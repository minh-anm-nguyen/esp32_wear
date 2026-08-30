// Tests for the QMI8658C logic layer, run on a PC with g++ against
// i2c::FakeBus. No chip, no ESP-IDF.
//   ./run_tests.sh
//
// The point of most of these is ORDER. A CTRL9 handshake or a WoM setup that
// writes the right bytes in the wrong sequence produces no error anywhere --
// the chip simply ignores it. FakeBus's transaction log is what makes that
// failure visible on a laptop instead of on a wrist.
#include "qmi8658.hpp"

#include "i2c_fake_bus.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace imu;

// ------------------------------------------------------------ test harness

static int g_checks = 0;
static int g_failed = 0;

#define EXPECT(cond, ...)                                            \
    do {                                                             \
        ++g_checks;                                                  \
        if (!(cond)) {                                               \
            ++g_failed;                                              \
            std::printf("  FAIL  %s:%d  ", __FILE__, __LINE__);      \
            std::printf(__VA_ARGS__);                                \
            std::printf("\n");                                       \
        }                                                            \
    } while (0)

static bool nearly(float a, float b, float tol = 0.01f)
{
    return std::fabs(a - b) <= tol;
}

static std::string describe(const i2c::FakeBus& bus)
{
    if (bus.logCount() == 0) return "(rong)";
    std::string s;
    char        buf[48];
    for (std::size_t i = 0; i < bus.logCount(); ++i) {
        if (i) s += " , ";
        std::snprintf(buf, sizeof(buf), "%s(0x%02X,%u)",
                      bus.log()[i].kind == i2c::FakeTransaction::Kind::Read ? "R" : "W",
                      bus.log()[i].reg,
                      static_cast<unsigned>(bus.log()[i].len));
        s += buf;
    }
    return s;
}

// A chip that is present and answers correctly.
static void primeIdentity(i2c::FakeBus& bus)
{
    bus.setReg(reg::WHO_AM_I, reg::WHO_AM_I_VALUE);
    bus.setReg(reg::REVISION_ID, reg::REVISION_ID_VALUE);
}

// ----------------------------------------------------------------- the tests

static void testProbeAcceptsCorrectChip()
{
    i2c::FakeBus bus;
    primeIdentity(bus);
    Qmi8658 dev(bus);

    EXPECT(dev.probe(), "chip dung phai duoc chap nhan");

    // identify() reports what it saw, so a failure can be explained instead of
    // just announced.
    Qmi8658::IdentityReport r{};
    EXPECT(dev.identify(r), "identify() phai thanh cong");
    EXPECT(r.isQmi8658, "phai nhan ra la QMI8658C");
    EXPECT(r.autoIncrement, "FakeBus co tang con tro -> phai bao auto-increment OK");
    EXPECT(!r.needsHighBitBurst, "khong can quy uoc reg|0x80");
    EXPECT(r.whoAmI == 0x05 && r.revision == 0x68, "bao cao dung hai byte");
}

static void testIdentifyAcceptsUnexpectedRevision()
{
    // A part whose REVISION_ID is not 0x68 is not a broken part. The old check
    // compared against a hard coded 0x68 and would have condemned it; the
    // auto-increment test now compares the burst against what the SINGLE reads
    // actually returned.
    i2c::FakeBus bus;
    bus.setReg(reg::WHO_AM_I, reg::WHO_AM_I_VALUE);
    bus.setReg(reg::REVISION_ID, 0x7C);  // some other revision
    Qmi8658 dev(bus);

    Qmi8658::IdentityReport r{};
    EXPECT(dev.identify(r), "revision la phai van duoc chap nhan");
    EXPECT(r.isQmi8658 && r.autoIncrement, "van la chip dung, van burst duoc");
    EXPECT(r.revision == 0x7C, "phai bao cao dung revision doc duoc");
}

static void testProbeRejectsWrongChip()
{
    i2c::FakeBus bus;
    bus.setReg(reg::WHO_AM_I, 0x6A);
    Qmi8658 dev(bus);
    EXPECT(!dev.probe(), "chip la phai bi tu choi");
}

static void testResetSequence()
{
    i2c::FakeBus bus;
    Qmi8658      dev(bus);

    EXPECT(dev.beginReset(), "ghi lenh reset phai thanh cong");
    EXPECT(bus.getReg(reg::RESET) == 0xB0, "phai ghi 0xB0 (KHONG phai 0x0B)");

    EXPECT(!dev.resetSucceeded(), "chua co 0x80 thi phai bao that bai");
    bus.setReg(reg::RESET_RESULT, reg::RESET_RESULT_VALUE);
    EXPECT(dev.resetSucceeded(), "0x4D == 0x80 phai la thanh cong");
}

static void testBoardSetupBits()
{
    i2c::FakeBus bus;
    Qmi8658      dev(bus);

    EXPECT(dev.setAddressAutoIncrement(true), "dat ADDR_AI phai thanh cong");
    EXPECT((bus.getReg(reg::CTRL1) & reg::ctrl1::ADDR_AI) != 0, "CTRL1.bit6 phai len");

    EXPECT(dev.setCtrl9Handshake(Ctrl9Handshake::StatusRegister), "dat handshake");
    EXPECT((bus.getReg(reg::CTRL8) & reg::ctrl8::HANDSHAKE_VIA_STATUS) != 0,
           "CTRL8.bit7 phai len -- neu khong moi lenh CTRL9 se treo");

    EXPECT(dev.setActivityIntPin(IntPin::Int2), "dat chan ngat");
    EXPECT((bus.getReg(reg::CTRL8) & reg::ctrl8::ACTIVITY_INT_SEL_INT1) == 0,
           "CTRL8.bit6 phai la 0 = INT2");

    // Read-modify-write must not stomp the neighbouring bit.
    EXPECT((bus.getReg(reg::CTRL8) & reg::ctrl8::HANDSHAKE_VIA_STATUS) != 0,
           "dat bit6 KHONG duoc xoa bit7");
}

static void testCtrl9HandshakeOrder()
{
    i2c::FakeBus bus;
    bus.setReg(reg::STATUSINT, reg::statusint::CMD_DONE);  // device reports done
    Qmi8658 dev(bus);
    bus.clearLog();

    EXPECT(dev.ctrl9Command(reg::cmd::WRITE_WOM_SETTING), "bat tay phai thanh cong");

    const std::vector<i2c::FakeTransaction> want = {
        {i2c::FakeTransaction::Kind::Write, reg::CTRL9, 1},     // command
        {i2c::FakeTransaction::Kind::Read, reg::STATUSINT, 1},  // poll CmdDone
        {i2c::FakeTransaction::Kind::Write, reg::CTRL9, 1},     // ACK
    };
    const std::vector<i2c::FakeTransaction> got(bus.log(), bus.log() + bus.logCount());
    EXPECT(got == want, "sai thu tu bat tay: %s", describe(bus).c_str());
    EXPECT(bus.getReg(reg::CTRL9) == reg::cmd::ACK,
           "gia tri cuoi cung o CTRL9 phai la ACK (0x00), duoc 0x%02X",
           bus.getReg(reg::CTRL9));
}

static void testCtrl9TimesOutInsteadOfHanging()
{
    // STATUSINT never sets CmdDone -- exactly what happens if CTRL8.bit7 was
    // left clear on a board with no INT1. This must END, not spin forever.
    i2c::FakeBus bus;
    Qmi8658      dev(bus);

    EXPECT(!dev.ctrl9Command(reg::cmd::WRITE_WOM_SETTING),
           "khong bao gio CmdDone thi phai tra ve that bai");
    EXPECT(bus.getReg(reg::CTRL9) == reg::cmd::WRITE_WOM_SETTING,
           "khong duoc ACK mot lenh chua hoan tat");
}

static void testCtrl9PropagatesBusFailure()
{
    i2c::FakeBus bus;
    bus.setReg(reg::STATUSINT, reg::statusint::CMD_DONE);
    Qmi8658 dev(bus);

    bus.failNext(1);
    EXPECT(!dev.ctrl9Command(reg::cmd::RST_FIFO), "loi bus phai noi ra ngoai");
}

static void testWakeOnMotionSequence()
{
    i2c::FakeBus bus;
    bus.setReg(reg::STATUSINT, reg::statusint::CMD_DONE);
    Qmi8658 dev(bus);

    WomConfig wom{};
    wom.thresholdMg     = 200;
    wom.blankingSamples = 8;
    wom.intPin          = IntPin::Int2;
    dev.setWomConfig(wom);
    dev.setRanges(AccelRange::G4, GyroRange::DPS256);
    bus.clearLog();

    EXPECT(dev.configureWakeOnMotion(), "cau hinh WoM phai thanh cong");

    // Rev A section 9.4 order: sensors off -> CTRL2 -> CAL1 -> CTRL9 -> aEN.
    const std::vector<i2c::FakeTransaction> want = {
        {i2c::FakeTransaction::Kind::Write, reg::CTRL7, 1},      // 1. all off
        {i2c::FakeTransaction::Kind::Write, reg::CTRL2, 1},      // 2. FS + LP ODR
        {i2c::FakeTransaction::Kind::Write, reg::CAL1_L, 2},     // 3. thr + blanking
        {i2c::FakeTransaction::Kind::Write, reg::CTRL9, 1},      // 4. command
        {i2c::FakeTransaction::Kind::Read, reg::STATUSINT, 1},   //    poll
        {i2c::FakeTransaction::Kind::Write, reg::CTRL9, 1},      //    ACK
        {i2c::FakeTransaction::Kind::Write, reg::CTRL7, 1},      // 5. accel on
    };
    const std::vector<i2c::FakeTransaction> got(bus.log(), bus.log() + bus.logCount());
    EXPECT(got == want, "sai trinh tu WoM: %s", describe(bus).c_str());

    // The two CAL1 bytes must go as ONE transaction, not two.
    EXPECT(bus.getReg(reg::CAL1_L) == 200, "nguong phai la 200 mg");
    EXPECT((bus.getReg(reg::CAL1_H) & 0xC0) == 0x40, "CAL1_H[7:6] phai chon INT2");
    EXPECT((bus.getReg(reg::CAL1_H) & 0x3F) == 8, "blanking phai la 8 mau");

    // Low-power ODR, and only the accelerometer enabled.
    EXPECT((bus.getReg(reg::CTRL2) & 0x0F) == 0x0E, "aODR phai la low-power 11 Hz");
    EXPECT(bus.getReg(reg::CTRL7) == reg::ctrl7::AEN, "chi aEN, tuyet doi khong gEN");
}

static void testLeavingWomRunsTheExitSequence()
{
    // Rev A section 12.6. Writing new CTRL2/CTRL7 values does NOT take the
    // part out of WoM -- the mode ends only when the threshold is zeroed and
    // the CTRL9 command is re-issued. Without this the chip keeps behaving as
    // WoM (no data-ready, no samples) while every register reads back exactly
    // as the new mode intends. That is a failure with no symptom anywhere
    // except an accelerometer that never produces a sample.
    i2c::FakeBus bus;
    bus.setReg(reg::STATUSINT, reg::statusint::CMD_DONE);
    Qmi8658  dev(bus);
    uint32_t settle = 0;

    EXPECT(dev.applyMode(PowerMode::WOM, settle), "vao WOM");
    EXPECT(dev.womArmed(), "sau WOM phai danh dau la dang armed");
    EXPECT(bus.getReg(reg::CAL1_L) == 200, "nguong WoM phai khac 0");
    bus.clearLog();

    EXPECT(dev.applyMode(PowerMode::ACTIVITY, settle), "chuyen sang ACTIVITY");
    EXPECT(!dev.womArmed(), "ra khoi WOM phai xoa co armed");
    EXPECT(bus.getReg(reg::CAL1_L) == 0,
           "nguong WoM PHAI ve 0, duoc %u", bus.getReg(reg::CAL1_L));

    // The exit must come FIRST, before the new mode is written -- and it must
    // be the full CTRL7=0 / threshold=0 / CTRL9 sequence.
    const std::vector<i2c::FakeTransaction> got(bus.log(), bus.log() + bus.logCount());
    EXPECT(got.size() >= 6, "trinh tu thoat WoM qua ngan: %s", describe(bus).c_str());
    if (got.size() >= 6) {
        EXPECT(got[0] == (i2c::FakeTransaction{i2c::FakeTransaction::Kind::Write, reg::CTRL7, 1}),
               "buoc 1 phai tat het cam bien");
        EXPECT(got[1] == (i2c::FakeTransaction{i2c::FakeTransaction::Kind::Write, reg::CAL1_L, 2}),
               "buoc 2 phai ghi nguong 0");
        EXPECT(got[2] == (i2c::FakeTransaction{i2c::FakeTransaction::Kind::Write, reg::CTRL9, 1}),
               "buoc 3 phai la lenh CTRL9");
    }

    EXPECT(bus.getReg(reg::CTRL7) == reg::ctrl7::AEN, "ket thuc o ACTIVITY: chi aEN");
    EXPECT((bus.getReg(reg::CTRL2) & 0x0F) == 0x07, "ket thuc o aODR 62.5 Hz");
}

static void testPowerModes()
{
    i2c::FakeBus bus;
    bus.setReg(reg::STATUSINT, reg::statusint::CMD_DONE);
    Qmi8658  dev(bus);
    uint32_t settle = 0;

    EXPECT(dev.applyMode(PowerMode::ACTIVITY, settle), "vao ACTIVITY");
    EXPECT(dev.mode() == PowerMode::ACTIVITY, "mode phai duoc ghi nhan");
    EXPECT(bus.getReg(reg::CTRL7) == reg::ctrl7::AEN, "ACTIVITY: chi accel");
    EXPECT((bus.getReg(reg::CTRL2) & 0x0F) == 0x07, "ACTIVITY: aODR = 62.5 Hz");
    EXPECT(nearly(dev.accelOdrHz(), 62.5f), "ACTIVITY ODR that = 62.5, duoc %.2f",
           dev.accelOdrHz());
    EXPECT(nearly(dev.gyroOdrHz(), 0.0f), "gyro tat thi gyroOdrHz() = 0");
    EXPECT(settle >= 3 + 48 && settle <= 3 + 48 + 2,
           "settle = wake 3ms + 3/ODR ~ 48ms, duoc %u", settle);

    EXPECT(dev.applyMode(PowerMode::IMU6, settle), "vao IMU6");
    EXPECT(bus.getReg(reg::CTRL7) == (reg::ctrl7::AEN | reg::ctrl7::GEN),
           "IMU6: ca accel lan gyro");
    EXPECT(settle >= 60, "IMU6 phai cong thoi gian danh thuc gyro 60 ms, duoc %u",
           settle);

    EXPECT(dev.applyMode(PowerMode::OFF, settle), "vao OFF");
    EXPECT(bus.getReg(reg::CTRL7) == 0x00, "OFF: CTRL7 = 0");
    EXPECT((bus.getReg(reg::CTRL1) & reg::ctrl1::SENSOR_DISABLE) == 0,
           "OFF KHONG duoc dung sensorDisable -- quay lai ton 1.75 s");
}

static void testAccelOdrChangesWhenGyroTurnsOn()
{
    // The headline trap of this chip: the same aODR nibble means one frequency
    // with the gyro off and another with it on. A dt computed from a constant
    // is 11% wrong the moment gesture recognition switches the gyro on.
    i2c::FakeBus bus;
    bus.setReg(reg::STATUSINT, reg::statusint::CMD_DONE);
    Qmi8658  dev(bus);
    uint32_t settle = 0;

    EXPECT(nearly(accelOdrHzFor(0x06, false), 125.0f), "code 6, gyro OFF -> 125 Hz");
    EXPECT(nearly(accelOdrHzFor(0x06, true), 112.1f), "code 6, gyro ON  -> 112.1 Hz");
    EXPECT(nearly(accelOdrHzFor(0x07, false), 62.5f), "code 7, gyro OFF -> 62.5 Hz");
    EXPECT(nearly(accelOdrHzFor(0x07, true), 56.05f), "code 7, gyro ON  -> 56.05 Hz");

    // Low power exists ONLY with the gyro off -- the register-level reason WoM
    // and the gyro are mutually exclusive.
    EXPECT(nearly(accelOdrHzFor(0x0E, false), 11.0f), "low-power 11 Hz khi gyro tat");
    EXPECT(nearly(accelOdrHzFor(0x0E, true), 0.0f), "low-power KHONG ton tai o 6DOF");

    dev.applyMode(PowerMode::IMU6, settle);
    EXPECT(nearly(dev.accelOdrHz(), 112.1f), "IMU6: accel that su chay 112.1 Hz");
    EXPECT(nearly(dev.gyroOdrHz(), 112.1f), "IMU6: gyro 112.1 Hz");
}

static void testSampleDecoding()
{
    i2c::FakeBus bus;
    Qmi8658      dev(bus);

    // Little endian, low byte at the lower address.
    const int16_t want[6] = {1000, -2000, 16384, 300, -300, 32767};
    for (int i = 0; i < 6; ++i) {
        const uint16_t u = static_cast<uint16_t>(want[i]);
        bus.setReg(static_cast<uint8_t>(reg::AX_L + 2 * i),
                   static_cast<uint8_t>(u & 0xFF));
        bus.setReg(static_cast<uint8_t>(reg::AX_L + 2 * i + 1),
                   static_cast<uint8_t>(u >> 8));
    }
    bus.clearLog();

    RawSample s{};
    EXPECT(dev.readSample(s), "doc mau phai thanh cong");
    EXPECT(bus.logCount() == 1 && bus.log()[0].len == 12,
           "12 byte phai di trong MOT burst, duoc %s", describe(bus).c_str());

    EXPECT(s.accel[0] == 1000, "ax");
    EXPECT(s.accel[1] == -2000, "ay am phai giai ma dung, duoc %d", s.accel[1]);
    EXPECT(s.accel[2] == 16384, "az");
    EXPECT(s.gyro[0] == 300, "gx");
    EXPECT(s.gyro[1] == -300, "gy am, duoc %d", s.gyro[1]);
    EXPECT(s.gyro[2] == 32767, "gz bien tren");
}

static void testUnitConversion()
{
    EXPECT(nearly(accelLsbPerG(AccelRange::G2), 16384.0f), "+/-2g");
    EXPECT(nearly(accelLsbPerG(AccelRange::G4), 8192.0f), "+/-4g");
    EXPECT(nearly(accelLsbPerG(AccelRange::G8), 4096.0f), "+/-8g");
    EXPECT(nearly(accelLsbPerG(AccelRange::G16), 2048.0f), "+/-16g");

    EXPECT(nearly(gyroLsbPerDps(GyroRange::DPS16), 2048.0f), "+/-16dps");
    EXPECT(nearly(gyroLsbPerDps(GyroRange::DPS256), 128.0f), "+/-256dps");
    EXPECT(nearly(gyroLsbPerDps(GyroRange::DPS1024), 32.0f), "+/-1024dps");

    // A watch lying still must read 1 g. This is the acceptance criterion that
    // replaces "measure it with a meter" -- section 15.
    const float oneG = 8192.0f / accelLsbPerG(AccelRange::G4);
    EXPECT(nearly(oneG, 1.0f), "8192 LSB o +/-4g phai la 1.00 g, duoc %.3f", oneG);

    EXPECT(nearly(tempLsbToC(256 * 25), 25.0f), "nhiet do 256 LSB/degC");
    EXPECT(nearly(tempLsbToC(-256 * 10), -10.0f), "nhiet do am");
}

static void testStatusDecodedInOneRead()
{
    // STATUS1 is read-to-clear, so the driver gets exactly one look. Everything
    // must be decoded from that single burst.
    i2c::FakeBus bus;
    bus.setReg(reg::STATUS0, reg::status0::ADA | reg::status0::GDA);
    bus.setReg(reg::STATUS1, reg::status1::WOM | reg::status1::PEDOMETER |
                                 reg::status1::NO_MOTION);
    Qmi8658 dev(bus);
    bus.clearLog();

    StatusFlags f{};
    EXPECT(dev.readStatus(f), "doc trang thai phai thanh cong");
    EXPECT(bus.logCount() == 1 && bus.log()[0].len == 2,
           "STATUS0+STATUS1 phai doc trong MOT burst, duoc %s", describe(bus).c_str());

    EXPECT(f.accelReady, "aDA");
    EXPECT(f.gyroReady, "gDA");
    EXPECT(f.wakeOnMotion, "WoM");
    EXPECT(f.step, "pedometer");
    EXPECT(f.noMotion, "no-motion");
    EXPECT(!f.tap, "tap khong duoc bao khi khong co");
    EXPECT(!f.sigMotion, "sig-motion khong duoc bao khi khong co");
}

static void testStepCount24Bit()
{
    i2c::FakeBus bus;
    bus.setReg(reg::STEP_CNT_LOW, 0x10);
    bus.setReg(static_cast<uint8_t>(reg::STEP_CNT_LOW + 1), 0x27);
    bus.setReg(static_cast<uint8_t>(reg::STEP_CNT_LOW + 2), 0x01);
    Qmi8658 dev(bus);

    uint32_t steps = 0;
    EXPECT(dev.readStepCount(steps), "doc so buoc");
    EXPECT(steps == 0x012710u, "24-bit little endian: mong doi 75536, duoc %u", steps);
}

static void testBusFailuresPropagate()
{
    i2c::FakeBus bus;
    primeIdentity(bus);
    Qmi8658 dev(bus);

    bus.failNext(1);
    EXPECT(!dev.probe(), "probe phai that bai khi bus loi");

    bus.failNext(1);
    RawSample s{};
    EXPECT(!dev.readSample(s), "readSample phai that bai khi bus loi");

    bus.failNext(1);
    uint32_t settle = 0;
    EXPECT(!dev.applyMode(PowerMode::ACTIVITY, settle),
           "applyMode phai that bai khi bus loi");
    EXPECT(dev.mode() == PowerMode::OFF,
           "applyMode that bai KHONG duoc doi mode_ da luu");
}

int main()
{
    struct Test {
        const char* name;
        void (*fn)();
    };

    const Test tests[] = {
        {"probe() chap nhan chip dung",              testProbeAcceptsCorrectChip},
        {"identify() chap nhan revision la",         testIdentifyAcceptsUnexpectedRevision},
        {"probe() tu choi chip la",                  testProbeRejectsWrongChip},
        {"reset ghi 0xB0 va kiem 0x4D",              testResetSequence},
        {"bit cau hinh bo mach (CTRL1/CTRL8)",       testBoardSetupBits},
        {"CTRL9: dung THU TU lenh-cho-ACK",          testCtrl9HandshakeOrder},
        {"CTRL9 het gio thay vi treo",               testCtrl9TimesOutInsteadOfHanging},
        {"CTRL9 bao loi bus ra ngoai",               testCtrl9PropagatesBusFailure},
        {"WoM dung trinh tu 5 buoc cua Rev A",       testWakeOnMotionSequence},
        {"ra khoi WOM chay dung trinh tu thoat",      testLeavingWomRunsTheExitSequence},
        {"chuyen che do dien + thoi gian on dinh",   testPowerModes},
        {"BAY: aODR doi nghia khi bat gyro",         testAccelOdrChangesWhenGyroTurnsOn},
        {"giai ma 12 byte mau trong mot burst",      testSampleDecoding},
        {"quy doi don vi (g, dps, degC)",            testUnitConversion},
        {"STATUS0+STATUS1 giai ma trong MOT lan doc", testStatusDecodedInOneRead},
        {"so buoc 24-bit",                           testStepCount24Bit},
        {"loi bus lan truyen ra ngoai",              testBusFailuresPropagate},
    };

    std::printf("=== test Qmi8658 (logic thuan) ===\n");
    for (auto& t : tests) {
        int before = g_failed;
        t.fn();
        std::printf("  [%s] %s\n", g_failed == before ? "OK  " : "FAIL", t.name);
    }
    std::printf("--- %d kiem tra, %d that bai ---\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
