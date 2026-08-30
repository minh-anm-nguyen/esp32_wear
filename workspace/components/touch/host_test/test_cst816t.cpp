// Tests for the CST816T logic layer, run on a PC with g++ against
// i2c::FakeBus. No chip, no ESP-IDF.
//   ./run_tests.sh
//
// Two things here are worth more than the rest.
//
// ORDER: applyConfig() must write IRQ_CTL last. Get that wrong and the chip
// starts pulsing the interrupt line while the rest of its configuration is
// half applied -- which produces no error anywhere, just occasional strange
// behaviour at boot. FakeBus's transaction log is what makes it visible.
//
// MASK ORDER: the event flag lives in the top two bits of the SAME byte as the
// high nibble of X. Masking the coordinate out first destroys it, and the
// resulting bug is invisible: coordinates stay perfect and only press/release
// goes wrong.
#include "cst816t.hpp"

#include "i2c_fake_bus.hpp"

#include <cstdio>
#include <string>

using namespace touch;

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

// Builds the six bytes exactly as the chip lays them out at 0x01..0x06.
static void makeFrame(uint8_t* f, uint8_t gesture, uint8_t fingers, EventFlag ev,
                      uint16_t x, uint16_t y)
{
    f[0] = gesture;
    f[1] = fingers;
    f[2] = static_cast<uint8_t>((static_cast<uint8_t>(ev) << 6) | ((x >> 8) & 0x0F));
    f[3] = static_cast<uint8_t>(x & 0xFF);
    f[4] = static_cast<uint8_t>((y >> 8) & 0x0F);
    f[5] = static_cast<uint8_t>(y & 0xFF);
}

static void primeFrame(i2c::FakeBus& bus, uint8_t gesture, uint8_t fingers, EventFlag ev,
                       uint16_t x, uint16_t y)
{
    uint8_t f[6];
    makeFrame(f, gesture, fingers, ev, x, y);
    for (uint8_t i = 0; i < 6; ++i) {
        bus.setReg(static_cast<uint8_t>(reg::FRAME_BASE + i), f[i]);
    }
}

// ------------------------------------------------------------------- parse

static void testParseValid()
{
    const RawLimits lim{};
    uint8_t         f[6];

    makeFrame(f, reg::GESTURE_NONE, 1, EventFlag::Contact, 100, 200);
    ParsedFrame p = parseFrame(f, sizeof(f), lim);

    EXPECT(p.status == FrameStatus::Valid, "status=%s", toString(p.status));
    EXPECT(p.fingers == 1, "fingers=%u", p.fingers);
    EXPECT(p.event == EventFlag::Contact, "event=%s", toString(p.event));
    EXPECT(p.rawX == 100, "rawX=%u", p.rawX);
    EXPECT(p.rawY == 200, "rawY=%u", p.rawY);
    EXPECT(!p.edgeClamped, "khong duoc clamp");
    EXPECT(!p.gestureUnknown, "gesture 0x00 phai la known");
    EXPECT(p.pressed(), "pressed()");
}

// The one that catches a mask-before-read bug: a 12-bit X large enough to use
// the high nibble, carried in the same byte as an event flag that is not zero.
static void testParseEventBitsSurviveCoordinate()
{
    const RawLimits lim{4096, 4096, 0};
    uint8_t         f[6];

    makeFrame(f, reg::GESTURE_NONE, 1, EventFlag::Down, 0x123, 0x0AB);
    ParsedFrame p = parseFrame(f, sizeof(f), lim);

    EXPECT(p.event == EventFlag::Down, "event=%s (mask lam hong event bits?)",
           toString(p.event));
    EXPECT(p.rawX == 0x123, "rawX=0x%03X", p.rawX);
    EXPECT(p.rawY == 0x0AB, "rawY=0x%03X", p.rawY);

    // And the reverse: a Contact flag must not leak into the coordinate.
    makeFrame(f, reg::GESTURE_NONE, 1, EventFlag::Contact, 0x0FF, 0x100);
    p = parseFrame(f, sizeof(f), lim);
    EXPECT(p.rawX == 0x0FF, "rawX=0x%03X (event bits lot vao toa do?)", p.rawX);
    EXPECT(p.event == EventFlag::Contact, "event=%s", toString(p.event));
}

static void testParseRejects()
{
    const RawLimits lim{};
    uint8_t         f[6];

    uint8_t allFf[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    ParsedFrame p    = parseFrame(allFf, sizeof(allFf), lim);
    EXPECT(p.status == FrameStatus::AllOnes, "0xFF -> %s", toString(p.status));
    EXPECT(!p.pressed(), "frame 0xFF khong duoc thanh mot cu nhan");

    makeFrame(f, reg::GESTURE_NONE, 0, EventFlag::Up, 0, 0);
    p = parseFrame(f, sizeof(f), lim);
    EXPECT(p.status == FrameStatus::NoFinger, "fingers=0 -> %s", toString(p.status));

    makeFrame(f, reg::GESTURE_NONE, 2, EventFlag::Contact, 10, 10);
    p = parseFrame(f, sizeof(f), lim);
    EXPECT(p.status == FrameStatus::BadFingerCount, "fingers=2 -> %s", toString(p.status));

    makeFrame(f, reg::GESTURE_NONE, 1, EventFlag::Reserved, 10, 10);
    p = parseFrame(f, sizeof(f), lim);
    EXPECT(p.status == FrameStatus::ReservedEvent, "event=11 -> %s", toString(p.status));

    // Short read must never be decoded.
    makeFrame(f, reg::GESTURE_NONE, 1, EventFlag::Contact, 10, 10);
    p = parseFrame(f, 5, lim);
    EXPECT(p.status == FrameStatus::AllOnes, "len=5 -> %s", toString(p.status));
    p = parseFrame(nullptr, 6, lim);
    EXPECT(p.status == FrameStatus::AllOnes, "nullptr -> %s", toString(p.status));
}

// The policy that keeps a corrupted frame from becoming a press on whatever
// widget sits at the screen edge.
static void testParseEdgeToleranceButNotClamping()
{
    const RawLimits lim{};  // 240x280, tolerance 1
    uint8_t         f[6];

    makeFrame(f, reg::GESTURE_NONE, 1, EventFlag::Contact, 240, 100);
    ParsedFrame p = parseFrame(f, sizeof(f), lim);
    EXPECT(p.status == FrameStatus::Valid, "x=240 -> %s", toString(p.status));
    EXPECT(p.rawX == 239, "x=240 phai keo ve 239, duoc %u", p.rawX);
    EXPECT(p.edgeClamped, "phai danh dau edgeClamped");

    makeFrame(f, reg::GESTURE_NONE, 1, EventFlag::Contact, 100, 280);
    p = parseFrame(f, sizeof(f), lim);
    EXPECT(p.status == FrameStatus::Valid, "y=280 -> %s", toString(p.status));
    EXPECT(p.rawY == 279, "y=280 phai keo ve 279, duoc %u", p.rawY);

    // Two past the edge is corruption, not a panel quirk.
    makeFrame(f, reg::GESTURE_NONE, 1, EventFlag::Contact, 241, 100);
    p = parseFrame(f, sizeof(f), lim);
    EXPECT(p.status == FrameStatus::OutOfRange, "x=241 -> %s", toString(p.status));

    makeFrame(f, reg::GESTURE_NONE, 1, EventFlag::Contact, 2000, 2000);
    p = parseFrame(f, sizeof(f), lim);
    EXPECT(p.status == FrameStatus::OutOfRange, "x=2000 -> %s", toString(p.status));
}

static void testParseGestureAndMismatch()
{
    const RawLimits lim{};
    uint8_t         f[6];

    makeFrame(f, 0x77, 1, EventFlag::Contact, 10, 10);
    ParsedFrame p = parseFrame(f, sizeof(f), lim);
    EXPECT(p.gestureUnknown, "0x77 phai la unknown gesture");
    EXPECT(p.status == FrameStatus::Valid, "gesture la khong lam hong frame: %s",
           toString(p.status));
    EXPECT(p.gestureCode == 0x77, "gestureCode=0x%02X", p.gestureCode);

    makeFrame(f, reg::GESTURE_BIG_PALM, 1, EventFlag::Contact, 10, 10);
    p = parseFrame(f, sizeof(f), lim);
    EXPECT(!p.gestureUnknown, "0xAA big palm la gesture cua rieng ban T");

    // finger says present, event says gone.
    makeFrame(f, reg::GESTURE_NONE, 1, EventFlag::Up, 10, 10);
    p = parseFrame(f, sizeof(f), lim);
    EXPECT(p.fingerEventMismatch, "fingers=1 + event=Up phai bao mismatch");

    makeFrame(f, reg::GESTURE_NONE, 0, EventFlag::Down, 10, 10);
    p = parseFrame(f, sizeof(f), lim);
    EXPECT(p.fingerEventMismatch, "fingers=0 + event=Down phai bao mismatch");
}

// ---------------------------------------------------------------- identify

static void testIdentify()
{
    i2c::FakeBus bus;
    Cst816t      dev{bus};

    bus.setReg(reg::CHIP_ID, 0xB6);
    bus.setReg(reg::PROJ_ID, 0x01);
    bus.setReg(reg::FW_VERSION, 0x03);
    bus.setReg(reg::FACTORY_ID, 0x12);

    ChipInfo info = dev.identify();
    EXPECT(info.read, "identify phai doc duoc");
    EXPECT(info.chipId == 0xB6, "chipId=0x%02X", info.chipId);
    EXPECT(info.factoryId == 0x12, "factoryId=0x%02X", info.factoryId);
    EXPECT(!info.implausible(), "gia tri that khong duoc coi la implausible");

    // ONE burst, not four single reads: the four identity registers are
    // contiguous precisely so this costs one transaction.
    EXPECT(bus.logCount() == 1, "log=%s", describe(bus).c_str());
    EXPECT(bus.log()[0].reg == reg::IDENT_BASE && bus.log()[0].len == 4,
           "log=%s", describe(bus).c_str());

    bus.reset();
    bus.failNext(1);
    info = dev.identify();
    EXPECT(!info.read, "NACK phai bao read=false");

    // All zeroes is what an absent chip reads back through a pull-down, and
    // all 0xFF through a pull-up. Neither is an identity.
    bus.reset();
    info = dev.identify();
    EXPECT(info.read, "doc thanh cong");
    EXPECT(info.implausible(), "toan 0x00 phai la implausible");
}

// ------------------------------------------------------------- applyConfig

static void testApplyConfigOrder()
{
    i2c::FakeBus bus;
    Cst816t      dev{bus};

    ChipConfig cfg{};
    cfg.verifyWrites = false;  // keep the log to writes only, for the order check

    ConfigResult res = dev.applyConfig(cfg);

    EXPECT(res.count == 5, "count=%u", static_cast<unsigned>(res.count));
    EXPECT(res.allWritesOk(), "moi write phai thanh cong");

    // The order that matters. IRQ_CTL last.
    const uint8_t want[5] = {reg::DIS_AUTO_SLEEP, reg::IRQ_PULSE_WIDTH, reg::MOTION_MASK,
                             reg::ERR_RESET_CTL, reg::IRQ_CTL};
    for (std::size_t i = 0; i < 5; ++i) {
        EXPECT(res.entries[i].regAddr == want[i],
               "entry %u: 0x%02X, mong doi 0x%02X", static_cast<unsigned>(i),
               res.entries[i].regAddr, want[i]);
    }
    EXPECT(bus.logCount() == 5, "verifyWrites=false thi khong duoc doc lai: %s",
           describe(bus).c_str());
    EXPECT(bus.log()[4].reg == reg::IRQ_CTL, "IRQ_CTL phai la lenh cuoi: %s",
           describe(bus).c_str());
}

static void testApplyConfigVerifies()
{
    i2c::FakeBus bus;
    Cst816t      dev{bus};

    ChipConfig cfg{};  // verifyWrites defaults true
    ConfigResult res = dev.applyConfig(cfg);

    EXPECT(res.allVerified(), "FakeBus luu lai gia tri nen phai verify duoc");
    EXPECT(res.firstMismatch() == 0, "firstMismatch=0x%02X", res.firstMismatch());
    EXPECT(bus.getReg(reg::IRQ_CTL) == (reg::IRQ_EN_TOUCH | reg::IRQ_EN_CHANGE),
           "IRQ_CTL=0x%02X", bus.getReg(reg::IRQ_CTL));
    EXPECT(bus.getReg(reg::DIS_AUTO_SLEEP) == reg::AUTO_SLEEP_ENABLED,
           "auto-sleep phai duoc BAT (0x00) mac dinh: 0x%02X",
           bus.getReg(reg::DIS_AUTO_SLEEP));

    // A chip that refuses one register must be named, not summarised.
    bus.reset();
    bus.failNext(1);  // the very first write, DIS_AUTO_SLEEP
    res = dev.applyConfig(cfg);
    EXPECT(!res.allWritesOk(), "write dau tien fail thi allWritesOk phai false");
    EXPECT(res.count == 5, "cac register con lai VAN phai duoc thu: count=%u",
           static_cast<unsigned>(res.count));
    EXPECT(res.entries[0].regAddr == reg::DIS_AUTO_SLEEP && !res.entries[0].writeOk,
           "entry 0 phai la 0xFE va that bai");
    EXPECT(res.entries[4].writeOk, "IRQ_CTL van phai duoc ghi");
}

static void testConfigByteEncoding()
{
    ChipConfig cfg{};
    EXPECT(Cst816t::irqCtlByte(cfg) == 0x60, "mac dinh EnTouch|EnChange = 0x60, duoc 0x%02X",
           Cst816t::irqCtlByte(cfg));

    cfg.irqOnMotion = true;
    EXPECT(Cst816t::irqCtlByte(cfg) == 0x70, "them EnMotion -> 0x70, duoc 0x%02X",
           Cst816t::irqCtlByte(cfg));

    cfg               = ChipConfig{};
    cfg.irqSelfTest   = true;
    cfg.irqOnTouch    = false;
    cfg.irqOnChange   = false;
    EXPECT(Cst816t::irqCtlByte(cfg) == 0x80, "chi EnTest -> 0x80, duoc 0x%02X",
           Cst816t::irqCtlByte(cfg));

    // The trap this project exists to avoid: an S-derived value of 10 means
    // 1.0 ms on an S and is outside the range on a T.
    cfg            = ChipConfig{};
    cfg.irqPulseMs = 10;
    EXPECT(Cst816t::irqPulseByte(cfg) == reg::IRQ_PULSE_MAX_MS,
           "10 phai bi clamp ve %u, duoc %u", reg::IRQ_PULSE_MAX_MS,
           Cst816t::irqPulseByte(cfg));
    cfg.irqPulseMs = 0;
    EXPECT(Cst816t::irqPulseByte(cfg) == reg::IRQ_PULSE_MIN_MS,
           "0 phai bi clamp len %u, duoc %u", reg::IRQ_PULSE_MIN_MS,
           Cst816t::irqPulseByte(cfg));

    cfg                  = ChipConfig{};
    cfg.disableAutoSleep = true;
    EXPECT(Cst816t::autoSleepByte(cfg) == reg::AUTO_SLEEP_DISABLED,
           "0x%02X", Cst816t::autoSleepByte(cfg));
    EXPECT(Cst816t::autoSleepByte(cfg) < 0xF0,
           "gia tri >= 0xF0 bi tai lieu T loai tru");
}

// ---------------------------------------------------------------- readFrame

static void testReadFrame()
{
    i2c::FakeBus bus;
    Cst816t      dev{bus};
    const RawLimits lim{};

    primeFrame(bus, reg::GESTURE_NONE, 1, EventFlag::Contact, 123, 45);

    bool        busOk = false;
    ParsedFrame p     = dev.readFrame(lim, busOk);

    EXPECT(busOk, "bus phai OK");
    EXPECT(p.status == FrameStatus::Valid, "status=%s", toString(p.status));
    EXPECT(p.rawX == 123 && p.rawY == 45, "(%u,%u)", p.rawX, p.rawY);

    // ONE transaction with a repeated START. Two reads would let the finger
    // move between X and Y, and would let another task take the shared bus in
    // the middle of a frame.
    EXPECT(bus.logCount() == 1, "phai la MOT giao dich: %s", describe(bus).c_str());
    EXPECT(bus.log()[0].reg == reg::FRAME_BASE && bus.log()[0].len == 6,
           "phai doc 6 byte tu 0x01: %s", describe(bus).c_str());

    // A NACK is not "no finger", and the caller must be able to tell.
    bus.clearLog();
    bus.failNext(1);
    p = dev.readFrame(lim, busOk);
    EXPECT(!busOk, "NACK phai bao busOk=false");

    // Raw bytes come back for the bring-up log.
    uint8_t raw[6] = {0};
    bus.clearLog();
    p = dev.readFrame(lim, busOk, raw);
    EXPECT(busOk && raw[1] == 1, "raw[1] (FingerNum) = %u", raw[1]);
    EXPECT(raw[3] == 123, "raw[3] (XposL) = %u", raw[3]);
}

static void testEnterDeepSleep()
{
    i2c::FakeBus bus;
    Cst816t      dev{bus};

    EXPECT(dev.enterDeepSleep(), "write phai thanh cong");
    EXPECT(bus.getReg(reg::SLEEP_MODE) == reg::SLEEP_MODE_ENTER,
           "0xE5 = 0x%02X", bus.getReg(reg::SLEEP_MODE));
    EXPECT(bus.logCount() == 1 && bus.log()[0].kind == i2c::FakeTransaction::Kind::Write,
           "log=%s", describe(bus).c_str());
}

// ---------------------------------------------------------------------- main

int main()
{
    std::printf("cst816t (lop logic thuan)\n");

    testParseValid();
    testParseEventBitsSurviveCoordinate();
    testParseRejects();
    testParseEdgeToleranceButNotClamping();
    testParseGestureAndMismatch();
    testIdentify();
    testApplyConfigOrder();
    testApplyConfigVerifies();
    testConfigByteEncoding();
    testReadFrame();
    testEnterDeepSleep();

    std::printf("  %d kiem tra, %d that bai\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
