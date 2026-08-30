// Tests for i2c::FakeBus, run on a PC with g++. No chip, no ESP-IDF.
//   ./run_tests.sh
//
// FakeBus is test infrastructure, so it gets tested itself. Every driver in
// this project will build its own tests on top of it, and a fake that lies
// about transaction ORDER would let a broken CTRL9 handshake pass.
#include "i2c_fake_bus.hpp"

#include <cstdio>
#include <string>
#include <vector>

using namespace i2c;

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

static std::vector<FakeTransaction> logOf(const FakeBus& bus)
{
    return std::vector<FakeTransaction>(bus.log(), bus.log() + bus.logCount());
}

static std::string describe(const std::vector<FakeTransaction>& v)
{
    if (v.empty()) return "(rong)";
    std::string s;
    char        buf[48];
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i) s += " , ";
        std::snprintf(buf, sizeof(buf), "%s(0x%02X,%u)",
                      v[i].kind == FakeTransaction::Kind::Read ? "R" : "W",
                      v[i].reg, static_cast<unsigned>(v[i].len));
        s += buf;
    }
    return s;
}

// ----------------------------------------------------------------- the tests

static void testReadWriteRoundTrip()
{
    FakeBus bus;
    EXPECT(bus.writeReg(0x2A, 0x5C), "writeReg phai thanh cong");
    EXPECT(bus.getReg(0x2A) == 0x5C, "gia tri phai vao dung thanh ghi");

    uint8_t got = 0;
    EXPECT(bus.readRegs(0x2A, &got, 1), "readRegs phai thanh cong");
    EXPECT(got == 0x5C, "doc lai phai ra 0x5C, duoc 0x%02X", got);
}

static void testBurstReadIsOneTransaction()
{
    // The whole point of readRegs(): the QMI8658C's 12 sample bytes must leave
    // as ONE repeated-START transaction, not twelve.
    FakeBus bus;
    for (uint8_t i = 0; i < 12; ++i) {
        bus.setReg(static_cast<uint8_t>(0x35 + i), static_cast<uint8_t>(0xA0 + i));
    }
    bus.clearLog();

    uint8_t buf[12]{};
    EXPECT(bus.readRegs(0x35, buf, sizeof(buf)), "burst read phai thanh cong");

    for (uint8_t i = 0; i < 12; ++i) {
        EXPECT(buf[i] == static_cast<uint8_t>(0xA0 + i),
               "byte %u phai la 0x%02X, duoc 0x%02X", i, 0xA0 + i, buf[i]);
    }

    const auto lg = logOf(bus);
    EXPECT(lg.size() == 1, "phai la DUNG MOT giao dich, duoc %s",
           describe(lg).c_str());
    EXPECT(lg.size() == 1 && lg[0].len == 12, "giao dich phai dai 12 byte");
}

static void testWriteRegsIsOneTransaction()
{
    // The QMI8658C pedometer load writes CAL1_L..CAL4_H, 8 consecutive
    // registers. Eight separate writes would release the bus seven times.
    FakeBus bus;
    const uint8_t params[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    EXPECT(bus.writeRegs(0x0B, params, sizeof(params)), "writeRegs phai thanh cong");

    for (uint8_t i = 0; i < 8; ++i) {
        EXPECT(bus.getReg(static_cast<uint8_t>(0x0B + i)) == params[i],
               "CAL byte %u sai", i);
    }

    const auto lg = logOf(bus);
    EXPECT(lg.size() == 1, "8 thanh ghi phai di trong MOT giao dich, duoc %s",
           describe(lg).c_str());
}

static void testTransactionOrderIsRecorded()
{
    // A CTRL9 handshake in miniature: params -> command -> poll status -> ACK.
    // The test that matters for a driver is that these happened IN THIS ORDER.
    FakeBus bus;
    const uint8_t params[2] = {0xCC, 0x00};

    bus.writeRegs(0x0B, params, sizeof(params));  // CAL1_L, CAL1_H
    bus.writeReg(0x0A, 0x08);                     // CTRL9 = WRITE_WOM_SETTING
    uint8_t status = 0;
    bus.readRegs(0x2D, &status, 1);               // poll STATUSINT
    bus.writeReg(0x0A, 0x00);                     // CTRL_CMD_ACK

    const std::vector<FakeTransaction> want = {
        {FakeTransaction::Kind::Write, 0x0B, 2},  // CAL1_L, CAL1_H
        {FakeTransaction::Kind::Write, 0x0A, 1},  // CTRL9 = cmd
        {FakeTransaction::Kind::Read, 0x2D, 1},   // STATUSINT
        {FakeTransaction::Kind::Write, 0x0A, 1},  // CTRL9 = ACK
    };
    const auto got = logOf(bus);
    EXPECT(got == want, "thu tu sai.\n        duoc : %s\n        mong doi: %s",
           describe(got).c_str(), describe(want).c_str());
}

static void testFailInjection()
{
    FakeBus bus;
    bus.setReg(0x00, 0x05);

    bus.failNext(2);

    uint8_t v = 0;
    EXPECT(!bus.readRegs(0x00, &v, 1), "lan 1 phai that bai");
    EXPECT(!bus.writeReg(0x01, 0xFF), "lan 2 phai that bai");
    EXPECT(bus.readRegs(0x00, &v, 1), "lan 3 phai thanh cong tro lai");
    EXPECT(v == 0x05, "gia tri sau khi khoi phuc phai dung");

    // A NACKed write must not have landed.
    EXPECT(bus.getReg(0x01) == 0x00, "ghi that bai KHONG duoc thay doi thanh ghi");

    // Failed transactions never reached the device, so they are not logged.
    const auto lg = logOf(bus);
    EXPECT(lg.size() == 1, "chi 1 giao dich thanh cong duoc ghi log, duoc %s",
           describe(lg).c_str());
}

static void testRegisterPointerWraps()
{
    // The register pointer is 8-bit and wraps, exactly as a real chip's does.
    // A driver that reads past 0xFF has a bug; the fake must not hide it by
    // reading out of bounds.
    FakeBus bus;
    bus.setReg(0xFF, 0xAA);
    bus.setReg(0x00, 0xBB);

    uint8_t buf[2]{};
    EXPECT(bus.readRegs(0xFF, buf, 2), "doc vat qua bien phai thanh cong");
    EXPECT(buf[0] == 0xAA, "byte 0 phai la 0xAA");
    EXPECT(buf[1] == 0xBB, "byte 1 phai cuon ve 0x00 -> 0xBB");
}

static void testLogOverflowIsVisible()
{
    // A silently truncated log would make an order assertion pass by accident.
    FakeBus bus;
    for (std::size_t i = 0; i < FakeBus::kLogCapacity + 5; ++i) {
        bus.writeReg(0x10, 0x00);
    }
    EXPECT(bus.logCount() == FakeBus::kLogCapacity, "log phai dung o suc chua");
    EXPECT(bus.logOverflowed(), "tran log phai duoc bao, khong duoc im lang");

    bus.clearLog();
    EXPECT(bus.logCount() == 0, "clearLog() phai xoa");
    EXPECT(!bus.logOverflowed(), "clearLog() phai xoa ca co tran");
}

static void testResetClearsEverything()
{
    FakeBus bus;
    bus.setReg(0x42, 0x99);
    bus.writeReg(0x10, 0x11);
    bus.failNext(3);

    bus.reset();

    EXPECT(bus.getReg(0x42) == 0x00, "reset() phai xoa thanh ghi");
    EXPECT(bus.logCount() == 0, "reset() phai xoa log");
    uint8_t v = 0;
    EXPECT(bus.readRegs(0x00, &v, 1), "reset() phai huy bom loi con lai");
}

static void testUsableThroughBaseInterface()
{
    // Every driver takes an i2c::Bus&, never a FakeBus&. If the fake were not
    // substitutable through the base interface it would be testing the wrong
    // thing -- this is the Liskov check, cheap and worth having.
    FakeBus concrete;
    Bus&    bus = concrete;

    EXPECT(bus.writeReg(0x20, 0x77), "ghi qua Bus& phai thanh cong");
    uint8_t v = 0;
    EXPECT(bus.readRegs(0x20, &v, 1) && v == 0x77, "doc qua Bus& phai dung");
    EXPECT(concrete.logCount() == 2, "trang thai cu the van quan sat duoc");
}

int main()
{
    struct Test {
        const char* name;
        void (*fn)();
    };

    const Test tests[] = {
        {"doc/ghi mot thanh ghi",                     testReadWriteRoundTrip},
        {"burst 12 byte = mot giao dich",             testBurstReadIsOneTransaction},
        {"ghi 8 thanh ghi lien tiep = mot giao dich", testWriteRegsIsOneTransaction},
        {"nhat ky giu dung THU TU giao dich",         testTransactionOrderIsRecorded},
        {"bom loi roi tu khoi phuc",                  testFailInjection},
        {"con tro thanh ghi cuon vong tai 0xFF",      testRegisterPointerWraps},
        {"tran nhat ky duoc bao ro",                  testLogOverflowIsVisible},
        {"reset()",                                   testResetClearsEverything},
        {"thay the duoc qua i2c::Bus&",               testUsableThroughBaseInterface},
    };

    std::printf("=== test i2c::FakeBus (logic thuan) ===\n");
    for (auto& t : tests) {
        int before = g_failed;
        t.fn();
        std::printf("  [%s] %s\n", g_failed == before ? "OK  " : "FAIL", t.name);
    }
    std::printf("--- %d kiem tra, %d that bai ---\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
