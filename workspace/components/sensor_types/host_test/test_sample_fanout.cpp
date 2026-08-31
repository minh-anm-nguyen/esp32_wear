// Tests for the one-producer/many-consumer fan-out. Pure logic, runs on a PC.
//   ./run_tests.sh
//
// The bugs this file exists to prevent, in order of how quietly they fail:
//   1. A sink registered twice -> every accumulator downstream doubles.
//   2. add() failing (full/sealed) and nobody checking -> a consumer that is
//      simply never called, with no error anywhere.
//   3. The sample being altered on the way through -> consumers disagree about
//      what the sensor said.
#include "sample_fanout.hpp"

#include <cstdio>

using namespace sensors;

static int g_checks = 0;
static int g_failed = 0;

#define EXPECT(cond, ...)                                       \
    do {                                                        \
        ++g_checks;                                             \
        if (!(cond)) {                                          \
            ++g_failed;                                         \
            std::printf("  FAIL  %s:%d  ", __FILE__, __LINE__); \
            std::printf(__VA_ARGS__);                           \
            std::printf("\n");                                  \
        }                                                       \
    } while (0)

// Records what it was handed, so the test can prove the payload survived.
class Recorder final : public ISampleSink {
public:
    explicit Recorder(int id) : id_(id) {}

    void onSample(const Sample& s, uint32_t nowMs) override
    {
        ++calls;
        last    = s;
        lastNow = nowMs;
        if (sharedOrder != nullptr && *sharedCount < kOrderMax) {
            sharedOrder[(*sharedCount)++] = id_;
        }
    }

    static constexpr int kOrderMax = 16;

    int      id_{};
    uint32_t calls{0};
    Sample   last{};
    uint32_t lastNow{0};

    // When set, every delivery appends this recorder's id -- that is how the
    // order test observes the sequence without the fan-out exposing it.
    int* sharedOrder{nullptr};
    int* sharedCount{nullptr};
};

static Sample makeSample(float ax, uint32_t ts)
{
    Sample s{};
    s.accelG[0]   = ax;
    s.accelG[1]   = 0.25f;
    s.accelG[2]   = -1.0f;
    s.gyroDps[0]  = 12.5f;
    s.gyroDps[1]  = -3.0f;
    s.gyroDps[2]  = 0.5f;
    s.timestampMs = ts;
    return s;
}

// --------------------------------------------------------------- basic wiring

static void testEmptyFanoutIsHarmless()
{
    SampleFanout<4> f;
    EXPECT(f.count() == 0, "fanout rong phai co count()==0");
    EXPECT(f.capacity() == 4, "capacity() phai bang N");

    // Must not crash, must still count the sample as seen.
    f.onSample(makeSample(1.0f, 100), 100);
    EXPECT(f.forwarded() == 1, "forwarded() phai dem ca khi khong co sink");
}

static void testSingleSinkGetsExactPayload()
{
    SampleFanout<4> f;
    Recorder        r{1};
    EXPECT(f.add(&r), "them sink dau tien phai thanh cong");
    EXPECT(f.count() == 1, "count() sau mot add");

    const Sample sent = makeSample(0.75f, 4242);
    f.onSample(sent, 9999);

    EXPECT(r.calls == 1, "sink phai duoc goi dung mot lan");
    EXPECT(r.lastNow == 9999, "nowMs phai di qua nguyen ven");
    EXPECT(r.last.timestampMs == 4242, "timestampMs phai di qua nguyen ven");
    EXPECT(r.last.accelG[0] == 0.75f && r.last.accelG[1] == 0.25f
               && r.last.accelG[2] == -1.0f,
           "accelG phai di qua nguyen ven");
    EXPECT(r.last.gyroDps[0] == 12.5f && r.last.gyroDps[1] == -3.0f
               && r.last.gyroDps[2] == 0.5f,
           "gyroDps phai di qua nguyen ven");
}

static void testAllSinksSeeTheSameSample()
{
    SampleFanout<4> f;
    Recorder        a{1}, b{2}, c{3};
    EXPECT(f.add(&a) && f.add(&b) && f.add(&c), "them ba sink");
    EXPECT(f.count() == 3, "count() sau ba add");

    f.onSample(makeSample(2.5f, 77), 88);

    EXPECT(a.calls == 1 && b.calls == 1 && c.calls == 1,
           "ca ba sink deu phai duoc goi dung mot lan");
    EXPECT(a.last.timestampMs == 77 && b.last.timestampMs == 77
               && c.last.timestampMs == 77,
           "ca ba phai thay CUNG mot sample");
    EXPECT(a.last.accelG[0] == 2.5f && b.last.accelG[0] == 2.5f
               && c.last.accelG[0] == 2.5f,
           "gia tri khong duoc bi sua tren duong di");
}

static void testDeliveryOrderIsRegistrationOrder()
{
    SampleFanout<4> f;
    int             log[Recorder::kOrderMax]{};
    int             n = 0;

    Recorder  a{10}, b{20}, c{30};
    Recorder* all[] = {&a, &b, &c};
    for (Recorder* r : all) {
        r->sharedOrder = log;
        r->sharedCount = &n;
    }
    f.add(&a);
    f.add(&b);
    f.add(&c);

    f.onSample(makeSample(0.0f, 1), 1);

    // Not a cosmetic property: a deterministic order is what makes a
    // reproduction of "consumer 3 saw a stale value" possible at all.
    EXPECT(n == 3, "phai ghi ba luot goi, thuc te %d", n);
    EXPECT(log[0] == 10 && log[1] == 20 && log[2] == 30,
           "thu tu phat phai dung thu tu dang ky: %d,%d,%d", log[0], log[1],
           log[2]);
}

// ------------------------------------------------------------ add() rejections

static void testRejectsNull()
{
    SampleFanout<4> f;
    EXPECT(!f.add(nullptr), "add(nullptr) phai bi tu choi");
    EXPECT(f.count() == 0, "nullptr khong duoc lam tang count()");
}

static void testRejectsDuplicate()
{
    SampleFanout<4> f;
    Recorder        r{1};

    EXPECT(f.add(&r), "lan add dau phai thanh cong");
    EXPECT(!f.add(&r), "add TRUNG phai bi tu choi");
    EXPECT(f.count() == 1, "trung lap khong duoc lam tang count()");

    f.onSample(makeSample(1.0f, 5), 5);
    // The whole point: one registration, one delivery. If the duplicate had
    // been accepted a step counter downstream would count every step twice.
    EXPECT(r.calls == 1, "sink trung phai chi nhan mot lan, thuc te %u",
           r.calls);
}

static void testRejectsWhenFull()
{
    SampleFanout<2> f;
    Recorder        a{1}, b{2}, c{3};

    EXPECT(f.add(&a), "sink 1 vao duoc");
    EXPECT(f.add(&b), "sink 2 vao duoc");
    EXPECT(!f.add(&c), "sink 3 phai bi tu choi khi day");
    EXPECT(f.count() == 2, "count() phai dung o capacity");

    f.onSample(makeSample(1.0f, 5), 5);
    EXPECT(a.calls == 1 && b.calls == 1, "hai sink dau van chay binh thuong");
    EXPECT(c.calls == 0, "sink bi tu choi khong duoc nhan gi");
}

static void testSealBlocksFurtherAdd()
{
    SampleFanout<4> f;
    Recorder        a{1}, b{2};

    EXPECT(f.add(&a), "them truoc khi seal");
    EXPECT(!f.isSealed(), "chua seal");

    f.seal();

    EXPECT(f.isSealed(), "da seal");
    EXPECT(!f.add(&b), "add sau seal phai bi tu choi");
    EXPECT(f.count() == 1, "seal roi thi count() khong doi");

    f.onSample(makeSample(1.0f, 5), 5);
    EXPECT(a.calls == 1, "sink da dang ky van chay sau khi seal");
    EXPECT(b.calls == 0, "sink bi chan boi seal khong nhan gi");
}

// ------------------------------------------------------------------ counters

static void testForwardedCounts()
{
    SampleFanout<2> f;
    Recorder        a{1};
    f.add(&a);

    for (uint32_t i = 0; i < 10; ++i) {
        f.onSample(makeSample(0.0f, i), i);
    }

    // forwarded() counts SAMPLES, not deliveries: one sample to two sinks is
    // still one sample. It answers "is the IMU pushing", not "how busy are the
    // consumers".
    EXPECT(f.forwarded() == 10, "forwarded()==10, thuc te %u", f.forwarded());
    EXPECT(a.calls == 10, "sink nhan du 10 mau, thuc te %u", a.calls);
    EXPECT(a.last.timestampMs == 9, "sink giu mau cuoi cung");
}

static void testCapacityOfOne()
{
    // N == 1 is the degenerate case that must behave exactly like the old
    // single-pointer sink, so switching to the fan-out cannot regress it.
    SampleFanout<1> f;
    Recorder        a{1}, b{2};

    EXPECT(f.add(&a), "sink duy nhat vao duoc");
    EXPECT(!f.add(&b), "sink thu hai bi tu choi");

    f.onSample(makeSample(3.0f, 1), 1);
    EXPECT(a.calls == 1 && b.calls == 0, "hanh vi giong het mot con tro don");
}

int main()
{
    std::printf("sample_fanout\n");

    testEmptyFanoutIsHarmless();
    testSingleSinkGetsExactPayload();
    testAllSinksSeeTheSameSample();
    testDeliveryOrderIsRegistrationOrder();
    testRejectsNull();
    testRejectsDuplicate();
    testRejectsWhenFull();
    testSealBlocksFurtherAdd();
    testForwardedCounts();
    testCapacityOfOne();

    std::printf("  %d kiem tra, %d that bai\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
