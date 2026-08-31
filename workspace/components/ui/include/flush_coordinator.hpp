// The rule "exactly one lv_display_flush_ready() per flush", as a state machine
// that can be tested without a panel.
//
// WHY THIS IS ITS OWN CLASS
//
// It is the single most dangerous piece of the LVGL integration and the one
// least amenable to debugging on hardware:
//
//   - report completion twice  -> LVGL may hand out a buffer the DMA is still
//                                 reading; corruption, far from its cause
//   - never report completion  -> LVGL waits forever. The screen freezes, the
//                                 UI task is still alive, nothing is logged
//   - report it too early      -> tearing and stray pixels
//
// And the trap that makes the second one likely: DisplayManager::drawRgb565()
// returns ESP_ERR_INVALID_STATE at PANEL_SLEEP -- deliberately, so a lost frame
// is not silent -- while LVGL waits for flush_ready UNCONDITIONALLY. A flush_cb
// that simply returns on error hangs the UI at the first sleep.
// doc-design/LVGL-UI-technical-challenges.md trap #16.
//
// Pure: includes only <cstdint>, so every path above is a host test.
#pragma once

#include <atomic>
#include <cstdint>

namespace ui {

// What the flush callback still owes LVGL when begin() returns.
enum class FlushAction : uint8_t {
    // The transfer is in flight. The DMA ISR will complete it. The callback
    // must NOT call lv_display_flush_ready().
    WaitForIsr,

    // Nothing was queued. The callback MUST call lv_display_flush_ready()
    // itself, right now, or LVGL never advances again.
    ReadyNow,
};

class FlushCoordinator {
public:
    // RESERVE THE FLUSH, then hand the pixels to the driver, then confirm.
    //
    // The order is the whole point, and getting it wrong cost a broken
    // invariant on real hardware. The first version queued the DMA first and
    // recorded it afterwards:
    //
    //     drawRgb565(...);          // DMA starts
    //     flush_.begin(...);        // pending_ = true
    //
    // For a small dirty rectangle the transfer finished BETWEEN those two
    // lines. The ISR then found pending_ == false, correctly refused a
    // completion nobody was waiting for, and the flush was never reported to
    // LVGL. Board log: "bat dau=56 isr=55 thua=1 can bang=KHONG".
    //
    // Reserving first makes that window impossible: pending_ is true before the
    // driver is ever asked to move a byte.
    //
    // Returns false when the caller must NOT draw -- see blocked().
    bool reserve()
    {
        ++started_;
        if (blocked_) {
            ++readyNowBlocked_;
            return false;
        }
        pending_.store(true, std::memory_order_release);
        return true;
    }

    // The driver refused the buffer after all. Undo the reservation; the caller
    // owes LVGL an immediate lv_display_flush_ready().
    void abandon()
    {
        pending_.store(false, std::memory_order_release);
        ++readyNowRejected_;
    }

    // Call from the DMA completion ISR.
    //
    // Returns false when nothing was pending, which means a completion arrived
    // that nobody was waiting for -- a stale callback from an abandoned
    // transfer, or a driver reporting twice. The caller must NOT report that
    // one to LVGL: doing so would release a buffer belonging to the NEXT flush.
    // RUNS IN ISR CONTEXT, concurrently with the UI task calling reserve().
    // Hence the atomics: pending_ was a plain bool, written from both sides.
    //
    // exchange() makes "was something pending, and claim it" one indivisible
    // step. A check followed by a store would let two completions both believe
    // they were the first.
    bool completeFromIsr()
    {
        if (!pending_.exchange(false, std::memory_order_acq_rel)) {
            spurious_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        completedByIsr_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    // Call when the wait for a completion ran out of time.
    //
    // This is not a slow frame, it is a dead panel or a dead DMA channel: the
    // transfer was 3.84 ms of work. LVGL has already given up and will reuse the
    // buffer, so the only safe response is to stop issuing flushes entirely
    // until something re-initialises the display.
    void onTimeout()
    {
        if (pending_.exchange(false, std::memory_order_acq_rel)) {
            ++timedOut_;
        }
        blocked_ = true;
    }

    // After the display stack has been re-initialised.
    void resumeAfterRecovery()
    {
        pending_.store(false, std::memory_order_release);
        blocked_ = false;
        ++recoveries_;
    }

    bool pending() const { return pending_.load(std::memory_order_acquire); }
    bool blocked() const { return blocked_; }

    uint32_t started() const { return started_; }
    uint32_t completedByIsr() const { return completedByIsr_.load(std::memory_order_relaxed); }
    uint32_t readyNowRejected() const { return readyNowRejected_; }
    uint32_t readyNowBlocked() const { return readyNowBlocked_; }
    uint32_t timedOut() const { return timedOut_; }
    uint32_t spurious() const { return spurious_.load(std::memory_order_relaxed); }
    uint32_t recoveries() const { return recoveries_; }

    // THE invariant, in one call, so a diagnostics screen and a host test can
    // ask the same question: every flush that began has been completed exactly
    // once, by exactly one route.
    bool balanced() const
    {
        const uint32_t settled =
            completedByIsr() + readyNowRejected_ + readyNowBlocked_ + timedOut_;
        return settled + (pending() ? 1u : 0u) == started_;
    }

private:
    // Touched by the ISR as well as the UI task.
    std::atomic<bool>     pending_{false};
    std::atomic<uint32_t> completedByIsr_{0};
    std::atomic<uint32_t> spurious_{0};

    // UI task only.
    bool     blocked_{false};
    uint32_t started_{0};
    uint32_t readyNowRejected_{0};  // display refused the draw (asleep)
    uint32_t readyNowBlocked_{0};   // we refused to draw (post-timeout)
    uint32_t timedOut_{0};
    uint32_t recoveries_{0};
};

}  // namespace ui
