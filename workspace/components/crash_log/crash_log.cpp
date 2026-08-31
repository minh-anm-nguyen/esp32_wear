#include "crash_log.hpp"

#include <cinttypes>
#include <cstring>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"

namespace forensics {
namespace {

constexpr const char* TAG = "forensic";

// Bumped whenever the layout below changes, so a firmware update never reads a
// previous build's bytes as if they meant the same thing.
constexpr uint32_t kMagic = 0x57415431;  // "WAT1"

constexpr uint8_t kIdLen = 16;

struct Record {
    uint32_t magic;
    uint32_t boots;
    char     current[kIdLen];   // on screen right now; cleared at every boot
    char     culprit[kIdLen];   // on screen when the board last died
    uint8_t  strikes;           // consecutive deaths blamed on `culprit`
    uint32_t quarantinedAt;     // boot number the lock-out started
};

// RTC slow memory, deliberately NOT zeroed at startup -- that is the whole
// point. The linker puts it out of reach of the normal .bss wipe, so what the
// dying firmware wrote is still there when the next one starts reading.
RTC_NOINIT_ATTR Record g_rec;

void copyId(char* dst, const char* src)
{
    if (src == nullptr) {
        dst[0] = '\0';
        return;
    }
    std::strncpy(dst, src, kIdLen - 1);
    dst[kIdLen - 1] = '\0';
}

const char* reasonName(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:  return "bat nguon";
    case ESP_RST_EXT:      return "reset ngoai";
    case ESP_RST_SW:       return "reset phan mem";
    case ESP_RST_PANIC:    return "PANIC (ngoai le CPU)";
    case ESP_RST_INT_WDT:  return "watchdog ngat";
    case ESP_RST_TASK_WDT: return "watchdog task";
    case ESP_RST_WDT:      return "watchdog khac";
    case ESP_RST_BROWNOUT: return "sut ap";
    case ESP_RST_DEEPSLEEP:return "thuc tu deep sleep";
    case ESP_RST_SDIO:     return "SDIO";
    default:               return "khong ro";
    }
}

// Only these mean "the firmware died". A brownout is deliberately NOT one:
// the board browning out while an app happened to be open says something about
// the power supply, not about the app, and blaming the app would quarantine
// innocent code and hide the real cause.
bool isCrash(esp_reset_reason_t r)
{
    return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT
           || r == ESP_RST_WDT;
}

}  // namespace

void reportBoot()
{
    const esp_reset_reason_t reason = esp_reset_reason();

    if (g_rec.magic != kMagic) {
        // Cold boot, or a firmware whose Record had a different shape. Either
        // way the bytes are not ours and must not be read as if they were.
        std::memset(&g_rec, 0, sizeof(g_rec));
        g_rec.magic = kMagic;
    }
    ++g_rec.boots;

    if (isCrash(reason)) {
        if (g_rec.current[0] != '\0') {
            if (std::strncmp(g_rec.current, g_rec.culprit, kIdLen) == 0) {
                if (g_rec.strikes < 255) {
                    ++g_rec.strikes;
                }
            } else {
                copyId(g_rec.culprit, g_rec.current);
                g_rec.strikes = 1;
            }
            ESP_LOGE(TAG,
                     "LAN BOOT TRUOC BOARD CHET: %s -- luc do app '%s' dang mo "
                     "(lan thu %u lien tiep)",
                     reasonName(reason), g_rec.culprit,
                     static_cast<unsigned>(g_rec.strikes));

            if (g_rec.strikes >= kQuarantineStrikes) {
                if (g_rec.quarantinedAt == 0) {
                    g_rec.quarantinedAt = g_rec.boots;
                }
                ESP_LOGE(TAG,
                         "'%s' BI CACH LY: se khong duoc mo nua cho den khi no "
                         "chay sach mot lan. Cac app khac khong bi anh huong.",
                         g_rec.culprit);
            }
        } else {
            // The UI was not showing an app: a driver, a daemon, or boot code.
            // The panic report names the TASK, and for a daemon that task is
            // named after the daemon.
            ESP_LOGE(TAG,
                     "LAN BOOT TRUOC BOARD CHET: %s -- khong app UI nao dang mo. "
                     "Xem ten task trong bao cao panic o tren.",
                     reasonName(reason));
        }
    } else {
        ESP_LOGI(TAG, "khoi dong lan %" PRIu32 " (%s)", g_rec.boots,
                 reasonName(reason));
    }

    // THE WAY OUT.
    //
    // A quarantined app cannot be opened, so it can never run clean, so it can
    // never clear its own record -- a life sentence, since RTC memory survives
    // every reset and only a flat battery wipes it. That is too strong for
    // evidence this circumstantial: "was on screen when the board died" is a
    // correlation, and an app can be convicted by a driver's bug.
    //
    // So the lock-out expires. It exists to break a boot loop, not to delete a
    // feature, and after this many further boots the app gets another chance
    // with a clean sheet.
    if (g_rec.strikes >= kQuarantineStrikes
        && (g_rec.boots - g_rec.quarantinedAt) >= kQuarantineBoots) {
        ESP_LOGW(TAG,
                 "'%s' het han cach ly sau %u lan khoi dong -- duoc mo lai. "
                 "Neu no lai lam board chet, ban dem se bat dau lai.",
                 g_rec.culprit, static_cast<unsigned>(kQuarantineBoots));
        g_rec.culprit[0]    = '\0';
        g_rec.strikes       = 0;
        g_rec.quarantinedAt = 0;
    }

    // Nothing is on screen yet. Leaving the previous value here would blame the
    // last app for a crash that happened during bring-up.
    g_rec.current[0] = '\0';
}

void setCurrentApp(const char* id)
{
    copyId(g_rec.current, id);
}

uint8_t strikes(const char* id)
{
    if (id == nullptr || g_rec.magic != kMagic || g_rec.culprit[0] == '\0') {
        return 0;
    }
    return (std::strncmp(id, g_rec.culprit, kIdLen) == 0) ? g_rec.strikes : 0;
}

bool quarantined(const char* id)
{
    return strikes(id) >= kQuarantineStrikes;
}

void clearStrikes(const char* id)
{
    if (id == nullptr || g_rec.magic != kMagic) {
        return;
    }
    if (std::strncmp(id, g_rec.culprit, kIdLen) == 0) {
        if (g_rec.strikes > 0) {
            ESP_LOGI(TAG, "'%s' chay sach -- xoa %u lan chet truoc do", id,
                     static_cast<unsigned>(g_rec.strikes));
        }
        g_rec.culprit[0]    = '\0';
        g_rec.strikes       = 0;
        g_rec.quarantinedAt = 0;
    }
}

const char* culprit()
{
    return (g_rec.magic == kMagic && g_rec.culprit[0] != '\0') ? g_rec.culprit
                                                               : "";
}

uint32_t bootCount()
{
    return (g_rec.magic == kMagic) ? g_rec.boots : 0;
}

}  // namespace forensics
