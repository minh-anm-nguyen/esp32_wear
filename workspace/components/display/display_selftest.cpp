#include "display_selftest.hpp"

#include <atomic>
#include <cinttypes>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace display {
namespace {

constexpr const char* TAG = "display.test";

// RGB565, native byte order. Config::dataEndian is LCD_RGB_DATA_ENDIAN_LITTLE,
// so the panel reads these words directly -- if the colours come out as noise
// rather than as flat fields, that field is the first suspect.
constexpr uint16_t kBlack = 0x0000;
constexpr uint16_t kWhite = 0xFFFF;
constexpr uint16_t kRed   = 0xF800;
constexpr uint16_t kGreen = 0x07E0;
constexpr uint16_t kBlue  = 0x001F;
constexpr uint16_t kGrey   = 0x39E7;
constexpr uint16_t kYellow = 0xFFE0;

std::atomic<uint32_t> g_callbackCount{0};

// ISR context. Nothing here may log, allocate or block.
bool IRAM_ATTR countTransfer(void*)
{
    g_callbackCount.fetch_add(1, std::memory_order_relaxed);
    return false;  // no task to wake in the self test
}

// Paints the whole screen one band at a time through a single buffer.
//
// One buffer, not two, on purpose: waiting for each band before refilling is
// what makes this also a test of waitIdle(). A torn band would mean the DMA
// completion path is lying.
template <typename Fn>
esp_err_t paint(DisplayManager& dm, uint16_t* buf, Fn&& pixelAt)
{
    const Geometry& g     = dm.geometry();
    const uint16_t  lines = dm.config().flushBufferLines;

    for (uint16_t y0 = 0; y0 < g.height; y0 += lines) {
        const uint16_t y1 = (y0 + lines > g.height) ? g.height
                                                    : static_cast<uint16_t>(y0 + lines);

        for (uint16_t y = y0; y < y1; ++y) {
            uint16_t* row = buf + static_cast<size_t>(y - y0) * g.width;
            for (uint16_t x = 0; x < g.width; ++x) {
                row[x] = pixelAt(x, y);
            }
        }

        const Area band{0, y0, g.width, y1};
        const esp_err_t err = dm.drawRgb565(band, buf);
        if (err != ESP_OK) {
            return err;
        }
        const esp_err_t werr = dm.waitIdle(dm.config().transferTimeoutMs);
        if (werr != ESP_OK) {
            return werr;
        }
    }
    return ESP_OK;
}

esp_err_t solid(DisplayManager& dm, uint16_t* buf, uint16_t colour, uint32_t holdMs)
{
    const esp_err_t err = paint(dm, buf, [colour](uint16_t, uint16_t) { return colour; });
    vTaskDelay(pdMS_TO_TICKS(holdMs));
    return err;
}


// One STATIC frame that answers every remaining question at once.
//
// The timed patterns above have a design flaw the first bring-up exposed: each
// one shows for 800 ms and then is gone, so answering "were the three colours in
// the right ORDER" requires the person to act as a stopwatch and a memory. This
// frame is read by POSITION instead, and it is the last thing drawn, so it stays
// on the glass for as long as anyone wants to look at it.
//
// NOTHING IN THIS FRAME IS WHITE EXCEPT ONE SQUARE IN THE MIDDLE.
//
// That is the point of the current revision. The border used to be a 1 px white
// line, which made a real question unanswerable: a viewer reporting "white
// stripes down all four edges" could be describing the border working perfectly
// OR some artifact sitting in the same place, and no amount of describing it
// separates the two. Give each edge its own colour and the ambiguity is gone --
// any white left at an edge is, by construction, not something this code drew.
//
// The edge colours double as the mirror test, and the three bands have
// DECREASING HEIGHT so that mirrorY can be judged without trusting colour at
// all: a flip and a red/blue swap both put "blue at the top", but only a flip
// moves the thin band there.
uint16_t diagnosticPixel(const Geometry& g, uint16_t x, uint16_t y)
{
    constexpr uint16_t kEdge = 2;  // 2 px so it cannot be mistaken for a stray line

    // Border, one colour per edge. Top and bottom win the corners.
    if (y < kEdge)                { return kRed; }
    if (y >= g.height - kEdge)    { return kBlue; }
    if (x < kEdge)                { return kYellow; }
    if (x >= g.width - kEdge)     { return kGreen; }

    // Three bands, thick to thin, top to bottom -> rgbOrder, and mirrorY again
    // by a route that does not depend on colour at all.
    if (x >= 8 && x < g.width - 8) {
        if (y >= 60  && y < 100) { return kRed; }    // 40 px
        if (y >= 112 && y < 136) { return kGreen; }  // 24 px
        if (y >= 148 && y < 160) { return kBlue; }   // 12 px
    }

    // The ONLY white in the frame: a brightness reference, deliberately far from
    // every edge so it can never be confused with one.
    if (x >= 105 && x < 135 && y >= 195 && y < 225) {
        return kWhite;
    }

    return kBlack;
}

}  // namespace

esp_err_t runSelfTest(DisplayManager& dm, SelfTestResult& out, uint32_t holdMs)
{
    out = SelfTestResult{};

    if (!dm.isInitialized()) {
        ESP_LOGE(TAG, "DisplayManager chua init");
        return ESP_ERR_INVALID_STATE;
    }

    const Geometry& g = dm.geometry();

    ESP_LOGI(TAG, "--- self test man hinh %ux%u ---", g.width, g.height);

    auto* buf = static_cast<uint16_t*>(dm.allocFlushBuffer());
    if (buf == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    out.bufferAllocated = true;
    ESP_LOGI(TAG, "  buffer DMA: %u byte internal RAM",
             static_cast<unsigned>(dm.flushBufferBytes()));

    g_callbackCount.store(0);
    dm.setTransferDoneCallback(&countTransfer, nullptr);

    // Keeps the FIRST error rather than OR-ing codes together: esp_err_t
    // values are not flags, and |= would produce a code that names nothing.
    esp_err_t err  = ESP_OK;
    auto      keep = [&err](esp_err_t e) {
        if (e != ESP_OK && err == ESP_OK) {
            err = e;
        }
    };

    // ---- 1. flat colours ----------------------------------------------------
    //
    // Proves rgb_ele_order (red and blue swapped means BGR), data_endian (noise
    // instead of a flat field) and invertColors (a red field showing as cyan).
    // Printed BEFORE the first pattern so the log records whether the screen was
    // even lit while these ran. The first version of this test did not, and four
    // patterns played to a dark panel without a single line saying so.
    dm.logDiagnostics("truoc mau dau tien");

    ESP_LOGI(TAG, "[1] ba mau day man hinh -- mong doi DO, roi LUC, roi LAM");
    ESP_LOGI(TAG, "    do <-> lam bi trao   => rgbOrder sai (dat LCD_RGB_ELEMENT_ORDER_BGR)");
    ESP_LOGI(TAG, "    ra nhieu hat         => dataEndian sai");
    ESP_LOGI(TAG, "    mau bi dao nguoc     => invertColors sai");
    keep(solid(dm, buf, kRed, holdMs));
    keep(solid(dm, buf, kGreen, holdMs));
    keep(solid(dm, buf, kBlue, holdMs));

    // ---- 2. border ----------------------------------------------------------
    //
    // The single most important pattern here. The ST7789 carries 240x320 of RAM
    // while the glass only lights 240x280, so an image that is simply shifted by
    // 20 rows still looks like a working display -- until a one pixel border
    // shows one of its edges missing.
    ESP_LOGI(TAG, "[2] vien trang 1 pixel -- PHAI thay du BON canh");
    ESP_LOGI(TAG, "    thieu canh tren/duoi => yGap sai (dang %u)", g.yGap);
    ESP_LOGI(TAG, "    thieu canh trai/phai => xGap sai (dang %u)", g.xGap);
    keep(paint(dm, buf, [&g](uint16_t x, uint16_t y) -> uint16_t {
        const bool edge = (x == 0) || (y == 0) ||
                          (x == g.width - 1) || (y == g.height - 1);
        return edge ? kWhite : kBlack;
    }));
    vTaskDelay(pdMS_TO_TICKS(holdMs));

    // ---- 3. corner markers --------------------------------------------------
    //
    // Four different colours, so a mirror error names itself instead of just
    // looking "somehow rotated".
    ESP_LOGI(TAG, "[3] bon goc -- tren-trai DO, tren-phai LUC, duoi-trai LAM, duoi-phai TRANG");
    ESP_LOGI(TAG, "    trai <-> phai bi trao => mirrorX sai (dang %d)",
             static_cast<int>(g.mirrorX));
    ESP_LOGI(TAG, "    tren <-> duoi bi trao => mirrorY sai (dang %d)",
             static_cast<int>(g.mirrorY));
    keep(paint(dm, buf, [&g](uint16_t x, uint16_t y) -> uint16_t {
        constexpr uint16_t kSize = 24;
        const bool left = x < kSize;
        const bool right = x >= g.width - kSize;
        const bool top = y < kSize;
        const bool bottom = y >= g.height - kSize;

        if (top && left) {
            return kRed;
        }
        if (top && right) {
            return kGreen;
        }
        if (bottom && left) {
            return kBlue;
        }
        if (bottom && right) {
            return kWhite;
        }
        const bool edge = (x == 0) || (y == 0) ||
                          (x == g.width - 1) || (y == g.height - 1);
        return edge ? kGrey : kBlack;
    }));
    vTaskDelay(pdMS_TO_TICKS(holdMs));

    // ---- 4. coordinate grid -------------------------------------------------
    ESP_LOGI(TAG, "[4] luoi 20px -- o cuoi moi chieu phai day du, khong bi cat");
    keep(paint(dm, buf, [&g](uint16_t x, uint16_t y) -> uint16_t {
        if (x % 20 == 0 || y % 20 == 0) {
            return kGrey;
        }
        const bool edge = (x == 0) || (y == 0) ||
                          (x == g.width - 1) || (y == g.height - 1);
        return edge ? kWhite : kBlack;
    }));
    vTaskDelay(pdMS_TO_TICKS(holdMs));

    // The DMA completion path, proved rather than observed. Every band above
    // waited on it, so a zero here would mean waitIdle() returned for some other
    // reason entirely.
    out.transfersCompleted = g_callbackCount.load();
    out.callbackFired      = out.transfersCompleted > 0;
    ESP_LOGI(TAG, "  callback DMA: %" PRIu32 " lan -> %s", out.transfersCompleted,
             out.callbackFired ? "OK" : "KHONG CHAY (bat buoc phai sua)");

    // ---- 5. brightness ------------------------------------------------------
    ESP_LOGI(TAG, "[5] do sang 100 -> 50 -> 25 -> 0 -> 80");
    keep(solid(dm, buf, kWhite, 200));
    bool sweepOk = true;
    static constexpr uint8_t kSweep[] = {100, 50, 25, 0};
    for (uint8_t pct : kSweep) {
        const esp_err_t berr = dm.setBrightness(pct, 300);
        if (berr != ESP_OK) {
            ESP_LOGE(TAG, "  setBrightness(%u) that bai: %s", pct,
                     esp_err_to_name(berr));
            sweepOk = false;
        }
        vTaskDelay(pdMS_TO_TICKS(holdMs / 2));
    }
    dm.setBrightness(dm.config().defaultBrightness, 300);
    out.brightnessSweepOk = sweepOk;
    dm.logDiagnostics("sau quet do sang");

    // ---- 6. sleep / wake ----------------------------------------------------
    //
    // Five cycles, not 500: this runs at boot. The 500 cycle soak is a separate
    // exercise, and the state machine half of it already passes on the host.
    ESP_LOGI(TAG, "[6] sleep/wake 5 lan -- man hinh phai tat han roi sang lai co hinh");
    bool cyclesOk = true;
    for (int i = 0; i < 5; ++i) {
        if (dm.enterSleep() != ESP_OK || dm.state() != State::PANEL_SLEEP) {
            ESP_LOGE(TAG, "  chu ky %d: khong vao duoc PANEL_SLEEP", i);
            cyclesOk = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(400));

        if (dm.exitSleep() != ESP_OK || dm.state() != State::AWAKE) {
            ESP_LOGE(TAG, "  chu ky %d: khong quay lai duoc AWAKE", i);
            cyclesOk = false;
            break;
        }
        // The policy raises this on every wake: GRAM is undefined across
        // SLPIN/SLPOUT, so somebody has to repaint. Here that somebody is us.
        if (dm.takeRedrawRequest()) {
            keep(paint(dm, buf, [&g](uint16_t x, uint16_t y) -> uint16_t {
                const bool edge = (x == 0) || (y == 0) ||
                                  (x == g.width - 1) || (y == g.height - 1);
                return edge ? kWhite : ((x / 20 + y / 20) % 2 ? kBlue : kBlack);
            }));
        } else {
            ESP_LOGW(TAG, "  chu ky %d: khong co yeu cau ve lai sau khi thuc", i);
            cyclesOk = false;
        }
        out.sleepWakeCycles++;
        vTaskDelay(pdMS_TO_TICKS(400));
    }
    out.sleepWakeOk = cyclesOk;

    // ---- 7. the frame that stays up ----------------------------------------
    ESP_LOGI(TAG, "[7] KHUNG CHAN DOAN -- o lai tren man hinh, xem bao lau tuy y");
    keep(paint(dm, buf, [&g](uint16_t x, uint16_t y) -> uint16_t {
        return diagnosticPixel(g, x, y);
    }));
    ESP_LOGI(TAG, "    VIEN GIO CO MAU, KHONG CON TRANG:");
    ESP_LOGI(TAG, "      canh TREN = DO   canh DUOI = LAM");
    ESP_LOGI(TAG, "      canh TRAI = VANG canh PHAI = LUC");
    ESP_LOGI(TAG, "    => Neu van thay soc TRANG o vien thi do KHONG phai thu firmware ve.");
    ESP_LOGI(TAG, "       Mau trang duy nhat trong khung la o vuong o GIUA man hinh.");
    ESP_LOGI(TAG, "    Bon cau hoi:");
    ESP_LOGI(TAG, "    1. Canh TREN mau gi?                      -> mirrorY + rgbOrder");
    ESP_LOGI(TAG, "    2. Canh TRAI mau gi?                      -> mirrorX");
    ESP_LOGI(TAG, "    3. Dai ngang DAY nhat o TREN hay DUOI?    -> mirrorY, khong phu thuoc mau");
    ESP_LOGI(TAG, "    4. Ba dai tu tren xuong mau gi?           -> rgbOrder");
    ESP_LOGI(TAG, "    Dung la: tren DO, trai VANG, dai day o TREN, ba dai DO-LUC-LAM.");

    dm.logDiagnostics("ket thuc self test");
    dm.setTransferDoneCallback(nullptr, nullptr);
    dm.freeFlushBuffer(buf);

    ESP_LOGI(TAG, "--- ket qua: buffer=%d callback=%d(%" PRIu32 ") sang=%d chu-ky=%d(%" PRIu32 ") ---",
             out.bufferAllocated, out.callbackFired, out.transfersCompleted,
             out.brightnessSweepOk, out.sleepWakeOk, out.sleepWakeCycles);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  co it nhat mot loi ve trong qua trinh test");
    }
    return err;
}

}  // namespace display
