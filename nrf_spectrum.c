/*
 * [NRF24] Spectrum — 2.4 GHz Spectrum Analyzer for Flipper Zero
 *
 * NRF24 driver: Jammer's nrf24_device_t with cs_pin=PC3 for Apex 5G
 * Scan: Binary RPD detection
 * Logging: SD Card debug logging
 */

#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_power.h>
#include <gui/gui.h>
#include <input/input.h>
#include <storage/storage.h>

#include "nrf24.h"

/* Channels 0..80 = 2400..2480 MHz = 2.40..2.48 GHz */
#define SCAN_MIN   0
#define SCAN_MAX   80
#define SCAN_COUNT (SCAN_MAX - SCAN_MIN + 1)

#define BAR_HEIGHT 40

static nrf24_device_t nrf_dev;

typedef struct {
    uint8_t level[SCAN_COUNT];
    bool running;
    bool show_grid;
    bool nrf_found;
    FuriMutex* mutex;
} SpectrumState;

/* Map channel index (0..80) to screen x (0..127) */
static inline int ch_to_x(int ch) {
    return (ch * 127) / SCAN_MAX;
}

/* ── Rendering ─────────────────────────────────────────── */

static void draw_spectrum(Canvas* canvas, SpectrumState* state) {
    canvas_clear(canvas);
    canvas_set_color(canvas, ColorBlack);

    if(!state->nrf_found) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str_aligned(canvas, 64, 24, AlignCenter, AlignCenter, "NRF24 not found");
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str_aligned(canvas, 64, 38, AlignCenter, AlignCenter, "Check connection");
        return;
    }

    const int baseline = 51;
    const int bar_max  = 43;

    /* Baseline */
    canvas_draw_line(canvas, 0, baseline, 127, baseline);

    /* Tick marks on baseline — every 10 channels (10 MHz) */
    for(int ch = 0; ch <= SCAN_MAX; ch += 10) {
        int x = ch_to_x(ch);
        canvas_draw_line(canvas, x, baseline, x, baseline + 2);
    }

    /* Frequency labels — below tick marks */
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 0, 63, "2.4");
    canvas_draw_str_aligned(canvas, ch_to_x(40), 63, AlignCenter, AlignBottom, "2.44");
    canvas_draw_str_aligned(canvas, 127, 63, AlignRight, AlignBottom, "2.48");

    /* Channel labels (OK toggle): Show 0, 20, 40, 60, 80 to ensure spacing */
    if(state->show_grid) {
        canvas_set_font(canvas, FontSecondary);
        for(int ch = 0; ch <= SCAN_MAX; ch += 20) {
            int x = ch_to_x(ch);
            char buf[4];
            snprintf(buf, sizeof(buf), "%d", ch);
            if(ch == 0) {
                canvas_draw_str_aligned(canvas, x, 0, AlignLeft, AlignTop, buf);
            } else if (ch == SCAN_MAX) {
                canvas_draw_str_aligned(canvas, 127, 0, AlignRight, AlignTop, buf);
            } else {
                canvas_draw_str_aligned(canvas, x, 0, AlignCenter, AlignTop, buf);
            }
        }
    }

    /* Spectrum bars */
    for(int i = 0; i < SCAN_COUNT; i++) {
        if(state->level[i] == 0) continue;
        int x = ch_to_x(i);
        int bar_h = (state->level[i] * bar_max) / BAR_HEIGHT;
        if(bar_h > bar_max) bar_h = bar_max;
        if(bar_h > 0) {
            canvas_draw_line(canvas, x, baseline - 1, x, baseline - 1 - bar_h);
        }
    }
}

static void render_cb(Canvas* canvas, void* ctx) {
    SpectrumState* state = ctx;
    if(furi_mutex_acquire(state->mutex, 25) != FuriStatusOk) return;
    draw_spectrum(canvas, state);
    furi_mutex_release(state->mutex);
}

static void input_cb(InputEvent* event, void* ctx) {
    FuriMessageQueue* queue = ctx;
    furi_message_queue_put(queue, event, FuriWaitForever);
}

/* ── Scan worker ───────────────────────────────────────── */

static int32_t scan_worker(void* ctx) {
    SpectrumState* state = ctx;
    if(!state->nrf_found) return 0;

    while(state->running) {
        for(int ch = SCAN_MIN; ch <= SCAN_MAX; ch++) {
            if(!state->running) break;

            furi_hal_gpio_write(nrf_dev.ce_pin, false);
            nrf24_write_reg(&nrf_dev, REG_RF_CH, ch);
            nrf24_flush_rx(&nrf_dev);
            
            /* Clear interrupts and power up */
            nrf24_write_reg(&nrf_dev, REG_STATUS, 0x70);
            nrf24_write_reg(&nrf_dev, REG_CONFIG, 0x03);

            /* Enter RX Mode */
            furi_hal_gpio_write(nrf_dev.ce_pin, true);
            
            /* Wait for PLL lock + signal measurement. 
             * Increased to 1000us (1ms) to support slower NRF clones.
             */
            furi_delay_us(1000);

            /* CRITICAL FIX: Read RPD *WHILE* CE is HIGH.
             * The NRF24 clears RPD immediately when CE goes low.
             */
            uint8_t rpd = 0;
            nrf24_read_reg(&nrf_dev, REG_RPD, &rpd, 1);

            /* Exit RX Mode */
            furi_hal_gpio_write(nrf_dev.ce_pin, false);

            if(furi_mutex_acquire(state->mutex, 50) == FuriStatusOk) {
                int idx = ch - SCAN_MIN;
                if(rpd & 0x01) {
                    state->level[idx] = BAR_HEIGHT;
                } else {
                    if(state->level[idx] > 4)
                        state->level[idx] -= 4;
                    else
                        state->level[idx] = 0;
                }
                furi_mutex_release(state->mutex);
            }
        }
    }

    return 0;
}

/* ── Main ──────────────────────────────────────────────── */

int32_t nrf_spectrum_app(void* p) {
    UNUSED(p);

    SpectrumState* state = malloc(sizeof(SpectrumState));
    memset(state, 0, sizeof(SpectrumState));
    state->running = true;
    state->mutex = furi_mutex_alloc(FuriMutexTypeNormal);

    if(!furi_hal_power_is_otg_enabled()) {
        furi_hal_power_enable_otg();
    }
    furi_delay_ms(500);

    /* Apex 5G: CS=PC3 (Pin 7), CE=PB2 (Pin 6) */
    nrf_dev.spi_handle = (FuriHalSpiBusHandle*)&furi_hal_spi_bus_handle_external;
    nrf_dev.ce_pin = &gpio_ext_pb2;
    nrf_dev.cs_pin = &gpio_ext_pc3;
    nrf_dev.initialized = false;

    nrf24_init(&nrf_dev);
    state->nrf_found = nrf24_check_connected(&nrf_dev);

    if(state->nrf_found) {
        nrf24_write_reg(&nrf_dev, REG_CONFIG, 0x00);
        furi_hal_gpio_write(nrf_dev.ce_pin, false);
        furi_delay_ms(10);
        nrf24_write_reg(&nrf_dev, REG_STATUS, 0x70);

        nrf24_write_reg(&nrf_dev, REG_EN_AA, 0x00);
        nrf24_write_reg(&nrf_dev, REG_EN_RXADDR, 0x01);
        nrf24_write_reg(&nrf_dev, REG_SETUP_AW, 0x01);
        nrf24_write_reg(&nrf_dev, REG_SETUP_RETR, 0x00);
        nrf24_write_reg(&nrf_dev, REG_RF_SETUP, 0x08);  /* 2 Mbps */
        nrf24_write_reg(&nrf_dev, REG_DYNPD, 0x00);
        nrf24_write_reg(&nrf_dev, REG_FEATURE, 0x00);

        uint8_t addr[] = {0xAA, 0xBB, 0xCC};
        nrf24_write_buf_reg(&nrf_dev, REG_RX_ADDR_P0, addr, 3);
        nrf24_write_reg(&nrf_dev, RX_PW_P0, 32);

        nrf24_flush_rx(&nrf_dev);
        nrf24_write_reg(&nrf_dev, REG_CONFIG, 0x03);
        furi_delay_ms(5);
    }

    FuriMessageQueue* queue = furi_message_queue_alloc(8, sizeof(InputEvent));
    ViewPort* vp = view_port_alloc();
    view_port_draw_callback_set(vp, render_cb, state);
    view_port_input_callback_set(vp, input_cb, queue);

    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, vp, GuiLayerFullscreen);

    FuriThread* thread = furi_thread_alloc_ex("NrfScan", 2048, scan_worker, state);
    furi_thread_start(thread);

    InputEvent event;
    while(state->running) {
        if(furi_message_queue_get(queue, &event, 100) == FuriStatusOk) {
            if(event.type == InputTypeShort) {
                if(event.key == InputKeyBack) {
                    state->running = false;
                } else if(event.key == InputKeyOk) {
                    state->show_grid = !state->show_grid;
                }
            } else if(event.type == InputTypeLong) {
                if(event.key == InputKeyOk) {
                    /* Reset all bars */
                    furi_mutex_acquire(state->mutex, FuriWaitForever);
                    memset(state->level, 0, SCAN_COUNT);
                    furi_mutex_release(state->mutex);
                }
            }
        }
        view_port_update(vp);
    }

    furi_thread_join(thread);
    furi_thread_free(thread);
    gui_remove_view_port(gui, vp);
    view_port_free(vp);
    furi_message_queue_free(queue);

    nrf24_set_idle(&nrf_dev);
    nrf24_deinit(&nrf_dev);

    furi_mutex_free(state->mutex);
    furi_hal_power_disable_otg();
    furi_record_close(RECORD_GUI);
    free(state);

    return 0;
}
