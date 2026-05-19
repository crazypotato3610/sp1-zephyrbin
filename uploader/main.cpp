/// @file main.cpp
/// @brief SP-1 USB album uploader firmware — streaming double-buffer protocol.
///
/// Protocol (v2):
///   Session header (host→fw, 8B):  "SPUL" | total_chunks u32LE
///   Chunk stream  (host→fw):       32768B × total_chunks  (sequential, no addr prefix)
///   Session ack   (fw→host, 1B):   0x02=DONE | 0x01=RETRY
///
/// Double-buffer pipeline: fill thread (main, priority 5) drains USB ring→chunk_data[fill_idx]
/// while write thread (priority 2) writes chunk_data[write_idx] to eMMC.
/// sem_empty/sem_filled (both init to N_BUFS/0) coordinate the handoff.
/// Write thread preempts fill thread the moment a filled buffer is ready.

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/usbd.h>
#include <cstdint>
#include <cstring>

#include "Leds.hpp"
#include "system/Watchdog.hpp"
#include "EmmcDriver.hpp"

// ── USB CDC ACM ─────────────────────────────────────────────────────────────

#define SP_USB_VID 0x1209
#define SP_USB_PID 0x0002

USBD_DEVICE_DEFINE(sp_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), SP_USB_VID, SP_USB_PID);
USBD_DESC_LANG_DEFINE(sp_lang);
USBD_DESC_MANUFACTURER_DEFINE(sp_mfr, "Teenage Engineering");
USBD_DESC_PRODUCT_DEFINE(sp_product, "SP-1 Album Uploader");
USBD_DESC_CONFIG_DEFINE(sp_fs_cfg, "FS Configuration");
USBD_CONFIGURATION_DEFINE(sp_fs_config, 0, 125, &sp_fs_cfg);

static int init_usb(void) {
    int err;
    if ((err = usbd_add_descriptor(&sp_usbd, &sp_lang)))    return err;
    if ((err = usbd_add_descriptor(&sp_usbd, &sp_mfr)))     return err;
    if ((err = usbd_add_descriptor(&sp_usbd, &sp_product))) return err;
    if ((err = usbd_add_configuration(&sp_usbd, USBD_SPEED_FS, &sp_fs_config))) return err;
    if ((err = usbd_register_all_classes(&sp_usbd, USBD_SPEED_FS, 1, nullptr))) return err;
    if ((err = usbd_init(&sp_usbd)))   return err;
    if ((err = usbd_enable(&sp_usbd))) return err;
    return 0;
}

// ── Constants ────────────────────────────────────────────────────────────────

static constexpr size_t CHUNK_SIZE        = 32768;
static constexpr size_t BLOCKS_PER_CHUNK  = CHUNK_SIZE / 512;  // 64 eMMC blocks
static constexpr size_t TERMINATOR_SIZE   = 8192;
static constexpr size_t HEADER_SIZE       = 8;   // "SPUL" + total_chunks u32LE
static constexpr size_t RX_RING_SIZE      = 32768;
static constexpr size_t N_BUFS           = 2;
static constexpr uint8_t ACK_RETRY       = 0x01;
static constexpr uint8_t ACK_DONE        = 0x02;
static constexpr int     MAX_RETRIES     = 3;
static constexpr int     WRITE_THREAD_PRIO = 2;  // higher than main (CONFIG_MAIN_THREAD_PRIORITY=5)

// ── Globals ──────────────────────────────────────────────────────────────────

static leds::Leds          g_leds;
static hardware::EmmcDriver g_emmc;
static const struct device *uart_dev;
static int                  g_wdt_ch;

// ── Buffers ──────────────────────────────────────────────────────────────────

static uint8_t rx_ring_buf[RX_RING_SIZE];
static ring_buf rx_ring;

alignas(4) static uint8_t chunk_data[N_BUFS][CHUNK_SIZE];
alignas(4) static uint8_t term_buf[TERMINATOR_SIZE];

// ── Pipeline sync ────────────────────────────────────────────────────────────

static K_SEM_DEFINE(sem_empty,        N_BUFS, N_BUFS);
static K_SEM_DEFINE(sem_filled,       0,      N_BUFS);
static K_SEM_DEFINE(sem_session_done, 0,      1);

static volatile uint32_t g_total_chunks;
static volatile uint32_t g_album_length_sectors;
static volatile bool     g_write_error;

// ── UART IRQ → ring buffer ───────────────────────────────────────────────────

static void uart_rx_cb(const struct device *dev, void *) {
    if (!uart_irq_update(dev)) return;
    while (uart_irq_rx_ready(dev)) {
        uint8_t b;
        if (uart_fifo_read(dev, &b, 1) != 1) break;
        ring_buf_put(&rx_ring, &b, 1);
    }
}

// ── I/O helpers ──────────────────────────────────────────────────────────────

static void recv_bytes(uint8_t *dst, size_t n) {
    size_t got = 0;
    while (got < n) {
        const uint32_t r = ring_buf_get(&rx_ring, dst + got, n - got);
        if (r == 0) {
            core::watchdog::feed(g_wdt_ch);
            k_sleep(K_MSEC(1));
        } else {
            got += r;
        }
    }
}

static void send_byte(uint8_t b) {
    uart_poll_out(uart_dev, b);
    core::watchdog::feed(g_wdt_ch);
}

// ── LED progress ─────────────────────────────────────────────────────────────

static void update_progress_leds(uint32_t done, uint32_t total) {
    if (total == 0) return;
    const uint8_t bright = 200;
    if (done >= total / 4)       g_leds.set_track(0, bright);
    if (done >= total / 2)       g_leds.set_track(1, bright);
    if (done >= (total * 3) / 4) g_leds.set_track(2, bright);
    if (done >= total)           g_leds.set_track(3, bright);
}

// ── eMMC write thread ────────────────────────────────────────────────────────

#define WRITE_STACK_SIZE 4096
K_THREAD_STACK_DEFINE(write_stack, WRITE_STACK_SIZE);
static struct k_thread write_thread_data;

static void emmc_write_thread_fn(void *, void *, void *) {
    int      write_idx     = 0;
    uint32_t chunks_written = 0;

    while (true) {
        k_sem_take(&sem_filled, K_FOREVER);
        core::watchdog::feed(g_wdt_ch);

        const uint32_t block_addr = chunks_written * BLOCKS_PER_CHUNK;
        const uint8_t *data       = chunk_data[write_idx];

        bool ok = false;
        for (int attempt = 0; attempt < MAX_RETRIES && !ok; ++attempt) {
            if (g_emmc.cmd25_write_multiple(block_addr, data, BLOCKS_PER_CHUNK))
                ok = true;
            else {
                k_sleep(K_MSEC(50));
                core::watchdog::feed(g_wdt_ch);
            }
        }

        if (!ok) {
            g_write_error = true;
            send_byte(ACK_RETRY);
            // Drain any queued buffers so fill thread can unblock
            while (k_sem_take(&sem_filled, K_NO_WAIT) == 0)
                k_sem_give(&sem_empty);
            k_sem_give(&sem_session_done);
            write_idx     = 0;
            chunks_written = 0;
            continue;
        }

        write_idx = (write_idx + 1) % N_BUFS;
        k_sem_give(&sem_empty);
        ++chunks_written;
        update_progress_leds(chunks_written, g_total_chunks);

        if (chunks_written == g_total_chunks) {
            bool term_ok = true;
            if (g_album_length_sectors > 0) {
                memset(term_buf, 0, TERMINATOR_SIZE);
                static const char marker[] = "ALBUM_PRESENT";
                memcpy(term_buf + 8179, marker, 13);
                const uint32_t term_block = (g_album_length_sectors - 1u) * 16u;
                term_ok = false;
                for (int attempt = 0; attempt < MAX_RETRIES && !term_ok; ++attempt) {
                    term_ok = g_emmc.cmd25_write_multiple(term_block, term_buf, 16);
                    if (!term_ok) {
                        k_sleep(K_MSEC(50));
                        core::watchdog::feed(g_wdt_ch);
                    }
                }
            }
            send_byte(term_ok ? ACK_DONE : ACK_RETRY);
            k_sem_give(&sem_session_done);
            write_idx     = 0;
            chunks_written = 0;
        }
    }
}

// ── Session loop ─────────────────────────────────────────────────────────────

static void run_session(void) {
    // Sliding-window scan for "SPUL" magic
    uint8_t hdr[HEADER_SIZE];
    recv_bytes(hdr, 4);
    while (!(hdr[0] == 'S' && hdr[1] == 'P' && hdr[2] == 'U' && hdr[3] == 'L')) {
        hdr[0] = hdr[1]; hdr[1] = hdr[2]; hdr[2] = hdr[3];
        recv_bytes(hdr + 3, 1);
        core::watchdog::feed(g_wdt_ch);
    }
    recv_bytes(hdr + 4, HEADER_SIZE - 4);

    uint32_t total_chunks;
    memcpy(&total_chunks, hdr + 4, 4);

    g_leds.all_off();
    g_total_chunks         = total_chunks;
    g_album_length_sectors = 0;
    g_write_error          = false;

    k_sem_init(&sem_empty,  N_BUFS, N_BUFS);
    k_sem_init(&sem_filled, 0,      N_BUFS);

    int fill_idx = 0;
    for (uint32_t i = 0; i < total_chunks && !g_write_error; ++i) {
        k_sem_take(&sem_empty, K_FOREVER);
        core::watchdog::feed(g_wdt_ch);

        recv_bytes(chunk_data[fill_idx], CHUNK_SIZE);

        if (i == 0) {
            uint32_t len;
            memcpy(&len, chunk_data[fill_idx] + 13, 4);
            g_album_length_sectors = len;
        }

        fill_idx = (fill_idx + 1) % N_BUFS;
        k_sem_give(&sem_filled);
    }

    k_sem_take(&sem_session_done, K_FOREVER);
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(void) {
    core::watchdog::start();
    g_wdt_ch = core::watchdog::register_channel("uploader", 10000);
    core::watchdog::feed(g_wdt_ch);

    ring_buf_init(&rx_ring, sizeof(rx_ring_buf), rx_ring_buf);

    g_leds.track[0] = PWM_DT_SPEC_GET(DT_NODELABEL(led_track_1));
    g_leds.track[1] = PWM_DT_SPEC_GET(DT_NODELABEL(led_track_2));
    g_leds.track[2] = PWM_DT_SPEC_GET(DT_NODELABEL(led_track_3));
    g_leds.track[3] = PWM_DT_SPEC_GET(DT_NODELABEL(led_track_4));
    g_leds.play[0]  = PWM_DT_SPEC_GET(DT_NODELABEL(led_play_1));
    g_leds.play[1]  = PWM_DT_SPEC_GET(DT_NODELABEL(led_play_2));
    g_leds.play[2]  = PWM_DT_SPEC_GET(DT_NODELABEL(led_play_3));
    g_leds.play[3]  = PWM_DT_SPEC_GET(DT_NODELABEL(led_play_4));
    g_leds.all_off();
    core::watchdog::feed(g_wdt_ch);

    const gpio_dt_spec emmc_supply = GPIO_DT_SPEC_GET(DT_NODELABEL(emmc), supply_gpios);
    const gpio_dt_spec emmc_reset  = GPIO_DT_SPEC_GET(DT_NODELABEL(emmc), reset_gpios);
    g_emmc.configure_control_gpios(emmc_supply, emmc_reset);
    if (!g_emmc.init()) {
        g_leds.set_play(0, 255);
        while (true) { core::watchdog::feed(g_wdt_ch); k_sleep(K_MSEC(500)); }
    }
    core::watchdog::feed(g_wdt_ch);

    if (init_usb() != 0) {
        g_leds.set_play(1, 255);
        while (true) { core::watchdog::feed(g_wdt_ch); k_sleep(K_MSEC(500)); }
    }

    uart_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));
    while (!device_is_ready(uart_dev)) {
        k_sleep(K_MSEC(100));
        core::watchdog::feed(g_wdt_ch);
    }
    k_sleep(K_MSEC(2000));
    core::watchdog::feed(g_wdt_ch);

    uart_irq_callback_user_data_set(uart_dev, uart_rx_cb, nullptr);
    uart_irq_rx_enable(uart_dev);

    k_thread_create(&write_thread_data, write_stack, WRITE_STACK_SIZE,
                    emmc_write_thread_fn, nullptr, nullptr, nullptr,
                    WRITE_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&write_thread_data, "emmc_write");

    g_leds.set_play(3, 64);

    while (true) {
        run_session();
        g_leds.all_off();
        g_leds.set_play(3, 64);
    }
}
