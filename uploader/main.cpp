/// @file main.cpp
/// @brief SP-1 USB album uploader firmware.
///
/// Receives album data over USB CDC ACM and writes it to eMMC.
/// Protocol:
///   Session header (host -> fw, 12B): "SPUL" | total_chunks u32LE | start_chunk u32LE
///   Session ack    (fw -> host,  4B): resume_from_chunk u32LE (echoes host's start_chunk)
///   Per chunk (host -> fw, 4100B): block_addr u32LE | 4096B data
///   Per chunk ack (fw -> host,  1B): 0x00 OK | 0x01 retry | 0x02 done
///
/// Album magic injection: on block_addr==0, read albumLength from chunk[13..16] (LE32).
/// After last chunk, write 8192B terminator at eMMC block (albumLength-1)*16 with
/// "ALBUM_PRESENT" at byte offset 8179. Then send ACK_DONE.
///
/// LED progress: T1 @25%, T2 @50%, T3 @75%, T4 @100% of chunks written.

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

// ── USB CDC ACM device ──────────────────────────────────────────

#define SP_USB_VID 0x1209
#define SP_USB_PID 0x0002

USBD_DEVICE_DEFINE(sp_usbd,
                   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
                   SP_USB_VID, SP_USB_PID);

USBD_DESC_LANG_DEFINE(sp_lang);
USBD_DESC_MANUFACTURER_DEFINE(sp_mfr, "Teenage Engineering");
USBD_DESC_PRODUCT_DEFINE(sp_product, "SP-1 Album Uploader");
USBD_DESC_CONFIG_DEFINE(sp_fs_cfg, "FS Configuration");
USBD_CONFIGURATION_DEFINE(sp_fs_config, 0, 125, &sp_fs_cfg);

static int init_usb(void) {
    int err;
    if ((err = usbd_add_descriptor(&sp_usbd, &sp_lang))) return err;
    if ((err = usbd_add_descriptor(&sp_usbd, &sp_mfr))) return err;
    if ((err = usbd_add_descriptor(&sp_usbd, &sp_product))) return err;
    if ((err = usbd_add_configuration(&sp_usbd, USBD_SPEED_FS, &sp_fs_config))) return err;
    if ((err = usbd_register_all_classes(&sp_usbd, USBD_SPEED_FS, 1, nullptr))) return err;
    if ((err = usbd_init(&sp_usbd))) return err;
    if ((err = usbd_enable(&sp_usbd))) return err;
    return 0;
}

// ── Globals (large buffers + drivers stay off the stack) ────────

static leds::Leds g_leds;
static hardware::EmmcDriver g_emmc;

static constexpr size_t CHUNK_DATA_SIZE = 4096;
static constexpr size_t CHUNK_WIRE_SIZE = CHUNK_DATA_SIZE + 4;  // addr + data
static constexpr size_t TERMINATOR_SIZE = 8192;
static constexpr size_t HEADER_SIZE = 12;
// 50 ms eMMC retry stall × ~400 KB/s USB throughput = ~20 KB in flight.
// 32 KB gives comfortable margin so the IRQ ring never silently drops bytes.
static constexpr size_t RX_RING_SIZE = 32768;

static uint8_t rx_ring_buf[RX_RING_SIZE];
static ring_buf rx_ring;

alignas(4) static uint8_t chunk_buf[CHUNK_WIRE_SIZE];
alignas(4) static uint8_t term_buf[TERMINATOR_SIZE];

static const struct device *uart_dev;
static int g_wdt_ch;

// ── UART RX IRQ → ring buffer ───────────────────────────────────

static void uart_rx_cb(const struct device *dev, void *) {
    if (!uart_irq_update(dev)) return;
    while (uart_irq_rx_ready(dev)) {
        uint8_t b;
        if (uart_fifo_read(dev, &b, 1) != 1) break;
        ring_buf_put(&rx_ring, &b, 1);
    }
}

// ── I/O helpers ─────────────────────────────────────────────────

static void recv_bytes(uint8_t *dst, size_t n, int wdt_ch) {
    size_t got = 0;
    while (got < n) {
        const uint32_t r = ring_buf_get(&rx_ring, dst + got, n - got);
        if (r == 0) {
            core::watchdog::feed(wdt_ch);
            k_sleep(K_MSEC(1));
        } else {
            got += r;
        }
    }
}

static void send_bytes(const struct device *dev, const uint8_t *src, size_t len, int wdt_ch) {
    for (size_t i = 0; i < len; i++)
        uart_poll_out(dev, src[i]);
    core::watchdog::feed(wdt_ch);
}

// ── Protocol ack codes ──────────────────────────────────────────

static constexpr uint8_t ACK_OK    = 0x00;
static constexpr uint8_t ACK_RETRY = 0x01;
static constexpr uint8_t ACK_DONE  = 0x02;

static constexpr int MAX_RETRIES = 3;
static constexpr size_t BLOCKS_PER_CHUNK = 8;

// ── LED progress (T1@25%, T2@50%, T3@75%, T4@100%) ─────────────

static void update_progress_leds(uint32_t done, uint32_t total) {
    if (total == 0) return;
    const uint8_t bright = 200;
    if (done >= total / 4)       g_leds.set_track(0, bright);
    if (done >= total / 2)       g_leds.set_track(1, bright);
    if (done >= (total * 3) / 4) g_leds.set_track(2, bright);
    if (done >= total)           g_leds.set_track(3, bright);
}

// ── Session: receive one album upload ──────────────────────────

static void run_session(void) {
    const int wdt_ch = g_wdt_ch;

    // Sliding-window scan for "SPUL" magic so a single noise byte before a
    // valid header doesn't consume the next 4 bytes of real data.
    uint8_t hdr[HEADER_SIZE];
    recv_bytes(hdr, 4, wdt_ch);
    while (!(hdr[0] == 'S' && hdr[1] == 'P' && hdr[2] == 'U' && hdr[3] == 'L')) {
        hdr[0] = hdr[1]; hdr[1] = hdr[2]; hdr[2] = hdr[3];
        recv_bytes(hdr + 3, 1, wdt_ch);
        core::watchdog::feed(wdt_ch);
    }
    recv_bytes(hdr + 4, HEADER_SIZE - 4, wdt_ch);

    uint32_t total_chunks, start_chunk;
    memcpy(&total_chunks, &hdr[4], 4);
    memcpy(&start_chunk,  &hdr[8], 4);

    g_leds.all_off();

    // Session ack: echo resume_from_chunk back to host.
    uint8_t resume_ack[4];
    memcpy(resume_ack, &start_chunk, 4);
    send_bytes(uart_dev, resume_ack, 4, wdt_ch);

    // NOTE: if start_chunk > 0, the block_addr==0 chunk won't be received,
    // so album_length stays 0 and the terminator write is skipped. Acceptable
    // for MVP — host is responsible for re-sending chunk 0 if it wants the
    // album marker written on resume.
    uint32_t album_length_sectors = 0;
    uint32_t chunks_done = start_chunk;

    for (uint32_t i = start_chunk; i < total_chunks; i++) {
        core::watchdog::feed(wdt_ch);
        recv_bytes(chunk_buf, CHUNK_WIRE_SIZE, wdt_ch);

        uint32_t block_addr;
        memcpy(&block_addr, chunk_buf, 4);
        const uint8_t *data = chunk_buf + 4;

        // Parse albumLength from the metadata sector — only on the first chunk.
        // For resumes (start_chunk > 0) the metadata chunk is not resent,
        // so album_length_sectors stays 0 and the terminator is skipped.
        if (i == 0) {
            memcpy(&album_length_sectors, data + 13, 4);
        }

        bool written = false;
        for (int attempt = 0; attempt < MAX_RETRIES && !written; ++attempt) {
            if (g_emmc.cmd25_write_multiple(block_addr, data, BLOCKS_PER_CHUNK))
                written = true;
            else {
                k_sleep(K_MSEC(50));
                core::watchdog::feed(wdt_ch);
            }
        }
        if (!written) {
            uint8_t ack = ACK_RETRY;
            send_bytes(uart_dev, &ack, 1, wdt_ch);
            return;  // abort session — caller's while(true) will re-enter
        }

        uint8_t ack = ACK_OK;
        send_bytes(uart_dev, &ack, 1, wdt_ch);
        ++chunks_done;
        update_progress_leds(chunks_done, total_chunks);
    }

    // Album terminator: 8192B at eMMC block (album_length - 1) * 16.
    // On resume (album_length_sectors == 0) we skip the terminator write but
    // still send ACK_DONE — the chunk data was written, only the marker is absent.
    bool term_ok = false;
    if (album_length_sectors > 0) {
        memset(term_buf, 0, TERMINATOR_SIZE);
        static const char marker[] = "ALBUM_PRESENT";  // 13 bytes (no NUL)
        memcpy(term_buf + 8179, marker, 13);
        core::watchdog::feed(wdt_ch);
        const uint32_t term_block = (album_length_sectors - 1u) * 16u;
        for (int attempt = 0; attempt < MAX_RETRIES && !term_ok; ++attempt) {
            term_ok = g_emmc.cmd25_write_multiple(term_block, term_buf, 16);
            if (!term_ok) {
                k_sleep(K_MSEC(50));
                core::watchdog::feed(wdt_ch);
            }
        }
    }

    const uint8_t ack = (album_length_sectors == 0 || term_ok) ? ACK_DONE : ACK_RETRY;
    send_bytes(uart_dev, &ack, 1, wdt_ch);
}

// ── main ────────────────────────────────────────────────────────

int main(void) {
    core::watchdog::start();
    g_wdt_ch = core::watchdog::register_channel("uploader", 10000);
    core::watchdog::feed(g_wdt_ch);

    ring_buf_init(&rx_ring, sizeof(rx_ring_buf), rx_ring_buf);

    // LEDs
    g_leds.track[0] = PWM_DT_SPEC_GET(DT_NODELABEL(led_track_1));
    g_leds.track[1] = PWM_DT_SPEC_GET(DT_NODELABEL(led_track_2));
    g_leds.track[2] = PWM_DT_SPEC_GET(DT_NODELABEL(led_track_3));
    g_leds.track[3] = PWM_DT_SPEC_GET(DT_NODELABEL(led_track_4));
    g_leds.play[0] = PWM_DT_SPEC_GET(DT_NODELABEL(led_play_1));
    g_leds.play[1] = PWM_DT_SPEC_GET(DT_NODELABEL(led_play_2));
    g_leds.play[2] = PWM_DT_SPEC_GET(DT_NODELABEL(led_play_3));
    g_leds.play[3] = PWM_DT_SPEC_GET(DT_NODELABEL(led_play_4));
    g_leds.all_off();
    core::watchdog::feed(g_wdt_ch);

    // eMMC
    const gpio_dt_spec emmc_supply = GPIO_DT_SPEC_GET(DT_NODELABEL(emmc), supply_gpios);
    const gpio_dt_spec emmc_reset  = GPIO_DT_SPEC_GET(DT_NODELABEL(emmc), reset_gpios);
    g_emmc.configure_control_gpios(emmc_supply, emmc_reset);
    if (!g_emmc.init()) {
        // eMMC init failure: solid play LED 1 + halt-spin (still feed WDT).
        g_leds.set_play(0, 255);
        while (true) { core::watchdog::feed(g_wdt_ch); k_sleep(K_MSEC(500)); }
    }
    core::watchdog::feed(g_wdt_ch);

    // USB
    if (init_usb() != 0) {
        g_leds.set_play(1, 255);
        while (true) { core::watchdog::feed(g_wdt_ch); k_sleep(K_MSEC(500)); }
    }

    // CDC ACM UART
    uart_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));
    while (!device_is_ready(uart_dev)) {
        k_sleep(K_MSEC(100));
        core::watchdog::feed(g_wdt_ch);
    }
    k_sleep(K_MSEC(2000));  // USB enumeration grace period
    core::watchdog::feed(g_wdt_ch);

    uart_irq_callback_user_data_set(uart_dev, uart_rx_cb, nullptr);
    uart_irq_rx_enable(uart_dev);

    // Ready indicator: play LED 4.
    g_leds.set_play(3, 64);

    while (true) {
        run_session();
        g_leds.all_off();
        g_leds.set_play(3, 64);
    }
}
