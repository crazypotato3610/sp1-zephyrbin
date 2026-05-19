#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_midi2.h>
#include <hal/nrf_power.h>
#include <zephyr/sys/poweroff.h>

extern "C" {
#include <ump_stream_responder.h>
}

#include "Leds.hpp"
#include "MidiController.hpp"
#include "power/PowerManager.hpp"
#include "system/Watchdog.hpp"
#include "EmmcDriver.hpp"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// ── USB composite device (CDC ACM + MIDI) ───────────────────────

#define SP_USB_VID  0x1209
#define SP_USB_PID  0x0001

USBD_DEVICE_DEFINE(sp_usbd,
                   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
                   SP_USB_VID, SP_USB_PID);

USBD_DESC_LANG_DEFINE(sp_lang);
USBD_DESC_MANUFACTURER_DEFINE(sp_mfr, "Teenage Engineering");
USBD_DESC_PRODUCT_DEFINE(sp_product, "SP-1 MIDI Controller");
USBD_DESC_CONFIG_DEFINE(sp_fs_cfg, "FS Configuration");

USBD_CONFIGURATION_DEFINE(sp_fs_config, 0, 125, &sp_fs_cfg);

static struct usbd_context *init_usb(void) {
    int err;

    printk("USB: adding descriptors\r\n");
    err = usbd_add_descriptor(&sp_usbd, &sp_lang);
    if (err) { printk("USB lang desc: %d\r\n", err); return nullptr; }

    err = usbd_add_descriptor(&sp_usbd, &sp_mfr);
    if (err) { printk("USB mfr desc: %d\r\n", err); return nullptr; }

    err = usbd_add_descriptor(&sp_usbd, &sp_product);
    if (err) { printk("USB product desc: %d\r\n", err); return nullptr; }

    printk("USB: adding FS configuration\r\n");
    err = usbd_add_configuration(&sp_usbd, USBD_SPEED_FS, &sp_fs_config);
    if (err) { printk("USB config: %d\r\n", err); return nullptr; }

    printk("USB: registering all classes\r\n");
    err = usbd_register_all_classes(&sp_usbd, USBD_SPEED_FS, 1, nullptr);
    if (err) { printk("USB register classes: %d\r\n", err); return nullptr; }

    // Composite device: use IAD triple (Misc/0x02/0x01)
    usbd_device_set_code_triple(&sp_usbd, USBD_SPEED_FS,
                                USB_BCC_MISCELLANEOUS, 0x02, 0x01);

    printk("USB: init\r\n");
    err = usbd_init(&sp_usbd);
    if (err) { printk("USB init: %d\r\n", err); return nullptr; }

    printk("USB: enable\r\n");
    err = usbd_enable(&sp_usbd);
    if (err) { printk("USB enable: %d\r\n", err); return nullptr; }

    printk("USB: composite device ready\r\n");
    return &sp_usbd;
}

// ── USB MIDI device from device tree ────────────────────────────

#define USB_MIDI_DT_NODE DT_NODELABEL(usb_midi)
static const struct device *const midi_dev = DEVICE_DT_GET(USB_MIDI_DT_NODE);

// ── Globals ─────────────────────────────────────────────────────

static midi::MidiController g_midi;
static leds::Leds g_leds;
static power::PowerManager g_power;

// ── UMP Stream Responder ────────────────────────────────────────

static const struct ump_endpoint_dt_spec ump_ep_dt =
    UMP_ENDPOINT_DT_SPEC_GET(USB_MIDI_DT_NODE);

static const struct ump_stream_responder_cfg responder_cfg =
    UMP_STREAM_RESPONDER(midi_dev, usbd_midi_send, &ump_ep_dt);

// ── USB MIDI callbacks ──────────────────────────────────────────

static void on_midi_rx(const struct device*, const struct midi_ump ump) {
    if (UMP_MT(ump) == UMP_MT_UMP_STREAM) {
        ump_stream_respond(&responder_cfg, ump);
        return;
    }
    g_midi.on_rx(ump);
}

static void on_midi_ready(const struct device*, const bool ready) {
    LOG_INF("USB MIDI %s", ready ? "ready" : "disconnected");
    g_midi.ready.store(ready, std::memory_order_relaxed);

    // Status LED: play LED 1
    g_leds.set_play(0, ready ? 128 : 0);
}

static const struct usbd_midi_ops midi_ops = {
    .rx_packet_cb = on_midi_rx,
    .ready_cb = on_midi_ready,
};

// ── Input subsystem callback (buttons → MIDI) ───────────────────

static void on_input_event(struct input_event *evt, void*) {
    if (evt == nullptr || evt->type != INPUT_EV_KEY) {
        return;
    }

    // Only act on press edges (not release) for toggle controls
    const bool pressed = (evt->value != 0);

    switch (evt->code) {
    // ── Track buttons → MIDI notes (offset by current octave) ────
    case INPUT_KEY_1:
    case INPUT_KEY_2:
    case INPUT_KEY_3:
    case INPUT_KEY_4: {
        const int base = midi::kNoteTrack1 + (evt->code - INPUT_KEY_1);
        const int note = base + (g_midi.octave_offset * 12);
        if (note >= 0 && note <= 127) {
            g_midi.send_note(static_cast<uint8_t>(note), pressed);
        }
        const int led_idx = evt->code - INPUT_KEY_1;
        if (led_idx < leds::kNumTrack) {
            g_leds.set_track(led_idx, pressed ? 200 : 0);
        }
        break;
    }

    // ── Play → MMC Play/Stop toggle ─────────────────────────────
    case INPUT_KEY_PLAY:
        if (pressed) {
            g_midi.playing = !g_midi.playing;
            g_midi.send_mmc(g_midi.playing ? midi::kMmcPlay : midi::kMmcStop);
        }
        break;

    // ── Rockers → octave shift ──────────────────────────────────
    case INPUT_KEY_FASTFORWARD:
        if (pressed && g_midi.octave_offset < 5) {
            g_midi.octave_offset++;
            LOG_INF("Octave: %+d (base C%d)",
                    g_midi.octave_offset,
                    (midi::kNoteTrack1 / 12) + g_midi.octave_offset - 1);
        }
        break;
    case INPUT_KEY_BACK:
        if (pressed && g_midi.octave_offset > -5) {
            g_midi.octave_offset--;
            LOG_INF("Octave: %+d (base C%d)",
                    g_midi.octave_offset,
                    (midi::kNoteTrack1 / 12) + g_midi.octave_offset - 1);
        }
        break;

    // ── Volume up/down → CC 7 step ──────────────────────────────
    case INPUT_KEY_VOLUMEUP:
        if (pressed && g_midi.volume_level < 127) {
            g_midi.volume_level = static_cast<uint8_t>(
                std::min<int>(g_midi.volume_level + 10, 127));
            g_midi.send_cc(midi::kCcVolume, g_midi.volume_level);
        }
        break;
    case INPUT_KEY_VOLUMEDOWN:
        if (pressed && g_midi.volume_level > 0) {
            g_midi.volume_level = static_cast<uint8_t>(
                std::max<int>(g_midi.volume_level - 10, 0));
            g_midi.send_cc(midi::kCcVolume, g_midi.volume_level);
        }
        break;

    // ── Function → MMC Record on/off toggle ─────────────────────
    case INPUT_KEY_MENU:
        if (pressed) {
            g_midi.recording = !g_midi.recording;
            g_midi.send_mmc(g_midi.recording
                ? midi::kMmcRecordOn : midi::kMmcRecordOff);
        }
        break;

    default:
        break;
    }
}
INPUT_CALLBACK_DEFINE(NULL, on_input_event, NULL);

// ── main ────────────────────────────────────────────────────────

int main(void) {
    core::watchdog::start();
    const int main_wdt = core::watchdog::register_channel("main", 5000);
    core::watchdog::feed(main_wdt);

    LOG_INF("Stem Player MIDI Controller Booting...");

    // ── USB composite device (CDC ACM console + MIDI) ───────────
    if (init_usb() == nullptr) {
        LOG_ERR("USB init failed");
    } else {
        LOG_INF("USB composite device ready (CDC ACM + MIDI)");
    }
    core::watchdog::feed(main_wdt);

    // ── LEDs ────────────────────────────────────────────────────
    g_leds.track[0] = PWM_DT_SPEC_GET(DT_NODELABEL(led_track_1));
    g_leds.track[1] = PWM_DT_SPEC_GET(DT_NODELABEL(led_track_2));
    g_leds.track[2] = PWM_DT_SPEC_GET(DT_NODELABEL(led_track_3));
    g_leds.track[3] = PWM_DT_SPEC_GET(DT_NODELABEL(led_track_4));
    g_leds.play[0] = PWM_DT_SPEC_GET(DT_NODELABEL(led_play_1));
    g_leds.play[1] = PWM_DT_SPEC_GET(DT_NODELABEL(led_play_2));
    g_leds.play[2] = PWM_DT_SPEC_GET(DT_NODELABEL(led_play_3));
    g_leds.play[3] = PWM_DT_SPEC_GET(DT_NODELABEL(led_play_4));
    g_leds.all_off();
    core::watchdog::feed(main_wdt);

    // Give USB CDC ACM time to enumerate on the host before logging
    k_sleep(K_MSEC(3000));
    core::watchdog::feed(main_wdt);

    // Power monitor must start before the test block so the function-button
    // long-press power-off handler is active even if the test loops forever.
    g_power.init();
    core::watchdog::feed(main_wdt);

    // ── eMMC CMD25 multi-block write+read+verify self-test ──────
    // fail_stage: 0=pass 1=init 2=cmd25-write 3=cmd24-readback-read-fail 4=cmd18-mismatch 5=cmd17-spot-mismatch
    //             6=cmd24-write 7=cmd24-readback-mismatch 8=cmd17-spot-read-fail 9=cmd18-multi-read-fail
    // T2/T3/T4: stage 2 → last_write_error (+R1 nibbles); stage 4 → fail_byte high nibble + actual byte nibbles
    //           stage 5 → actual byte nibbles; stage 6 → last_write_error; stage 7 → fail_byte 32B-chunk + actual nibbles
    // T2/T3/T4 blink count = nibble_value + 1 (so 1 blink = nibble 0, 16 blinks = nibble 15)
    {
        static hardware::EmmcDriver emmc;
        const gpio_dt_spec emmc_supply =
            GPIO_DT_SPEC_GET(DT_NODELABEL(emmc), supply_gpios);
        const gpio_dt_spec emmc_reset =
            GPIO_DT_SPEC_GET(DT_NODELABEL(emmc), reset_gpios);
        emmc.configure_control_gpios(emmc_supply, emmc_reset);

        int fail_stage = 0;
        int fail_byte = -1;
        uint8_t spot_actual = 0;    // actual byte at block 1 byte 0 from CMD17 (for fail_stage=5)
        uint8_t rbuf_bad_byte = 0;  // actual wrong byte from CMD18 at rbuf[fail_byte] (for fail_stage=4)
        uint8_t c24_actual = 0;     // actual byte at CMD24 readback mismatch position (for fail_stage=7)
        uint32_t test_block = 0;

        alignas(4) static uint8_t wbuf[4096];
        alignas(4) static uint8_t rbuf[4098];  // +2: DMA may bleed past last block
        alignas(4) static uint8_t sblk[512];   // single-block spot-check buffer (CMD17)
        alignas(4) static uint8_t c24buf[512]; // CMD24 single-block write pattern

        // Block-unique pattern: block b, byte j → ((b * 37 + j + 1) & 0xFF)
        // Block 0 byte 0 = 0x01, block 1 byte 0 = 0x26, block 2 = 0x4B, ...
        // Each block's first byte is distinct so we can tell which block failed and read the actual flash value.
        for (int i = 0; i < 4096; ++i)
            wbuf[i] = static_cast<uint8_t>(((uint32_t)(i >> 9) * 37u + (uint32_t)(i & 0xFF) + 1u) & 0xFFu);

        if (!emmc.init()) {
            fail_stage = 1;
        } else if (emmc.capacity_blocks() < 65) {
            fail_stage = 1;
        } else {
            test_block = emmc.capacity_blocks() - 64;
            const uint32_t c24_block = test_block - 16;

            for (int i = 0; i < 512; ++i)
                c24buf[i] = static_cast<uint8_t>((c24_block * 37u + (uint32_t)i + 1u) & 0xFFu);

            // CMD24 single-block write+readback first: verifies SPIM3 infrastructure
            // independently of the CMD25 multi-block path.
            bool c24_ok = false;
            if (!emmc.cmd24_write_single(c24_block, c24buf)) {
                fail_stage = 6;
            } else if (!emmc.read_blocks_sync(c24_block, sblk, 1)) {
                fail_stage = 3;
            } else {
                c24_ok = true;
                for (int i = 0; i < 512; ++i) {
                    if (sblk[i] != c24buf[i]) {
                        fail_stage = 7;
                        fail_byte  = i;
                        c24_actual = sblk[i];
                        c24_ok = false;
                        break;
                    }
                }
            }

            if (c24_ok) {
                if (!emmc.cmd25_write_multiple(test_block, wbuf, 8)) {
                    fail_stage = 2;
                } else {
                    // Spot-check block 1 via CMD17 (single-block path, confirmed working for CMD24).
                    // This disambiguates: if block 1 is wrong via CMD17, the WRITE is failing.
                    // If block 1 is correct via CMD17 but wrong via CMD18, the multi-block READ is failing.
                    if (!emmc.read_blocks_sync(test_block + 1, sblk, 1)) {
                        fail_stage = 8;
                    } else if (sblk[0] != wbuf[512]) {
                        // Block 1 byte 0 wrong via CMD17 → write is definitely failing for block 1.
                        // T2 = upper nibble of actual flash value, T3 = lower nibble.
                        fail_stage = 5;
                        spot_actual = sblk[0];
                    } else {
                        // Block 1 confirmed correct via CMD17. Now verify all 8 blocks via CMD18.
                        if (!emmc.read_blocks_sync(test_block, rbuf, 8)) {
                            fail_stage = 9;
                        } else {
                            for (int i = 0; i < 4096; ++i) {
                                if (rbuf[i] != wbuf[i]) {
                                    fail_stage = 4;
                                    fail_byte = i;
                                    rbuf_bad_byte = rbuf[i];  // capture actual wrong value from CMD18
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
        core::watchdog::feed(main_wdt);

        // Hold function button 3 s to power off from either terminal state.
        int64_t fn_hold_start = 0;
        auto poll_poweroff = [&]() {
            const bool pressed = g_power.function_button_pressed();
            const int64_t now = k_uptime_get();
            if (pressed && fn_hold_start == 0) fn_hold_start = now;
            if (!pressed) fn_hold_start = 0;
            if (fn_hold_start != 0 && (now - fn_hold_start) >= 3000) {
                g_leds.all_off();
                sys_poweroff();
            }
        };

        if (fail_stage == 0) {
            for (int i = 0; i < leds::kNumTrack; ++i) g_leds.set_track(i, 255);
            while (true) { core::watchdog::feed(main_wdt); poll_poweroff(); k_sleep(K_MSEC(100)); }
        }

        // FAIL: T1 blinks stage, T2 blinks sub-error
        while (true) {
            core::watchdog::feed(main_wdt);
            poll_poweroff();
            for (int b = 0; b < fail_stage; ++b) {
                g_leds.set_track(0, 255); k_sleep(K_MSEC(300));
                g_leds.set_track(0, 0);   k_sleep(K_MSEC(300));
                core::watchdog::feed(main_wdt);
                poll_poweroff();
            }
            k_sleep(K_MSEC(800));
            core::watchdog::feed(main_wdt);
            poll_poweroff();
            int t2 = 0;
            if (fail_stage == 2) t2 = emmc.last_write_error;
            if (fail_stage == 4) t2 = (fail_byte >= 0) ? ((fail_byte >> 8) & 0xF) : 0;
            if (fail_stage == 5) t2 = (spot_actual >> 4) & 0x0F;  // upper nibble of actual flash byte
            if (fail_stage == 6) t2 = emmc.last_write_error;
            if (fail_stage == 7 && fail_byte >= 0) t2 = ((fail_byte & 0xFF) >> 5) & 0x7;  // 0..7 covering byte positions in 32-byte chunks
            for (int b = 0; b < t2 + 1; ++b) {
                g_leds.set_track(1, 255); k_sleep(K_MSEC(300));
                g_leds.set_track(1, 0);   k_sleep(K_MSEC(300));
                core::watchdog::feed(main_wdt);
                poll_poweroff();
            }
            // T3/T4: blink high byte of R1 as two nibbles when CMD25 returned R1 error
            // T3 = bits 31-28 of R1 (upper nibble), T4 = bits 27-24 of R1 (lower nibble)
            if (fail_stage == 2 && emmc.last_write_error == 11) {
                k_sleep(K_MSEC(800));
                core::watchdog::feed(main_wdt);
                poll_poweroff();
                const int t3 = static_cast<int>((emmc.last_failed_block_index >> 28) & 0x0F);
                for (int b = 0; b < t3 + 1; ++b) {
                    g_leds.set_track(2, 255); k_sleep(K_MSEC(300));
                    g_leds.set_track(2, 0);   k_sleep(K_MSEC(300));
                    core::watchdog::feed(main_wdt);
                    poll_poweroff();
                }
                k_sleep(K_MSEC(800));
                core::watchdog::feed(main_wdt);
                poll_poweroff();
                const int t4 = static_cast<int>((emmc.last_failed_block_index >> 24) & 0x0F);
                for (int b = 0; b < t4 + 1; ++b) {
                    g_leds.set_track(3, 255); k_sleep(K_MSEC(300));
                    g_leds.set_track(3, 0);   k_sleep(K_MSEC(300));
                    core::watchdog::feed(main_wdt);
                    poll_poweroff();
                }
            }
            // T3/T4: actual wrong byte value returned by CMD18 at rbuf[fail_byte].
            // T2 = upper nibble of fail_byte index (still shows WHICH byte failed).
            // T3 = upper nibble of actual CMD18 value, T4 = lower nibble.
            // Decode: actual = (T3 << 4) | T4. Expected block 1 byte 0 = 0x26.
            if (fail_stage == 4 && fail_byte >= 0) {
                k_sleep(K_MSEC(800));
                core::watchdog::feed(main_wdt);
                poll_poweroff();
                const int t3 = (rbuf_bad_byte >> 4) & 0x0F;
                for (int b = 0; b < t3 + 1; ++b) {
                    g_leds.set_track(2, 255); k_sleep(K_MSEC(300));
                    g_leds.set_track(2, 0);   k_sleep(K_MSEC(300));
                    core::watchdog::feed(main_wdt);
                    poll_poweroff();
                }
                k_sleep(K_MSEC(800));
                core::watchdog::feed(main_wdt);
                poll_poweroff();
                const int t4 = rbuf_bad_byte & 0x0F;
                for (int b = 0; b < t4 + 1; ++b) {
                    g_leds.set_track(3, 255); k_sleep(K_MSEC(300));
                    g_leds.set_track(3, 0);   k_sleep(K_MSEC(300));
                    core::watchdog::feed(main_wdt);
                    poll_poweroff();
                }
            }
            // fail_stage=5: block 1 wrong via CMD17 (write confirmed bad).
            // T2 = upper nibble of actual flash byte (already blinked above).
            // T3 = lower nibble of actual flash byte.
            // Decode: actual = (T2 << 4) | T3.
            // Expected block 1 byte 0 = 0x26 (block-unique pattern).
            // 0xFF = never written (erased state). 0x5A = old repeating-pattern data.
            if (fail_stage == 5) {
                k_sleep(K_MSEC(800));
                core::watchdog::feed(main_wdt);
                poll_poweroff();
                const int t3_spot = spot_actual & 0x0F;
                for (int b = 0; b < t3_spot + 1; ++b) {
                    g_leds.set_track(2, 255); k_sleep(K_MSEC(300));
                    g_leds.set_track(2, 0);   k_sleep(K_MSEC(300));
                    core::watchdog::feed(main_wdt);
                    poll_poweroff();
                }
            }
            // T3×1 = CRC error (0b101), T3×2 = write error (0b110)
            if (fail_stage == 2 && emmc.last_write_error == 4) {
                k_sleep(K_MSEC(800));
                core::watchdog::feed(main_wdt);
                poll_poweroff();
                const int t3 = (emmc.last_write_response_status == 0b101U) ? 1 : 2;
                for (int b = 0; b < t3; ++b) {
                    g_leds.set_track(2, 255); k_sleep(K_MSEC(300));
                    g_leds.set_track(2, 0);   k_sleep(K_MSEC(300));
                    core::watchdog::feed(main_wdt);
                    poll_poweroff();
                }
            }
            // fail_stage=7: CMD24 readback mismatch — same shape as stage 4.
            // T2 (already blinked) = upper nibble of fail_byte index.
            // T3 = upper nibble of actual readback byte, T4 = lower nibble.
            // Decode: actual = (T3 << 4) | T4. Expected = (c24_block * 37 + fail_byte + 1) & 0xFF.
            if (fail_stage == 7 && fail_byte >= 0) {
                k_sleep(K_MSEC(800));
                core::watchdog::feed(main_wdt);
                poll_poweroff();
                const int t3 = (c24_actual >> 4) & 0x0F;
                for (int b = 0; b < t3 + 1; ++b) {
                    g_leds.set_track(2, 255); k_sleep(K_MSEC(300));
                    g_leds.set_track(2, 0);   k_sleep(K_MSEC(300));
                    core::watchdog::feed(main_wdt);
                    poll_poweroff();
                }
                k_sleep(K_MSEC(800));
                core::watchdog::feed(main_wdt);
                poll_poweroff();
                const int t4 = c24_actual & 0x0F;
                for (int b = 0; b < t4 + 1; ++b) {
                    g_leds.set_track(3, 255); k_sleep(K_MSEC(300));
                    g_leds.set_track(3, 0);   k_sleep(K_MSEC(300));
                    core::watchdog::feed(main_wdt);
                    poll_poweroff();
                }
            }
            // fail_stage=6: CMD24 write failure. Mirror stage 2's badstatus sub-case.
            // T3×1 = CRC error (0b101), T3×2 = write error (0b110), only when last_write_error == 4.
            if (fail_stage == 6 && emmc.last_write_error == 4) {
                k_sleep(K_MSEC(800));
                core::watchdog::feed(main_wdt);
                poll_poweroff();
                const int t3 = (emmc.last_write_response_status == 0b101U) ? 1 : 2;
                for (int b = 0; b < t3; ++b) {
                    g_leds.set_track(2, 255); k_sleep(K_MSEC(300));
                    g_leds.set_track(2, 0);   k_sleep(K_MSEC(300));
                    core::watchdog::feed(main_wdt);
                    poll_poweroff();
                }
            }
            k_sleep(K_MSEC(800));
            core::watchdog::feed(main_wdt);
            poll_poweroff();
            k_sleep(K_MSEC(700));
        }
    }
}
