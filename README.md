# sp1-zephyrbin

SP-1 custom firmware — CMD24 write test build.

## What this is

Custom Zephyr firmware for the Teenage Engineering SP-1 stem player (nRF52840).
Based on `ericlewis/sp1-midi` BSP with CMD24 eMMC write path added.

## What it does

On boot:
1. Waits 3 seconds for USB CDC to enumerate
2. Initializes the eMMC
3. Writes a 512-byte test pattern to the last 64 blocks of the eMMC using CMD24
4. Reads it back with CMD17
5. Verifies byte-for-byte match
6. Indicates result via LEDs:
   - **All 4 track LEDs solid** = PASS
   - **T1 blinks N times, T2 blinks M times** = FAIL (see below)

## LED error codes

- T1 blinks 1x = eMMC init failed
- T1 blinks 2x, T2 blinks 1x = CMD24 R1 response error
- T1 blinks 2x, T2 blinks 2x = SPIM3 transmit timeout
- T1 blinks 2x, T2 blinks 3x = no write response token from card
- T1 blinks 2x, T2 blinks 4x = bad write response status (CRC or write error)
- T1 blinks 2x, T2 blinks 5x = card busy timeout
- T1 blinks 3x = CMD17 read-back failed
- T1 blinks 4x = data mismatch (write+read succeeded but bytes differ)

## Key implementation notes

- **CPHA=0 required for SPIM3 TX path**: The driver uses CONFIG=2 (CPHA=1) for reads.
  Writes require CONFIG=0 (CPHA=0) so MOSI changes on the falling CLK edge and is
  stable by the time the eMMC samples on the rising edge. Using CPHA=1 for writes
  causes a timing race and the card returns a CRC error.
- **CONFIG restored to 2 after transmit** so the read path is unaffected.
- **alignas(4) static** (not `static alignas(4)`) — required syntax in C++17 strict mode.
- **ZEPHYR_HAL_NORDIC_MODULE_DIR/nrfx/bsp/stable/mdk** must be in include dirs —
  nrf.h moved to this location in nrfx 3.x+ (Zephyr 4.4.99 / SDK 1.0.1).

## Build environment

- Zephyr: 4.4.99 (`west init` + `west update` as of 2026-05-18)
- Zephyr SDK: 1.0.1 (via `west sdk install`)
- Board: `stem_player` (nrf52840)
- Snippet: `cdc-acm-console`

## Flashing

Hold Track 1 + Track 4, plug in USB-C, flash via solderless.engineering firmware utility.

## Source files

| File | Description |
|------|-------------|
| `zephyr.bin` | Pre-built binary, ready to flash |
| `EmmcDriver.cpp` | eMMC driver with CMD24 write path added |
| `EmmcDriver.hpp` | Header with cmd24_write_single, test_cmd24_roundtrip, last_write_error |
| `main.cpp` | Boot sequence with LED diagnostic test |
| `CMakeLists.txt` | Build config (includes storagethingies, nrf MDK path) |
| `prj.conf` | Kconfig (stack size bumped to 4096) |
