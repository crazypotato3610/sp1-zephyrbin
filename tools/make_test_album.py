#!/usr/bin/env python3
"""
Generate a minimal SP-1 test album binary for uploader testing.

Creates an 8-sector album (8 × 8192 = 65536 bytes) with:
  - Sector 0: metadata (ALBUM_PRESENT header, 1 dummy song entry)
  - Sectors 1-6: dummy audio data (incrementing byte pattern per sector)
  - Sector 7: terminator (written by firmware automatically — not included here)

Usage:
  python3 tools/make_test_album.py [output_path]
  python3 tools/make_test_album.py test_album.bin
"""

import struct
import sys
from pathlib import Path

SECTOR_SIZE  = 8192   # 16 eMMC blocks × 512 bytes
CHUNK_SIZE   = 4096   # one upload chunk = 8 eMMC blocks

# Album layout:
#   Sector 0:     metadata
#   Sectors 1-6:  audio (6 sectors of silence/pattern)
#   Sector 7:     terminator ← firmware writes this, we don't include it
ALBUM_LENGTH = 8   # total sectors including terminator (but we send 7)
AUDIO_SECTORS = ALBUM_LENGTH - 2  # exclude metadata and terminator = 6

def make_metadata_sector():
    buf = bytearray(SECTOR_SIZE)
    # Magic
    buf[0:13] = b'ALBUM_PRESENT'
    # albumLength: total sectors including terminator
    struct.pack_into('<I', buf, 13, ALBUM_LENGTH)
    # numSongs
    buf[17] = 1
    # Album title (null-terminated, padded with 0x58 'X' to 64 bytes)
    title = b'SP-1 Uploader Test\x00'
    buf[18:18 + len(title)] = title
    buf[18 + len(title):82] = b'X' * (64 - len(title))
    # Song entry at offset 82 (136 bytes):
    #   offset 0-3:  song start sector (uint32 LE) = 1 (first sector after metadata)
    #   offset 4-7:  song length in sectors (uint32 LE) = AUDIO_SECTORS
    #   offset 8-71:  artist name
    #   offset 72-135: song title
    song_entry = bytearray(136)
    struct.pack_into('<I', song_entry, 0, 1)             # start sector
    struct.pack_into('<I', song_entry, 4, AUDIO_SECTORS) # length
    artist = b'Test Artist\x00'
    song_entry[8:8 + len(artist)] = artist
    song_entry[8 + len(artist):72] = b'X' * (64 - len(artist))
    title_s = b'Test Song\x00'
    song_entry[72:72 + len(title_s)] = title_s
    song_entry[72 + len(title_s):136] = b'X' * (64 - len(title_s))
    buf[82:218] = song_entry
    return bytes(buf)

def make_audio_sector(sector_index):
    # Each byte encodes sector index in the high nibble, byte position in the low nibble.
    # Makes it easy to eyeball whether the right data landed on the right sector.
    buf = bytearray(SECTOR_SIZE)
    for i in range(SECTOR_SIZE):
        buf[i] = ((sector_index & 0xF) << 4) | (i & 0xF)
    return bytes(buf)

def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('test_album.bin')

    sectors = [make_metadata_sector()]
    for s in range(1, ALBUM_LENGTH - 1):   # sectors 1..6 (no terminator)
        sectors.append(make_audio_sector(s))

    data = b''.join(sectors)
    out.write_bytes(data)

    total_chunks = len(data) // CHUNK_SIZE
    print(f"Written: {out}  ({len(data)} bytes, {total_chunks} chunks)")
    print(f"Album layout: 1 metadata + {AUDIO_SECTORS} audio sectors (terminator written by firmware)")
    print(f"Upload: python3 tools/upload.py /dev/ttyACM0 {out}")

if __name__ == '__main__':
    main()
