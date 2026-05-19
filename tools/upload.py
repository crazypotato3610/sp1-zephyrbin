#!/usr/bin/env python3
"""Host-side album uploader for the SP-1 music player (streaming protocol)."""

import argparse
import struct
import sys
import time

import serial

MAGIC          = b'SPUL'
CHUNK_SIZE     = 32768
BLOCKS_PER_CHUNK = CHUNK_SIZE // 512
HEADER_FMT     = '<4sI'   # magic + total_chunks (8 bytes total, no start_chunk)
SERIAL_TIMEOUT = 120.0
BAUDRATE       = 115200
ACK_RETRY      = 0x01
ACK_DONE       = 0x02


def load_album(path):
    with open(path, 'rb') as f:
        data = f.read()
    pad = (-len(data)) % CHUNK_SIZE
    if pad:
        data += b'\x00' * pad
    return data


def read_exact(ser, n):
    buf = ser.read(n)
    if len(buf) != n:
        raise IOError(f'serial read timed out: expected {n} bytes, got {len(buf)}')
    return buf


def upload(port, album_path):
    album = load_album(album_path)
    total_chunks = len(album) // CHUNK_SIZE

    print(f'Album: {album_path} ({len(album)} bytes, {total_chunks} × {CHUNK_SIZE // 1024} KB chunks)')

    with serial.Serial(port, baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
        ser.write(struct.pack(HEADER_FMT, MAGIC, total_chunks))

        start_time = time.monotonic()
        for i in range(total_chunks):
            chunk = album[i * CHUNK_SIZE:(i + 1) * CHUNK_SIZE]
            ser.write(chunk)

            elapsed = time.monotonic() - start_time
            bytes_sent = (i + 1) * CHUNK_SIZE
            kbps = (bytes_sent / 1024.0) / elapsed if elapsed > 0 else 0.0
            sys.stdout.write(
                f'\r{i + 1}/{total_chunks}  {100.0 * (i + 1) / total_chunks:.1f}%  {kbps:.0f} KB/s'
            )
            sys.stdout.flush()

        sys.stdout.write('\nWaiting for firmware ACK...\n')
        sys.stdout.flush()

        ack = read_exact(ser, 1)[0]
        elapsed = time.monotonic() - start_time
        kbps = (len(album) / 1024.0) / elapsed if elapsed > 0 else 0.0

        if ack == ACK_DONE:
            print(f'done: {len(album)} bytes in {elapsed:.2f}s ({kbps:.0f} KB/s)')
        elif ack == ACK_RETRY:
            sys.stderr.write('error: firmware reported write failure (ACK_RETRY)\n')
            sys.exit(1)
        else:
            sys.stderr.write(f'error: unknown ack 0x{ack:02x}\n')
            sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description='Upload an album binary to the SP-1 over serial.')
    parser.add_argument('port',  help='serial port (e.g. /dev/ttyACM0)')
    parser.add_argument('album', help='album binary file')
    args = parser.parse_args()

    try:
        upload(args.port, args.album)
    except (IOError, serial.SerialException) as e:
        sys.stderr.write(f'error: {e}\n')
        sys.exit(1)


if __name__ == '__main__':
    main()
