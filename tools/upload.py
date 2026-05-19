#!/usr/bin/env python3
"""Host-side album uploader for the SP-1 music player."""

import argparse
import struct
import sys
import time

import serial

MAGIC = b'SPUL'
CHUNK_SIZE = 4096
BLOCKS_PER_CHUNK = 8  # 4096 / 512
SESSION_HEADER_FMT = '<4sII'
SESSION_ACK_FMT = '<I'
CHUNK_HEADER_FMT = '<I'
SESSION_ACK_LEN = 4
CHUNK_ACK_LEN = 1
ACK_OK = 0x00
ACK_RETRY = 0x01
ACK_DONE = 0x02
MAX_RETRIES = 3
SERIAL_TIMEOUT = 30.0
BAUDRATE = 115200


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


def send_session_header(ser, total_chunks, start_chunk):
    ser.write(struct.pack(SESSION_HEADER_FMT, MAGIC, total_chunks, start_chunk))
    (resume_from,) = struct.unpack(SESSION_ACK_FMT, read_exact(ser, SESSION_ACK_LEN))
    return resume_from


def send_chunk(ser, chunk_index, data):
    block_addr = chunk_index * BLOCKS_PER_CHUNK
    ser.write(struct.pack(CHUNK_HEADER_FMT, block_addr) + data)
    return read_exact(ser, CHUNK_ACK_LEN)[0]


def print_progress(chunk, total, start_time, bytes_sent):
    pct = 100.0 * chunk / total
    elapsed = time.monotonic() - start_time
    kbps = (bytes_sent / 1024.0) / elapsed if elapsed > 0 else 0.0
    sys.stdout.write(f'\r{chunk}/{total}  {pct:.1f}%  {kbps:.0f} KB/s')
    sys.stdout.flush()


def upload(port, album_path, resume_from):
    album = load_album(album_path)
    total_chunks = len(album) // CHUNK_SIZE

    if resume_from < 0 or resume_from > total_chunks:
        raise ValueError(f'--resume-from {resume_from} out of range [0, {total_chunks}]')

    with serial.Serial(port, baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
        fw_resume = send_session_header(ser, total_chunks, resume_from)
        start = max(resume_from, fw_resume)
        if start > total_chunks:
            raise IOError(f'firmware reported resume_from={fw_resume} beyond total_chunks={total_chunks}')

        start_time = time.monotonic()
        bytes_sent = 0

        for i in range(start, total_chunks):
            chunk = album[i * CHUNK_SIZE:(i + 1) * CHUNK_SIZE]
            attempts = 0
            while True:
                ack = send_chunk(ser, i, chunk)
                if ack == ACK_OK:
                    bytes_sent += CHUNK_SIZE
                    print_progress(i + 1, total_chunks, start_time, bytes_sent)
                    break
                if ack == ACK_DONE:
                    bytes_sent += CHUNK_SIZE
                    elapsed = time.monotonic() - start_time
                    kbps = (bytes_sent / 1024.0) / elapsed if elapsed > 0 else 0.0
                    print_progress(i + 1, total_chunks, start_time, bytes_sent)
                    sys.stdout.write(f'\ndone: {bytes_sent} bytes in {elapsed:.2f}s ({kbps:.0f} KB/s)\n')
                    return
                if ack == ACK_RETRY:
                    attempts += 1
                    if attempts >= MAX_RETRIES:
                        raise IOError(f'chunk {i} failed after {MAX_RETRIES} retries')
                    continue
                raise IOError(f'unknown ack 0x{ack:02x} for chunk {i}')

        elapsed = time.monotonic() - start_time
        kbps = (bytes_sent / 1024.0) / elapsed if elapsed > 0 else 0.0
        sys.stdout.write(f'\ndone: {bytes_sent} bytes in {elapsed:.2f}s ({kbps:.0f} KB/s)\n')


def main():
    parser = argparse.ArgumentParser(description='Upload an album binary to the SP-1 over serial.')
    parser.add_argument('port', help='serial port (e.g. /dev/ttyACM0)')
    parser.add_argument('album', help='album binary file')
    parser.add_argument('--resume-from', type=int, default=0, metavar='CHUNK',
                        help='start_chunk to request from firmware (default: 0)')
    args = parser.parse_args()

    try:
        upload(args.port, args.album, args.resume_from)
    except (IOError, serial.SerialException) as e:
        sys.stderr.write(f'\nerror: {e}\n')
        sys.exit(1)


if __name__ == '__main__':
    main()
