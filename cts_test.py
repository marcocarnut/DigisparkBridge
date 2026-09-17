#!/usr/bin/env python3
"""Check the bridge's CTS input: tell it to wait, see that it stops sending
and that nothing is lost when it is let go again.

usage: cts_test.py [--adapter /dev/ttyUSB0] [--bridge /dev/ttyACM0]
                   [--baud 9600] [--bytes 200]

The adapter's RTS output, wired to the bridge's PA3, is driven by hand rather
than by the kernel's flow control, so this tests the bridge and not the two
drivers. TIOCM_RTS set means the signal is asserted, which for RTS# means the
pin is low: the bridge may send.
"""
import argparse
import fcntl
import os
import select
import struct
import termios
import time
import tty


def open_port(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = getattr(termios, f"B{baud}")
    attrs[2] &= ~termios.CRTSCTS  # we drive RTS ourselves
    attrs[2] |= termios.CLOCAL | termios.CREAD
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def set_rts(fd, asserted):
    op = termios.TIOCMBIS if asserted else termios.TIOCMBIC
    fcntl.ioctl(fd, op, struct.pack("I", termios.TIOCM_RTS))


def read_for(fd, seconds):
    out, end = b"", time.time() + seconds
    while time.time() < end:
        if select.select([fd], [], [], 0.1)[0]:
            out += os.read(fd, 4096)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--adapter", default="/dev/ttyUSB0")
    ap.add_argument("--bridge", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--bytes", type=int, default=200)
    args = ap.parse_args()

    adapter = open_port(args.adapter, args.baud)
    bridge = open_port(args.bridge, args.baud)
    time.sleep(0.3)
    read_for(adapter, 0.3)  # anything left over

    set_rts(adapter, False)  # RTS high: the bridge must wait
    time.sleep(0.1)
    data = bytes((i * 7 + 13) & 0xFF for i in range(args.bytes))
    sent = 0
    end = time.time() + 2
    while sent < len(data) and time.time() < end:
        if select.select([], [bridge], [], 0.1)[1]:
            try:
                sent += os.write(bridge, data[sent:])
            except BlockingIOError:
                pass
    got = read_for(adapter, 1.0)
    print(f"told to wait: the host got {sent} of {len(data)} bytes into the bridge"
          f" before it was pushed back, and the adapter saw {len(got)}")

    set_rts(adapter, True)  # RTS low: go ahead
    quiet = time.time() + 2
    while len(got) < len(data) and time.time() < quiet:
        writers = [bridge] if sent < len(data) else []
        readable, writable, _ = select.select([adapter], writers, [], 0.1)
        if writable:
            try:
                sent += os.write(bridge, data[sent:])
            except BlockingIOError:
                pass
        if readable:
            chunk = os.read(adapter, 4096)
            if chunk:
                got += chunk
                quiet = time.time() + 2
    print(f"let go: the adapter saw {len(got)} of {len(data)} bytes,"
          f" {'in order and intact' if got == data else 'WRONG'}")
    if got != data:
        print(f"  sent {data[:32].hex()} ...")
        print(f"  got  {got[:32].hex()} ...")
    os.close(bridge)
    os.close(adapter)


if __name__ == "__main__":
    main()
