#!/usr/bin/env python3
"""Test a USB-UART bridge against a reference USB-serial adapter wired to
its UART (TX to RX, RX to TX, GND to GND).

usage: bridge_test.py [--bridge PORT] [--adapter PORT] [--bytes N] [--rx-only] [--stats] BAUD...

For each bit rate: adapter -> bridge, bridge -> adapter, then both at once,
each with N pseudo-random bytes (default 100000). With --rx-only (for a
receive-only bridge): adapter -> bridge with the host idle, then again while
the host floods the bridge's USB port with data the bridge discards. Reports throughput and
bytes lost, extra or corrupted, found by realigning the received data with
what was sent. With --stats, also prints the bridge's diagnostic counters
after each transfer (a sketch that prints them when set to 110 bps).
"""
import argparse
import os
import re
import random
import select
import termios
import time
import tty


def open_port(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, f"B{baud}")
    attrs[4] = attrs[5] = speed
    attrs[2] &= ~termios.CRTSCTS
    attrs[2] |= termios.CLOCAL | termios.CREAD
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def drain(fd, quiet=0.3):
    while select.select([fd], [], [], quiet)[0]:
        os.read(fd, 4096)


def set_speed(fd, baud):
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = getattr(termios, f"B{baud}")
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def stats(fd, baud, stats_baud=110):
    """Switch the bridge to the stats rate, collect its counter line, switch back."""
    drain(fd, 0.1)
    set_speed(fd, stats_baud)
    out = b""
    end = time.time() + 1.5
    while time.time() < end and b"\n" not in out:
        if select.select([fd], [], [], 0.1)[0]:
            out += os.read(fd, 4096)
    set_speed(fd, baud)
    time.sleep(0.1)
    m = re.search(rb"S?( [a-z0-9][0-9a-f]{4})+", out)
    if not m:
        return "no stats"
    return " ".join(f"{f[0]}={int(f[1:], 16)}" for f in m.group(0)[2:].decode().split())


def align(sent, got, window=4096, probe=16):
    """Walk both streams; on a mismatch, resynchronize by finding the next
    few received bytes further on in what was sent (bytes lost) or the other
    way round (extra bytes). Returns (lost, extra, corrupted, events, where),
    where lists the offsets in what was sent of the first mismatches."""
    i = j = lost = extra = corrupted = events = 0
    where = []
    while i < len(sent) and j < len(got):
        if sent[i] == got[j]:
            i += 1
            j += 1
            continue
        events += 1
        if len(where) < 8:
            where.append(i)
        k = sent.find(got[j:j + probe], i, i + window)
        if k > i:
            lost += k - i
            i = k
            continue
        k = got.find(sent[i:i + probe], j, j + window)
        if k > j:
            extra += k - j
            j = k
            continue
        corrupted += 1
        i += 1
        j += 1
    lost += len(sent) - i
    extra += len(got) - j
    return lost, extra, corrupted, events, where


def transfer(pairs, idle_timeout=3.0):
    """pairs: list of (source fd, destination fd, data). Sends all data while
    reading every destination; returns received bytes per pair and seconds."""
    sent = [0] * len(pairs)
    got = [b""] * len(pairs)
    start = last = time.time()
    while True:
        writers = [p[0] for n, p in enumerate(pairs) if sent[n] < len(p[2])]
        readers = [p[1] for p in pairs]
        readable, writable, _ = select.select(readers, writers, [], 0.1)
        for n, (src, dst, data) in enumerate(pairs):
            if src in writable and sent[n] < len(data):
                try:
                    sent[n] += os.write(src, data[sent[n]:sent[n] + 256])
                except BlockingIOError:
                    pass
            if dst in readable:
                chunk = os.read(dst, 4096)
                if chunk:
                    got[n] += chunk
                    last = time.time()
        done = all(sent[n] >= len(p[2]) for n, p in enumerate(pairs))
        if done and all(len(got[n]) >= len(p[2]) for n, p in enumerate(pairs)):
            break
        if time.time() - last > idle_timeout and (done or not writers):
            break
        if time.time() - last > 30:
            break
    return got, last - start


def report(label, data, got, seconds):
    lost, extra, corrupted, events, where = align(data, got)
    ok = (lost, extra, corrupted) == (0, 0, 0)
    rate = len(got) / seconds if seconds > 0 else 0
    print(f"  {'PASS' if ok else 'FAIL'}  {label:18s} {len(got):7d}/{len(data)} bytes"
          f" at {rate:6.0f} bytes/s; lost {lost}, extra {extra}, corrupted {corrupted}"
          f" ({events} events{' at ' + ' '.join(map(str, where)) if where else ''})", flush=True)
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bridge", default="/dev/ttyACM0")
    ap.add_argument("--adapter", default="/dev/ttyUSB0")
    ap.add_argument("--bytes", type=int, default=100000)
    ap.add_argument("--rx-only", action="store_true")
    ap.add_argument("--stats", action="store_true")
    ap.add_argument("bauds", type=int, nargs="+")
    args = ap.parse_args()
    results = []
    for baud in args.bauds:
        print(f"== {baud} bps", flush=True)
        bridge = open_port(args.bridge, baud)
        adapter = open_port(args.adapter, baud)
        time.sleep(0.5)  # the bridge reconfigures its UART
        drain(bridge)
        drain(adapter)
        if args.stats:
            stats(bridge, baud)  # clear the counters
        rnd = random.Random(baud)
        a = rnd.randbytes(args.bytes)
        b = rnd.randbytes(args.bytes)
        (got,), secs = transfer([(adapter, bridge, a)])
        results.append(report("adapter -> bridge", a, got, secs))
        drain(bridge)
        if args.stats:
            print("        " + stats(bridge, baud), flush=True)
        if args.rx_only:
            (got, _), secs = transfer([(adapter, bridge, a), (bridge, adapter, b)])
            results.append(report("... host flooding", a, got, secs))
            if args.stats:
                drain(bridge)
                print("        " + stats(bridge, baud), flush=True)
            os.close(bridge)
            os.close(adapter)
            continue
        (got,), secs = transfer([(bridge, adapter, b)])
        results.append(report("bridge -> adapter", b, got, secs))
        drain(adapter)
        if args.stats:
            print("        " + stats(bridge, baud), flush=True)
        (got_a, got_b), secs = transfer([(adapter, bridge, a), (bridge, adapter, b)])
        results.append(report("both: to bridge", a, got_a, secs))
        results.append(report("both: to adapter", b, got_b, secs))
        if args.stats:
            drain(bridge)
            print("        " + stats(bridge, baud), flush=True)
        os.close(bridge)
        os.close(adapter)
    print(f"{sum(results)} of {len(results)} passed")


if __name__ == "__main__":
    main()
