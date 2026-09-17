#!/usr/bin/env python3
"""Measure how well the bridge carries frame-shaped traffic, repeatedly.

usage: frame_test.py [--bridge PORT] [--adapter PORT] [--baud N] [--frame N]
                     [--gap MS] [--seconds N] [--runs N] [--reboot]
                     [--direction both|to-adapter|to-bridge] [--label TEXT]
                     [--out FILE]

Frames of a few hundred bytes with a gap between them, in both directions at
once, which is what PPP and most protocols look like and what a stream test
does not: the transmitter starts from idle for every frame, and its planner's
idea of when USB is busy goes stale in the gaps.

The point of this tool is repetition. The same firmware corrupts anywhere
between 0 and 40 bytes per 20 kB from one run to the next, so a single run
says nothing at all about a change: it takes a set of runs, and the spread
matters as much as the total. Each run is measured on its own and can start
from a fresh boot (--reboot), since the bridge behaves consistently within a
boot and differently across boots.

With STATS compiled into the sketch, the diagnostic counters are read and
cleared around every run, so each line carries what the bridge thought was
happening: e late edges, w time-sharing holds, z bytes given up, r receive
overflows, k stack bytes never touched, P the USB frame period in 1/16 timer
ticks (4125 when the clock is exactly 16.5 MHz).
"""
import argparse
import json
import os
import random
import re
import select
import statistics
import subprocess
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bridge_test import align

COUNTERS = ("k", "n", "g", "o", "f", "e", "a", "h", "p", "z", "r", "w", "P", "c", "b")


def open_port(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    a = termios.tcgetattr(fd)
    a[4] = a[5] = getattr(termios, f"B{baud}")
    a[2] &= ~termios.CRTSCTS
    a[2] |= termios.CLOCAL | termios.CREAD
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd


def drain(fd, quiet=0.3):
    while select.select([fd], [], [], quiet)[0]:
        os.read(fd, 4096)


def wait_for_port(path, timeout=30):
    end = time.time() + timeout
    while time.time() < end:
        if os.path.exists(path):
            time.sleep(1.5)  # let the host finish enumerating it
            return True
        time.sleep(0.3)
    return False


def reboot(path):
    """The sketch jumps to the bootloader at 134 bps; it starts the sketch
    again by itself a few seconds later, so this is a reset without a flash."""
    try:
        subprocess.run(["stty", "-F", path, "134"], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass
    time.sleep(9)
    return wait_for_port(path)


def counters(path, baud, stats_baud=110):
    """Read and clear the sketch's counters, then put the rate back. Returns
    {} if the bridge is not there, which happens when it has just reset."""
    if not wait_for_port(path, 10):
        return {}
    try:
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return {}
    try:
        tty.setraw(fd)
        a = termios.tcgetattr(fd)
        a[4] = a[5] = getattr(termios, f"B{stats_baud}")
        termios.tcsetattr(fd, termios.TCSANOW, a)
        out, end = b"", time.time() + 2
        while time.time() < end and b"\n" not in out:
            if select.select([fd], [], [], 0.1)[0]:
                out += os.read(fd, 4096)
        a[4] = a[5] = getattr(termios, f"B{baud}")
        termios.tcsetattr(fd, termios.TCSANOW, a)
        time.sleep(0.2)
    except (termios.error, OSError):
        return {}
    finally:
        os.close(fd)
    found = dict(re.findall(rb"([a-zA-Z])([0-9a-f]{4})", out))
    return {k: int(v, 16) for k, v in
            ((k.decode(), v) for k, v in found.items()) if k in COUNTERS}


def one_run(args, seed):
    """Returns the measurements, or {"gone": True} if the bridge left the bus
    -- which is worth knowing: it means the sketch reset under the load."""
    if not wait_for_port(args.bridge, 15):
        return {"gone": True}
    try:
        return measure(args, seed)
    except (termios.error, OSError) as e:
        return {"gone": True, "error": str(e)}


def measure(args, seed):
    bridge = open_port(args.bridge, args.baud)
    adapter = open_port(args.adapter, args.baud)
    time.sleep(0.3)
    drain(bridge, 0.2)
    drain(adapter, 0.2)

    rnd = random.Random(seed)
    sending = []
    if args.direction in ("both", "to-adapter"):
        sending.append(bridge)
    if args.direction in ("both", "to-bridge"):
        sending.append(adapter)
    sent = {fd: bytearray() for fd in (bridge, adapter)}
    got = {fd: bytearray() for fd in (bridge, adapter)}
    pending = {fd: b"" for fd in (bridge, adapter)}
    ready_at = {fd: 0.0 for fd in (bridge, adapter)}
    gap = args.gap / 1000.0

    # Stop offering frames before the end, so what is still on its way can
    # arrive and a truncated tail is not counted as loss.
    stop_offering = time.time() + args.seconds
    while True:
        now = time.time()
        if now < stop_offering:
            for fd in sending:
                if not pending[fd] and now >= ready_at[fd]:
                    frame = bytes(rnd.randrange(256) for _ in range(args.frame))
                    sent[fd] += frame
                    pending[fd] = frame
        elif not any(pending.values()) and all(
                len(got[s]) >= len(sent[s]) for s in sending):
            break
        elif now > stop_offering + 15:
            break
        writers = [fd for fd in (bridge, adapter) if pending[fd]]
        readable, writable, _ = select.select([bridge, adapter], writers, [], 0.05)
        for fd in writable:
            try:
                n = os.write(fd, pending[fd])
                pending[fd] = pending[fd][n:]
                if not pending[fd]:
                    ready_at[fd] = time.time() + gap
            except BlockingIOError:
                pass
        for fd in readable:
            chunk = os.read(fd, 4096)
            if chunk:
                got[adapter if fd is bridge else bridge] += chunk

    result = {}
    for name, src in (("transmit", bridge), ("receive", adapter)):
        if src not in sending:
            continue
        a, b = bytes(sent[src]), bytes(got[src])
        lost, extra, corrupted, events, where = align(a, b)
        result[name] = dict(bytes=len(a), arrived=len(b), lost=lost, extra=extra,
                            corrupted=corrupted, events=events,
                            offsets=[w % args.frame for w in where[:16]],
                            at=where[:64])  # absolute, to see how they are spaced
    os.close(bridge)
    os.close(adapter)
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bridge", default="/dev/ttyACM0")
    ap.add_argument("--adapter", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--frame", type=int, default=170)
    ap.add_argument("--gap", type=float, default=30, help="ms between frames")
    ap.add_argument("--seconds", type=float, default=45, help="per run")
    ap.add_argument("--runs", type=int, default=6)
    ap.add_argument("--reboot", action="store_true",
                    help="reset the board between runs (134 bps), since it "
                         "behaves differently from one boot to the next")
    ap.add_argument("--direction", default="both",
                    choices=["both", "to-adapter", "to-bridge"])
    ap.add_argument("--label", default="")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    print(f"{args.runs} runs of {args.seconds:g} s, {args.frame}-byte frames,"
          f" {args.gap:g} ms apart, {args.baud} bps, {args.direction}"
          f"{', rebooting between runs' if args.reboot else ''}"
          f"{'  [' + args.label + ']' if args.label else ''}", flush=True)
    runs = []
    for i in range(args.runs):
        if args.reboot and i:
            if not reboot(args.bridge):
                print("  the bridge did not come back", flush=True)
                break
        counters(args.bridge, args.baud)  # clear
        result = one_run(args, seed=1000 + i)
        result["counters"] = counters(args.bridge, args.baud)
        runs.append(result)
        if result.get("gone"):
            print(f"  run {i + 1}: THE BRIDGE LEFT THE BUS", flush=True)
            continue
        line = f"  run {i + 1}: "
        for name in ("transmit", "receive"):
            if name in result:
                r = result[name]
                line += (f"{name} {r['corrupted']:4d} corrupted, {r['lost']:4d}"
                         f" lost of {r['bytes']}; ")
        c = result["counters"]
        if c:
            line += " ".join(f"{k}={c[k]}" for k in ("e", "w", "z", "r", "k", "P")
                             if k in c)
        print(line, flush=True)

    print()
    for name in ("transmit", "receive"):
        vals = [r[name] for r in runs if name in r]
        gone = sum(1 for r in runs if r.get("gone"))
        if not vals:
            continue
        total = sum(v["bytes"] for v in vals)
        bad = [v["corrupted"] for v in vals]
        lost = sum(v["lost"] for v in vals)
        per100k = sum(bad) / total * 100000 if total else 0
        print(f"{name}: {sum(bad)} corrupted and {lost} lost in {total} bytes"
              f" over {len(vals)} runs"
              f"  ->  {per100k:.1f} corrupted per 100 kB"
              f"  (per run: min {min(bad)}, median {statistics.median(bad):.0f},"
              f" max {max(bad)})"
              + (f"; the bridge left the bus in {gone} run(s)" if gone else ""))
    if args.out:
        with open(args.out, "w") as f:
            json.dump(dict(args=vars(args), runs=runs), f, indent=1)
        print(f"written to {args.out}")


if __name__ == "__main__":
    main()
