#!/usr/bin/env python3
"""Test a USB-UART bridge with bursty traffic: random bursts of 1 to MAX
bytes separated by random pauses, like protocols and interactive use rather
than a stream. Wiring as for bridge_test.py.

usage: burst_test.py [--bridge PORT] [--adapter PORT] [--bytes N]
                     [--max-burst MAX] [--pause MIN MAX] [--seed S]
                     [--direction both|to-bridge|to-adapter] BAUD

Defaults: 12000 bytes each way, bursts up to 100 bytes, pauses of 50-300 ms,
so at 9600 bps the line goes idle between bursts.
"""
import argparse
import os
import random
import select
import time

import bridge_test as bt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bridge", default="/dev/ttyACM0")
    ap.add_argument("--adapter", default="/dev/ttyUSB0")
    ap.add_argument("--bytes", type=int, default=12000)
    ap.add_argument("--max-burst", type=int, default=100)
    ap.add_argument("--pause", type=float, nargs=2, default=[0.05, 0.3])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--direction", choices=["both", "to-bridge", "to-adapter"], default="both")
    ap.add_argument("baud", type=int)
    args = ap.parse_args()

    bridge = bt.open_port(args.bridge, args.baud)
    adapter = bt.open_port(args.adapter, args.baud)
    time.sleep(0.5)  # the bridge reconfigures its UART
    bt.drain(bridge)
    bt.drain(adapter)
    rnd = random.Random(args.seed)
    flows = []
    if args.direction in ("both", "to-bridge"):
        flows.append(["to bridge", adapter, bridge])
    if args.direction in ("both", "to-adapter"):
        flows.append(["to adapter", bridge, adapter])
    flows = [dict(label=l, src=s, dst=d, data=rnd.randbytes(args.bytes), pos=0,
                  pending=b"", got=b"", due=time.time()) for l, s, d in flows]

    start = last = time.time()
    while True:
        now = time.time()
        for f in flows:
            if not f["pending"] and f["pos"] < args.bytes and now >= f["due"]:
                n = min(rnd.randint(1, args.max_burst), args.bytes - f["pos"])
                f["pending"] = f["data"][f["pos"]:f["pos"] + n]
                f["pos"] += n
            if f["pending"]:
                try:
                    f["pending"] = f["pending"][os.write(f["src"], f["pending"]):]
                    if not f["pending"]:
                        f["due"] = time.time() + rnd.uniform(*args.pause)
                except BlockingIOError:
                    pass
        readable = select.select([f["dst"] for f in flows], [], [], 0.002)[0]
        for f in flows:
            if f["dst"] in readable:
                chunk = os.read(f["dst"], 4096)
                if chunk:
                    f["got"] += chunk
                    last = time.time()
        sent = all(f["pos"] >= args.bytes and not f["pending"] for f in flows)
        if sent and (all(len(f["got"]) >= args.bytes for f in flows) or time.time() - last > 3):
            break
    ok = [bt.report(f["label"], f["data"], f["got"], last - start) for f in flows]
    print(f"{sum(ok)} of {len(ok)} passed")


if __name__ == "__main__":
    main()
