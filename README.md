# DigisparkBridge

A minimalistic (8N1 only, no flow control) USB-to-UART bridge for the
original Digispark (ATtiny85) and limited to 9,600 bps full duplex
(both directions at once) by taking turns around USB traffic
(see notes below); receiving alone works up to 19,200 bps.
Which is not too shabby for a device that has no UART
at all: it receives with the USI peripheral, oversampling in hardware, and
transmits with bit edges timed by a timer's compare output,
while [DigiCDCFast](https://github.com/marcocarnut/DigiCDCFast) runs
bitbanged USB in software on the same 8-bit chip.

**Is it useful?** At 1200 to 9600 bps, yes: every test passes, in either
direction and both at once, with no lost or corrupted bytes. The price is
throughput when both directions are busy at 9600 bps: the bridge and the
host take turns on USB, so each way carries about 600 bytes/s instead of
860, and received bytes can wait up to 20 ms. That is
[time sharing](#time-sharing-both-directions-at-once), which can be turned
off for full speed at the cost of the occasional corrupted byte
(`TIME_SHARING` in the sketch). Above 9600 bps only receiving works.

For a serial port at any rate, a USB-serial chip costs less than a dollar,
and the [DigisparkProBridge](https://github.com/marcocarnut/DigisparkProBridge)
has a hardware UART and is lossless to 38400 bps in both directions.

It is also a demonstration of what a fast USB serial library like
[DigiCDCFast](https://github.com/marcocarnut/DigiCDCFast) makes
possible on these boards, and of the techniques involved; the notes below
explain them.

## Wiring

| Digispark | Other device |
|-----------|--------------|
| PB0 (P0) | TX |
| PB1 (P1) | RX |
| GND | GND |

The Digispark's I/O is at 5 V; use a level shifter for 3.3 V devices. The
LED on PB1 flickers with the data, which doesn't matter.

No hardware flow control for now (maybe in the future).

## Requirements

- **Linux host.** Windows refuses low-speed USB serial devices; see
  [DigiCDCFast's README](https://github.com/marcocarnut/DigiCDCFast#windows-does-not-work).
- **Digistump AVR core 1.7.5**, from the board manager URL
  `https://raw.githubusercontent.com/ArminJo/DigistumpArduino/master/package_digistump_index.json`.
  No changes are needed for the ATtiny85.
- **DigiCDCFast** 1.2.0 or later, from the Arduino Library Manager (1.1.0
  works too, except that unsupported bit rates are then accepted silently).
- A Digispark with the micronucleus bootloader. Build for its 16.5 MHz
  clock setting: the board definition defaults to 16 MHz, where USB doesn't
  work.

## Using it

Plug it in, set the bit rate on the port and use it like any serial port:

```sh
stty -F /dev/ttyACM0 9600 raw -echo
```

The bit rates it supports are **1200, 2400, 4800 and 9600**, and **19200
for receiving alone**. Any other rate is refused: the bridge answers the
host's request with a STALL, as the CDC specification asks, and keeps the
rate it had. Linux doesn't pass that on, so `stty` still reports success and
the host then believes a rate the bridge isn't using; a program can check by
reading the settings back from the device. The format is always **8N1**: the
data bits, parity and stop bits the host asks for are ignored, as are DTR
and RTS, and there are no CTS or break signals. Two bit rates are commands
instead:

- **134 bps** jumps to the micronucleus bootloader, for reflashing without
  replugging.
- **110 bps** prints diagnostic counters as hex and clears them, leaving the
  UART as it was (see [Diagnostics](#diagnostics)).

## Building and flashing

With the Arduino IDE: open `TinyBridge/TinyBridge.ino`, select the board
*Digispark* and *Tools → Clock → 16.5 MHz - For V-USB*, and upload; plug the
board in when asked.

With arduino-cli:

```sh
arduino-cli compile --fqbn digistump:avr:digispark-tiny:clock=clock165 --output-dir build TinyBridge
stty -F /dev/ttyACM0 134        # only if a bridge is already running
~/.arduino15/packages/digistump/tools/micronucleus/2.6/micronucleus --run build/TinyBridge.ino.hex
```

### Build options

Two settings at the top of `TinyBridge.ino`:

- `TIME_SHARING` (1): hold data going to the host while transmitting, for
  reliable full duplex at 9600 bps (see
  [Time sharing](#time-sharing-both-directions-at-once)). 0 is faster and
  corrupts a byte now and then, and saves 350 bytes of flash.
- `STATS` (1): the diagnostic counters and the 110 bps command that prints
  them (see [Diagnostics](#diagnostics)). 0 saves 722 bytes of flash and 25
  bytes of RAM.

With both on the sketch uses 6542 of the 6650 bytes available; with both off,
5514.

## How it works

### Receiving: hardware oversampling with USI

V-USB handles every USB transaction with interrupts off, for up to ~200 µs,
so an interrupt that catches a start bit can be most of a bit late at
9600 bps. Instead, Timer0 clocks the USI shift register at three samples per
bit, so every sample is taken on time in hardware whatever the interrupts
are doing. USI's 4-bit counter overflows every 8 samples; the handler copies
the 8 samples and rewrites the counter so the next overflow comes exactly 8
samples after the last one, however late the handler ran. Its deadline is
8 samples, 2 2/3 bits (278 µs at 9600 bps, 139 µs at 19200). `loop()` finds
start bits in the sample stream and reads each bit near its middle.

One subtlety: rewriting the counter must not race with a sample shifting in.
Samples shift at Timer0's compare match, but USI's counter changes slightly
later than TCNT0 does, so the handler waits out the timer ticks on both
sides of the match.

### Transmitting: hardware-timed edges, and bytes planned around USB

Timer1 keeps running freely for `millis()`. Each bit edge is a compare match
on OC1A (set or clear), so the hardware makes it on time; the handler only
schedules the next edge, and has one bit to do so (104 µs at 9600 bps).

USB transactions are longer than that. While data flows, the host makes one
or two every millisecond, each starting at the same point of the frame. So
each byte is planned: the handler's late runs show when the transactions
start, and a byte starts a little later, or after an idle bit, when one would
begin near an edge whose next edge changes the level. Bytes go out between
the host's transactions, which is why a busy full-duplex link carries a bit
less than the line rate. The handler keeps running on idle bits for a second
after the last byte, so the next burst starts with current timing.

That covers the host's own traffic. The data the bridge sends to the host is
traffic too, and it is up to the bridge when to send it: see
[Time sharing](#time-sharing-both-directions-at-once).

### Shorter USB packets: DigiCDCMedium.h

The sketch includes `DigiCDCMedium.h` instead of `DigiCDCFast.h`: 2-byte USB
packets instead of 8-byte ones. Each transaction then keeps interrupts off
for ~73 µs instead of ~110 µs, less than a bit at 9600 bps, which is what
makes the planning work reliably. It limits USB to about 2000 bytes/s each
way, plenty for 9600 bps.

### Time sharing: both directions at once

At 9600 bps a transmitted bit lasts 104 us and a USB transaction keeps
V-USB's interrupts off for ~73 us, so a transaction that starts at the
wrong moment delays an edge enough to corrupt a byte. Planning bytes around
the host's transactions (above) handles the host's own traffic, but the
data the bridge sends to the host is extra traffic at times of the bridge's
choosing. Transmitting alone is clean; adding traffic the other way is what
corrupts bytes.

So, with `TIME_SHARING` (on by default), no data goes to the host while a
byte is being transmitted. Received bytes collect in a 32-byte buffer until
20 of them wait or the oldest is 20 ms old; then the transmitter finishes
its byte and holds the line idle while they go to the host, and resumes
when the host has taken them all. A hold ends after 100 ms regardless, in
case no program is reading the port. Below 9600 bps nothing is held.

The cost is throughput and a little latency: at 9600 bps with both
directions saturated, each carries about 600 bytes/s instead of 860, and a
received byte can wait up to 20 ms. One direction at a time is unaffected.
To trade that back for speed, set `TIME_SHARING` to 0 at the top of
`TinyBridge.ino` (330 bytes less flash), and the bridge behaves as it did
before: a few corrupted bytes per 10 kB transmitted while receiving, on some
hosts (see below).

## Results

Time sharing is on unless the table says otherwise. Against a CH340
USB-serial adapter wired to the bridge, on a PC (Intel
xHCI, Linux 6.8) with the board in a port of its own, the host sending as
fast as the bridge accepts (`bridge_test.py`). Bytes corrupted; none lost
unless noted; throughput per direction:

| Bit rate | Receive | Transmit | Both at once, time sharing on (default) | Both at once, time sharing off |
|----------|---------|----------|------------------------------------------|--------------------------------|
| 1200 (3 kB) | 0, 120 B/s | 0, 120 B/s | 0 / 0, 120 B/s | 0 / 0, 120 B/s |
| 2400 (3 kB) | 0, 240 B/s | 0, 240 B/s | 0 / 0, 240 B/s | 0 / 0, 240 B/s |
| 4800 (10 kB) | 0, 481 B/s | 0, 479 B/s | 0 / 0, 479 B/s | 0 / 0, 480 B/s |
| 9600 (20 kB) | 0, 961 B/s | 0, 859 B/s | 0 / 0, 601 B/s | receive 0; transmit 2, 858 B/s |
| 9600 (50 kB, 3 runs) | | | 0 / 0, 600 B/s | receive 0; transmit 8, 10, 14; 859 B/s |
| 19200 (10 kB) | 0, 1921 B/s | 15, 1293 B/s | receive 846 lost, 1271 corrupted; transmit 35 | receive 465 lost, 489 corrupted; transmit 221 |

Below 9600 bps time sharing does nothing: the bits are long enough that USB
traffic can't spoil them, and both directions run at the line rate. At
19200 bps transmitting is beyond reach either way, and in full duplex the
receiver loses sample windows: it has 139 us to service them, less than the
~200 us V-USB can keep interrupts off when it handles a transaction in each
direction back to back.

Other tests at 9600 bps, with time sharing on except where noted:

- **Bursty traffic** (`burst_test.py`, bursts of up to 100 bytes with
  50-300 ms pauses, both directions at once): clean, 40 kB each way
  (2 corrupted bytes in 48 kB with time sharing off).
- **Zmodem** (lrzsz), a 62 kB file each way, time sharing off: both copies
  identical; 784-836 bytes/s through the bridge's transmitter, 911-930
  bytes/s through its receiver.
- **PPP** (`novj`, MTU 296) with iperf3 in both directions, time sharing
  off: works, with about 1.6% of the frames from the bridge failing their
  checksum and being retransmitted by TCP.

The planner is unchanged since the first release, but a bug in how the
transmit handler returned is fixed (see
[Lessons](#lessons-from-the-hardware)), and with time sharing off the sketch
is smaller (flash 6638 -> 6092 bytes, RAM 388 -> 281, with DigiCDCFast
1.1.0; 6442 and 302 with time sharing). Side by side with the first release
on the same bench, three boots each, time sharing off: bit rate changes
requested right after traffic were missed 5 times in 27 by the first
release and never since; receiving and 4800 bps were clean for both;
transmitting at 9600 bps in both directions corrupted 12 bytes in 120 kB
with the first release and 18 since, and bursts 3 and 7 in 72 kB, which is
within chance.

### Hubs and hosts

With time sharing off, how often transmitted bytes get corrupted depends on
where the host's USB controller places the bridge's transactions in each
frame, and that changes with the host and with a hub in between. The same
sketch and adapter at 9600 bps, both directions at once, 50 kB each, three
runs per setup (`e` is the bridge's count of edges made late, see
[Diagnostics](#diagnostics)):

| Host | Connection | Corrupted bytes to the adapter | Edges made late |
|------|------------|--------------------------------|-----------------|
| PC (Intel Meteor Lake xHCI, Linux 6.8) | directly | 8, 10, 14 | ~37,400 |
| PC (Intel Meteor Lake xHCI, Linux 6.8) | through a USB 3 hub (Genesys Logic, USB ID 05e3:0610) | 1, 0, 2 and 0, 0, 0 | ~7,000 |
| Raspberry Pi 5 (RP1, Linux 6.12) | directly | 5, 2, 4 | ~38,800 |
| Raspberry Pi 5 (RP1, Linux 6.12) | through the same hub | 7, 9, 9 | ~37,300 |

Behind a high-speed hub, the host controller schedules a low-speed device's
transactions through the hub's transaction translator. On the PC that left
the transmitter's handlers five times fewer late edges and made full duplex
practically clean (and slightly faster, 873 bytes/s); on the Raspberry Pi
the same hub changed nothing. It isn't the Digispark's clock: its frame
length (`P`) was the same within 0.1% in the best and worst of these
sessions. So if you turn time sharing off and full duplex matters, try the
bridge with and without a hub on your host. With time sharing on, both
setups were clean.

Things tried along the way:

- **8-byte USB packets** (plain `DigiCDCFast.h`): transmit 3-7 corrupted per
  10-20 kB, both directions 12-19 per 10 kB. So it almost works at 9600 bps
  with normal packets; without planning, ~35% of transmitted bytes were
  corrupted.
- **3-byte packets**: as good as 2-byte ones at 9600 bps, and better at
  transmitting at 19200 bps (17-22 corrupted per 10 kB), which still isn't
  usable.
- **Exact USB timing instead of the handler's late runs.** A variant of
  DigiCDCFast recorded when V-USB's interrupt ended (an experiment, not
  published). It showed each frame's
  transactions at fixed places (in full duplex, an OUT transaction ending at
  +0 and +13 ticks, an IN one at +39 and +43), so the frame's timing can be
  known exactly. Planners built on it did no better than the estimate from
  late runs: seeding the estimate at the start of each burst corrupted 1-6
  bytes per 48 kB of bursts where the estimate alone corrupted none; planning
  around one region per frame transmitted cleanly one way but corrupted 7-11
  per 20 kB in both directions, against ~3; and learning a map of where
  handlers ran late made the handler too slow and too deep for the ATtiny85.
  Late runs measure what actually matters, whatever causes it, at almost no
  cost per edge.

## The sketches

- `TinyBridge/`: the bridge described above.
- `TinyBridgeUsi3x/`: the receiver on its own (receive only, PB0).
- `TinyBridgeUsi1x/`: the classic approach, for comparison: INT0 catches
  the start bit and USI takes one sample per bit. It needs the UART's TX on
  both PB0 and PB2. It loses 0.56% of bytes at 9600 bps with USB idle, and
  3.7-12.8% at 19200 bps, as V-USB delays the start-bit interrupt.

## Diagnostics

Setting the port to 110 bps makes the sketch print a line of hex counters
and clear them:

- `n` bytes received, `g` sample windows lost, `o` capture queue overflows,
  `f` framing errors;
- TinyBridge also: `k` stack bytes never used, `e` edges made late, `a` USB
  transactions seen, `h` bytes started later, `p` idle bits spent looking,
  `z` bytes sent after 10 idle bits without a safe place, `r` received bytes
  dropped because loop() hadn't forwarded them, `w` times transmitting was
  held for data going to the host, `P` the USB frame length it measures, in
  1/16 Timer1 ticks (4125 with an exact 16.5 MHz clock; 0.1% is about 4),
  `c` and `b` the CRC-16 (XMODEM) and count of the bytes read from USB, to
  compare with what the host sent;
- TinyBridgeUsi3x also: `l` glitches, `r` receive buffer overflows.

`bridge_test.py --stats` prints them after each transfer, with the host's
own `c` and `b` for the data it sent to the bridge.

## Tests

Both scripts use a reference USB-serial adapter wired to the bridge (TX to
RX, RX to TX, GND to GND), and default to `/dev/ttyACM0` for the bridge and
`/dev/ttyUSB0` for the adapter. They report bytes lost, extra or corrupted,
with the offsets of the first mismatches.

- `bridge_test.py [--bytes N] [--rx-only] [--duplex-only] [--stats]
  [--stats-wait SECONDS] BAUD...` sends pseudo-random data one direction at
  a time, then both at once. `--rx-only` is for the receive-only sketches,
  `--duplex-only` runs only the test in both directions, and `--stats-wait`
  sets the quiet time before asking for the counters.
- `burst_test.py [--bytes N] [--max-burst MAX] [--pause MIN MAX]
  [--direction both|to-bridge|to-adapter] BAUD` sends random bursts with
  random pauses.

## Lessons from the hardware

- Forcing a compare match with FOC1A right after changing COM1A made no
  edge: the output kept its level. A late edge is made as a compare match 2
  ticks ahead instead.
- Extending Timer1's count with the core's `millis_timer_overflow_count`
  was occasionally off by 256 ticks, as its non-blocking handler counts late;
  the transmitter extends TCNT1 itself.
- A byte whose level changes at many edges may fit nowhere between the USB
  transactions. The planner gives up after 10 idle bits: otherwise the
  bridge stops sending, its buffer fills, and DigiCDCFast's flow control then
  refuses even the host's line-coding requests.
- A plain `ISR()` for the transmit handler saved 27 registers with
  interrupts off (~5 µs), long enough to worry V-USB. An entry stub now
  masks the handler's interrupt and turns interrupts on before saving
  anything.
- **The way out matters as much as the way in.** The handler and its stub
  restored their registers with interrupts off: over 100 CPU cycles after
  every edge. V-USB must start within about 40 cycles of a packet, and at
  16.5 MHz it doesn't check CRCs, so a packet arriving then could be misread
  and accepted. Because the planner keeps the handler at a fixed place in
  the USB frame, some sessions lost whole 2-byte packets from the host (seen
  as pairs of corrupted bytes, confirmed by the `c` counter disagreeing with
  the host), and requests to change the bit rate were lost about half the
  time right after traffic. Restoring registers with interrupts on, and
  unmasking the handler's interrupt in one short step at the end, fixed both.
- A USB-UART bridge makes such bugs hard to see: corrupted data looks like a
  transmit timing error. Checksumming what the sketch received over USB told
  the two apart.

## License and credits

GPL version 2 or version 3, at your choice; see [LICENSE](LICENSE).

- [DigiCDCFast](https://github.com/marcocarnut/DigiCDCFast), and through it
  [V-USB](https://www.obdev.at/vusb/) by OBJECTIVE DEVELOPMENT Software GmbH.
- The Digistump AVR core, by Digistump LLC, as maintained by ArminJo.
- DigisparkBridge by Marco Carnut.
