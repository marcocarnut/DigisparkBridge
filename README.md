# DigisparkBridge

A minimalistic (8N1 only, RTS flow control) USB-to-UART bridge for the
original Digispark (ATtiny85) and limited to 9,600 bps "quasi-full duplex"
(both directions at once, almost) by taking turns around USB traffic
(see notes below); receiving alone works up to 19,200 bps.
Which is not too shabby for a device that has no UART
at all: it receives with the USI peripheral, oversampling in hardware, and
transmits with bit edges timed by a timer's compare output,
while [DigiCDCFast](https://github.com/marcocarnut/DigiCDCFast) runs
bitbanged USB in software on the same 8-bit chip.

**Is it useful?** At 1200 to 4800 bps, yes: every test passes, in either
direction and both at once, with no lost or corrupted bytes. At 9600 bps it
works almost perfectly -- either direction alone is clean, and only with both
near saturation does it corrupt a few bytes per 100 kB on the way out. The
other price is throughput: the bridge and the host take turns on USB, so each
direction carries about 600 bytes/s instead of 860 transmitting and 960
receiving, and received bytes can wait up to 20 ms. That is
[time sharing](#time-sharing-both-directions-at-once), which can be switched
off for full speed at the cost of more corruption (`TIME_SHARING` in the
sketch). That's why "quasi full duplex". It's pretty usable, but not perfect.
Above 9600 bps only receiving works.

For a 9600 serial GPS, for instance, those imperfections are inconsequential;
so, yes, it is useful -- if anything, to give those aging Digisparks a better
meaning to their lives than sitting unused in your drawer.

For a true serial port at any rate, a USB-serial chip costs less than a dollar,
and the [DigisparkProBridge](https://github.com/marcocarnut/DigisparkProBridge)
has a hardware UART and is lossless to 57600 bps in both directions.

It is also a demonstration of what a fast USB serial library like
[DigiCDCFast](https://github.com/marcocarnut/DigiCDCFast) makes
possible on these boards, and of the techniques involved; the notes below
explain them.

## Wiring

| Digispark | Other device |
|-----------|--------------|
| PB0 (P0) | TX |
| PB1 (P1) | RX |
| PB2 (P2) | CTS (optional, see [Flow control](#flow-control)) |
| PB5 (P5) | RTS (optional; read the warning under [Flow control](#flow-control) first) |
| GND | GND |

The Digispark's I/O is at 5 V; use a level shifter for 3.3 V devices. The
LED on PB1 flickers with the data, which doesn't matter.

PB2 asks the other device to pause when the bridge's buffer fills; leave it
unconnected if the device has no CTS input. There is no CTS input on the
bridge's side: it has no pin left for one.

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

- **134 bps, asked for three times within two seconds**, jumps to the
  micronucleus bootloader, for reflashing without
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

Settings at the top of `TinyBridge.ino`, and two in DigiCDCFast:

- `TIME_SHARING` (1): hold data going to the host while transmitting, for
  reliable full duplex at 9600 bps (see
  [Time sharing](#time-sharing-both-directions-at-once)). 0 is faster and
  corrupts a byte now and then, and saves 284 bytes of flash.
- `STATS` (1): the diagnostic counters and the 110 bps command that prints
  them (see [Diagnostics](#diagnostics)). They cost 510 bytes of flash and 28
  of RAM, which this sketch can ill afford -- but turning them off is not
  free either. The transmitter's timing was tuned with them compiled in, and
  without them three runs of 50 kB in both directions corrupted two bytes,
  where the same code with them corrupted none. They stay on until the
  planner is retuned without them. With `WIDE_STATS` (0) the byte counter is
  32-bit, printed as two words, the low one first (`n` `N`): it wraps at
  65535 otherwise, which a long run will do.
- `CRC_STATS` (0): `c` and `b`, the bridge's own CRC and count of the bytes
  read from USB, to compare with what the host sent, at 94 bytes of flash and
  4 of RAM. They answered one question -- whether USB itself was corrupting
  anything, which it was not -- so they are off.
- `RTS_OUTPUT` (0): PB2 as an RTS output (see
  [Flow control](#flow-control)), 20 bytes of flash. Harmless if you switch
  it on without wiring it: the bridge drives a pin nobody reads.
- `CTS_INPUT` (0): PB5 as a CTS input (see
  [Flow control](#flow-control)), 10 bytes of flash. **Switch this on only
  with the wire, and only on a board whose reset pin has been given up**: see
  the warnings under [Flow control](#flow-control). With it on and nothing
  driving PB5, the pull-up reads "wait" and the bridge never sends a byte --
  it still receives, so the link looks half dead rather than broken.

- `CALL_HOOK` (1): the largest single thing that reduces corruption here (see
  [When the driver makes the edges](#when-the-driver-makes-the-edges)), and it
  needs **no changes to DigiCDCFast at all** -- it fills in
  `usbTransactionEnd()`, the weak hook the library has called since 1.3.0. It
  does need `USB_PACKET_SIZE` 2, the sketch's default: the build fails
  otherwise, because the hook assumes no transaction outlasts a bit time. 138
  bytes of flash.

With the defaults the sketch uses 6544 of the 6650 bytes available and 309 of
the 512 bytes of RAM; with both flow control lines, 6574; without the hook,
6406; without the counters, 6034 and 281 -- and see what that costs, above.

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
`TinyBridge.ino` (284 bytes less flash), and the bridge behaves as it did
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

Later, against an FT232R, three rounds of 50 kB in both directions at once
with the shipping defaults: nothing lost, nothing corrupted. The same rounds
with `STATS` at 0 corrupted two transmitted bytes, which is how we learned
that the counters are part of what the transmitter's timing was tuned
around.

### Frames, which are harder than streams

Protocols do not send streams; they send frames with gaps, and the bridge
carries those less well. `frame_test.py` measures it: 170-byte frames 30 ms
apart, both directions at once. Over 80 runs of 20 s with a fresh boot before
each -- 748 kB transmitted, 1.6 MB received -- **2.5 bytes per 100 kB were
corrupted on the way out, and not one of the 1.6 MB coming in**. Most runs are
perfectly clean; the bad ones come in episodes lasting minutes, during which a
run can corrupt hundreds of bytes, and no counter the bridge keeps says
anything is wrong. That is what a PPP link over this bridge runs into: TCP
retransmits the frames, so it works, but it is not free.

**With the driver making the edges** (see
[When the driver makes the edges](#when-the-driver-makes-the-edges)) the same
test gives 2.6 per 100 kB -- but the distribution is what changed. Over 20
runs with a reboot before each, fourteen were perfectly clean and the other
six corrupted **exactly one byte**, where before a single run could corrupt
eleven. Receiving stayed perfect: 0 in 492 kB.

Over a real PPP link carrying two 400 kB file transfers at once, with both
hooks on: **1.12 MB received without a single error**, and 41 damaged frames
in 7281 sent, which is 0.56% of frames or about 4.3 corrupted bytes per
100 kB. Before the hooks the same test damaged about 34 frames per 100 kB.

Anyone measuring this should know that one run tells you nothing. The same
firmware gave 6.0 and 446.7 corrupted per 100 kB in two consecutive sets of
20 runs. Comparing two configurations means interleaving them run by run in
one binary (`AB_TEST` in the sketch does this for the hold thresholds): done
that way, holding at 8 bytes / 8 ms measured 4.3 per 100 kB against 0.8 for
the 20/20 the sketch ships with, over 40 runs each -- the opposite of what the
same comparison said when the two were measured one after the other. Part of
the reason is that the board boots into one of two regimes and stays there;
see [Two regimes](#two-regimes).
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
- **PPP** (`pppd` at both ends, MTU/MRU 296, `novj`, `nocrtscts`), a 62 kB
  file over TCP in each direction at once: **no frame errors either way**,
  85 kB carried each way. The transfer into the host finishes well before
  the one out of it, since time sharing gives receiving priority. (Earlier
  runs showed 6-14 bad frames per 95 kB from the bridge, all of them bytes
  the planner gave up on; see
  [When a byte fits nowhere](#when-a-byte-fits-nowhere).)

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

### Flow control

The bridge has 32 bytes to hold what it receives until USB takes it. If the
host stops reading its port, that fills, and the bytes that arrive next are
lost (counter `r`: 20 kB sent to a port nobody read lost 15956 of them).

PB2 prevents that, with `RTS_OUTPUT` set to 1 and the other device's CTS
wired to it: the bridge holds it low while it can take data, and raises it once 22 bytes are waiting, until
the buffer is down to 8 again. Wired to an FT232R's CTS, with `crtscts` set
on that side, the same test lost nothing: the adapter paused, accepting
8704 bytes in 150 s while the host read nothing, and the bridge's `r`
stayed 0.

Not every adapter obeys CTS. A CH340 kept sending regardless (and dropped
16793 bytes), because Linux's `ch341` driver accepts `crtscts` without
implementing it; the bridge's line was correct all the while, as the
adapter's own CTS pin showed. PB2 is left alone until `RTS_OUTPUT` is set to
1 in the sketch (20 bytes of flash), so that a board with nothing wired to it
behaves as it always did.

The other direction, a device asking the *bridge* to pause, is `CTS_INPUT` on
PB5 -- and it needs a board whose reset pin has been given up. PB0, PB1 and
PB2 are the UART and RTS, PB3 and PB4 are USB, and PB5 is the reset pin
unless the `RSTDISBL` fuse is programmed (AVR fuses read 0 when programmed,
so RSTDISBL = 0 is what you want).

> **Check the fuse before wiring anything to P5.** On a board where RSTDISBL
> is not programmed, P5 is still the reset pin, and reset is active low --
> which is the same level the other device's RTS uses to say "go ahead". Wire
> them together and the board sits in reset exactly when it is being told it
> may send. It will look dead, not slow. Some Digisparks are said to ship
> with the fuse programmed and many clones, "rev3" boards among them, not, so
> read it rather than assume it.

Programming it costs the reset pin for good: micronucleus becomes the only
way in, and a high-voltage programmer (an ISP one cannot do it) the only way
back if the bootloader is ever damaged. For a bridge that transmits at most
860 bytes/s into devices that usually have a UART buffer, it is rarely worth
it.

With it wired and switched on, the bridge finishes the byte it is sending and
stops while PB5 is high; `loop()` starts the transmitter again when it goes
low. Told to wait by an FT232R's RTS driven by hand, the bridge sent 0 bytes,
and then all 200 of them in order and intact once let go (`cts_test.py`).
With `CTS_INPUT` on and nothing driving PB5, the pin's pull-up reads "wait"
and the bridge never transmits, while still receiving normally: switch it on
only together with the wire.

**It leaves the sketch little room to spare, though.** The ATtiny85 build
runs close to the edge of its RAM: the `k` counter, which reports the stack
bytes never touched, ranges from about 20 to 140 between runs with everything
else equal -- one reading says nothing -- and with `CTS_INPUT` on it has been
seen at 0, the stack reaching the first byte past the globals without yet
passing it. No test lost or corrupted anything at 9600 bps because of it.
The flags the sketch used to keep in bytes now live in the bits of GPIOR0,
which gave back 6 bytes of RAM and 62 of flash; `STATS` at 0 gives back
another 25 and 726, at the price of the counters that would tell you.

Nothing needs to be told to the host, and CDC has no way to tell it: the
`SERIAL_STATE` notification carries carrier, ring, break, framing, parity and
overrun, and no CTS bit. It is not needed either: when the bridge can't take
more, USB makes the host wait.

### When a byte fits nowhere

A byte whose level changes at many edges sometimes can't be placed safely
anywhere in the frame. The planner then sends idle bits and tries again, and
after `IDLE_LIMIT` of them (40, about 4 ms at 9600 bps) it gives up and
sends the byte where it is, corrupted now and then. The limit has to exist:
without it the bridge would stop sending, its buffers would fill, and
DigiCDCFast's flow control would then refuse even the host's line-coding
requests.

The limit was 10 idle bits until it turned out to be the main source of
errors under PPP: every corrupted byte in a long test was one of these
(counter `z`). At 40 the counter stays at 0 in the same tests, and
throughput is unchanged (601 bytes/s each way in both directions at once).
Raising it further only delays a stubborn byte more.

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

## When the driver makes the edges

A bit edge is a compare match on OC1A, so the hardware places it on time
whatever the processor is doing. What the handler must do is load the *next*
one, and it has one bit time to do it -- 104 µs at 9600 bps. V-USB keeps
interrupts off for a whole transaction, and for a whole run of back-to-back
transactions: up to about 200 µs. Planning each byte around the transactions
(below) helps, but nothing the sketch does can help an edge that came due
*during* a blackout, because the sketch is not running. Only the driver is.

So the driver makes those edges. DigiCDCFast calls `usbTransactionEnd()` at
the end of every transaction -- a weak, do-nothing function since 1.3.0 -- and
the sketch fills it in with about sixty instructions of assembler
(`CALL_HOOK`). Nothing in the library changes. At the end of every transaction
it shifts the sketch's frame along, puts the
next bit on the compare output and moves the compare one bit further --
repeatedly, for as long as the transactions keep coming, so a blackout of any
length up to the end of a byte is covered. It stops before the edge that ends
a byte, which wants planning, and counts what it did so the handler can catch
up.

The handler does not recompute where the driver got to: it reads `OCR1A`, the
driver's own latest edge, and adjusts the high byte for the single wrap that
is possible. Only one of them ever does the arithmetic, so the two clocks
cannot disagree. The fraction of a bit time is shared for the same reason,
in 256ths of a tick -- a tick is 64 CPU cycles, so the conversion is exact,
and the driver carries it with a single `adc`. An earlier version added whole
ticks only and drifted about a tick every four bits, after which the
handler's next edge read as displaced by all of it at once.

The same hook does the receiver the same favour first, because its deadline is
tighter: it takes USI's window of 8 samples into a stash and re-arms `USISR`.
Re-arming clears `USIOIF`, so the sketch's own overflow interrupt does not run
for that window at all -- which is most of what it is worth, since that
interrupt is also what delays the transmitter.

The rules for that function are DigiCDCFast's, and they are strict: naked
assembler, and only the registers the driver saved. See *Borrowing the
interrupt* in [DigiCDCFast's README](https://github.com/marcocarnut/DigiCDCFast).
An earlier version of this work inlined the same instructions into the driver
instead, on the assumption that the seven cycles of `rcall` and `ret` were
unaffordable. They are not: the inlined form has to sit at the end of the
driver's assembler file and jump there and back, which costs eight. Measured
against each other the two are indistinguishable -- forced edges 12014 against
12061 per run -- so the call wins on the only ground that separates them,
which is that it leaves the library alone.

Measured per 8 kB in both directions at once, edges the handler had to force:

| | transmitting only | both directions |
|---|---|---|
| no hook | ~4950 | -- |
| edge hook, one stashed edge | ~1340 | ~5000 |
| the rest of the byte, fraction carried | 553 | 4257 |

## Two regimes

The same firmware sometimes behaves quite differently from one run to the
next. In a good run the handler forces about 4200 edges per 8 kB of duplex
traffic and loses one sample window; in a bad one, about 10000 and some 400
windows. Nothing in the sketch chooses this, and it is not the build: the
same hex, reflashed, gave one and then the other, and two hex files that
measured twelvefold apart on this turned out to be byte-identical.

It often persists for a whole session, which is what first suggested that the
host's frame schedule sets it -- whether this device's two transactions land
back to back, one blackout of 200 µs rather than two of 110. But it has also
flipped between consecutive runs with no re-enumeration at all, which that
explanation does not allow. The other candidate is the sketch's own planner
latching a good or a bad estimate when a run starts: `probeBits` and the
activity estimate are reset in `txStart()`, and the planner learns from there.
Unresolved, and worth resolving.

It is worth knowing about because it makes measurements lie. Two consecutive
sets of 20 runs of the same firmware measured 6.0 and 446.7 corrupted bytes
per 100 kB. **Compare configurations interleaved in one binary, switched at
run time, never in blocks** -- `frame_test.py --modes` does this -- and read
`g` and `e` to see which regime a run was in.

With the edges made from the transaction hook the difference mostly goes away:
across 20 runs each preceded by a reboot, forced edges stayed within 1.3% of
each other and no run corrupted more than a single byte. It has not gone
entirely -- the twelvefold pair above was measured after that -- so read `g`
and `e` before trusting any run.

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
  `z` bytes sent without a safe place (see
  [When a byte fits nowhere](#when-a-byte-fits-nowhere)), `r` received bytes
  dropped for want of room, `w` times transmitting was held for data going
  to the host, `H` times the handler found edges the driver had made for it,
  `P` the USB frame length it measures, in 1/16 Timer1 ticks
  (4125 with an exact 16.5 MHz clock; 0.1% is about 4), and with `CRC_STATS`
  on, `c` and `b`, the CRC-16 (XMODEM) and count of the bytes read from USB,
  to compare with what the host sent;
- TinyBridgeUsi3x also: `l` glitches, `r` receive buffer overflows.

Each counter is tagged by the letter at its own position in `COUNTER_TAGS`,
so adding one means adding a letter there too; a `static_assert` fails the
build if the two ever drift apart. `g` and `e` are worth reading first: they
say whether a run went well or badly, which varies from one power-up to the
next (see [Two regimes](#two-regimes)).

All of them are 16-bit and wrap silently, so `n` reads 34464 after 100000
bytes, unless `WIDE_STATS` is set (see [Build options](#build-options)). `r`
rises when the host stops reading the port and the other device keeps
sending without watching the bridge's RTS (see
[Flow control](#flow-control)). Ask for the counters
with the port drained, or the line comes back truncated: the host's buffer
is then full, and DigiCDCFast drops what it can't deliver after 50 ms.

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
- `cts_test.py [--baud N] [--bytes N]` drives the adapter's RTS by hand,
  rather than leaving it to the kernel's flow control, and checks that the
  bridge stops while told to wait and loses nothing when let go.
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
