# DigisparkBridge

An experimental USB-to-UART bridge for the original Digispark (ATtiny85)
but limited to 9,600 bps full duplex (barely, see notes below); receiving
alone works up to 19,200 bps. Which is not too shabby for a device that has no UART
at all: it receives with the USI peripheral, oversampling in hardware, and
transmits with bit edges timed by a timer's compare output,
while [DigiCDCFast](https://github.com/marcocarnut/DigiCDCFast) runs
bitbanged USB in software on the same 8-bit chip.

**Is it useful?** For some things. Receiving is solid: lossless up to
19200 bps in every test, even while the bridge also transmits. Transmitting
at 9600 bps is clean one way at a time, but under heavy traffic in both
directions about 1 byte in 20000 still goes out corrupted (details below).
So it suits receive-mostly devices, such as a GPS module at 9600 bps,
request-response protocols and half-duplex links, or anything that
retransmits on errors. For a serial port you can rely on in every case, a
USB-serial chip costs less than a dollar, and the
[DigisparkProBridge](https://github.com/marcocarnut/DigisparkProBridge)
has a hardware UART.

It is also a demonstration of what a fast USB serial library makes possible
on these boards, and of the techniques involved; the notes below explain
them.

## Wiring

| Digispark | Other device |
|-----------|--------------|
| PB0 (P0) | TX |
| PB1 (P1) | RX |
| GND | GND |

The Digispark's I/O is at 5 V; use a level shifter for 3.3 V devices. The
LED on PB1 flickers with the data, which doesn't matter.

## Requirements

- **Linux host.** Windows refuses low-speed USB serial devices; see
  [DigiCDCFast's README](https://github.com/marcocarnut/DigiCDCFast#windows-does-not-work).
- **Digistump AVR core 1.7.5**, from the board manager URL
  `https://raw.githubusercontent.com/ArminJo/DigistumpArduino/master/package_digistump_index.json`.
  No changes are needed for the ATtiny85.
- **DigiCDCFast** 1.0.0 or later, from the Arduino Library Manager.
- A Digispark with the micronucleus bootloader. Build for its 16.5 MHz
  clock setting: the board definition defaults to 16 MHz, where USB doesn't
  work.

## Using it

Plug it in, set the bit rate on the port and use it like any serial port:

```sh
stty -F /dev/ttyACM0 9600 raw -echo
```

8N1, 1200 to 9600 bps for both directions; receiving alone also works at
19200 bps. Two bit rates are commands instead:

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

### Shorter USB packets: DigiCDCMedium.h

The sketch includes `DigiCDCMedium.h` instead of `DigiCDCFast.h`: 2-byte USB
packets instead of 8-byte ones. Each transaction then keeps interrupts off
for ~73 µs instead of ~110 µs, less than a bit at 9600 bps, which is what
makes the planning work reliably. It limits USB to about 2000 bytes/s each
way, plenty for 9600 bps.

## Results

Against a CH340 USB-serial adapter wired to the bridge, a Linux PC (xHCI),
with the host sending as fast as the bridge accepts (`bridge_test.py`).
Bytes corrupted; none lost unless noted:

| Bit rate | Receive | Transmit | Both directions at once |
|----------|---------|----------|-------------------------|
| 1200, 2400 (3 kB) | 0 | 0 | 0 / 0 |
| 4800 (10 kB) | 0 | 0 | 0 / 0 |
| 9600 (50 kB) | 0 | 0, 859 bytes/s | receive 0; transmit 1-4 per 50 kB, 856 bytes/s |
| 19200 (10 kB) | 0 | 56-84 | fails, loses received bytes too |

Other tests at 9600 bps:

- **Bursty traffic** (`burst_test.py`, bursts of up to 100 bytes with
  50-300 ms pauses, both directions at once): 2 corrupted bytes in 48 kB.
- **Zmodem** (lrzsz), a 62 kB file each way: both copies identical; 784-836
  bytes/s through the bridge's transmitter, 911-930 bytes/s through its
  receiver.
- **PPP** (`novj`, MTU 296) with iperf3 in both directions: works, with about
  1.6% of the frames from the bridge failing their checksum and being
  retransmitted by TCP.

Things tried along the way:

- **8-byte USB packets** (plain `DigiCDCFast.h`): transmit 3-7 corrupted per
  10-20 kB, both directions 12-19 per 10 kB. So it almost works at 9600 bps
  with normal packets; without planning, ~35% of transmitted bytes were
  corrupted.
- **3-byte packets**: as good as 2-byte ones at 9600 bps, and better at
  transmitting at 19200 bps (17-22 corrupted per 10 kB), which still isn't
  usable.

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
- TinyBridge also: `e` edges made late, `a` USB transactions seen, `h`
  bytes started later, `p` idle bits spent looking;
- TinyBridgeUsi3x also: `l` glitches, `r` receive buffer overflows.

`bridge_test.py --stats` prints them after each transfer.

## Tests

Both scripts use a reference USB-serial adapter wired to the bridge (TX to
RX, RX to TX, GND to GND), and default to `/dev/ttyACM0` for the bridge and
`/dev/ttyUSB0` for the adapter. They report bytes lost, extra or corrupted,
with the offsets of the first mismatches.

- `bridge_test.py [--bytes N] [--rx-only] [--stats] BAUD...` sends
  pseudo-random data one direction at a time, then both at once.
  `--rx-only` is for the receive-only sketches.
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
  interrupts off (~5 µs), long enough to worry V-USB.

## License and credits

GPL version 2 or version 3, at your choice; see [LICENSE](LICENSE).

- [DigiCDCFast](https://github.com/marcocarnut/DigiCDCFast), and through it
  [V-USB](https://www.obdev.at/vusb/) by OBJECTIVE DEVELOPMENT Software GmbH.
- The Digistump AVR core, by Digistump LLC, as maintained by ArminJo.
- DigisparkBridge by Marco Carnut.
