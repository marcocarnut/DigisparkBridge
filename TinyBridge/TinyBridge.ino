/*
  TinyBridge -- USB-UART bridge on the Digispark (ATtiny85): a 3x oversampling
  USI receiver (TinyBridgeUsi3x) and a transmitter whose bit edges are timed
  by Timer1's compare output.

  Wiring: UART RX to PB0 (USI DI), UART TX from PB1 (OC1A; the LED on PB1
  doesn't matter), GND.
  Bit rate: the rate the host sets for the USB port (stty), 8N1, 1200 to
  9600 bps (receiving alone also 19200). 134 bps jumps to the micronucleus bootloader; 110 bps prints the
  diagnostic counters as hex and clears them, leaving the UART as it was.

  Receiving: Timer0 clocks USI three times per bit, so every sample is taken
  on time in hardware, whatever the interrupts are doing. USI's counter
  overflows every 8 samples and USIBR then holds those 8 samples; the handler
  only has to collect them before the next overflow (2 2/3 bits: 139 us at
  19200 bps) and keep the overflows exactly 8 samples apart. loop() finds
  start bits in the sample stream and reads each bit near its middle.

  Transmitting: Timer1 keeps running freely for millis() (64 CPU cycles per
  tick, 256 ticks per overflow). Each bit edge is a compare match on OC1A,
  set or clear, so the hardware makes it on time. Its handler schedules the
  next edge, and has until that edge to do it: one bit (104 us at 9600 bps).
  An edge it misses is made 2 ticks after the handler runs.

  V-USB keeps interrupts off for each USB transaction, and while data flows
  the host makes one or two every millisecond, each starting at the same
  point of the frame. With DigiCDCFast's usual 8-byte packets each takes
  ~110 us, longer than the deadline; built for 2-byte packets (below), ~73 us.
  A transaction starting just before an edge, or while its handler runs, can
  still hold the handler past the next edge. So each byte is planned: the
  handler's late runs show when the transactions start, and a byte starts
  later (or after an idle bit) when one would begin near an edge whose next
  edge changes the level. Bytes go out between the host's transactions.

  Latest measurements of V-USB delaying other interrupts: up to ~200 us.
*/

// DigiCDCFast with 2-byte USB packets: V-USB's interrupts-off stretches
// (~73 us) then stay shorter than a bit at 9600 bps. With 8-byte packets
// (~110 us) the planning alone still let ~1 byte in 1000 through corrupted in
// full duplex.
#ifndef USB_PACKET_SIZE
#define USB_PACKET_SIZE 2  // bytes per USB packet: the smaller, the shorter
#endif                     // V-USB holds the processor, and the less it carries
#if USB_PACKET_SIZE == 2
#include <DigiCDCMedium.h>
#else
#include <DigiCDCFast.h>
#include <DigiCDCDescriptor.h>
const uchar digiCdcConfigDescriptor[DIGICDC_DESCRIPTOR_SIZE] PROGMEM =
    DIGICDC_CONFIG_DESCRIPTOR(USB_PACKET_SIZE, USB_PACKET_SIZE);
DIGICDC_BUFFERS(8, 8);
#endif
#include <util/crc16.h>

#ifndef STATS
#define STATS           1    // 1: 110 bps prints and clears diagnostic counters.
                             //    Not free to turn off: the transmit timing was
                             //    tuned with the counters in, and without them a
                             //    byte in ~75 kB came out corrupted in testing.
#endif
#ifndef TIME_SHARING
#define TIME_SHARING    1    // 1: no USB data to the host while transmitting (reliable full duplex,
                             //    ~600 bytes/s each way); 0: both at once (~870 bytes/s, some
                             //    bytes corrupted on some hosts; see README)
#endif
#define BOOTLOADER_BAUD 134
#define STATS_BAUD      110
#define SAMPLES_PER_BIT 3
#define CAPTURE_SIZE    16   // 16: its gap flags are the bits of GPIOR1 and GPIOR2
#if TIME_SHARING
#define RX_SIZE         32   // powers of 2; received bytes wait here while the transmitter works
#ifndef RX_HOLD_BYTES
#define RX_HOLD_BYTES   20   // ... until this many,
#endif
#ifndef RX_HOLD_MS
#define RX_HOLD_MS      20   // or the oldest is this old
#endif
#ifndef AB_TEST
#define AB_TEST         0    // 1: 150 and 200 bps switch one stop bit and two,
#endif                       //    so the two can be interleaved run by run
#ifndef STOP_BITS
#define STOP_BITS       1    // 2: a second stop bit, a tenth of the transmit
#endif                       //    rate spent giving the far end an idle bit to
#define FRAME_1STOP     0x600  // start bit, 8 data bits, stop bit, end marker
#define FRAME_2STOP     0xE00  // ... and a second stop bit
#define STARTS_1STOP    0x400  // the frame's value while its start bit is next
#define STARTS_2STOP    0x800
#if AB_TEST                  //    switched at run time, to interleave the two
static uint16_t frameBase = FRAME_1STOP, frameStarts = STARTS_1STOP;
#define FRAME_OF(b)     (frameBase | ((b) << 1))
#define FRAME_STARTS    frameStarts
#elif STOP_BITS == 2
#define FRAME_OF(b)     (FRAME_2STOP | ((b) << 1))
#define FRAME_STARTS    STARTS_2STOP
#else
#define FRAME_OF(b)     (FRAME_1STOP | ((b) << 1))
#define FRAME_STARTS    STARTS_1STOP
#endif
#ifndef SPAN_STATS
#define SPAN_STATS      0    // 1: measure how long each byte takes on the line
#endif                       //    (D, x), in place of the CRC counters (c, b)
#if AB_TEST
#define AB_SLOW    150
#define AB_FAST    200
#endif
#define HOLD_BYTES RX_HOLD_BYTES
#define HOLD_MS    RX_HOLD_MS
#else
#define RX_SIZE         16   // powers of 2
#endif
#define TX_SIZE         16

// Bit rates: Timer0 clocks USI at 3 samples per bit (period OCR0A + 1), and a
// bit lasts `bitCycles` CPU cycles. A table spares the 32-bit arithmetic.
struct Rate {
  uint16_t baud;
  uint8_t ocr0a, prescaler;
  uint16_t bitCycles;
};
static const Rate rates[] PROGMEM = {
  { 1200,  71, _BV(CS01) | _BV(CS00), 13750},
  { 2400,  35, _BV(CS01) | _BV(CS00),  6875},
  { 4800, 142, _BV(CS01),              3438},
  { 9600,  71, _BV(CS01),              1719},
  {19200,  35, _BV(CS01),               859},  // receiving only
};

// Sample bytes from USI (oldest sample in bit 7). A capture's gap flag (samples
// were lost before it) is bit (slot & 7) of GPIOR1 for slots 0-7, GPIOR2 for
// 8-15, written only by the handler.
static uint8_t captured[CAPTURE_SIZE];
static volatile uint8_t captureHead;
static uint8_t captureTail;

static uint8_t rxBuf[RX_SIZE];
static uint8_t rxHead, rxTail;

// Transmitter: the line and the next edge. Times are in Timer1 ticks (64 CPU
// cycles), modulo 2^16.
static uint8_t txBuf[TX_SIZE];
static uint8_t txHead;                 // written by loop()
static volatile uint8_t txTail;        // written by the handler
// Flags live in the bits of GPIOR0 rather than in bytes of RAM, which this
// sketch has less of than it would like. GPIOR0 is in the bit-addressable I/O
// range, so each is set or cleared in a single instruction that no interrupt
// can land inside -- which is what the ones shared with the handlers need.
// The edge after the one the timer holds, ready for usbTransactionEnd() to
// load while V-USB has the processor: OCR1A reaches only one edge ahead, and a
// run of USB transactions can outlast a bit. EDGE_READY says the stash is
// valid; the handler clears it while it programs, so the hook cannot land in
// the middle of that, and the hook clears it when it uses the stash.
static uint8_t nextOcr, nextTccr;

#define txFlags    GPIOR0
#define TX_ACTIVE  _BV(0)  // an edge is scheduled
#define TX_IDLE    _BV(1)  // the bit on the line is idle line after a stop bit
#define TX_MOVED   _BV(2)  // the edge just made was moved later
#define TX_HOLD    _BV(3)  // loop() has asked the transmitter to hold
#define TX_HELD    _BV(4)  // ... and it has, between bytes
#define SHARE_USB  _BV(5)  // bits short enough for USB data to the host to corrupt them
#define EDGE_READY _BV(6)  // nextOcr/nextTccr hold the edge after the timer's
// Time sharing: USB data to the host (IN packets) never goes out while a
// byte is being transmitted. loop() asks the transmitter to hold (txHold);
// it finishes the byte on the line and sends idle bits (txHeld) while
// received bytes go to the host, then loop() lets it go on.

static uint16_t txIdleBits;            // idle bits since the last byte
#define LINGER_BITS 10000              // how long the handler keeps running after the last byte (~1 s at 9600 bps)
static uint16_t txFrame;               // bit 0: the bit on the line, then the rest of the byte
static uint16_t txEdge;                // when the next edge is due
static uint8_t txEdgeFrac;             // and its 1/64 ticks
static uint16_t bitCycles;
static uint8_t bitTicks, bitFrac;      // bitCycles in ticks and 1/64 ticks
static uint8_t edgeTicks[10];          // edge k of a byte, ticks after its start edge

// USB activity: the host's transactions come every millisecond, and V-USB
// keeps interrupts off while it handles them (measured: one or two stretches
// per millisecond, each starting at the same point)
#ifndef WINDOW_TICKS
#define WINDOW_TICKS 52                // activity to plan around, found best by testing (200 us)
#endif
#define SEEN_TICKS   8                 // a handler this late (31 us) was held by activity
#define HANDLER_TICKS 10                // a handler's own run time
#define STALE_TICKS  4096              // after 16 ms without seeing activity, look again
#define PROBE_BITS   11                // idle bits spent looking, before a byte
#ifndef RTS_OUTPUT
#define RTS_OUTPUT   0   // 1: PB2 is an RTS output (low: the other device may send)
#endif
#define RTS_HIGH  (RX_SIZE - 10)  // stop it with this many received bytes waiting,
#define RTS_LOW   8               // let it send again with this many
#ifndef CTS_INPUT
#define CTS_INPUT    0   // 1: PB5 is a CTS input, and the bridge only sends while
                         //    it is low. Needs RSTDISBL burned, which costs the
                         //    reset pin: micronucleus becomes the only way in, and
                         //    a high-voltage programmer the only way back. The pin
                         //    is pulled up, so unwired it would read "wait".
#endif
#if CTS_INPUT
#define maySend() (!(PINB & _BV(PB5)))
#else
#define maySend() true   // the compiler then drops every test of it
#endif
#ifndef IDLE_LIMIT
#define IDLE_LIMIT   40                // idle bits a byte may wait for a safe place (4 ms at 9600 bps)
#endif
static uint16_t activityStart;         // when the latest activity started (estimated)
static uint16_t lastEnd;               // when the latest activity seen ended
static uint16_t framePeriod = 16500 / 4;  // USB frame period, 1/16 ticks
static uint8_t probeBits;
static uint8_t bandBefore;             // activity starting this soon before an edge holds its handler too long

#define COM1A_MASK (_BV(COM1A1) | _BV(COM1A0))
#define COM1A_SET  (_BV(COM1A1) | _BV(COM1A0))
#define COM1A_CLR  _BV(COM1A1)


// Diagnostic counters, printed at STATS_BAUD
#if STATS
#ifndef WIDE_STATS
#define WIDE_STATS 0     // 1: the byte counters (n, b) are 32-bit, printed as two halves
#endif
#if WIDE_STATS
#define COUNTER uint32_t
#else
#define COUNTER uint16_t
#endif
#define COUNT(counter) (stats.counter++)
static struct {
  COUNTER received;                             // bytes: 16 bits wrap in a long test
  uint16_t gaps, framingErrors;
  uint16_t forcedEdges, seen, shifted, probed, gaveUp;  // updated by the transmitter
  uint16_t rxOverflows, holds;
  uint16_t hooked;                              // edges the USB interrupt loaded
#if SPAN_STATS
  uint8_t worstShift;                           // furthest an edge inside a byte was
  uint16_t badBits;                             // moved, and how many moved by a whole
#endif                                          // bit or more, which must misread
  uint8_t captureOverflows;                     // updated by the receive handler
#if !SPAN_STATS
  uint16_t usbCrc;                              // CRC-XMODEM of the bytes read from USB
  COUNTER usbBytes;
#endif
} stats;
#else
#define COUNT(counter) ((void)0)
#endif

// Timer1 ticks, counted on from the last call: call at least every 256 ticks
// (the transmitter does, while active), with interrupts off
static uint16_t clockNow;
extern volatile unsigned long millis_timer_overflow_count;  // the core's
static uint16_t ticksNow()
{
  return clockNow += (uint8_t)(TCNT1 - (uint8_t)clockNow);
}

// Collect USI's samples. Runs with USI's interrupt masked, interrupts on,
// and unmasks it: the handler restores its registers with interrupts on (V-USB
// must not wait; USI's next overflow is at least a sample away, so it nests
// only when this ran far too late anyway).
extern "C" void rxCapture() __attribute__((used));
void rxCapture()
{
  // A sample shifting in between reading and rewriting the counter would be
  // lost. It shifts at the compare match, when TCNT0 goes from OCR0A to 0,
  // but USI's counter changes slightly later: with only TCNT0 == OCR0A-1
  // avoided, about 1 byte in 5000 was lost or corrupted at 9600 bps. Keep
  // clear of the ticks on both sides of the match.
  cli();
  for (uint8_t t; (t = TCNT0) == 0 || (uint8_t)(OCR0A - t) <= 1; )
    ;
  uint8_t count = USISR & 0x0F;  // samples since the overflow
  USISR = _BV(USIOIF) | ((8 + count) & 0x0F);  // next overflow 8 samples after it
  uint8_t samples = USIBR;
  sei();

  static bool lost;
  uint8_t slot = captureHead, bit = 1 << (slot & 7);
  captured[slot] = samples;
  if (count >= 8 || lost) {  // too late: a window is lost
    if (slot & 8) GPIOR2 |= bit; else GPIOR1 |= bit;
  } else {
    if (slot & 8) GPIOR2 &= ~bit; else GPIOR1 &= ~bit;
  }
  uint8_t next = (slot + 1) & (CAPTURE_SIZE - 1);
  lost = next == captureTail;
  if (!lost)
    captureHead = next;
  else
    COUNT(captureOverflows);

  USICR = _BV(USICS0) | _BV(USIOIE);
}

// Entry stub for the transmit handler. V-USB must never wait for it (it has
// to start within a few dozen cycles of a packet, or may misread it), and the
// stack is small: it masks its own interrupt and millis()' (whose
// non-blocking handler would otherwise nest in it, deepening the stack), lets
// other interrupts in, and only then saves the registers a C function may
// change. It restores them with interrupts still on, then in one short
// interrupts-off step unmasks its interrupt if asked (r24 from the handler)
// and puts millis()' back as it found it. (A plain ISR saved 27 registers with
// interrupts off; restoring them with interrupts off, with the handler's own,
// took over 100 cycles and let USB packets from the host arrive corrupted.)
#define STUB_SAVE \
    "sei\n" \
    "push r0\n push r1\n clr r1\n" \
    "push r18\n push r19\n push r20\n push r21\n push r22\n push r23\n" \
    "push r25\n push r26\n push r27\n push r30\n push r31\n"
#define STUB_RESTORE \
    "pop r31\n pop r30\n pop r27\n pop r26\n pop r25\n" \
    "pop r23\n pop r22\n pop r21\n pop r20\n pop r19\n pop r18\n" \
    "pop r1\n pop r0\n" \
    "cli\n bst r24, 6\n pop r24\n bld r24, 6\n out 0x39, r24\n"  /* TIMSK as found, OCIE1A as asked */ \
    "pop r24\n out 0x3f, r24\n pop r24\n reti\n"

// A plain handler: an entry stub let the transmit handler nest before the
// counter is read, which then sometimes came more than 8 samples late
ISR(USI_OVF_vect)
{
  USICR = _BV(USICS0);  // mask USI's interrupt, keep clocking it from Timer0
  sei();
  rxCapture();
}

static void txAdvance()  // txEdge one bit later
{
  txEdgeFrac += bitFrac;
  txEdge += bitTicks + (txEdgeFrac >> 6);
  txEdgeFrac &= 63;
}

// USB activity ended at `end`, having held the handler of an edge made `late`
// ticks earlier
static void activitySeen(uint16_t end, uint8_t late)
{
  uint16_t d = (end - lastEnd) << 4;
  if (d >= 16500 / 4 - 40 && d <= 16500 / 4 + 40)  // a frame after the last (within 1%)
    framePeriod += (int16_t)(d - framePeriod) >> 4;
  lastEnd = end;
  // It started by the edge, and after the previous edge's handler had run
  uint16_t hi = end - late, lo = hi - bitTicks + HANDLER_TICKS;
  uint16_t start = activityStart, period = framePeriod >> 4;
  if ((uint16_t)(hi - start) > STALE_TICKS)
    start = hi;
  while ((int16_t)(hi - start) > (int16_t)(period >> 1))  // the predicted start nearest to it
    start += period;
  if ((int16_t)(start - hi) > 0)
    start = hi;
  else if ((int16_t)(lo - start) > 0)
    start = lo;
  activityStart = start;
  COUNT(seen);
}

// How much later a byte (frame, start bit first) must start, in ticks, so
// that USB activity starting `d` ticks after its start edge can't hold the
// handler of an edge past the next one where the level changes. 0 when it can't.
static uint8_t shiftNeeded(uint16_t frame, int16_t d)
{
  for (uint8_t k = 0; k < 9; k++, frame >>= 1) {
    int16_t from = edgeTicks[k] - bandBefore;
    if (((frame ^ (frame >> 1)) & 1) && d > from && d <= edgeTicks[k] + HANDLER_TICKS)
      return d - from;
  }
  return 0;
}

// Choose when the byte in `frame` starts, at txEdge or later. Returns false
// to send an idle bit first instead (looking for USB activity).
static bool txPlan(uint16_t frame)
{
  cli();
  uint16_t now = ticksNow();
  sei();
  if ((int16_t)(txEdge - now) < 2) {  // the handler ran late: start from now
    txEdge = now + 2;
    txEdgeFrac = 0;
  }
  if (bitTicks >= WINDOW_TICKS)  // bits long enough to ride out any activity
    return true;
  if ((uint16_t)(now - lastEnd) > STALE_TICKS) {
    if (probeBits) {
      probeBits--;
      COUNT(probed);
      return false;
    }
    return true;  // none seen: whatever there is, is short
  }
  // The first activity predicted to start after (or just before) the start edge
  uint16_t start = activityStart, period = framePeriod >> 4;
  while ((int16_t)(start - txEdge) <= -(int16_t)bandBefore)
    start += period;
  // Start later until safe. Beyond 128 ticks (OCR1A's reach), send an idle
  // bit and plan again; a byte that fits nowhere goes anyway after
  // IDLE_LIMIT idle bits, corrupted now and then, rather than stopping the
  // bridge (its buffers fill, and DigiCDCFast's flow control then refuses
  // even the host's line-coding requests).
  static uint8_t idle;
  uint8_t total = 0;
  for (uint8_t shift; (shift = shiftNeeded(frame, start - txEdge - total)); total += shift)
    if (total + shift > 128) {
      if (++idle < IDLE_LIMIT)
        return false;
      COUNT(gaveUp);  // sent where it doesn't fit
      total = 0;
      break;
    }
  idle = 0;
  if (total) {
    txEdge += total;
    COUNT(shifted);
  }
  return true;
}

// Schedule the next edge after the one on the line. Runs with Timer1's
// compare (and millis()') interrupts masked, interrupts on. Returns the TIMSK
// bit to unmask (OCIE1A, or 0 when done). edgeMade: called from the compare
// interrupt.
// Called by DigiCDCFast at the end of every USB transaction, with the driver's
// registers already saved: r0, r16-r22, Y and the flags are ours, nothing
// else. While a run of transactions holds the processor the handler cannot
// run, so the edge it would have loaded is loaded here instead -- the one
// thing that can be done from inside someone else's interrupt. Every
// instruction here delays the packet already arriving, so there are nine.
extern "C" void usbTransactionEnd() __attribute__((naked, used, externally_visible));
void usbTransactionEnd()
{
  asm volatile(
      "        sbis %[flags], %[ready]  \n"  // nothing stashed: nothing to do
      "        ret                      \n"
      "        in   r16, %[tifr]        \n"
      "        sbrs r16, %[ocf]         \n"  // the timer's edge has not fired yet
      "        ret                      \n"
      "        lds  r16, %[tccr]        \n"  // the level the stashed edge makes
      "        out  %[tccr1], r16       \n"
      "        lds  r16, %[ocr]         \n"
      "        out  %[ocr1a], r16       \n"
      "        ldi  r16, %[ocfbit]      \n"  // the handler is not wanted for the
      "        out  %[tifr], r16        \n"  // edge just made, but for the next
      "        cbi  %[flags], %[ready]  \n"
      "        ret                      \n"
      :
      : [flags] "I"(_SFR_IO_ADDR(GPIOR0)), [ready] "I"(6),
        [tifr] "I"(_SFR_IO_ADDR(TIFR)), [ocf] "I"(OCF1A),
        [ocfbit] "M"(_BV(OCF1A)), [tccr1] "I"(_SFR_IO_ADDR(TCCR1)),
        [ocr1a] "I"(_SFR_IO_ADDR(OCR1A)), [tccr] "i"(&nextTccr), [ocr] "i"(&nextOcr));
}

extern "C" __attribute__((used)) uint8_t txSchedule(uint8_t edgeMade)
{
  if (edgeMade) {
    cli();
    uint16_t now = ticksNow();
    sei();
    int16_t late = now - txEdge;
    if (!(txFlags & TX_MOVED) && late >= SEEN_TICKS && late < 128)
      activitySeen(now, late);
    txFlags &= ~TX_MOVED;
  }
  static bool stashed;
  if (stashed && !(txFlags & EDGE_READY)) {
    // usbTransactionEnd() loaded the edge this call would have: catch up with
    // it, silently, and go on to the one after. (Only ever inside a byte: a
    // boundary needs planning, so nothing is stashed there.)
    txFrame >>= 1;
    txAdvance();
    COUNT(hooked);
  }
  stashed = false;
  txFlags &= ~EDGE_READY;  // the hook must not fire while we are programming
  for (;;) {
    uint16_t frame = txFrame >> 1;  // bit 0: the bit that starts at the next edge
    txAdvance();
    if (frame == 1) {               // the next edge ends a stop bit
#if TIME_SHARING
      // Between bytes, so loop() can send received ones to the host: say so
      // whether or not a byte is waiting, or it would wait for the timeout
      if (txFlags & TX_HOLD) {
        txFlags |= TX_HELD;
        frame = 3;                  // an idle bit
        txFlags |= TX_IDLE;
      } else
#endif
      if (txTail != txHead && maySend()) {
        frame = FRAME_OF(txBuf[txTail]);  // start bit, 8 data bits, stop, end marker
        if (txPlan(frame)) {
          txTail = (txTail + 1) & (TX_SIZE - 1);
          txFlags &= ~TX_IDLE;
        } else {
          frame = 3;                // an idle bit
          txFlags |= TX_IDLE;
        }
      } else if (!(txFlags & TX_IDLE)) {
        frame = 3;                  // a bit of idle line, so the stop bit ends before a later start
        txFlags |= TX_IDLE;
        txIdleBits = 0;
      } else if (++txIdleBits < LINGER_BITS) {
        frame = 3;                  // keep running on idle bits, watching USB activity for the next burst
      } else {
        txFlags &= ~TX_ACTIVE;
        return 0;
      }
    }
    txFrame = frame;

    // With interrupts on: OCR1A still holds a time well past (the edge just
    // made, or txStart()'s), so no match comes between these; the check
    // below catches an edge already too late
    TCCR1 = (TCCR1 & ~COM1A_MASK) | (frame & 1 ? COM1A_SET : COM1A_CLR);
    OCR1A = txEdge;
    cli();
    uint16_t now = ticksNow();
    int16_t slack = (int16_t)(txEdge - now);
#if STATS && SPAN_STATS
    bool insideByte = false;
#endif
    if (slack < 2) {                    // too late for a match at its time
      if (frame >= FRAME_STARTS) {      // a start bit: start the byte later instead
        txEdge = now + 2;               // (the whole byte moves: nothing is stretched)
        txEdgeFrac = 0;
      } else {
        COUNT(forcedEdges);
        txFlags |= TX_MOVED;
#if STATS && SPAN_STATS
        insideByte = true;  // nothing but a flag here: see below
#endif
      }
      // Make the edge 2 ticks from now. (Forcing a match with FOC1A right
      // after changing COM1A made no edge: the output kept its level.)
      OCR1A = now + 2;
    }
    TIFR = _BV(OCF1A);  // an old match would call the handler early
    sei();
    // Stash the edge after this one, if it needs no planning -- which is to
    // say anywhere inside a byte. The times are the scheduled ones, as the
    // handler's own are: a forced edge does not move those that follow.
    if ((frame >> 1) != 1) {
      uint8_t frac = txEdgeFrac + bitFrac;
      nextOcr = (uint8_t)(txEdge + bitTicks + (frac >> 6));
      nextTccr = (TCCR1 & ~COM1A_MASK) | ((frame >> 1) & 1 ? COM1A_SET : COM1A_CLR);
      stashed = true;
      txFlags |= EDGE_READY;
    }
#if STATS && SPAN_STATS
    // After sei(), and after the edge has been programmed: an edge inside a
    // byte that moves late stretches the bit before it and shortens the one
    // after, while the byte's total length stays right -- so this has to be
    // measured per edge, and measuring it must not delay it (counting here
    // rather than up there is worth 400 times the corruption).
    if (insideByte) {
      uint8_t moved = (uint8_t)(2 - slack);
      if (moved > stats.worstShift)
        stats.worstShift = moved;
      if (moved >= bitTicks)  // a whole bit: the far end samples the wrong one
        COUNT(badBits);
    }
#endif
    return _BV(OCIE1A);
  }
}

ISR(TIMER1_COMPA_vect, ISR_NAKED)  // an edge was made
{
  __asm__(
    "push r24\n in r24, 0x3f\n push r24\n"
    "in r24, 0x39\n push r24\n andi r24, 0xbb\n out 0x39, r24\n" /* TIMSK: mask OCIE1A, TOIE1 */
    STUB_SAVE
    "ldi r24, 1\n rcall txSchedule\n"
    STUB_RESTORE);
}

static void txStart()
{
  txFlags |= TX_ACTIVE;
  txFrame = 2;  // "a stop bit on the line": the next edge starts a byte
  txFlags |= TX_IDLE;
  probeBits = PROBE_BITS;
  // The clock may have missed Timer1 overflows while idle: set it from the
  // core's count of them, so what was seen of USB activity stays usable.
  // (In loop() that count is up to date but for a pending overflow.)
  uint8_t t = TCNT1;
  clockNow = ((uint16_t)(uint8_t)millis_timer_overflow_count << 8) | t;
  if ((TIFR & _BV(TOV1)) && t < 128)
    clockNow += 256;
  txEdge = clockNow;  // txPlan() moves it to now
  OCR1A = t + 128;    // no match while txSchedule() changes the level
  txEdgeFrac = 0;
  TIFR = _BV(OCF1A);
  sei();
  uint8_t unmask = txSchedule(0);
  cli();
  TIMSK |= unmask;
  sei();
}

static void uartBegin(const Rate *entry)  // entry: in flash
{
  Rate rate;
  memcpy_P(&rate, entry, sizeof rate);
  const Rate *r = &rate;
  cli();
  TIMSK &= ~_BV(OCIE1A);  // abandon a byte being sent
  txFlags &= ~TX_ACTIVE;
  txHead = txTail = 0;
#if TIME_SHARING
  txFlags &= ~TX_HOLD;  // one bit at a time: each is a single
  txFlags &= ~TX_HELD;  // instruction, and the handler writes here too
#endif
  TCCR1 = (TCCR1 & ~COM1A_MASK) | COM1A_SET;
  GTCCR |= _BV(FOC1A);    // idle line
  bitCycles = r->bitCycles;
  bitTicks = bitCycles >> 6;
  bitFrac = bitCycles & 63;
#if TIME_SHARING
  if (bitTicks < 40)
    txFlags |= SHARE_USB;
  else
    txFlags &= ~SHARE_USB;
#endif
  uint8_t ticks = 0;
  uint16_t frac = 0;  // bitFrac * k / 64 < 9, but k * bitFrac needs 16 bits
  for (uint8_t k = 0; k < 10; k++, ticks += bitTicks, frac += bitFrac)
    edgeTicks[k] = ticks + (frac >> 6);
  bandBefore = WINDOW_TICKS + HANDLER_TICKS > bitTicks ? WINDOW_TICKS + HANDLER_TICKS - bitTicks : 0;
  USICR = 0;
  TCCR0B = 0;
  TCCR0A = _BV(WGM01);  // CTC: period is OCR0A + 1
  OCR0A = r->ocr0a;
  TCNT0 = 0;
  captureHead = captureTail = 0;
  rxHead = rxTail = 0;
  USISR = _BV(USIOIF) | 8;
  USICR = _BV(USICS0) | _BV(USIOIE);
  TCCR0B = r->prescaler;
  sei();
}

// Decoder state, driven one sample at a time
static int8_t framePos = -1;  // samples since the start bit's first low sample, -1 when idle
static uint8_t nextSample, bitsRead, rxByte;
static bool lastSample = false;

static void decode(uint8_t s)
{
  if (framePos < 0) {
    if (lastSample && !s) {  // falling edge: start bit
      framePos = 0;
      nextSample = 4;        // 1.5 bits after the edge, which lies 0-1/3 bit earlier
      bitsRead = 0;
    }
  } else if (++framePos == 1 && s) {
    framePos = -1;           // start bit not low in its middle: a glitch
  } else if (framePos == nextSample) {
    if (bitsRead < 8) {
      rxByte = (rxByte >> 1) | (s ? 0x80 : 0);
      bitsRead++;
      nextSample += SAMPLES_PER_BIT;
    } else {
      if (s) {               // stop bit high: keep the byte
        uint8_t next = (rxHead + 1) & (RX_SIZE - 1);
        COUNT(received);
        if (next != rxTail) {
          rxBuf[rxHead] = rxByte;
          rxHead = next;
        } else
          COUNT(rxOverflows);
      } else
        COUNT(framingErrors);
      framePos = -1;
    }
  }
  lastSample = s;
}

static void enterBootloader()
{
  SerialUSB.delay(100);  // let the host's SET_LINE_CODING request complete
  cli();
  USICR = 0;             // no sketch interrupts may fire once the bootloader runs
  TIMSK = 0;
  TCCR0B = 0;
  TCCR1 = 0;
  ((void (*)())0)();     // micronucleus points the reset vector at itself
}

#if STATS
extern uint8_t _end;  // the first byte above the variables: the stack's limit

static void writeHex(char tag, uint16_t v)
{
  SerialUSB.write(' ');
  SerialUSB.write(tag);
  for (int8_t shift = 12; shift >= 0; shift -= 4) {
    uint8_t d = (v >> shift) & 0x0F;
    SerialUSB.write(d < 10 ? '0' + d : 'a' + d - 10);
  }
}

static void printStats()
{
  cli();
  uint8_t o = stats.captureOverflows;
  sei();
  uint16_t unused = 0;  // stack bytes never touched
  for (uint8_t *p = &_end; *p == 0xC5; p++)
    unused++;
  SerialUSB.write('S');
  writeHex('k', unused);
#if WIDE_STATS
  writeHex('N', stats.received >> 16);
#endif
  writeHex('n', stats.received);
  writeHex('g', stats.gaps);
  writeHex('o', o);
  writeHex('f', stats.framingErrors);
  writeHex('e', stats.forcedEdges);
  writeHex('a', stats.seen);
  writeHex('h', stats.shifted);
  writeHex('p', stats.probed);
  writeHex('z', stats.gaveUp);
  writeHex('r', stats.rxOverflows);
  writeHex('w', stats.holds);
  writeHex('H', stats.hooked);
#if AB_TEST
  writeHex('F', frameBase);  // which frame format is in force
#endif
#if SPAN_STATS
  writeHex('D', stats.worstShift);
  writeHex('x', stats.badBits);
#endif
  writeHex('P', framePeriod);  // USB frame in 1/16 Timer1 ticks: 4125 with an exact 16.5 MHz clock
#if !SPAN_STATS
  writeHex('c', stats.usbCrc);
#if WIDE_STATS
  writeHex('B', stats.usbBytes >> 16);
#endif
  writeHex('b', stats.usbBytes);
#endif
  SerialUSB.write('\r');
  SerialUSB.write('\n');
  // Printing kept loop() busy: drop the samples it couldn't decode meanwhile
  cli();
  captureTail = captureHead;
  framePos = -1;
  lastSample = false;
  memset(&stats, 0, sizeof stats);
  sei();
}
#endif

// Refuse bit rates the bridge can't provide: DigiCDCFast then answers the
// host's request with a STALL and keeps reporting the settings in use. (The
// data bits, parity and stop bits are not checked: the UART is always 8N1.)
extern "C" uint8_t digiCdcAcceptLineCoding(const uint8_t *coding)
{
  if (coding[2] | coding[3])  // above 65535 bps
    return 0;
  uint16_t baud = coding[0] | coding[1] << 8;
  if (baud == BOOTLOADER_BAUD || baud == STATS_BAUD)
    return 1;
#if AB_TEST
  if (baud == AB_SLOW || baud == AB_FAST)
    return 1;
#endif
  for (const Rate *r = rates; r < rates + sizeof rates / sizeof *rates; r++)
    if (pgm_read_word(&r->baud) == baud)
      return 1;
  return 0;
}

void setup()
{
#if STATS
  for (uint8_t *p = &_end; p < (uint8_t *)SP - 32; p++)  // paint the free stack
    *p = 0xC5;
#endif
  PORTB |= _BV(PB1);  // TX idles high (PB0, RX, is an input from reset)
  DDRB |= _BV(PB1);
#if RTS_OUTPUT
  PORTB &= ~_BV(PB2);  // RTS: the other device may send
  DDRB |= _BV(PB2);
#endif
#if CTS_INPUT
  PORTB |= _BV(PB5);   // CTS input with its pull-up (PB5 is reset until RSTDISBL)
#endif
  // Timer1: normal mode, same clock and period for millis(), OC1A sets PB1
  cli();
  TCCR1 = (TCCR1 & 0x0F) | COM1A_SET;
  GTCCR = (GTCCR & ~(_BV(PWM1B) | _BV(COM1B1) | _BV(COM1B0))) | _BV(FOC1A);
  sei();
  SerialUSB.begin();
}

void loop()
{
  static uint16_t lineBaud, uartBaud;
  unsigned long rate = SerialUSB.baud();
  uint16_t baud = rate >> 16 ? 0 : (uint16_t)rate;
  if (baud != lineBaud) {
    lineBaud = baud;
    if (baud == BOOTLOADER_BAUD)
      enterBootloader();
#if STATS
    else if (baud == STATS_BAUD)
      printStats();
#endif
#if AB_TEST
    else if (baud == AB_SLOW || baud == AB_FAST) {
      frameBase = baud == AB_SLOW ? FRAME_1STOP : FRAME_2STOP;
      frameStarts = baud == AB_SLOW ? STARTS_1STOP : STARTS_2STOP;
    }
#endif
    else if (baud != uartBaud)  // other rates leave the UART as it is
      for (const Rate *r = rates; r < rates + sizeof rates / sizeof *rates; r++)
        if (pgm_read_word(&r->baud) == baud) {
          uartBegin(r);
          uartBaud = baud;
        }
  }

  while (captureTail != captureHead) {
    uint8_t samples = captured[captureTail];
    if ((captureTail & 8 ? GPIOR2 : GPIOR1) & (1 << (captureTail & 7))) {
      COUNT(gaps);
      framePos = -1;         // the byte in progress is incomplete
      lastSample = false;    // wait for the line to be seen idle again
    }
    captureTail = (captureTail + 1) & (CAPTURE_SIZE - 1);
    for (uint8_t mask = 0x80; mask; mask >>= 1)
      decode(samples & mask);
  }

#if TIME_SHARING
  // Received bytes go to the host. Each packet keeps V-USB busy, which the
  // transmitter has to plan around, so they go 8 at a time (or once 4 ms
  // old), and while there are bytes to transmit they wait for a pause in
  // transmitting (RX_HOLD_BYTES, or RX_HOLD_MS old).
  static uint16_t rxSince, holdSince;
  uint8_t rxCount = (rxHead - rxTail) & (RX_SIZE - 1);
  if (!rxCount)
    rxSince = millis();
#if RTS_OUTPUT
  // Ask the other device to pause before the buffer is full, and to go on
  // once it has drained (its CTS input; low means it may send)
  if (rxCount >= RTS_HIGH)
    PORTB |= _BV(PB2);
  else if (rxCount <= RTS_LOW)
    PORTB &= ~_BV(PB2);
#endif
  bool sending = txHead != txTail;  // (a byte is on the line, or will be)
  if (txFlags & TX_HOLD) {
    if ((txFlags & TX_HELD) || !(txFlags & TX_ACTIVE) || txHead == txTail) {
      for (int room = SerialUSB.availableForWrite(); room > 0 && rxTail != rxHead; room--) {
        SerialUSB.write(rxBuf[rxTail]);
        rxTail = (rxTail + 1) & (RX_SIZE - 1);
      }
      if (rxTail == rxHead && SerialUSB.availableForWrite() == pgm_read_byte(&digiCdcBufferSizes[0])
          && usbInterruptIsReady()) {  // all taken by the host: transmit again
        txFlags &= ~TX_HOLD;  // one bit at a time: each is a single
        txFlags &= ~TX_HELD;  // instruction, and the handler writes here too
        holdSince = millis() - 100;
      }
    }
    if ((txFlags & TX_HOLD) && (uint16_t)millis() - holdSince >= 100) {  // the host isn't taking data: don't stop transmitting for it
      txFlags &= ~TX_HOLD;  // one bit at a time: each is a single
      txFlags &= ~TX_HELD;  // instruction, and the handler writes here too
      holdSince = millis();  // and don't hold again for 100 ms
    }
  } else if (rxCount && sending && (txFlags & SHARE_USB) && (uint16_t)millis() - holdSince >= 100) {
    if (rxCount >= HOLD_BYTES || (uint16_t)millis() - rxSince >= HOLD_MS) {
      COUNT(holds);
      holdSince = millis();
      txFlags |= TX_HOLD;
    }
  } else if (rxCount >= 8 || (rxCount && (uint16_t)millis() - rxSince >= 4))
    for (int room = SerialUSB.availableForWrite(); room > 0 && rxTail != rxHead; room--) {
      SerialUSB.write(rxBuf[rxTail]);
      rxTail = (rxTail + 1) & (RX_SIZE - 1);
    }
#else
  // Received bytes go to the host 8 at a time, or once 4 ms old: each packet
  // keeps V-USB busy, which the transmitter has to plan around
  static uint16_t rxSince;
  if (rxTail == rxHead)
    rxSince = millis();
  else if (((rxHead - rxTail) & (RX_SIZE - 1)) >= 8 || (uint16_t)millis() - rxSince >= 4)
    for (int room = SerialUSB.availableForWrite(); room > 0 && rxTail != rxHead; room--) {
      SerialUSB.write(rxBuf[rxTail]);
      rxTail = (rxTail + 1) & (RX_SIZE - 1);
    }
#endif

  while (SerialUSB.available()) {
    uint8_t next = (txHead + 1) & (TX_SIZE - 1);
    if (next == txTail)
      break;
    txBuf[txHead] = SerialUSB.read();
#if STATS
#if !SPAN_STATS
    stats.usbCrc = _crc_xmodem_update(stats.usbCrc, txBuf[txHead]);
    stats.usbBytes++;
#endif
#endif
    txHead = next;
  }
  // While the other device says wait, the transmitter runs out its idle bits
  // and stops; this is what picks it up again once the line goes low.
  if (!(txFlags & TX_ACTIVE) && txHead != txTail && maySend()) {
    cli();
    txStart();
  }
}
