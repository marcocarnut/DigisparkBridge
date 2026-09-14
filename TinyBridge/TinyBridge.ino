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
#include <DigiCDCMedium.h>

#define BOOTLOADER_BAUD 134
#define STATS_BAUD      110  // diagnostics: print and clear the counters
#define SAMPLES_PER_BIT 3
#define CAPTURE_SIZE    16  // powers of 2
#define RX_SIZE         16
#define TX_SIZE         16
#define GAP             1   // capture flag: samples were lost before these

// Sample bytes from USI (oldest sample in bit 7) and their flags
static uint8_t captured[CAPTURE_SIZE], capturedFlags[CAPTURE_SIZE];
static volatile uint8_t captureHead;
static uint8_t captureTail;

static uint8_t rxBuf[RX_SIZE];
static uint8_t rxHead, rxTail;
static uint8_t prescaler;  // TCCR0B clock select

// Transmitter: the line and the next edge. Times are in Timer1 ticks (64 CPU
// cycles), modulo 2^16.
static uint8_t txBuf[TX_SIZE];
static uint8_t txHead;                 // written by loop()
static volatile uint8_t txTail;        // written by the handler
static volatile bool txActive;         // an edge is scheduled
static bool txIdle;                    // the bit on the line is idle line after a stop bit
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
#define WINDOW_TICKS 52                // activity to plan around, found best by testing (200 us)
#define SEEN_TICKS   8                 // a handler this late (31 us) was held by activity
#define HANDLER_TICKS 10                // a handler's own run time
#define STALE_TICKS  4096              // after 16 ms without seeing activity, look again
#define PROBE_BITS   11                // idle bits spent looking, before a byte
static uint16_t activityStart;         // when the latest activity started (estimated)
static uint16_t lastEnd;               // when the latest activity seen ended
static uint16_t framePeriod = 16500 / 4;  // USB frame period, 1/16 ticks
static uint8_t probeBits;
static uint8_t bandBefore;             // activity starting this soon before an edge holds its handler too long
static bool txMoved;                   // the edge just made was moved later

#define COM1A_MASK (_BV(COM1A1) | _BV(COM1A0))
#define COM1A_SET  (_BV(COM1A1) | _BV(COM1A0))
#define COM1A_CLR  _BV(COM1A1)


// Diagnostic counters
static volatile uint8_t captureOverflows;  // updated by the handler
static uint16_t gaps, framingErrors, received;
static uint16_t forcedEdges, seen, shifted, probed;  // updated by the transmitter

// V-USB must never wait for this handler, so it masks its own interrupt and
// lets others in; only the counter update runs with interrupts off.
ISR(USI_OVF_vect)
{
  USICR = _BV(USICS0);  // mask USI's interrupt, keep clocking it from Timer0
  sei();

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

  static uint8_t lost;
  uint8_t next = (captureHead + 1) & (CAPTURE_SIZE - 1);
  captured[captureHead] = samples;
  capturedFlags[captureHead] = count >= 8 || lost ? GAP : 0;  // too late: a window is lost
  lost = 0;
  if (next != captureTail)
    captureHead = next;
  else {
    lost = 1;
    captureOverflows++;
  }

  cli();
  USICR = _BV(USICS0) | _BV(USIOIE);
}

// Timer1 ticks, counted on from the last call: call at least every 256 ticks
// (the transmitter does, while active), with interrupts off
static uint16_t clockNow;
extern volatile unsigned long millis_timer_overflow_count;  // the core's
static uint16_t ticksNow()
{
  return clockNow += (uint8_t)(TCNT1 - (uint8_t)clockNow);
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
  seen++;
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
      probed++;
      return false;
    }
    return true;  // none seen: whatever there is, is short
  }
  // The first activity predicted to start after (or just before) the start edge
  uint16_t start = activityStart, period = framePeriod >> 4;
  while ((int16_t)(start - txEdge) <= -(int16_t)bandBefore)
    start += period;
  // Start later until safe. Beyond 128 ticks (OCR1A's reach), send an idle
  // bit and plan again; a byte that fits nowhere goes after 10 idle bits.
  static uint8_t idle;
  uint8_t total = 0;
  for (uint8_t shift; (shift = shiftNeeded(frame, start - txEdge - total)); total += shift)
    if (total + shift > 128) {
      if (++idle < 10)
        return false;
      total = 0;
      break;
    }
  idle = 0;
  if (total) {
    txEdge += total;
    shifted++;
  }
  return true;
}

// Schedule the next edge after the one on the line. Runs with Timer1's
// compare interrupt masked and interrupts on; returns with interrupts off.
static void txSchedule()
{
  for (;;) {
    uint16_t frame = txFrame >> 1;  // bit 0: the bit that starts at the next edge
    txAdvance();
    if (frame == 1) {               // the next edge ends a stop bit
      if (txTail != txHead) {
        frame = 0x600 | (txBuf[txTail] << 1);  // start bit, 8 data bits, stop bit, end marker
        if (txPlan(frame)) {
          txTail = (txTail + 1) & (TX_SIZE - 1);
          txIdle = false;
        } else {
          frame = 3;                // an idle bit
          txIdle = true;
        }
      } else if (!txIdle) {
        frame = 3;                  // a bit of idle line, so the stop bit ends before a later start
        txIdle = true;
        txIdleBits = 0;
      } else if (++txIdleBits < LINGER_BITS) {
        frame = 3;                  // keep running on idle bits, watching USB activity for the next burst
      } else {
        cli();
        txActive = false;
        return;
      }
    }
    txFrame = frame;

    cli();
    TCCR1 = (TCCR1 & ~COM1A_MASK) | (frame & 1 ? COM1A_SET : COM1A_CLR);
    OCR1A = txEdge;
    uint16_t now = ticksNow();
    if ((int16_t)(txEdge - now) < 2) {  // too late for a match at its time
      if (frame >= 0x400) {             // a start bit: start the byte later instead
        txEdge = now + 2;
        txEdgeFrac = 0;
      } else {
        forcedEdges++;
        txMoved = true;
      }
      // Make the edge 2 ticks from now. (Forcing a match with FOC1A right
      // after changing COM1A made no edge: the output kept its level.)
      OCR1A = now + 2;
    }
    TIFR = _BV(OCF1A);  // an old match would call the handler early
    TIMSK |= _BV(OCIE1A);
    return;
  }
}

ISR(TIMER1_COMPA_vect)  // an edge was made
{
  TIMSK &= ~_BV(OCIE1A);  // V-USB must never wait for this handler
  uint16_t now = ticksNow();
  sei();
  int16_t late = now - txEdge;
  if (!txMoved && late >= SEEN_TICKS && late < 128) {
    activitySeen(now, late);
  }
  txMoved = false;
  txSchedule();
}

static void txStart()
{
  txActive = true;
  txFrame = 2;  // "a stop bit on the line": the next edge starts a byte
  txIdle = true;
  probeBits = PROBE_BITS;
  // The clock may have missed Timer1 overflows while idle: set it from the
  // core's count of them, so what was seen of USB activity stays usable.
  // (In loop() that count is up to date but for a pending overflow.)
  uint8_t t = TCNT1;
  clockNow = ((uint16_t)(uint8_t)millis_timer_overflow_count << 8) | t;
  if ((TIFR & _BV(TOV1)) && t < 128)
    clockNow += 256;
  txEdge = clockNow;  // txPlan() moves it to now
  txEdgeFrac = 0;
  TIFR = _BV(OCF1A);
  sei();
  txSchedule();
  sei();
}

static void uartBegin(unsigned long baud)
{
  unsigned long rate = baud * SAMPLES_PER_BIT;
  unsigned long ticks = (F_CPU / 8 + rate / 2) / rate;
  prescaler = _BV(CS01);                  // clk/8
  if (ticks > 256) {
    ticks = (F_CPU / 64 + rate / 2) / rate;
    prescaler = _BV(CS01) | _BV(CS00);    // clk/64
  }
  cli();
  TIMSK &= ~_BV(OCIE1A);  // abandon a byte being sent
  txActive = false;
  txHead = txTail = 0;
  TCCR1 = (TCCR1 & ~COM1A_MASK) | COM1A_SET;
  GTCCR |= _BV(FOC1A);    // idle line
  bitCycles = (F_CPU + baud / 2) / baud;
  bitTicks = bitCycles >> 6;
  bitFrac = bitCycles & 63;
  for (uint8_t k = 0; k < 10; k++)
    edgeTicks[k] = (k * (unsigned long)bitCycles) >> 6;
  bandBefore = WINDOW_TICKS + HANDLER_TICKS > bitTicks ? WINDOW_TICKS + HANDLER_TICKS - bitTicks : 0;
  USICR = 0;
  TCCR0B = 0;
  TCCR0A = _BV(WGM01);  // CTC: period is OCR0A + 1
  OCR0A = ticks - 1;
  TCNT0 = 0;
  captureHead = captureTail = 0;
  rxHead = rxTail = 0;
  USISR = _BV(USIOIF) | 8;
  USICR = _BV(USICS0) | _BV(USIOIE);
  TCCR0B = prescaler;
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
        received++;
        if (next != rxTail) {  // (loop() forwards faster than bytes arrive)
          rxBuf[rxHead] = rxByte;
          rxHead = next;
        }
      } else
        framingErrors++;
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
  uint8_t o = captureOverflows;
  SerialUSB.write('S');
  writeHex('n', received);
  writeHex('g', gaps);
  writeHex('o', o);
  writeHex('f', framingErrors);
  writeHex('e', forcedEdges);
  writeHex('a', seen);
  writeHex('h', shifted);
  writeHex('p', probed);
  SerialUSB.write('\r');
  SerialUSB.write('\n');
  // Printing kept loop() busy: drop the samples it couldn't decode meanwhile
  cli();
  captureTail = captureHead;
  captureOverflows = 0;
  framePos = -1;
  lastSample = false;
  received = gaps = framingErrors = 0;
  forcedEdges = seen = shifted = probed = 0;
  sei();
}

void setup()
{
  pinMode(PB0, INPUT);
  digitalWrite(PB1, HIGH);
  pinMode(PB1, OUTPUT);
  // Timer1: normal mode, same clock and period for millis(), OC1A sets PB1
  cli();
  TCCR1 = (TCCR1 & 0x0F) | COM1A_SET;
  GTCCR = (GTCCR & ~(_BV(PWM1B) | _BV(COM1B1) | _BV(COM1B0))) | _BV(FOC1A);
  sei();
  SerialUSB.begin();
}

void loop()
{
  static unsigned long lineBaud, uartBaud;
  unsigned long baud = SerialUSB.baud();
  if (baud != lineBaud) {
    lineBaud = baud;
    if (baud == BOOTLOADER_BAUD)
      enterBootloader();
    else if (baud == STATS_BAUD)
      printStats();
    else if (baud != uartBaud) {
      uartBegin(baud);
      uartBaud = baud;
    }
  }

  while (captureTail != captureHead) {
    uint8_t samples = captured[captureTail];
    if (capturedFlags[captureTail] & GAP) {
      gaps++;
      framePos = -1;         // the byte in progress is incomplete
      lastSample = false;    // wait for the line to be seen idle again
    }
    captureTail = (captureTail + 1) & (CAPTURE_SIZE - 1);
    for (uint8_t mask = 0x80; mask; mask >>= 1)
      decode(samples & mask);
  }

  // Received bytes go to the host 8 at a time, or once 4 ms old: each packet
  // keeps V-USB busy, which the transmitter has to plan around
  static unsigned long rxSince;
  if (rxTail == rxHead)
    rxSince = millis();
  else if (((rxHead - rxTail) & (RX_SIZE - 1)) >= 8 || millis() - rxSince >= 4)
    for (int room = SerialUSB.availableForWrite(); room > 0 && rxTail != rxHead; room--) {
      SerialUSB.write(rxBuf[rxTail]);
      rxTail = (rxTail + 1) & (RX_SIZE - 1);
    }

  while (SerialUSB.available()) {
    uint8_t next = (txHead + 1) & (TX_SIZE - 1);
    if (next == txTail)
      break;
    txBuf[txHead] = SerialUSB.read();
    txHead = next;
  }
  if (!txActive && txHead != txTail) {
    cli();
    txStart();
  }
}
