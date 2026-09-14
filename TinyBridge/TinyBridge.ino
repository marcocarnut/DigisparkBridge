/*
  TinyBridge -- USB-UART bridge on the Digispark (ATtiny85): a 3x oversampling
  USI receiver (TinyBridgeUsi3x) and a transmitter whose bit edges are timed
  by Timer1's compare output.

  Wiring: UART RX to PB0 (USI DI), UART TX from PB1 (OC1A; the LED on PB1
  doesn't matter), GND.
  Bit rate: the rate the host sets for the USB port (stty), 8N1, 1200 to
  19200 bps. 134 bps jumps to the micronucleus bootloader; 110 bps prints the
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
  An edge it misses is made as soon as the handler runs; a little late is
  harmless, as the receiver samples near the middle of each bit.

  Latest measurements of V-USB delaying other interrupts: up to ~200 us.
*/

#include <DigiCDCFast.h>

#define BOOTLOADER_BAUD 134
#define STATS_BAUD      110  // diagnostics: print and clear the counters
#define SAMPLES_PER_BIT 3
#define CAPTURE_SIZE    16  // powers of 2
#define RX_SIZE         32
#define TX_SIZE         32
#define GAP             1   // capture flag: samples were lost before these

// Sample bytes from USI (oldest sample in bit 7) and their flags
static uint8_t captured[CAPTURE_SIZE], capturedFlags[CAPTURE_SIZE];
static volatile uint8_t captureHead;
static uint8_t captureTail;

static uint8_t rxBuf[RX_SIZE];
static uint8_t rxHead, rxTail;
static uint8_t prescaler;  // TCCR0B clock select

// Transmitter: the line and the next edge
static uint8_t txBuf[TX_SIZE];
static uint8_t txHead;                 // written by loop()
static volatile uint8_t txTail;        // written by the handler
static volatile bool txActive;         // an edge is scheduled
static bool txIdle;                    // the bit on the line is idle line after a stop bit
static uint16_t txFrame;               // bit 0: the bit on the line, then the rest of the byte
static uint16_t txEdge;                // CPU cycles, modulo 64 * 256; OCR1A = txEdge >> 6
static uint16_t bitCycles;
static uint8_t bitTicks;               // bitCycles in Timer1 ticks, rounded up
static uint8_t lateTicks;              // edges later than this (1/4 bit) count as late

#define COM1A_MASK (_BV(COM1A1) | _BV(COM1A0))
#define COM1A_SET  (_BV(COM1A1) | _BV(COM1A0))
#define COM1A_CLR  _BV(COM1A1)

// Diagnostic counters
static volatile uint8_t captureOverflows;  // updated by the handler
static uint16_t gaps, glitches, framingErrors, rxOverflows, received;
static uint16_t sent, forcedEdges, lateEdges;  // updated by the transmitter
static uint16_t entryLate[8];  // handler entries by ticks after the match, in steps of 7 (27 µs)

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

// Schedule the next edge after the one on the line. Runs with Timer1's
// compare interrupt masked and interrupts on; returns with interrupts off.
static void txSchedule()
{
  for (;;) {
    uint16_t frame = txFrame >> 1;  // bit 0: the bit that starts at the next edge
    if (frame == 1) {               // the next edge ends a stop bit
      if (txTail != txHead) {
        frame = 0x600 | (txBuf[txTail] << 1);  // start bit, 8 data bits, stop bit, end marker
        txTail = (txTail + 1) & (TX_SIZE - 1);
        txIdle = false;
        sent++;
      } else if (!txIdle) {
        frame = 3;                  // a bit of idle line, so the stop bit ends before a later start
        txIdle = true;
      } else {
        cli();
        txActive = false;
        return;
      }
    }
    txFrame = frame;
    txEdge += bitCycles;
    uint8_t edge = txEdge >> 6;

    cli();
    TCCR1 = (TCCR1 & ~COM1A_MASK) | (frame & 1 ? COM1A_SET : COM1A_CLR);
    OCR1A = edge;
    uint8_t ahead = edge - TCNT1;
    if (ahead >= 2 && ahead <= bitTicks) {  // the hardware will make the edge
      TIFR = _BV(OCF1A);  // a match from an edge made below would call the handler early
      TIMSK |= _BV(OCIE1A);
      return;
    }
    sei();

    if (ahead < 2) {  // due within 2 ticks: wait until it's past
      while ((uint8_t)(edge - TCNT1) < 2)
        ;
    } else {
      forcedEdges++;  // the handler ran too late for the hardware
      if ((uint8_t)(TCNT1 - edge) > lateTicks)
        lateEdges++;
    }
    // Make the edge now in case no match did (set and clear are idempotent)
    GTCCR |= _BV(FOC1A);
  }
}

ISR(TIMER1_COMPA_vect)  // an edge was made
{
  TIMSK &= ~_BV(OCIE1A);  // V-USB must never wait for this handler
  uint8_t late = TCNT1 - OCR1A;
  sei();
  late /= 7;
  entryLate[late < 7 ? late : 7]++;
  txSchedule();
}

static void txStart()
{
  txActive = true;
  txFrame = 2;  // "a stop bit on the line": the next edge starts a byte
  txIdle = true;
  txEdge = ((TCNT1 + 3) << 6) - bitCycles;  // 3 ticks from now
  TIFR = _BV(OCF1A);
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
  bitTicks = (bitCycles >> 6) + 1;
  lateTicks = bitCycles >> 8;
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
    glitches++;
  } else if (framePos == nextSample) {
    if (bitsRead < 8) {
      rxByte = (rxByte >> 1) | (s ? 0x80 : 0);
      bitsRead++;
      nextSample += SAMPLES_PER_BIT;
    } else {
      if (s) {               // stop bit high: keep the byte
        uint8_t next = (rxHead + 1) & (RX_SIZE - 1);
        received++;
        if (next != rxTail) {
          rxBuf[rxHead] = rxByte;
          rxHead = next;
        } else
          rxOverflows++;
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
  writeHex('l', glitches);
  writeHex('f', framingErrors);
  writeHex('r', rxOverflows);
  writeHex('t', sent);
  writeHex('e', forcedEdges);
  writeHex('x', lateEdges);
  for (uint8_t i = 0; i < 8; i++)
    writeHex('0' + i, entryLate[i]);
  SerialUSB.write('\r');
  SerialUSB.write('\n');
  // Printing kept loop() busy: drop the samples it couldn't decode meanwhile
  cli();
  captureTail = captureHead;
  captureOverflows = 0;
  framePos = -1;
  lastSample = false;
  received = gaps = glitches = framingErrors = rxOverflows = 0;
  sent = forcedEdges = lateEdges = 0;
  for (uint8_t i = 0; i < 8; i++)
    entryLate[i] = 0;
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
