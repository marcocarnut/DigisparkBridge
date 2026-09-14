/*
  TinyBridgeUsi3x -- UART receive -> USB on the Digispark (ATtiny85) with a
  free-running, 3x oversampling USI receiver.

  Wiring: UART RX to PB0 (USI DI), GND. Receive only (PB2 is not used).
  Bit rate: the rate the host sets for the USB port (stty), 8N1, 1200 to
  19200 bps (lossless on the bench, even with the host flooding the USB
  port). 134 bps jumps to the micronucleus bootloader; 110 bps prints the
  diagnostic counters as hex and clears them, leaving the UART as it was.

  Timer0 clocks USI three times per bit, so every sample is taken on time in
  hardware, whatever the interrupts are doing. USI's counter overflows every
  8 samples and USIBR then holds those 8 samples; the handler only has to
  collect them before the next overflow (2 2/3 bits: 139 us at 19200 bps)
  and keep the overflows exactly 8 samples apart. loop() finds start bits in
  the sample stream and reads each bit near its middle.

  Latest measurements of V-USB delaying other interrupts: up to ~200 us.
*/

#include <DigiCDCFast.h>

#define BOOTLOADER_BAUD 134
#define STATS_BAUD      110  // diagnostics: print and clear the counters
#define SAMPLES_PER_BIT 3
#define CAPTURE_SIZE    16  // powers of 2
#define RX_SIZE         64
#define GAP             1   // capture flag: samples were lost before these

// Sample bytes from USI (oldest sample in bit 7) and their flags
static uint8_t captured[CAPTURE_SIZE], capturedFlags[CAPTURE_SIZE];
static volatile uint8_t captureHead;
static uint8_t captureTail;

static uint8_t rxBuf[RX_SIZE];
static uint8_t rxHead, rxTail;
static uint8_t prescaler;  // TCCR0B clock select

// Diagnostic counters
static volatile uint8_t captureOverflows;  // updated by the handler
static uint16_t gaps, glitches, framingErrors, rxOverflows, received;

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
  cli();
  uint8_t o = captureOverflows;
  captureOverflows = 0;
  sei();
  SerialUSB.write('S');
  writeHex('n', received);
  writeHex('g', gaps);
  writeHex('o', o);
  writeHex('l', glitches);
  writeHex('f', framingErrors);
  writeHex('r', rxOverflows);
  SerialUSB.write('\r');
  SerialUSB.write('\n');
  received = gaps = glitches = framingErrors = rxOverflows = 0;
}

void setup()
{
  pinMode(PB0, INPUT);
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

  while (SerialUSB.available())  // receive only: discard what the host sends
    SerialUSB.read();
}
