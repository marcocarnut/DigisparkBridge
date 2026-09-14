/*
  TinyBridgeUsi1x -- UART receive -> USB on the Digispark (ATtiny85), the
  classic way: INT0 catches the start bit, then USI samples each bit once,
  clocked by Timer0. A baseline for comparison with oversampled USI.

  Wiring: UART RX to PB0 (USI DI) and PB2 (INT0), GND. Receive only.
  Bit rate: the rate the host sets for the USB port (stty), 8N1, up to about
  38400 bps. 134 bps jumps to the micronucleus bootloader.

  The weak point is the start bit: INT0's handler can run late (V-USB keeps
  interrupts off for up to ~200 us while it handles USB traffic), and every
  sample of the byte is then late by the same amount. More than half a bit
  corrupts the byte.
*/

#include <DigiCDCFast.h>

#define BOOTLOADER_BAUD 134
#define RX_SIZE         64  // power of 2

static uint8_t rxBuf[RX_SIZE];
static volatile uint8_t rxHead;
static uint8_t rxTail;
static uint8_t bitTicks;   // Timer0 compare value (period - 1)
static uint8_t prescaler;  // TCCR0B clock select

ISR(INT0_vect)  // start bit
{
  GIMSK &= ~_BV(INT0);
  TCCR0B = 0;
  TCNT0 = bitTicks / 2;     // first sample in the middle of the start bit
  TIFR = _BV(OCF0A);
  USISR = _BV(USIOIF) | 7;  // overflow after 9 samples: start bit + 8 data bits
  USICR = _BV(USICS0) | _BV(USIOIE);  // clock USI from Timer0 compare match
  TCCR0B = prescaler;
}

ISR(USI_OVF_vect)  // 8 data bits sampled
{
  USICR = 0;
  TCCR0B = 0;
  uint8_t next = (rxHead + 1) & (RX_SIZE - 1);
  if (next != rxTail) {
    rxBuf[rxHead] = USIBR;  // bit 0 was sampled first, so it is now bit 7
    rxHead = next;
  }
  GIFR = _BV(INTF0);        // drop edges seen during the data bits
  GIMSK |= _BV(INT0);
}

static void uartBegin(unsigned long baud)
{
  // Timer0 period in ticks of F_CPU/8, or F_CPU/64 when that doesn't fit 8 bits
  unsigned long ticks = (F_CPU / 8 + baud / 2) / baud;
  prescaler = _BV(CS01);                 // clk/8
  if (ticks > 256) {
    ticks = (F_CPU / 64 + baud / 2) / baud;
    prescaler = _BV(CS01) | _BV(CS00);   // clk/64
  }
  cli();
  GIMSK &= ~_BV(INT0);
  USICR = 0;
  TCCR0B = 0;
  TCCR0A = _BV(WGM01);  // CTC: period is OCR0A + 1
  bitTicks = ticks - 1;
  OCR0A = bitTicks;
  MCUCR = (MCUCR & ~(_BV(ISC01) | _BV(ISC00))) | _BV(ISC01);  // INT0 on falling edge
  rxHead = rxTail = 0;
  GIFR = _BV(INTF0);
  GIMSK |= _BV(INT0);
  sei();
}

static void enterBootloader()
{
  SerialUSB.delay(100);  // let the host's SET_LINE_CODING request complete
  cli();
  GIMSK &= ~_BV(INT0);   // no sketch interrupts may fire once the bootloader runs
  USICR = 0;
  TIMSK = 0;
  TCCR0B = 0;
  TCCR1 = 0;
  ((void (*)())0)();     // micronucleus points the reset vector at itself
}

void setup()
{
  pinMode(PB0, INPUT);
  pinMode(PB2, INPUT_PULLUP);
  SerialUSB.begin();
}

void loop()
{
  static unsigned long currentBaud;
  unsigned long baud = SerialUSB.baud();
  if (baud != currentBaud) {
    if (baud == BOOTLOADER_BAUD)
      enterBootloader();
    uartBegin(baud);
    currentBaud = baud;
  }

  for (int room = SerialUSB.availableForWrite(); room > 0 && rxTail != rxHead; room--) {
    uint8_t sampled = rxBuf[rxTail], c = 0;
    for (uint8_t i = 0; i < 8; i++, sampled >>= 1)
      c = (c << 1) | (sampled & 1);
    SerialUSB.write(c);
    rxTail = (rxTail + 1) & (RX_SIZE - 1);
  }

  while (SerialUSB.available())  // receive only: discard what the host sends
    SerialUSB.read();
}
