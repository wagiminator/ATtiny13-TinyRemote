// tinyIRremote for ATtiny13A - RC-5, 4 Buttons
// 
// IR remote control using an ATtiny 13A. Timer0 generates a 36kHz
// carrier frequency with a duty cycle of 25% on the output pin to the
// IR LED. The signal (RC-5 protocol) is modulated by toggling the pin
// to input/output. The protocol uses bi-phase modulation (Manchester
// coding).
//
//   +-------+                     +-------+
//           |                     |
//     889us | 889us         889us | 889us
//           |                     |
//           +-------+     +-------+
//
//   |<-- Bit "0" -->|     |<-- Bit "1" -->|
//
// IR telegram starts with two start bits. The first bit is always "1",
// the second bit is "1" in the original protocol and inverted 7th bit
// of the command in the extended RC-5 protocol. The third bit toggles
// after each button release. The next five bits represent the device
// address, MSB first and the last six bits represent the command, MSB
// first.
//
// As long as a key remains down the telegram will be repeated every
// 114ms without changing the toggle bit.
//
// The code utilizes the sleep mode power down function. The device will
// work several months on a CR2032 battery.
//
//                        +-\/-+
// KEY5 --- A0 (D5) PB5  1|    |8  Vcc
// KEY3 --- A3 (D3) PB3  2|    |7  PB2 (D2) A1 --- KEY2
// KEY4 --- A2 (D4) PB4  3|    |6  PB1 (D1) ------ IR LED
//                  GND  4|    |5  PB0 (D0) ------ KEY1
//                        +----+    
//
// Controller: ATtiny13A
// Clockspeed: 1.2 MHz internal
//
// Note: The internal oscillator may need to be calibrated for the device
//       to function properly.
//
// 2020 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/
//
// Based on the work of:
// - San Bergmans (https://www.sbprojects.net/knowledge/ir/index.php),
// - Christoph Niessen (http://chris.cnie.de/avr/tcm231421.html),
// - David Johnson-Davies (http://www.technoblogy.com/show?UVE).


// oscillator calibration value (uncomment and set if necessary)
//#define OSCCAL_VAL  48

// libraries
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// IR codes
#define ADDR  0x00  // Address: Philips TV
#define KEY1  0x0D  // Command: Volume+
#define KEY2  0x11  // Command: Channel+
#define KEY3  0x0E  // Command: Volume-
#define KEY4  0x12  // Command: Channel-

// define values for 36kHz PWM frequency and 25% duty cycle
#define TOP   32                      // 1200kHz / 36kHz - 1 = 32
#define DUTY  7                       // 1200kHz / 36kHz / 4 - 1 = 7

// macros to switch on/off IR LED
#define IRon()   DDRB |= 0b00000010   // PB1 as output = IR at OC0B (36 kHz)
#define IRoff()  DDRB &= 0b11111101   // PB1 as input  = LED off

// macros to modulate the signals according to RC-5 protocol with compensated timings
#define bit0Pulse()     {IRon();  _delay_us(889); IRoff(); _delay_us(884);}
#define bit1Pulse()     {IRoff(); _delay_us(889); IRon();  _delay_us(884);}
#define repeatDelay()   _delay_ms(89) // 114ms - 14 * 2 * 889us

// bitmasks
#define startBit  0b0010000000000000
#define cmdBit7   0b0001000000000000
#define toggleBit 0b0000100000000000

// toggle variable
uint8_t toggle = 0;

// send complete telegram (startbits + togglebit + address + command) via IR
void sendCode(uint8_t cmd) {
  // prepare the message
  uint16_t message = ADDR << 6;         // shift address to the right position
  message |= (cmd & 0x3f);              // add the low 6 bits of the command
  if (~cmd & 0x40) message |= cmdBit7;  // add inverse of 7th command bit
  message |= startBit;                  // add start bit
  if (toggle) message |= toggleBit;     // add toggle bit

  // send the message
  do {
    uint16_t bitmask = startBit;        // set the bitmask to first bit to send
    for(uint8_t i=14; i; i--, bitmask>>=1) {                // 14 bits, MSB first
      (message & bitmask) ? (bit1Pulse()) : (bit0Pulse());  // send the bit
    }
    IRoff();                            // switch off IR LED
    repeatDelay();                      // wait for next repeat
  } while(~PINB & 0b00011101);          // repeat sending until button is released
  toggle ^= 1;                          // toggle the toggle bit
}

// main function
int main(void) {
  // set oscillator calibration value
  #ifdef OSCCAL_VAL
    OSCCAL = OSCCAL_VAL;                // set the value if defined above
  #endif

  // setup pins
  DDRB  = 0b00000000;                   // all pins are input pins by now
  PORTB = 0b00011101;                   // pull-up for button pins
  
  // set timer0 to toggle IR pin at 38 kHz
  TCCR0A = 0b00100011;                  // PWM on OC0B (PB1)
  TCCR0B = 0b00001001;                  // no prescaler
  OCR0A  = TOP;                         // 38 kHz PWM frequency
  OCR0B  = DUTY;                        // 25 % duty cycle

  // setup pin change interrupt
  GIMSK = 0b00100000;                   // turn on pin change interrupts
  PCMSK = 0b00011101;                   // turn on interrupt on button pins
  SREG |= 0b10000000;                   // enable global interrupts

  // disable unused peripherals and set sleep mode to save power
  ADCSRA = 0b00000000;                  // disable ADC
  ACSR   = 0b10000000;                  // disable analog comperator
  PRR    = 0b00000001;                  // shut down ADC
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode to power down

  // main loop
  while(1) {
    sleep_mode();                             // sleep until button is pressed
    _delay_ms(1);                             // debounce
    uint8_t buttons = ~PINB & 0b00011101;     // read button pins
    switch (buttons) {                        // send corresponding IR code
      case 0b00000001: sendCode(KEY1); break;
      case 0b00000100: sendCode(KEY2); break;
      case 0b00001000: sendCode(KEY3); break;
      case 0b00010000: sendCode(KEY4); break;
      default: break;
    }
  }
}

// pin change interrupt service routine
EMPTY_INTERRUPT (PCINT0_vect);                // nothing to be done here, just wake up from sleep
