// tinyIRremote for ATtiny13A - SONY SIRC, 4 Buttons
// 
// IR remote control using an ATtiny 13A. Timer0 generates a 40kHz
// carrier frequency with a duty cycle of 25% on the output pin to the
// IR LED. The message (SONY protocol) is modulated by toggling the pin
// to input/output. The protocol uses pulse length modulation.
//
//       +--------------------+     +-----+     +----------+     +-- ON
//       |                    |     |     |     |          |     |
//       |       2400us       |600us|600us|600us|  1200us  |600us|   ...
//       |                    |     |     |     |          |     |
// ------+                    +-----+     +-----+          +-----+   OFF
//
//       |<------ Start Frame ----->|<- Bit=0 ->|<--- Bit=1 ---->| 
//
// A "0" bit is a 600us burst followed by a 600us space, a "1" bit is a
// 1200us burst followed by a 600us space. An IR telegram starts with a
// 2400us leading burst followed by a 600us space. The command and
// address bits are then transmitted, LSB first. Depending on the
// protocol version, these are in detail:
// - 12-bit version: 7 command bits, 5 address bits
// - 15-bit version: 7 command bits, 8 address bits
// - 20-bit version: 7 command bits, 5 address bits, 8 extended bits
//
// As long as a key remains down the message will be repeated every 45ms.
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
// Controller: ATtiny13
// Core:       MicroCore (https://github.com/MCUdude/MicroCore)
// Clockspeed: 1.2 MHz internal
// BOD:        BOD disabled (energy saving)
// Timing:     Micros disabled (Timer0 is in use)
//
// Note: The internal oscillator may need to be calibrated for the device
//       to function properly.
//
// 2019 by Stefan Wagner 
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
#define BITS  12    // SIRC version: 12, 15 or 20 bits
#define EXTB  0x00  // Extended byte in 20 bits version
#define ADDR  0x01  // Address: SONY TV
#define KEY1  0x12  // Command: Volume+
#define KEY2  0x10  // Command: Channel+
#define KEY3  0x13  // Command: Volume-
#define KEY4  0x11  // Command: Channel-
//#define KEY5  0x15  // Command: Power

// define values for 40kHz PWM frequency and 25% duty cycle
#define TOP   29                      // 1200kHz / 40kHz - 1 = 29
#define DUTY  7                       // 1200kHz / 40kHz / 4 - 1 = 7

// macros to switch on/off IR LED
#define IRon()   DDRB |= 0b00000010   // PB1 as output = IR at OC0B (40kHz)
#define IRoff()  DDRB &= 0b11111101   // PB1 as input  = LED off

// macros to modulate the signals according to SONY protocol with compensated timings
#define startPulse()    {IRon(); _delay_us(2400); IRoff(); _delay_us( 595);}
#define bit0Pulse()     {IRon(); _delay_us( 600); IRoff(); _delay_us( 595);}
#define bit1Pulse()     {IRon(); _delay_us(1200); IRoff(); _delay_us( 595);}
#define repeatPause()   _delay_ms(27)


// send "number" of bits of "value" via IR
void sendByte(uint8_t value, uint8_t number) {
  do {                                    // send number of bits, LSB first
    (value & 1) ? (bit1Pulse()) : (bit0Pulse());  // send bit
    value>>=1;                            // next bit
  } while(--number);
}

// send complete telegram (start frame + command + address) via IR
void sendCode(uint8_t cmd) {
  do {
    startPulse();               // signify start of transmission
    sendByte(cmd, 7);           // send 7 command bits
    #if BITS == 12              // if 12-bit version:
      sendByte(ADDR, 5);        // send 5 address bits
    #elif BITS == 15            // if 15-bit version:
      sendByte(ADDR, 8);        // send 8 address bits
    #elif BITS == 20            // if 20-bit version:
      sendByte(ADDR, 5);        // send 5 address bits
      sendByte(EXTB, 8);        // send 8 extended bits
    #endif
    repeatPause();              // wait until next repeat
  } while (~PINB & 0b00011101); // repeat sending until button is released
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
