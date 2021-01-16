// tinyIRremote for ATtiny13A - Multi Protocol, 4 Buttons
// 
// IR remote control using an ATtiny 13A. Timer0 generates a carrier
// frequency with a duty cycle of 25% on the output pin to the
// IR LED. The signal is modulated by toggling the pin to input/output.
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
//#define OSCCAL_VAL  0x48

// libraries
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// assign IR commands to the keys
#define KEY1  NEC_sendCode(0x04,0x08)     // LG TV Power: addr 0x04, cmd 0x08
#define KEY2  RC5_sendCode(0x00,0x0B)     // Philips TV Power: addr 0x00, cmd 0x0B
#define KEY3  SON_sendCode(0x01,0x15,12)  // Sony TV Power: addr 0x01, cmd 0x15, 12-bit version
#define KEY4  SAM_sendCode(0x07,0x02)     // Samsung TV Power: addr: 07, cmd: 02
#define KEY5  _delay_ms(10)               // nothing

// macros to switch on/off IR LED
#define IRon()    DDRB |= 0b00000010      // PB1 as output = IR at OC0B (38 kHz)
#define IRoff()   DDRB &= 0b11111101      // PB1 as input  = LED off

// button mask
#define BT_MASK   0b00011101              // port mask for button pins

// -----------------------------------------------------------------------------------
// NEC Protocol Implementation
// -----------------------------------------------------------------------------------
//
// The NEC protocol uses pulse distance modulation.
//
//       +---------+     +-+ +-+   +-+   +-+ +-    ON
//       |         |     | | | |   | |   | | |          bit0:  562.5us
//       |   9ms   |4.5ms| |0| | 1 | | 1 | |0| ...
//       |         |     | | | |   | |   | | |          bit1: 1687.5us
// ------+         +-----+ +-+ +---+ +---+ +-+     OFF
//
// IR telegram starts with a 9ms leading burst followed by a 4.5ms pause.
// Afterwards 4 data bytes are transmitted, least significant bit first.
// A "0" bit is a 562.5us burst followed by a 562.5us pause, a "1" bit is
// a 562.5us burst followed by a 1687.5us pause. A final 562.5us burst
// signifies the end of the transmission. The four data bytes are in order:
// - the 8-bit address for the receiving device,
// - the 8-bit logical inverse of the address,
// - the 8-bit command and
// - the 8-bit logical inverse of the command.
// The Extended NEC protocol uses 16-bit addresses. Instead of sending an
// 8-bit address and its logically inverse, first the low byte and then the
// high byte of the address is transmitted.
//
// If the key on the remote controller is kept depressed, a repeat code
// will be issued consisting of a 9ms leading burst, a 2.25ms pause and
// a 562.5us burst to mark the end. The repeat code will continue to be
// sent out at 108ms intervals, until the key is finally released.

// define values for 38kHz PWM frequency and 25% duty cycle
#define NEC_TOP   31          // 1200kHz / 38kHz - 1 = 31
#define NEC_DUTY  7           // 1200kHz / 38kHz / 4 - 1 = 7

// macros to modulate the signals according to NEC protocol with compensated timings
#define NEC_startPulse()    {IRon(); _delay_us(9000); IRoff(); _delay_us(4500);}
#define NEC_repeatPulse()   {IRon(); _delay_us(9000); IRoff(); _delay_us(2250);}
#define NEC_normalPulse()   {IRon(); _delay_us( 562); IRoff(); _delay_us( 557);}
#define NEC_bit1Pause()     _delay_us(1120) // 1687.5us - 562.5us = 1125us
#define NEC_repeatCode()    {_delay_ms(40); NEC_repeatPulse(); NEC_normalPulse(); _delay_ms(56);}

// send a single byte via IR
void NEC_sendByte(uint8_t value) {
  for (uint8_t i=8; i; i--, value>>=1) {  // send 8 bits, LSB first
    NEC_normalPulse();                    // 562us burst, 562us pause
    if (value & 1) NEC_bit1Pause();       // extend pause if bit is 1
  }
}

// send complete telegram (start frame + address + command) via IR
void NEC_sendCode(uint16_t addr, uint8_t cmd) {
  // prepare carrier wave
  OCR0A  = NEC_TOP;           // set PWM frequency
  OCR0B  = NEC_DUTY;          // set duty cycle

  // send telegram
  NEC_startPulse();           // 9ms burst + 4.5ms pause to signify start of transmission
  if (addr > 0xFF) {          // if extended NEC protocol (16-bit address):
    NEC_sendByte(addr);       // send address low byte
    NEC_sendByte(addr >> 8);  // send address high byte
  } else {                    // if standard NEC protocol (8-bit address):
    NEC_sendByte(addr);       // send address byte
    NEC_sendByte(~addr);      // send inverse of address byte
  }
  NEC_sendByte(cmd);          // send command byte
  NEC_sendByte(~cmd);         // send inverse of command byte
  NEC_normalPulse();          // 562us burst to signify end of transmission
  while(~PINB & BT_MASK) NEC_repeatCode();  // send repeat command until button is released
}

// -----------------------------------------------------------------------------------
// SAMSUNG Protocol Implementation
// -----------------------------------------------------------------------------------
//
// The SAMSUNG protocol corresponds to the NEC protocol, except that the start pulse is
// 4.5ms long and the address byte is sent twice. The telegram is repeated every 108ms
// as long as the button is pressed.

#define SAM_startPulse()    {IRon(); _delay_us(4500); IRoff(); _delay_us(4500);}
#define SAM_repeatPause()   _delay_ms(44)

// send complete telegram (start frame + address + command) via IR
void SAM_sendCode(uint8_t addr, uint8_t cmd) {
  // prepare carrier wave
  OCR0A  = NEC_TOP;           // set PWM frequency
  OCR0B  = NEC_DUTY;          // set duty cycle

  // send telegram
  do {
    SAM_startPulse();           // 9ms burst + 4.5ms pause to signify start of transmission
    NEC_sendByte(addr);         // send address byte
    NEC_sendByte(addr);         // send address byte again
    NEC_sendByte(cmd);          // send command byte
    NEC_sendByte(~cmd);         // send inverse of command byte
    NEC_normalPulse();          // 562us burst to signify end of transmission
    SAM_repeatPause();          // wait for next repeat
  } while(~PINB & BT_MASK);     // repeat sending until button is released
}

// -----------------------------------------------------------------------------------
// RC-5 Protocol Implementation
// -----------------------------------------------------------------------------------
//
// The RC-5 protocol uses bi-phase modulation (Manchester coding).
//
//   +-------+                     +-------+    ON
//           |                     |
//     889us | 889us         889us | 889us
//           |                     |
//           +-------+     +-------+            OFF
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

// define values for 36kHz PWM frequency and 25% duty cycle
#define RC5_TOP   32          // 1200kHz / 36kHz - 1 = 32
#define RC5_DUTY  7           // 1200kHz / 36kHz / 4 - 1 = 7

// macros to modulate the signals according to RC-5 protocol with compensated timings
#define RC5_bit0Pulse()     {IRon();  _delay_us(889); IRoff(); _delay_us(884);}
#define RC5_bit1Pulse()     {IRoff(); _delay_us(889); IRon();  _delay_us(884);}
#define RC5_repeatPause()   _delay_ms(89) // 114ms - 14 * 2 * 889us

// bitmasks
#define RC5_startBit  0b0010000000000000
#define RC5_cmdBit7   0b0001000000000000
#define RC5_toggleBit 0b0000100000000000

// toggle variable
uint8_t RC5_toggle = 0;

// send complete telegram (startbits + togglebit + address + command) via IR
void RC5_sendCode(uint8_t addr, uint8_t cmd) {
  // prepare carrier wave
  OCR0A  = RC5_TOP;                           // set PWM frequency
  OCR0B  = RC5_DUTY;                          // set duty cycle

  // prepare the message
  uint16_t message = addr << 6;               // shift address to the right position
  message |= (cmd & 0x3f);                    // add the low 6 bits of the command
  if (~cmd & 0x40) message |= RC5_cmdBit7;    // add inverse of 7th command bit
  message |= RC5_startBit;                    // add start bit
  if (RC5_toggle) message |= RC5_toggleBit;   // add toggle bit

  // send the message
  do {
    uint16_t bitmask = RC5_startBit;          // set the bitmask to first bit to send
    for(uint8_t i=14; i; i--, bitmask>>=1) {  // 14 bits, MSB first
      (message & bitmask) ? (RC5_bit1Pulse()) : (RC5_bit0Pulse());  // send the bit
    }
    IRoff();                                  // switch off IR LED
    RC5_repeatPause();                        // wait for next repeat
  } while(~PINB & BT_MASK);                   // repeat sending until button is released
  RC5_toggle ^= 1;                            // toggle the toggle bit
}

// -----------------------------------------------------------------------------------
// SONY SIRC Protocol Implementation
// -----------------------------------------------------------------------------------
//
// The SONY SIRC protocol uses pulse length modulation.
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

// define values for 40kHz PWM frequency and 25% duty cycle
#define SON_TOP   29                      // 1200kHz / 40kHz - 1 = 29
#define SON_DUTY  7                       // 1200kHz / 40kHz / 4 - 1 = 7

// macros to modulate the signals according to SONY protocol with compensated timings
#define SON_startPulse()    {IRon(); _delay_us(2400); IRoff(); _delay_us( 595);}
#define SON_bit0Pulse()     {IRon(); _delay_us( 600); IRoff(); _delay_us( 595);}
#define SON_bit1Pulse()     {IRon(); _delay_us(1200); IRoff(); _delay_us( 595);}
#define SON_repeatPause()   _delay_ms(27)

// send "number" of bits of "value" via IR
void SON_sendByte(uint8_t value, uint8_t number) {
  do {                                        // send number of bits, LSB first
    (value & 1) ? (SON_bit1Pulse()) : (SON_bit0Pulse());  // send bit
    value>>=1;                                // next bit
  } while(--number);
}

// send complete telegram (start frame + command + address) via IR
void SON_sendCode(uint16_t addr, uint8_t cmd, uint8_t bits) {
  // prepare carrier wave
  OCR0A  = SON_TOP;                           // set PWM frequency
  OCR0B  = SON_DUTY;                          // set duty cycle

  // send telegram
  do {
    SON_startPulse();                         // signify start of transmission
    SON_sendByte(cmd, 7);                     // send 7 command bits
    switch (bits) {
      case 12: SON_sendByte(addr, 5); break;  // 12-bit version: send 5 address bits
      case 15: SON_sendByte(addr, 8); break;  // 15-bit version: send 8 address bits
      case 20: SON_sendByte(addr, 8); SON_sendByte(addr>>8, 5); break; // 20-bit: 13 bits
      default: break;
    }
    SON_repeatPause();                        // wait until next repeat
  } while (~PINB & BT_MASK);                  // repeat sending until button is released
}

// -----------------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------------

// main function
int main(void) {
  // set oscillator calibration value
  #ifdef OSCCAL_VAL
    OSCCAL = OSCCAL_VAL;                // set the value if defined above
  #endif

  // setup pins
  DDRB  = 0b00000000;                   // all pins are input pins by now
  PORTB = BT_MASK;                      // pull-up for button pins
  
  // set timer0 to toggle IR pin at 38 kHz
  TCCR0A = 0b00100011;                  // PWM on OC0B (PB1)
  TCCR0B = 0b00001001;                  // no prescaler

  // setup pin change interrupt
  GIMSK = 0b00100000;                   // turn on pin change interrupts
  PCMSK = BT_MASK;                      // turn on interrupt on button pins
  SREG |= 0b10000000;                   // enable global interrupts

  // disable unused peripherals and set sleep mode to save power
  ADCSRA = 0b00000000;                  // disable ADC
  ACSR   = 0b10000000;                  // disable analog comperator
  PRR    = 0b00000001;                  // shut down ADC
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode to power down

  // main loop
  while(1) {
    sleep_mode();                       // sleep until button is pressed
    _delay_ms(1);                       // debounce
    uint8_t buttons = ~PINB & BT_MASK;  // read button pins
    switch (buttons) {                  // send corresponding IR code
      case 0b00000001: KEY1; break;
      case 0b00000100: KEY2; break;
      case 0b00001000: KEY3; break;
      case 0b00010000: KEY4; break;
      case 0b00100000: KEY5; break;
      default: break;
    }
  }
}

// pin change interrupt service routine
EMPTY_INTERRUPT (PCINT0_vect);                // nothing to be done here, just wake up from sleep
