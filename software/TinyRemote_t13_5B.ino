// tinyIRremote for ATtiny13 - 5 Buttons
// 
// IR remote control using an ATtiny 13. Timer 0 generates a 38kHz
// pulse frequency with a duty cycle of 25% on the output pin to the IR LED.
// The signal (NEC protocol) is modulated by toggling the pin to input/output.
//
//       +---------+     +-+ +-+   +-+   +-+ +-
//       |         |     | | | |   | |   | | |     bit0:  562.5us
//       |   9ms   |4.5ms| |0| | 1 | | 1 | |0| ...
//       |         |     | | | |   | |   | | |     bit1: 1687.5us
// ------+         +-----+ +-+ +---+ +---+ +-+
//
// IR message starts with a 9ms leading burst followed by a 4.5ms pause.
// Afterwards 4 data bytes are transmitted, least significant bit first.
// A "0" bit is a 562.5us burst followed by a 562.5us pause, a "1" bit is
// a 562.5us burst followed by a 1687.5us pause. A final 562.5us burst
// signifies the end of the transmission. The four data bytes are in order:
// - the 8-bit address for the receiving device,
// - the 8-bit logical inverse of the address,
// - the 8-bit command and
// - the 8-bit logical inverse of the command.
// If the key on the remote controller is kept depressed, a repeat code
// will be issued consisting of a 9ms leading burst, a 2.25ms pause and
// a 562.5us burst to mark the end. The repeat code will continue to be
// sent out at 108ms intervals, until the key is finally released.
//
// The code utilizes the sleep mode power down function. The device will
// work several months on an CR2032 battery.
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
// Reset pin must be disabled by writing respective fuse after uploading the code:
// avrdude -p attiny13 -c usbasp -U lfuse:w:0x2a:m -U hfuse:w:0xfe:m
// Warning: You will need a high voltage fuse resetter to undo this change!
//
// Note: The internal oscillator may need to be calibrated for the device
//       to function properly.
//
// 2019 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/
//
// Based on the work of Christoph Niessen (http://chris.cnie.de/avr/tcm231421.html)
// and David Johnson-Davies (http://www.technoblogy.com/show?UVE)


// oscillator calibration value (uncomment and set if necessary)
//#define OSCCAL_VAL  48

// libraries
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/delay.h>

// IR codes
#define ADDR  0x04  // Address of LG TV
#define KEY1  0x02  // Volume+
#define KEY2  0x00  // Channel+
#define KEY3  0x03  // Volume-
#define KEY4  0x01  // Channel-
#define KEY5  0x08  // Power

// define values for 38kHz PWM frequency and 25% duty cycle
#define TOP   31                      // 1200kHz / 38kHz - 1 = 31
#define DUTY  7                       // 1200kHz / 38kHz / 4 - 1 = 7

// commands to switch on/off IR LED
#define IRon()   DDRB |= 0b00000010   // PB1 as output = IR at OC0B (38 kHz)
#define IRoff()  DDRB &= 0b11111101   // PB1 as input  = LED off

// commands to modulate the signals according to NEC protocol
#define startPulse()    {IRon(); _delay_us(9000); IRoff();}
#define normalPulse()   {IRon(); _delay_us(562);  IRoff();}
#define bit0Pause()     _delay_us(562)
#define bit1Pause()     _delay_us(1687)
#define startPause()    _delay_us(4500)
#define repeatPause()   _delay_us(2250)
#define repeatCode()    {_delay_ms(40); startPulse(); repeatPause(); normalPulse(); _delay_ms(56);}


// send a single byte via IR
void sendByte(uint8_t value){
  for (uint8_t i=8; i; i--, value>>=1) {      // send 8 bits, LSB first
    normalPulse();                            // 562us burst
    (value & 1) ? bit1Pause() : bit0Pause();  // pause time depending on bit
  }
}

// send complete code (address + command) via IR
void sendCode(uint8_t code){
  startPulse();       // 9ms start burst
  startPause();       // 4.5ms pause
  sendByte(ADDR);     // send address byte
  sendByte(~ADDR);    // send inverse of address byte
  sendByte(code);     // send code byte
  sendByte(~code);    // send inverse of code byte
  normalPulse();      // 562us burst to signify end of transmission
}

// go to sleep in order to save energy, wake up again by pin change interrupt (button pressed)
void sleep(void) {
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);   // set sleep mode to power down
  GIFR |= 0b00100000;                     // clear any outstanding interrupts
  power_all_disable();                    // power everything off
  cli();                                  // timed sequence coming up
  sleep_enable();                         // ready to sleep
  sei();                                  // interrupts are required now
  sleep_cpu();                            // sleep              
  sleep_disable();                        // precaution
  power_all_enable();                     // power everything back on
}

// main function
int main(void){
  // set oscillator calibration value
  #ifdef OSCCAL_VAL
    OSCCAL = OSCCAL_VAL;                  // set the value if defined above
  #endif

  // setup pins
  DDRB  = 0b00000000;                     // all pins are input pins by now
  PORTB = 0b00111101;                     // pull-up for button pins
  
  // set timer0 to toggle IR pin at 38 kHz
  TCCR0A = 0b00100011;                    // PWM on OC0B (PB1)
  TCCR0B = 0b00001001;                    // no prescaler
  OCR0A  = TOP;                           // 38 kHz PWM frequency
  OCR0B  = DUTY;                          // 25 % duty cycle

  // setup pin change interrupt
  GIMSK = 0b00100000;                     // turn on pin change interrupts
  PCMSK = 0b00111101;                     // turn on interrupt on button pins

  // disable ADC and analog comperator for energy saving
  ADCSRA = 0b00000000;                    // disable ADC
  ACSR   = 0b10000000;                    // disable analog comperator

  // main loop
  while(1) {
    sleep();                              // sleep until button is pressed
    _delay_ms(1);                         // debounce
    uint8_t buttons = ~PINB & 0b00111101; // read button pins
    switch (buttons) {                    // send corresponding IR code
      case 0b00000001: sendCode(KEY1); break;
      case 0b00000100: sendCode(KEY2); break;
      case 0b00001000: sendCode(KEY3); break;
      case 0b00010000: sendCode(KEY4); break;
      case 0b00100000: sendCode(KEY5); break;
      default: break;
    }
    while (~PINB & 0b00111101) repeatCode();// send repeat command until button is released
  }
}

// pin change interrupt service routine
EMPTY_INTERRUPT (PCINT0_vect);              // nothing to be done here, just wake up from sleep
