
// tinyIRremote for ATtiny13 - 5 Buttons
// 
// IR remote control using an ATtiny 13. Timer 0 generates a 38kHz
// pulse frequency with a duty cycle of 25% on the output pin to the IR LED.
// The signal (NEC protocol) is modulated by toggling the pin to input/output.
//
//                        +-\/-+
// KEY5 --- A0 (D5) PB5  1|    |8  Vcc
// KEY3 --- A3 (D3) PB3  2|    |7  PB2 (D2) A1 --- KEY2
// KEY4 --- A2 (D4) PB4  3|    |6  PB1 (D1) ------ IR LED
//                  GND  4|    |5  PB0 (D0) ------ KEY1
//                        +----+    
//
// Clockspeed 1.2 MHz internal. BOD should be disabled to save energy.
// Millis/Tone must be disabled as Timer 0 is used for PWM.
// Reset pin must be disabled by writing respective fuse after uploading the code:
// avrdude -p attiny13 -c usbtiny -F -U lfuse:w:0x6a:m -U hfuse:w:0xfe:m
//
// 2019 by Stefan Wagner (https://easyeda.com/wagiminator)
// License: http://creativecommons.org/licenses/by-sa/3.0/
//
// based on the work of Christoph Niessen (http://chris.cnie.de/avr/tcm231421.html)
// and David Johnson-Davies (http://www.technoblogy.com/show?UVE)


// Libraries
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>

// IR codes
#define ADDR  0x20  // Address of LG TV
#define KEY1  0x40  // Volume+
#define KEY2  0x00  // Channel+
#define KEY3  0xc0  // Volume-
#define KEY4  0x80  // Channel-
#define KEY5  0x10  // Power

// Define values for 38kHz PWM frequency and 25% duty cycle
#define top   31                            // 1200kHz / 38kHz - 1 = 31
#define duty  7                             // 1200kHz / 38kHz / 4 - 1 = 7

// Commands to switch on/off IR LED
#define IRon   DDRB |= 0b00000010           // PB1 as output = IR at OC0B (38 kHz)
#define IRoff  DDRB &= 0b11111101           // PB1 as input  = LED off

// Commands to modulate the signals according to NEC protocol
#define startPulse    {IRon; delay(9); delayMicroseconds(100); IRoff;}
#define normalPulse   {IRon; delayMicroseconds(650); IRoff;}
#define bit0Pause     delayMicroseconds(450)
#define bit1Pause     delayMicroseconds(1600)
#define startPause    delayMicroseconds(4400)
#define repeatPause   delayMicroseconds(2200)
#define repeatCode    {delay(40); startPulse; repeatPause; normalPulse; delay(56);}


void setup(){
  // setup pins
  DDRB  = 0b00000000;                       // all pins are input pins by now
  PORTB = 0b00111101;                       // pull-up for button pins
  
  // set timer 1 to toggle IR pin at 38 kHz
  TCCR0A = 0b00100011;                      // PWM on OC0B (PB1)
  TCCR0B = 0b00001001;                      // no prescaler
  OCR0A  = top;                             // 38 kHz PWM frequency
  OCR0B  = duty;                            // 25 % duty cycle
  TIMSK0 = 0b00000000;                      // disable timer interrupts

  // setup pin change interrupt
  GIMSK = 0b00100000;                       // turn on pin change interrupts
  PCMSK = 0b00111101;                       // turn on interrupt on button pins

  // disable ADC for energy saving
  ADCSRA = 0;                               // disable ADC
}


void loop(){
  sleep();                                  // sleep until button is pressed
  delay(1);                                 // debounce
  uint8_t buttons = ~PINB & 0b00111101;     // read button pins
  switch (buttons) {                        // send corresponding IR code
    case 0b00000001: sendCode(KEY1); break;
    case 0b00000100: sendCode(KEY2); break;
    case 0b00001000: sendCode(KEY3); break;
    case 0b00010000: sendCode(KEY4); break;
    case 0b00100000: sendCode(KEY5); break;
    default: break;
  }
  while (~PINB & 0b00111101) repeatCode;    // send repeat command until button is released
}


// send a single byte via IR
void sendByte(uint8_t value){
  uint8_t bitmask = 0b10000000; 
  for (uint8_t i=0; i<8; i++) {
    normalPulse;
    if (value & bitmask) bit1Pause; else bit0Pause;
    bitmask = (bitmask >> 1);
  };
}


// send complete code (address + command) via IR
void sendCode(uint8_t code){
  startPulse;
  startPause;
  sendByte(ADDR);
  sendByte(~ADDR);
  sendByte(code);
  sendByte(~code);
  normalPulse;
}


// go to sleep in order to save energy, wake up again by pin change interrupt (button pressed)
void sleep() {
  set_sleep_mode (SLEEP_MODE_PWR_DOWN); // set sleep mode to power down
  bitSet (GIFR, PCIF);                  // clear any outstanding interrupts
  power_all_disable ();                 // power everything off
  noInterrupts ();                      // timed sequence coming up
  sleep_enable ();                      // ready to sleep
  interrupts ();                        // interrupts are required now
  sleep_cpu ();                         // sleep              
  sleep_disable ();                     // precaution
  power_all_enable ();                  // power everything back on
}


// pin change interrupt service routine
EMPTY_INTERRUPT (PCINT0_vect);          // nothing to be done here, just wake up from sleep
