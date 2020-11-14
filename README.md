# TinyRemote - IR Remote Control based on ATtiny13A
TinyRemote is an IR remote control based on an ATtiny13A powered by a CR2032 or LIR2032 coin cell battery. The code is a full implementation of the NEC protocol using only 370 bytes (36%) of program memory.

- Project Video: https://youtu.be/ad3eyNCov9c
- Project on EasyEDA: https://easyeda.com/wagiminator/attiny13-tinyremote

![IMG_20200105_115938_x.jpg](https://image.easyeda.com/pullimage/VIS5ZlaEejDmMenv7sVxYe85p3RsQkphLDLCliZ2.jpeg)

# Working Principle
Timer 0 generates a 38kHz pulse frequency with a duty cycle of 25% on the output pin to the IR LED. The signal (NEC protocol) is modulated by toggling the pin to input/output.

```c
// define values for 38kHz PWM frequency and 25% duty cycle
#define TOP   31          // 1200kHz / 38kHz - 1 = 31
#define DUTY  7           // 1200kHz / 38kHz / 4 - 1 = 7

// set timer0 to toggle IR pin at 38 kHz
TCCR0A = 0b00100011;      // PWM on OC0B (PB1)
TCCR0B = 0b00001001;      // no prescaler
OCR0A  = TOP;             // 38 kHz PWM frequency
OCR0B  = DUTY;            // 25 % duty cycle
```

Here's the result, captured with a logic analyzer:

![PWM.jpg](https://github.com/wagiminator/ATtiny13-TinyRemote/blob/master/documentation/TinyRemote_PWM.png)

IR message starts with a 9ms leading burst followed by a 4.5ms pause. Afterwards 4 data bytes are transmitted, least significant bit first. A "0" bit is a 562.5us burst followed by a 562.5us pause, a "1" bit is a 562.5us burst followed by a 1687.5us pause. A final 562.5us burst signifies the end of the transmission. The four data bytes are in order:
- the 8-bit address for the receiving device,
- the 8-bit logical inverse of the address,
- the 8-bit command and
- the 8-bit logical inverse of the command.

![NEC_transmission.png](https://techdocs.altium.com/sites/default/files/wiki_attachments/296329/NECMessageFrame.png)

```c
// macros to switch on/off IR LED
#define IRon()   DDRB |= 0b00000010   // PB1 as output = IR at OC0B (38 kHz)
#define IRoff()  DDRB &= 0b11111101   // PB1 as input  = LED off

// macros to modulate the signals according to NEC protocol with compensated timings
#define startPulse()    {IRon(); _delay_us(9000); IRoff(); _delay_us(4500);}
#define normalPulse()   {IRon(); _delay_us( 562); IRoff(); _delay_us( 557);}
#define bit1Pause()     _delay_us(1120) // 1687.5us - 562.5us = 1125us

// send a single byte via IR
void sendByte(uint8_t value){
  for (uint8_t i=8; i; i--, value>>=1) {  // send 8 bits, LSB first
    normalPulse();                        // 562us burst, 562us pause
    if (value & 1) bit1Pause();           // extend pause if bit is 1
  }
}

// send complete code (address + command) via IR
void sendCode(uint8_t code){
  startPulse();       // 9ms burst + 4.5ms pause to signify start of transmission
  sendByte(ADDR);     // send address byte
  sendByte(~ADDR);    // send inverse of address byte
  sendByte(code);     // send code byte
  sendByte(~code);    // send inverse of code byte
  normalPulse();      // 562us burst to signify end of transmission
}
```

Here's the result, captured with a logic analyzer:

![NEC_protocol.jpg](https://github.com/wagiminator/ATtiny13-TinyRemote/blob/master/documentation/TinyRemote_NEC.png)

If the key on the remote controller is kept depressed, a repeat code will be issued consisting of a 9ms leading burst, a 2.25ms pause and a 562.5us burst to mark the end. The repeat code will continue to be sent out at 108ms intervals, until the key is finally released.

![NEC_repeat.png](https://techdocs.altium.com/sites/default/files/wiki_attachments/296329/NECRepeatCodes.png)

```c
// macros to modulate the signals according to NEC protocol with compensated timings
#define repeatPulse()   {IRon(); _delay_us(9000); IRoff(); _delay_us(2250);}
#define repeatCode()    {_delay_ms(40); repeatPulse(); normalPulse(); _delay_ms(56);}

// send repeat command until button is released
while (~PINB & 0b00111101) repeatCode();
```

The code utilizes the sleep mode power down function. It wakes up on every button press by pin change interrupt. The device will work several months on a CR2032 battery.

```c
// setup pin change interrupt
GIMSK = 0b00100000;                   // turn on pin change interrupts
PCMSK = 0b00111101;                   // turn on interrupt on button pins
SREG |= 0b10000000;                   // enable global interrupts

// disable unused peripherals and set sleep mode to save power
ADCSRA = 0b00000000;                  // disable ADC
ACSR   = 0b10000000;                  // disable analog comperator
PRR    = 0b00000001;                  // shut down ADC
set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode to power down
```

![pic1.jpg](https://github.com/wagiminator/ATtiny13-TinyRemote/blob/master/documentation/TinyRemote_pic1.jpg)

# References, Links and Notes
- [NEC infrared transmission protocol (altium.com)](https://techdocs.altium.com/display/FPGA/NEC+Infrared+Transmission+Protocol)
- [NEC protocol explanation by David Johnson-Davies](http://www.technoblogy.com/show?UVE)
- [IR remote control by Christoph Niessen (german)](http://chris.cnie.de/avr/tcm231421.html)
