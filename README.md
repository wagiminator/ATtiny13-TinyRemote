# TinyRemote - IR Remote Control based on ATtiny13a
TinyRemote is an IR remote control based on an ATtiny13A powered by a CR2032 or LIR2032 coin cell battery. The code is a full implementation of the NEC protocol using only 370 bytes (36%) of program memory.
The code utilizes the sleep mode power down function. The device will work several months on a CR2032 battery.

Project Video: https://youtu.be/ad3eyNCov9c

Project on EasyEDA: https://easyeda.com/wagiminator/attiny13-tinyremote

![IMG_20200105_115938_x.jpg](https://image.easyeda.com/pullimage/VIS5ZlaEejDmMenv7sVxYe85p3RsQkphLDLCliZ2.jpeg)

Timer 0 generates a 38kHz pulse frequency with a duty cycle of 25% on the output pin to the IR LED. The signal (NEC protocol) is modulated by toggling the pin to input/output.

IR message starts with a 9ms leading burst followed by a 4.5ms pause. Afterwards 4 data bytes are transmitted, least significant bit first. A "0" bit is a 562.5us burst followed by a 562.5us pause, a "1" bit is a 562.5us burst followed by a 1687.5us pause. A final 562.5us burst signifies the end of the transmission. The four data bytes are in order:
- the 8-bit address for the receiving device,
- the 8-bit logical inverse of the address,
- the 8-bit command and
- the 8-bit logical inverse of the command.

If the key on the remote controller is kept depressed, a repeat code will be issued consisting of a 9ms leading burst, a 2.25ms pause and a 562.5us burst to mark the end. The repeat code will continue to be sent out at 108ms intervals, until the key is finally released.

Here's the result, captured with a logic analyzer:

![PWM.jpg](https://github.com/wagiminator/ATtiny13-TinyRemote/blob/master/documentation/TinyRemote_PWM.png)
![NEC_protocol.jpg](https://github.com/wagiminator/ATtiny13-TinyRemote/blob/master/documentation/TinyRemote_NEC.png)
![pic1.jpg](https://github.com/wagiminator/ATtiny13-TinyRemote/blob/master/documentation/TinyRemote_pic1.jpg)
