# IR_receiver-NEC

This library allows you to comfortably decode NEC protocol used by remote IR controls.

How to use:
1. Import header file "IR_NEC.h" to your project
2. Add "libIR_receiver-NEC-t85.a" or "libIR_receiver-NEC-t88.a" library (located inside the "Release" folder) to your project
3. Meet following instructions:

Timer/Counter and External interrupt is used to decode IR NEC protocol.
Hardware support:
ATtiny85: 8-bit Timer/Counter0, INT0
ATtiny88: 8-bit Timer/Counter0, INT0
CPU clock is 8 MHz.

Connect output pin of IR receiver to PB2->ATtiny85 / PD2->ATtiny88 (pin 7/4).
Execute IR_init(uint16_t clearBufferTime) function to set registers, interrupts and clearBufferTime (after this time, durationBuffer will be cleared).
It is useful, when some parts of running program cause delay. The clearBufferTime should be set the same as the duration of the delay. If nothing causes delay, just write 0.
Then just check IR_available() function in loop.
If IR_available() returns true, IR signal has been checked succesfully and then IR data (address and command) are available.
Access to IR data is through IR_data structure named IR (IR.address for address, IR.command for command).
Make sure to execute sei() function to enable global interrupts.
