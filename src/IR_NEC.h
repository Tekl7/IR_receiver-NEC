/*
 * IR_NEC.h
 *
 * Created: 12. 4. 2019 10:02:55
 *  Author: Tekl7
 *
 * This project uses Timer/Counter and External interrupt to decode IR NEC protocol.
 *
 * Hardware support:
 * ATtiny85: 8-bit Timer/Counter0, INT0
 * ATtiny88: 8-bit Timer/Counter0, INT0
 * CPU clock is 8 MHz.
 *
 * Connect output pin of IR receiver to PB2->ATtiny85 / PD2->ATtiny88 (pin 7/4).
 * Execute IR_init(uint16_t clearBufferTime) function to set registers, interrupts and clearBufferTime (after this time, durationBuffer will be cleared).
 * The clearBufferTime is useful, when some parts of running program cause delay. Its value should be set the same as the duration of the delay.
 * If nothing causes delay, just write 0.
 * Repetition for certain commands can be disabled using IR_disableRepetition(uint8_t command).
 * For each command one IR_disableRepetition(uint8_t command) execution is needed.
 * So while holding a button, your instructions related to the command are executed just once (IR_available() returns false).
 * Disabled repetition can be enabled using IR_enableRepetition(uint8_t command) again.
 * Then just check IR_available() function in loop.
 * If IR_available() returns true, IR signal has been checked succesfully and then IR data (address and command) are available.
 * Access to IR data is through IR_data structure named IR (IR.address for address, IR.command for command).
 * Make sure to execute sei() function to enable global interrupts.
 */ 


#ifndef IR_NEC_H_
#define IR_NEC_H_

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <stdbool.h>

// Choose a number and assign it to DEVICE macro to select the demanded device.
// ATtiny85 -> 0
// ATtiny88 -> 1
#define DEVICE 0

// Structure for address and command
typedef struct
{
	uint8_t address;
	uint8_t command;
} IR_data;

extern IR_data IR;

void IR_init(uint16_t clearBufferTime);
void IR_disableRepetition(uint8_t command);
void IR_enableRepetition(uint8_t command);
bool IR_available();

#endif /* IR_NEC_H_ */
