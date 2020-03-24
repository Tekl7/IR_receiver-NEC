/*
 * IR_NEC.c
 *
 * Created: 12. 4. 2019 10:00:18
 * Author : Tekl7
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <stdlib.h>

#include "IR_NEC.h"

// Times in µs
#define LEADING_PULSE 9000
#define INITIAL_SPACE 4500
#define REPEAT_SPACE 2250
#define FINAL_PULSE 563	// 562.5
#define LOGIC_SHORT 563	// 562.5
#define LOGIC_LONG 1688	// 1687.5

// One tick duration in µs
#define TICK 128
#define NUM_OF_TICKS 3

// States
#define SLEEP_STATE 0
#define LEADING_PULSE_STATE 1
#define LEADING_SPACE_STATE 2
#define ADDRESS_STATE 3
#define INVERTED_ADDRESS_STATE 4
#define COMMAND_STATE 5
#define INVERTED_COMMAND_STATE 6
#define FINAL_PULSE_STATE 7
#define WAIT_STATE 8

// Number of OVFs to SLEEP_STATE or clear buffer
#define NUM_OF_OVFS 3
#define NUM_OF_CLEAR_BUFFER_OVFS(clearBufferTime) (clearBufferTime + 16) / (1UL * 1000UL * 256UL * 1024UL / F_CPU)

// Number of elements in the durationBuffer
#define BUFFER_SIZE 64

IR_data IR;
static uint8_t numOfcmds = 0;
static uint8_t *cmdsPtr = NULL;
static uint16_t timeRange = TICK * NUM_OF_TICKS;
static uint16_t clearBTime = 0;
static volatile bool durationBuffer[BUFFER_SIZE];	// LOGIC_SHORT -> true, LOGIC_LONG -> false
static volatile uint8_t bufferIndex = 0;
static volatile bool bufferReady = false;
static volatile bool clearBuffer = false;
static volatile bool level = true;					// Pulse -> level = true, space -> level = false
static volatile uint8_t state = SLEEP_STATE;
static volatile uint16_t ovfCounter = 1;
static volatile bool available = false;

static bool isRepDisabled(uint8_t command);
static void writeBit(uint8_t i, uint8_t j, uint8_t *data);
static void setSleepState();

void IR_init(uint16_t clearBufferTime)
{
	#if DEVICE == 0
	// Normal mode
	// Prescaler 1024
	TCCR0B |= _BV(CS02) | _BV(CS00);
	
	// Overflow interrupt for Timer/Counter0 enabled
	TIMSK |= _BV(TOIE0);
	
	// External interrupt - any logical change
	MCUCR |= _BV(ISC00);
	
	// External interrupt for INT0 enabled
	GIMSK |= _BV(INT0);
	
	#elif DEVICE == 1
	// Normal mode
	// Prescaler 1024
	TCCR0A |= _BV(CS02) | _BV(CS00);
	
	// Overflow interrupt for Timer/Counter0 enabled
	TIMSK0 |= _BV(TOIE0);
	
	// External interrupt - any logical change
	EICRA |= _BV(ISC00);
	
	// External interrupt for INT0 enabled
	EIMSK |= _BV(INT0);
	
	#endif
	
	// Get the clearBufferTime time in ms (after this time, the buffer will be cleared)
	if (clearBufferTime < 83)
	{
		clearBTime = 83;
	}
	else
	{
		clearBTime = clearBufferTime;
	}
	
	// Initialize the buffer
	for (uint8_t i = 0; i < BUFFER_SIZE; i++)
	{
		durationBuffer[i] = 0;
	}
}

// Disables repetition of the entered command (while holding a button, your instructions related to the command are executed just once)
void IR_disableRepetition(uint8_t command)
{
	if(!isRepDisabled(command))
		{
		numOfcmds++;
		// Dynamic memory allocation
		cmdsPtr = realloc(cmdsPtr, sizeof(*cmdsPtr) * numOfcmds);
		if (cmdsPtr != NULL)
		{
			*(cmdsPtr + (numOfcmds-1)) = command;
		}
	}
}

// Enables repetition of the entered command, which were disabled earlier
void IR_enableRepetition(uint8_t command)
{
	// Iterates through whole dynamically allocated memory
	for (uint8_t i = 0; i < numOfcmds; i++)
	{
		if (*(cmdsPtr + i) == command)
		{
			// Copy all commands
			uint8_t *tempCmdsPtr = calloc(numOfcmds, sizeof(*cmdsPtr));
			if (tempCmdsPtr != NULL)
			{
				for (uint8_t j = 0; j < numOfcmds; j++)
				{
					*(tempCmdsPtr + j) = *(cmdsPtr + j);
				}
				
				// Free commands memory
				free(cmdsPtr);
				
				// Copy back all commands except the deleted one
				numOfcmds--;
				cmdsPtr = calloc(numOfcmds, sizeof(*cmdsPtr));
				if (cmdsPtr != NULL)
				{
					uint8_t k = 0;
					for (uint8_t j = 0; j < numOfcmds + 1; j++)
					{
						if (*(tempCmdsPtr + j) != command)
						{
							*(cmdsPtr + k) = *(tempCmdsPtr + j);
							k++;
						}
					}
				}
				
				// Free temporary commands memory
				free(tempCmdsPtr);
			}
			
			break;
		}
	}
}

// Returns true if receiving IR (initial pulses or repeat pulses)
bool IR_available()
{
	uint8_t address = 0, invertedAddress = 0, command = 0, invertedCommand = 0;
	static uint8_t lastCommand = 0;
	static bool lastAvailable = false;
	
	// Clearing the buffer
	if (clearBuffer)
	{
		for (uint8_t i = 0; i < BUFFER_SIZE; i++)
		{
			durationBuffer[i] = 0;
		}
		clearBuffer = false;
	}

	// Checking the buffer
	if (bufferReady)
	{
		available = true;
			
		for (uint8_t i = 0; i < 4; i++)
		{
			for (uint8_t j = i * 16 + 1; j < i * 16 + 16; j += 2)
			{
				switch (i)
				{
					case 0:
					writeBit(i, j, &address);
					break;
						
					case 1:
					writeBit(i, j, &invertedAddress);
					break;
						
					case 2:
					writeBit(i, j, &command);
					break;
						
					case 3:
					writeBit(i, j, &invertedCommand);
					break;
				}
			}
		}
			
		// Checks whether invertedAddress is really inverted address
		if ((address ^ invertedAddress) != 0xFF)
		{
			setSleepState();
		}
		// Checks whether invertedCommand is really inverted command
		if ((command ^ invertedCommand) != 0xFF)
		{
			setSleepState();
		}
			
		IR.address = address;	// Assigning address to structure's address member
		IR.command = command;	// Assigning command to structure's command member
		
		// Checks whether repetition is disabled
		if (IR.command == lastCommand && lastAvailable == true && isRepDisabled(IR.command))
		{
			bufferReady = false;
			return false;
		}
		// Repetition is allowed
		else
		{
			lastCommand = IR.command;
			bufferReady = false;
		}
	}
	// Checks whether repetition is disabled
	else if (available && isRepDisabled(IR.command))
	{
		return false;
	}
	
	lastAvailable = available;
	return available;
}

// Checks whether repetition is disabled
static bool isRepDisabled(uint8_t command)
{
	// Iterates through whole dynamically allocated memory
	for (uint8_t i = 0; i < numOfcmds; i++)
	{
		if (*(cmdsPtr + i) == command)
		{
			return true;
		}
	}
	return false;
}

static void writeBit(uint8_t i, uint8_t j, uint8_t *data)
{
	if (durationBuffer[j])
	{
		*data &= ~(1 << (((j-1)-i*16)/2));
	}
	else
	{
		*data |= (1 << (((j-1)-i*16)/2));
	}
}

static void setSleepState()
{
	clearBuffer = true;
	state = SLEEP_STATE;
	level = true;
	bufferIndex = 0;
	available = false;
}

// External interrupt
ISR(INT0_vect)
{
	if (!bufferReady)
	{
		switch (state)
		{
			case SLEEP_STATE:
			state = LEADING_PULSE_STATE;
			break;
			
			// Leading 9 ms pulse
			case LEADING_PULSE_STATE:
			if (TICK * TCNT0 >= LEADING_PULSE - timeRange && TICK * TCNT0 <= LEADING_PULSE + timeRange)
			{
				state = LEADING_SPACE_STATE;
			}
			else
			{
				setSleepState();
			}
			break;
			
			// Initial 4.5 ms space or repeat 2.25 ms space
			case LEADING_SPACE_STATE:
			if (TICK * TCNT0 >= INITIAL_SPACE - timeRange && TICK * TCNT0 <= INITIAL_SPACE + timeRange)
			{
				state = ADDRESS_STATE;
			}
			else if (TICK * TCNT0 >= REPEAT_SPACE - timeRange && TICK * TCNT0 <= REPEAT_SPACE + timeRange)
			{
				state = FINAL_PULSE_STATE;
			}
			else
			{
				setSleepState();
			}
			break;
			
			// 8-bit address
			case ADDRESS_STATE:
			// Short pulse or space
			if (TICK * TCNT0 >= LOGIC_SHORT - timeRange && TICK * TCNT0 <= LOGIC_SHORT + timeRange)
			{
				if (bufferIndex == BUFFER_SIZE / 4 - 1)
				{
					state = INVERTED_ADDRESS_STATE;
				}
				level = !level;
				durationBuffer[bufferIndex++] = true;
			}
			// Long space
			else if (TICK * TCNT0 >= LOGIC_LONG - timeRange && TICK * TCNT0 <= LOGIC_LONG + timeRange && !level)
			{
				if (bufferIndex == BUFFER_SIZE / 4 - 1)
				{
					state = INVERTED_ADDRESS_STATE;
				}
				level = !level;
				durationBuffer[bufferIndex++] = false;
			}
			else
			{
				setSleepState();
			}
			break;
			
			// 8-bit inverted address
			case INVERTED_ADDRESS_STATE:
			// Short pulse or space
			if (TICK * TCNT0 >= LOGIC_SHORT - timeRange && TICK * TCNT0 <= LOGIC_SHORT + timeRange)
			{
				if (bufferIndex == BUFFER_SIZE / 2 - 1)
				{
					state = COMMAND_STATE;
				}
				level = !level;
				durationBuffer[bufferIndex++] = true;
			}
			// Long space
			else if (TICK * TCNT0 >= LOGIC_LONG - timeRange && TICK * TCNT0 <= LOGIC_LONG + timeRange && !level)
			{
				if (bufferIndex == BUFFER_SIZE / 2 - 1)
				{
					state = COMMAND_STATE;
				}
				level = !level;
				durationBuffer[bufferIndex++] = false;
			}
			else
			{
				setSleepState();
			}
			break;
			
			// 8-bit command
			case COMMAND_STATE:
			// Short pulse or space
			if (TICK * TCNT0 >= LOGIC_SHORT - timeRange && TICK * TCNT0 <= LOGIC_SHORT + timeRange)
			{
				if (bufferIndex == (3 * BUFFER_SIZE) / 4 - 1)
				{
					state = INVERTED_COMMAND_STATE;
				}
				level = !level;
				durationBuffer[bufferIndex++] = true;
			}
			// Long space
			else if (TICK * TCNT0 >= LOGIC_LONG - timeRange && TICK * TCNT0 <= LOGIC_LONG + timeRange && !level)
			{
				if (bufferIndex == (3 * BUFFER_SIZE) / 4 - 1)
				{
					state = INVERTED_COMMAND_STATE;
				}
				level = !level;
				durationBuffer[bufferIndex++] = false;
			}
			else
			{
				setSleepState();
			}
			break;
			
			// 8-bit inverted command
			case INVERTED_COMMAND_STATE:
			// Short pulse or space
			if (TICK * TCNT0 >= LOGIC_SHORT - timeRange && TICK * TCNT0 <= LOGIC_SHORT + timeRange)
			{
				if (bufferIndex == BUFFER_SIZE - 1)
				{
					state = FINAL_PULSE_STATE;
				}
				level = !level;
				durationBuffer[bufferIndex++] = true;
			}
			// Long space
			else if (TICK * TCNT0 >= LOGIC_LONG - timeRange && TICK * TCNT0 <= LOGIC_LONG + timeRange && !level)
			{
				if (bufferIndex == BUFFER_SIZE - 1)
				{
					state = FINAL_PULSE_STATE;
				}
				level = !level;
				durationBuffer[bufferIndex++] = false;
			}
			else
			{
				setSleepState();
			}
			break;
			
			// Final 562.5 µs pulse
			case FINAL_PULSE_STATE:
			if (TICK * TCNT0 >= FINAL_PULSE - timeRange && TICK * TCNT0 <= FINAL_PULSE + timeRange)
			{
				clearBuffer = false;
				state = WAIT_STATE;
				ovfCounter = 1;
				level = true;
				bufferIndex = 0;
				bufferReady = true;
			}
			else
			{
				setSleepState();
			}
			break;
			
			// Waiting after final pulse
			case WAIT_STATE:
			state = LEADING_PULSE_STATE;
			break;
		}
	}
	
	TCNT0 = 0;
}

// Overflow interrupt
ISR(TIMER0_OVF_vect)
{
	// SLEEP_STATE begins after NUM_OF_OVFS overflow interrupts
	if (ovfCounter >= NUM_OF_OVFS)
	{
		state = SLEEP_STATE;
		level = true;
		bufferIndex = 0;
		available = false;
	}
	
	// Clearing buffer is allowed after NUM_OF_CLEAR_BUFFER_OVFS overflow interrupts
	if (ovfCounter >= NUM_OF_CLEAR_BUFFER_OVFS(clearBTime))
	{
		ovfCounter = 1;
		clearBuffer = true;
	}
	else
	{
		ovfCounter++;
	}
}
