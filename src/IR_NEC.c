/*
 * IR_NEC.c
 *
 * Created: 12. 4. 2019 10:00:18
 * Author : Tekl7
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/portpins.h>
#include <avr/sfr_defs.h>
#include <stdlib.h>

#include "IR_NEC.h"

// Set IR receive pin
#if defined(__AVR_ATtiny85__)
#define IR_RCV_PIN		PB2
#define IR_RCV_PIN_REG	PINB
#elif defined(__AVR_ATtiny88__)
#define IR_RCV_PIN		PD2
#define IR_RCV_PIN_REG	PIND
#endif

// Duration of one timer tick in µs
#define TICK 128

// Number of elements in the pulseBuffer
#define BUFFER_SIZE 32

/*
	Pulse tolerance multiplier - used to calculate pulse thresholds
	Use in interval from 1 to 2
	The greater the multiplier, the greater the tolerance
*/
#define PULSE_TOLERANCE_MULT	1.3

enum PulseType
{
	LEADING_PULSE = 0,
	INITIAL_SPACE,
	REPEAT_SPACE,
	FINAL_PULSE,
	LOGIC_SHORT,
	LOGIC_LONG,
	
	PULSE_COUNT
};

// In µs
enum PulseDuration
{
	LEADING_PULSE_DUR = 9500,
	INITIAL_SPACE_DUR = 4500,
	REPEAT_SPACE_DUR = 2250,
	FINAL_PULSE_DUR = 540,
	LOGIC_SHORT_DUR = 540,
	LOGIC_LONG_DUR = 1688
};

typedef enum
{
	WAIT_STATE,
	LEADING_PULSE_STATE,
	LEADING_SPACE_STATE,
	DATA_STATE,
	FINAL_PULSE_STATE
} IRCaptureState;

typedef enum
{
	// Newly captured IR data in pulse buffer
	BUF_READY,
	// Capturing IR repeat code (IR data still captured in pulse buffer)
	BUF_REPEAT,
	/*
		Nothing is happening (timeout) /
		receiving error /
		only if disabled repetition: pulse buffer already processed in IR_data_ready() function
	*/
	BUF_NOT_READY
} PulseBufferState;

// Lower thresholds of pulse duration
static /* const */ uint16_t lowPulseThrs[PULSE_COUNT] =
{
	LEADING_PULSE_DUR * (2 - PULSE_TOLERANCE_MULT),
	INITIAL_SPACE_DUR * (2 - PULSE_TOLERANCE_MULT),
	REPEAT_SPACE_DUR * (2 - PULSE_TOLERANCE_MULT),
	FINAL_PULSE_DUR * (2 - PULSE_TOLERANCE_MULT),
	LOGIC_SHORT_DUR * (2 - PULSE_TOLERANCE_MULT),
	LOGIC_LONG_DUR * (2 - PULSE_TOLERANCE_MULT)
};

// Upper thresholds of pulse duration
static /* const */ uint16_t upPulseThrs[PULSE_COUNT] =
{
	LEADING_PULSE_DUR * PULSE_TOLERANCE_MULT,
	INITIAL_SPACE_DUR * PULSE_TOLERANCE_MULT,
	REPEAT_SPACE_DUR * PULSE_TOLERANCE_MULT,
	FINAL_PULSE_DUR * PULSE_TOLERANCE_MULT,
	LOGIC_SHORT_DUR * PULSE_TOLERANCE_MULT,
	LOGIC_LONG_DUR * PULSE_TOLERANCE_MULT
};

// Main variable that stores IR data (address and command)
IR_data IR;
// Number of overflows to timeout
static uint8_t ovfsTimeout;
// Number of commands with disabled repetition
static uint8_t repDisCmdsCount = 0;
// Pointer to commands with disabled repetition
static uint8_t *repDisCmds = NULL;
// Stores captured data; LOGIC_SHORT -> false, LOGIC_LONG -> true
static volatile bool pulseBuffer[BUFFER_SIZE];
// Pulse buffer index (changes from 0 to BUFFER_SIZE)
static volatile uint8_t bufferIndex = 0;
// Current state of pulse buffer
static volatile PulseBufferState bufferState = BUF_NOT_READY;
// Current state of capturing
static volatile IRCaptureState captureState = WAIT_STATE;
// Overflow counter - used to check timeout
static volatile uint8_t ovfCounter = 0;

/**
 * Check whether repetition is disabled.
 *
 * @param command Command to check
 * @return true if repetition is disabled for command
 */
static bool repDisabled(uint8_t command);

/**
 * Write data from pulse buffer to IRData and invIRData.
 *
 * @param IRData Structure which stores non-inverted data
 * @param invIRData Structure which stores inverted data
 */
static void write_data(IR_data *IRData, IR_data *invIRData);



/* ---------------------------------------------------------------------- */



void IR_init(uint32_t bufferTimeout)
{
	#if defined(__AVR_ATtiny85__)
	// Normal mode, prescaler 1024
	TCCR0B |= _BV(CS02) | _BV(CS00);
	
	// External interrupt - any logical change
	MCUCR |= _BV(ISC00);
	
	// External interrupt for INT0 enabled
	GIMSK |= _BV(INT0);
	
	#elif defined(__AVR_ATtiny88__)
	// Normal mode, prescaler 1024
	TCCR0A |= _BV(CS02) | _BV(CS00);
	
	// External interrupt - any logical change
	EICRA |= _BV(ISC00);
	
	// External interrupt for INT0 enabled
	EIMSK |= _BV(INT0);
	
	#endif
	
	// Initialize the pulse buffer
	for (uint8_t i = 0; i < BUFFER_SIZE; i++)
	{
		pulseBuffer[i] = 0;
	}
	
	// Calculate number of overflows to timeout
	if (bufferTimeout < 132)
	{
		// 4 * 32.768 ms = 131.072 ms (32.768 ms is one period of overflow interrupt)
		ovfsTimeout = 4;
	}
	else
	{
		ovfsTimeout = (uint8_t) ((bufferTimeout / 32.768) + 1);
	}
}

/* ---------------------------------------------------------------------- */

void IR_disable_repetition(uint8_t command)
{
	if(!repDisabled(command))
	{
		/* Repetition is not enabled yet */

		repDisCmdsCount++;
		// Dynamic memory allocation
		repDisCmds = realloc(repDisCmds, sizeof(*repDisCmds) * repDisCmdsCount);
		
		if (repDisCmds != NULL)
		{
			// Add command to the array
			*(repDisCmds + (repDisCmdsCount-1)) = command;
		}
	}
}

/* ---------------------------------------------------------------------- */

void IR_enable_repetition(uint8_t command)
{
	// Iterate through whole dynamically allocated memory
	for (uint8_t i = 0; i < repDisCmdsCount; i++)
	{
		if (*(repDisCmds + i) == command)
		{
			/* Command to disable found */

			// Create memory for temporary pointer where commands will be copied to
			uint8_t *tempCmds = calloc(repDisCmdsCount, sizeof(*repDisCmds));
			
			if (tempCmds != NULL)
			{
				// Copy commands to the temporary pointer
				for (uint8_t j = 0; j < repDisCmdsCount; j++)
				{
					*(tempCmds + j) = *(repDisCmds + j);
				}
				
				// Free commands memory
				free(repDisCmds);
				
				// Copy back all commands except the deleted one
				repDisCmdsCount--;
				repDisCmds = calloc(repDisCmdsCount, sizeof(*repDisCmds));
				if (repDisCmds != NULL)
				{
					for (uint8_t j = 0, k = 0; j < repDisCmdsCount + 1; j++)
					{
						if (*(tempCmds + j) != command)
						{
							*(repDisCmds + k) = *(tempCmds + j);
							k++;
						}
					}
				}
				
				// Free temporary commands memory
				free(tempCmds);
			}
			
			break;
		}
	}
}

/* ---------------------------------------------------------------------- */

bool IR_data_ready()
{
	bool dataReadyFlag = false;
	
	// New IR data
	IR_data newIRData;
	// New inverted IR data
	IR_data newInvIRData;
		
	switch(bufferState)
	{
		case BUF_READY:
		// Write data from the pulse buffer to new IR data structures
		write_data(&newIRData, &newInvIRData);
			
		if ((newIRData.address ^ newInvIRData.address) != 0xFF || (newIRData.command ^ newInvIRData.command) != 0xFF)
		{
			/* Address or command inversion error */

			dataReadyFlag = false;
			bufferState = BUF_NOT_READY;
			break;
		}
		else
		{
			/* Address and command inversion successful */

			// Store IR data to the globally accessible variable
			IR.address = newIRData.address;
			IR.command = newIRData.command;
			
			dataReadyFlag = true;
			bufferState = repDisabled(IR.command) ? BUF_NOT_READY : BUF_REPEAT;
		}
		break;
			
		case BUF_REPEAT:
		dataReadyFlag = true;
		break;
			
		case BUF_NOT_READY:
		dataReadyFlag = false;
		break;
	}
	
	return dataReadyFlag;
}

/* ---------------------------------------------------------------------- */

// External interrupt (IR signal capture)
ISR(INT0_vect)
{
	uint32_t capturedPulse = TICK * TCNT0;
	// Pulse (HIGH) -> true, space (LOW) -> false
	uint8_t pulseLevel = (IR_RCV_PIN_REG & _BV(IR_RCV_PIN)) >> IR_RCV_PIN;
	// Bool to determine pulse buffer state in final pulse state
	static bool repeatCode = false;
		
	switch (captureState)
	{
		// Waiting for next data
		case WAIT_STATE:
		if (!pulseLevel)
		{
			captureState = LEADING_PULSE_STATE;
			#if defined(__AVR_ATtiny85__)
			// Clear overflow interrupt flag
			TIFR |= _BV(TOV0);
			// Enable overflow interrupt for Timer/Counter0
			TIMSK |= _BV(TOIE0);
			#elif defined(__AVR_ATtiny88__)
			// Clear overflow interrupt flag
			TIFR0 |= _BV(TOV0);
			// Enable overflow interrupt for Timer/Counter0
			TIMSK0 |= _BV(TOIE0);
			#endif
		}
		else
		{
			bufferState = BUF_NOT_READY;
		}
		break;
			
		// Leading pulse
		case LEADING_PULSE_STATE:
		if (pulseLevel && capturedPulse >= lowPulseThrs[LEADING_PULSE] && capturedPulse <= upPulseThrs[LEADING_PULSE])
		{
			captureState = LEADING_SPACE_STATE;
		}
		else
		{
			bufferState = BUF_NOT_READY;
			captureState = WAIT_STATE;
		}
		break;
			
		// Initial space or repeat space
		case LEADING_SPACE_STATE:
		if (!pulseLevel && capturedPulse >= lowPulseThrs[INITIAL_SPACE] && capturedPulse <= upPulseThrs[INITIAL_SPACE])
		{
			captureState = DATA_STATE;
			repeatCode = false;
			bufferIndex = 0;
		}
		else if (!pulseLevel && capturedPulse >= lowPulseThrs[REPEAT_SPACE] && capturedPulse <= upPulseThrs[REPEAT_SPACE])
		{
			captureState = FINAL_PULSE_STATE;
			repeatCode = true;
		}
		else
		{
			bufferState = BUF_NOT_READY;
			captureState = WAIT_STATE;
		}
		break;
			
		// 8-bit address
		case DATA_STATE:
		// Short pulse or space
		if (!pulseLevel && capturedPulse >= lowPulseThrs[LOGIC_SHORT] && capturedPulse <= upPulseThrs[LOGIC_SHORT])
		{
			pulseBuffer[bufferIndex++] = false;

			if (bufferIndex >= BUFFER_SIZE)
			{
				captureState = FINAL_PULSE_STATE;
			}
		}
		// Long space
		else if (!pulseLevel && capturedPulse >= lowPulseThrs[LOGIC_LONG] && capturedPulse <= upPulseThrs[LOGIC_LONG])
		{
			pulseBuffer[bufferIndex++] = true;

			if (bufferIndex >= BUFFER_SIZE)
			{
				captureState = FINAL_PULSE_STATE;
			}
		}
		else if (!pulseLevel)
		{
			bufferState = BUF_NOT_READY;
			captureState = WAIT_STATE;
		}
		break;
			
		// Final pulse
		case FINAL_PULSE_STATE:
		if (pulseLevel && capturedPulse >= lowPulseThrs[FINAL_PULSE] && capturedPulse <= upPulseThrs[FINAL_PULSE])
		{
			if (!repeatCode)
			{
				bufferState = BUF_READY;
			}
			
			ovfCounter = 0;
		}
		else
		{
			bufferState = BUF_NOT_READY;
		}
		
		captureState = WAIT_STATE;
		break;
	}
		
	TCNT0 = 0;
}

/* ---------------------------------------------------------------------- */

// Overflow interrupt (period = 32.768 ms)
ISR(TIMER0_OVF_vect)
{
	ovfCounter++;

	if (ovfCounter >= ovfsTimeout)
	{
		#if defined(__AVR_ATtiny85__)
		// Disable overflow interrupt for Timer/Counter0
		TIMSK |= _BV(TOIE0);
		#elif defined(__AVR_ATtiny88__)
		// Disable overflow interrupt for Timer/Counter0
		TIMSK0 &= ~_BV(TOIE0);		
		#endif
		
		bufferState = BUF_NOT_READY;
		captureState = WAIT_STATE;
		ovfCounter = 0;
	}
}

/* ---------------------------------------------------------------------- */

static bool repDisabled(uint8_t command)
{
	// Iterate through whole dynamically allocated memory
	for (uint8_t i = 0; i < repDisCmdsCount; i++)
	{
		if (*(repDisCmds + i) == command)
		{
			return true;
		}
	}
	return false;
}

/* ---------------------------------------------------------------------- */

static void write_data(IR_data *IRData, IR_data *invIRData)
{
	const uint8_t dataCount = 4;
	const uint8_t bufferFraction = BUFFER_SIZE / dataCount;
	
	uint8_t *allData[dataCount];
	allData[0] = &IRData->address;
	allData[1] = &invIRData->address;
	allData[2] = &IRData->command;
	allData[3] = &invIRData->command;
	
	// Write data
	for (uint8_t dataI = 0; dataI < dataCount; dataI++)
	{
		// dataI =: 0 -> address, 1 -> inverted address, 2 -> command, 3 -> inverted command
		for (uint8_t bufferI = 0; bufferI < bufferFraction; bufferI++)
		{
			if (pulseBuffer[bufferI+(dataI*bufferFraction)])
			{
				// Write 1
				*allData[dataI] |= (1 << bufferI);
			}
			else
			{
				// Write 0
				*allData[dataI] &= ~(1 << bufferI);
			}
		}
	}
}
