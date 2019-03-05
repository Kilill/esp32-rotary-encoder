#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define ONBOARD_LED_GPIO (CONFIG_ONBOARD_LED_GPIO)
#define ROT_ENC_A_GPIO (CONFIG_ROT_ENC_A_GPIO)
#define ROT_ENC_B_GPIO (CONFIG_ROT_ENC_B_GPIO)
#define ROT_ENC_SW_GPIO (CONFIG_ROT_ENC_SW_GPIO)
#define TAG "Rotary-enc"

// Based of es32 port of 
// convert to C++ by Kim Lilliestierna
// Based on https://github.com/buxtronix/arduino/tree/master/libraries/Rotary
// Original header:

/* Rotary encoder handler for arduino. v1.1
 *
 * Copyright 2011 Ben Buxton. Licenced under the GNU GPL Version 3.
 * Contact: bb@cactii.net
 *
 * A typical mechanical rotary encoder emits a two bit gray code
 * on 3 output pins. Every step in the output (often accompanied
 * by a physical 'click') generates a specific sequence of output
 * codes on the pins.
 *
 * There are 3 pins used for the rotary encoding - one common and
 * two 'bit' pins.
 *
 * The following is the typical sequence of code on the output when
 * moving from one step to the next:
 *
 *   Position   Bit1   Bit2
 *   ----------------------
 *     Step1     0      0
 *      1/4      1      0
 *      1/2      1      1
 *      3/4      0      1
 *     Step2     0      0
 *
 * From this table, we can see that when moving from one 'click' to
 * the next, there are 4 changes in the output code.
 *
 * - From an initial 0 - 0, Bit1 goes high, Bit0 stays low.
 * - Then both bits are high, halfway through the step.
 * - Then Bit1 goes low, but Bit2 stays high.
 * - Finally at the end of the step, both bits return to 0.
 *
 * Detecting the direction is easy - the table simply goes in the other
 * direction (read up instead of down).
 *
 * To decode this, we use a simple state machine. Every time the output
 * code changes, it follows state, until finally a full steps worth of
 * code is received (in the correct order). At the final 0-0, it returns
 * a value indicating a step in one direction or the other.
 *
 * It's also possible to use 'half-step' mode. This just emits an event
 * at both the 0-0 and 1-1 positions. This might be useful for some
 * encoders where you want to detect all positions.
 *
 * If an invalid state happens (for example we go from '0-1' straight
 * to '1-0'), the state machine resets to the start until 0-0 and the
 * next valid codes occur.
 *
 * The biggest advantage of using a state machine over other algorithms
 * is that this has inherent debounce built in. Other algorithms emit spurious
 * output with switch bounce, but this one will simply flip between
 * sub-states until the bounce settles, then continue along the state
 * machine.
 * A side effect of debounce is that fast rotations can cause steps to
 * be skipped. By not requiring debounce, fast rotations can be accurately
 * measured.
 * Another advantage is the ability to properly handle bad state, such
 * as due to EMI, etc.
 * It is also a lot simpler than others - a static state table and less
 * than 10 lines of logic.
 */

/*
 * The below state table has, for each state (row), the new state
 * to set based on the next encoder output. From left to right in,
 * the table, the encoder outputs are 00, 01, 10, 11, and the value
 * in that position is the new state to set.
 */


class RotaryEnc {

	public:


// event struct sent out on queue
typedef struct
{
    int32_t position;
		uint8_t type;
		int8_t error;
} rotaryEvent_t;

// Direction states used by state processor
enum Direction {
		DIR_NONE	=	0x0,		// No complete step yet.
		DIR_CW		=	0x10,		// Clockwise step.
		DIR_CCW		=	0x20		// Anti-clockwise step.
};

// Event types encoded in queue events
enum EventTypes {
	CW_EVENT,
	CCW_EVENT,
	SW_EVENT
};


RotaryEnc(gpio_num_t pin_CLK , gpio_num_t pin_DT,gpio_num_t pin_SW);

~RotaryEnc() {
		vQueueDelete(RotaryQueue);
}

int32_t				getPosition() 			{ return position; }

void					setPosition(int pos){ position = pos; }
QueueHandle_t getQueue()					{ return RotaryQueue; }
gpio_num_t 			getSwPin()				{ return swPin; }
gpio_num_t 			getClkPin()				{ return clkPin; }
gpio_num_t 			getDtPin()				{ return dtPin; }

// Process new state as each encoder tick happens
uint8_t rotaryProcessState();
uint8_t waitForEvent(rotaryEvent_t *eventP);


#ifdef HALF_STEP
// Use the half-step state table (emits a code at 00 and 11)
enum States {
	R_START				= 0x0,
	R_CCW_BEGIN		= 0x1,
	R_CW_BEGIN		= 0x2,
	R_START_M			= 0x3,
	R_CW_BEGIN_M	= 0x4,
	R_CCW_BEGIN_M	= 0x5
};
static const uint8_t ttable[6][4];
#else
// Use the full-step state table (emits a code at 00 only)
enum States {
	R_START				= 0x0,
	R_CW_FINAL		= 0x1,
	R_CW_BEGIN		= 0x2,
	R_CW_NEXT			= 0x3,
	R_CCW_BEGIN		= 0x4,
	R_CCW_FINAL		= 0x5,
	R_CCW_NEXT		= 0x6
};
static const uint8_t ttable[7][4];
#endif
private:

// GPIO direction and pull-ups should be set externally
	uint8_t state;
	gpio_num_t clkPin;		// A
	gpio_num_t dtPin;		// B
	gpio_num_t swPin;		// SW
	int32_t position;

 	QueueHandle_t RotaryQueue;

	int get_pins();

};


