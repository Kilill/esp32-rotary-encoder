#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "rotary.h"

#define TAG "Rotary-enc"

#ifdef HALF_STEP
const uint8_t RotaryEnc::ttable[6][4] = {
    // R_START (00)
    {R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
    // R_CCW_BEGIN
    {R_START_M | DIR_CCW,  R_START,        R_CCW_BEGIN,  R_START},
    // R_CW_BEGIN
    {R_START_M | DIR_CW,   R_CW_BEGIN,     R_START,      R_START},
    // R_START_M (11)
    {R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
    // R_CW_BEGIN_M
    {R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
    // R_CCW_BEGIN_M
    {R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
};


#else 
const uint8_t RotaryEnc::ttable[7][4] = {
    // 00        01           10           11                  // BA
    {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},           // R_START
    {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},  // R_CW_FINAL
    {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},           // R_CW_BEGIN
    {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},           // R_CW_NEXT
    {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},           // R_CCW_BEGIN
    {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW}, // R_CCW_FINAL
    {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},           // R_CCW_NEXT
};
#endif
// isr routines need to be C functions
static void rotEncIsr(void * args)
{
    //ESP_EARLY_LOGD(TAG, "intr");
		
    RotaryEnc * rp = (RotaryEnc *) args;

		RotaryEnc::rotaryEvent_t qEvent;	

		qEvent.position=rp->getPosition();

    uint8_t event = rp->rotaryProcessState();
    ESP_EARLY_LOGI(TAG, "rotEncIsr");
    switch (event)
    {
			case RotaryEnc::DIR_CW:
				qEvent.type=RotaryEnc::CW_EVENT;
        qEvent.position++;
				rp->setPosition(qEvent.position);
        ESP_EARLY_LOGI(TAG, "CW turn");
        xQueueSendToBackFromISR(rp->getQueue(), &qEvent, NULL);
        break;

			case RotaryEnc::DIR_CCW:
        qEvent.position--;
				qEvent.type=RotaryEnc::CCW_EVENT;
				rp->setPosition(qEvent.position);
        ESP_EARLY_LOGI(TAG, "CCW");
        xQueueSendToBackFromISR(rp->getQueue(), &qEvent, NULL);
        break;

			default:
        break;
    }
}

static void rotSwIsr(void * args)
{

		
    RotaryEnc * rp = (RotaryEnc *) args;
		RotaryEnc::rotaryEvent_t qEvent;	
		ESP_EARLY_LOGD(TAG, "Rotary Sw");
    if (gpio_get_level(rp->getSwPin()) == 0) 
    {
			qEvent.type=RotaryEnc::SW_EVENT;
      xQueueSendToBackFromISR(rp->getQueue(), &qEvent, NULL);
    }
}


RotaryEnc::RotaryEnc(gpio_num_t pin_CLK , gpio_num_t pin_DT,gpio_num_t pin_SW)
{
		clkPin = pin_CLK;
    dtPin =  pin_DT;
		swPin =  pin_SW;
    state = R_START;

		gpio_pad_select_gpio(swPin);
		gpio_set_pull_mode(swPin, GPIO_PULLUP_ONLY);
    gpio_set_direction(swPin, GPIO_MODE_INPUT);
    gpio_set_intr_type(swPin, GPIO_INTR_ANYEDGE);

    gpio_pad_select_gpio(clkPin);
    gpio_set_pull_mode(clkPin, GPIO_PULLUP_ONLY);
    gpio_set_direction(clkPin, GPIO_MODE_INPUT);
    gpio_set_intr_type(clkPin, GPIO_INTR_ANYEDGE);

    gpio_pad_select_gpio(dtPin);
    gpio_set_pull_mode(dtPin, GPIO_PULLUP_ONLY);
    gpio_set_direction(dtPin, GPIO_MODE_INPUT);
    gpio_set_intr_type(dtPin, GPIO_INTR_ANYEDGE);

    gpio_install_isr_service(0);

		// set up gpio isr handlers
    gpio_isr_handler_add(clkPin, rotEncIsr, this);
    gpio_isr_handler_add(dtPin,  rotEncIsr, this);
    gpio_isr_handler_add(swPin,  rotSwIsr, this);

    RotaryQueue = xQueueCreate(10, sizeof(rotaryEvent_t));
		ESP_LOGI(TAG, "Rotary Constructor");
}

uint8_t RotaryEnc::rotaryProcessState()
{
	uint8_t event = 0;
	// Get state of input pins.
	uint8_t pin_state = gpio_get_level(clkPin)<<1 | gpio_get_level(dtPin);	// get current status of pinns
	uint8_t old_state = state;										// remeber old state
	state = ttable[state & 0xf][pin_state];				// Determine new state from the pins and state table.
	event = state & 0x30;									// Return emit bits, i.e. the generated event.
	ESP_EARLY_LOGD(TAG, "Old state=%02x new state 0x%02x, event %d", state, event);
	return event;
}

uint8_t RotaryEnc::waitForEvent(rotaryEvent_t *eventP)
{
		if (eventP == NULL) return ESP_FAIL;
		eventP->error=ESP_OK;
		BaseType_t rc = xQueueReceive(RotaryQueue, eventP, portMAX_DELAY);
		if (rc == pdTRUE)
		{
				printf( "rc==PdTRUE, type: %0x, error: %0x pos: %d\n",eventP->type,eventP->error,position);
		} else {
			eventP->error++; 
			printf( "rc != pdTRUE, type: %0x, error: %0x pos: %d\n",eventP->type,eventP->error,position);
		}
		return eventP->error;
}

