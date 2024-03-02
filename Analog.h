#ifndef ANALOG_H
#define ANALOG_H

#include "configuration.h"
#include "hal.h"

#define ANALOG_REF_AREF                 0
#define ANALOG_REF_AVCC                 _BV(REFS0)
#define ANALOG_REF_INT_1_1              _BV(REFS1)
#define ANALOG_REF_INT_2_56             _BV(REFS0) | _BV(REFS1)
#define ANALOG_REF                      ANALOG_REF_AVCC
#define ANALOG_PRESCALER                _BV(ADPS0)|_BV(ADPS1)|_BV(ADPS2)
#define ANALOG_INPUT_SAMPLE             5
#define ANALOG_INPUT_BITS               10
#define ANALOG_OUTPUT_BITS              12
#define ANALOG_MAX_VALUE                ((1 << ANALOG_OUTPUT_BITS) - 1)

#if SPEED_DIAL && SPEED_DIAL_PIN > -1
#define SPEED_DIAL_ANALOG_INPUTS        1
#define SPEED_DIAL_ANALOG_CHANNEL       SPEED_DIAL_PIN
#define SPEED_DIAL_COMMA			    ,
#else
#define SPEED_DIAL_ANALOG_INPUTS        0
#define SPEED_DIAL_ANALOG_CHANNEL
#define SPEED_DIAL_COMMA			    
#endif
#define SPEED_DIAL_ANALOG_INDEX         0

#if SUPPORT_LASER && LASER_TEMP_PIN > -1
#define LASER_TEMP_ANALOG_INPUTS        1
#define LASER_TEMP_ANALOG_CHANNEL       SPEED_DIAL_COMMA LASER_TEMP_PIN
#define LASER_TEMP_COMMA			    ,
#else
#define LASER_TEMP_ANALOG_INPUTS        0
#define LASER_TEMP_ANALOG_CHANNEL
#define LASER_TEMP_COMMA				SPEED_DIAL_COMMA
#endif
#define LASER_TEMP_ANALOG_INDEX         SPEED_DIAL_ANALOG_INDEX + SPEED_DIAL_ANALOG_INPUTS


#define ANALOG_INPUTS                   (SPEED_DIAL_ANALOG_INPUTS + LASER_TEMP_ANALOG_INPUTS)
#define ANALOG_INPUT_CHANNELS           {SPEED_DIAL_ANALOG_CHANNEL LASER_TEMP_ANALOG_CHANNEL}

#if ANALOG_INPUTS > 0
class Analog {
public:
	Analog();

	void start();
	void read();

#if ANALOG_OUTPUT_BITS <= 8
	volatile uint8_t values[ANALOG_INPUTS];
#elif ANALOG_OUTPUT_BITS <= 16
	volatile uint16_t values[ANALOG_INPUTS];
#else
	volatile uint32_t values[ANALOG_INPUTS];
#endif

private:
	uint8_t inputPosition;
	uint8_t inputCounter[ANALOG_INPUTS];
#if (ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE)  <= 8
	uint8_t inputValues[ANALOG_INPUTS];
#elif (ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE)  <= 16
	uint16_t inputValues[ANALOG_INPUTS];
#else
	uint32_t inputValues[ANALOG_INPUTS];
#endif
	static const uint8_t inputChannels[ANALOG_INPUTS];
};
#endif

#endif