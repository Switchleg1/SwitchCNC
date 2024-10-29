#ifndef PWM_H
#define PWM_H

#include "configuration.h"
#include "hal.h"

#define SPINDLE_PWM_INDEX				0
#define FAN_BOARD_PWM_INDEX				SPINDLE_PWM_INDEX + 1
#define FAN_PWM_INDEX					FAN_BOARD_PWM_INDEX + 1
#define FAN2_PWM_INDEX					FAN_PWM_INDEX + 1
#define NUM_PWM							FAN2_PWM_INDEX + 1

class PWM {
public:
	static void doPWM(bool deincrementKickStart);
	static void clear();
	static void set(uint8_t index, uint8_t value);
	static inline uint8_t get(uint8_t index) {
		return softwareValues[index];
	}
	
private:
	static uint8_t softwarePosition;
	static uint8_t softwareValues[NUM_PWM];
	static uint8_t softwareKickstartValue[NUM_PWM];
	static const uint8_t softwareKickstart[NUM_PWM];
};

#endif