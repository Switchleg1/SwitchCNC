#include "../../SwitchCNC.h"

#if FEATURE_FAN_CONTROL

uint8_t FanDriver::nextSpeed;
uint8_t FanDriver::currentSpeed;

void FanDriver::initialize() {
	nextSpeed = 0;
	currentSpeed = 0;

	Machine::pwm.set(FAN_PWM_INDEX, 0);
	Machine::pwm.set(FAN2_PWM_INDEX, 0);
	Machine::pwm.set(FAN_BOARD_PWM_INDEX, FAN_BOARD_MIN_SPEED);
}

void FanDriver::setNextSpeed(uint8_t speed, bool immediately) {
	if (nextSpeed == speed) {
		return;
	}

	nextSpeed = speed;
	if (MachineLine::linesCount == 0 || immediately) {
		for (fast8_t i = 0; i < MACHINELINE_CACHE_SIZE; i++) {
			MachineLine::lines[i].fanSpeed = speed;
		}

		setSpeed(speed);
	}
}

void FanDriver::setSpeed(uint8_t speed, uint8_t fan) {
	switch(fan) {
	case FAN_INDEX:
		if (currentSpeed != speed) {
			Machine::pwm.set(FAN_PWM_INDEX, speed);
			currentSpeed = speed;
			Com::printFLN(Com::tFanspeed, speed);
		}
		break;
	case FAN2_INDEX:
		Machine::pwm.set(FAN2_PWM_INDEX, speed);
		Com::printFLN(Com::tFan2speed, speed);
		break;
	case FAN_BOARD_INDEX:
		if (speed < FAN_BOARD_MIN_SPEED) {
			speed = FAN_BOARD_MIN_SPEED;
		}
		if (speed > FAN_BOARD_MAX_SPEED) {
			speed = FAN_BOARD_MAX_SPEED;
		}
		Machine::pwm.set(FAN_BOARD_PWM_INDEX, speed);
		break;
	}
}

#endif