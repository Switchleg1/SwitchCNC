#include "../../SwitchCNC.h"

#if FAN_PIN >- 1 && FEATURE_FAN_CONTROL

uint8_t FanDriver::nextSpeed;
uint8_t FanDriver::currentSpeed;

void FanDriver::initialize() {
	nextSpeed = 0;
	currentSpeed = 0;

	Machine::pwm.set(FAN_PWM_INDEX, 0);
	Machine::pwm.set(FAN2_PWM_INDEX, 0);
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
#if FAN2_PIN >- 1 && FEATURE_FAN2_CONTROL
	if (fan) {
		Machine::pwm.set(FAN2_PWM_INDEX, speed);

		Com::printFLN(Com::tFan2speed, speed); // send only new values to break update loops!

		return;
	}
#endif

	if (currentSpeed == speed) {
		return;
	}

	Machine::pwm.set(FAN_PWM_INDEX, speed);

	currentSpeed = speed;

	Com::printFLN(Com::tFanspeed, speed); // send only new values to break update loops!
}

#endif