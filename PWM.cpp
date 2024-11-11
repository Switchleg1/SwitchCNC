#include "SwitchCNC.h"

const uint8_t PWM::softwareKickstart[NUM_PWM] PROGMEM = { SPINDLE_KICKSTART_TIME, FAN_KICKSTART_TIME, FAN2_KICKSTART_TIME, FAN_BOARD_KICKSTART_TIME };

PWM::PWM() {
	softwarePosition = 0;

#if SPINDLE_PWM_PIN > -1
	HAL::pinMode(SPINDLE_PWM_PIN, OUTPUT);
#endif

#if FAN_CONTROL_SUPPORT
#if FAN_PIN > -1
	HAL::pinMode(FAN_PIN, OUTPUT);
#endif
#if FAN2_PIN > -1
	HAL::pinMode(FAN2_PIN, OUTPUT);
#endif
#if FAN_BOARD_PIN > -1
	HAL::pinMode(FAN_BOARD_PIN, OUTPUT);
#endif
#endif

	clear();
}

void PWM::doPWM(bool deincrementKickStart) {
	if (softwarePosition++ == 0) {
#if SPINDLE_SUPPORT && SPINDLE_PWM_PIN > -1
		HAL::digitalWrite(SPINDLE_PWM_PIN, softwareValues[SPINDLE_PWM_INDEX] ? HIGH : LOW, false);
#endif
#if FAN_CONTROL_SUPPORT
#if FAN_PIN > -1
		HAL::digitalWrite(FAN_PIN, softwareValues[FAN_PWM_INDEX] ? HIGH : LOW, false);
#endif
#if FAN2_PIN > -1
		HAL::digitalWrite(FAN2_PIN, softwareValues[FAN2_PWM_INDEX] ? HIGH : LOW, false);
#endif
#if FAN_BOARD_PIN > -1
		HAL::digitalWrite(FAN_BOARD_PIN, softwareValues[FAN_BOARD_PWM_INDEX] ? HIGH : LOW, false);
#endif
#endif
	} else {
#if SPINDLE_SUPPORT && SPINDLE_PWM_PIN > -1
		if (deincrementKickStart && softwareKickstartValue[SPINDLE_PWM_INDEX]) softwareKickstartValue[SPINDLE_PWM_INDEX]--;
		if (softwareKickstartValue[SPINDLE_PWM_INDEX] == 0 && softwareValues[SPINDLE_PWM_INDEX] == softwarePosition) HAL::digitalWrite(SPINDLE_PWM_PIN, LOW, false);
#endif
#if FAN_CONTROL_SUPPORT
#if FAN_PIN > -1
		if (deincrementKickStart && softwareKickstartValue[FAN_PWM_INDEX]) softwareKickstartValue[FAN_PWM_INDEX]--;
		if (softwareKickstartValue[FAN_PWM_INDEX] == 0 && softwareValues[FAN_PWM_INDEX] == softwarePosition) HAL::digitalWrite(FAN_PIN, LOW, false);
#endif
#if FAN2_PIN > -1
		if (deincrementKickStart && softwareKickstartValue[FAN2_PWM_INDEX]) softwareKickstartValue[FAN2_PWM_INDEX]--;
		if (softwareKickstartValue[FAN2_PWM_INDEX] == 0 && softwareValues[FAN2_PWM_INDEX] == softwarePosition) HAL::digitalWrite(FAN2_PIN, LOW, false);
#endif
#if FAN_BOARD_PIN > -1
		if (deincrementKickStart && softwareKickstartValue[FAN_BOARD_PWM_INDEX]) softwareKickstartValue[FAN_BOARD_PWM_INDEX]--;
		if (softwareKickstartValue[FAN_BOARD_PWM_INDEX] == 0 && softwareValues[FAN_BOARD_PWM_INDEX] == softwarePosition) HAL::digitalWrite(FAN_BOARD_PIN, LOW, false);
#endif
#endif
	}
}

void PWM::clear() {
	for (uint8_t i = 0; i < NUM_PWM; i++) {
		softwareValues[i]			= 0;
		softwareKickstartValue[i]	= 0;
	}

#if SPINDLE_PWM_PIN > -1
	HAL::digitalWrite(SPINDLE_PWM_PIN, LOW);
#endif
#if FAN_CONTROL_SUPPORT
#if FAN_BOARD_PIN > -1
	HAL::digitalWrite(FAN_BOARD_PIN, LOW);
#endif
#if FAN_PIN > -1
	HAL::digitalWrite(FAN_PIN, LOW);
#endif
#if FAN2_PIN > -1
	HAL::digitalWrite(FAN2_PIN, LOW);
#endif
#endif
}

void PWM::set(uint8_t index, uint8_t value) {
	uint8_t pwm = softwareValues[index];
	if (pwm == value) {
		return;
	}

	uint8_t kickstart = softwareKickstart[index];
	if (kickstart && value > pwm) {
		softwareKickstartValue[index] = kickstart / (pwm ? 100 : 25);
	}

	softwareValues[index] = value;
}