#include "SwitchCNC.h"

const uint8_t PWM::softwareKickstart[NUM_PWM] PROGMEM = { SPINDLE_KICKSTART_TIME, BOARD_FAN_KICKSTART_TIME, FAN_KICKSTART_TIME, FAN2_KICKSTART_TIME };

PWM::PWM() {
	softwarePosition = 0;
}

void PWM::doPWM(bool deincrementKickStart) {
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
	if (softwareKickstartValue[FAN_PWM_INDEX]) softwareKickstartValue[FAN_PWM_INDEX]--;
#endif
#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
	if (softwareKickstartValue[FAN2_PWM_INDEX]) softwareKickstartValue[FAN2_PWM_INDEX]--;
#endif

	softwarePosition++;

	if (softwarePosition == 0) {
#if SPINDLE_PWM_PIN > -1
		WRITE(SPINDLE_PWM_PIN, softwareValues[SPINDLE_PWM_INDEX] ? 1 : 0);
#endif
#if FAN_BOARD_PIN > -1
		WRITE(FAN_BOARD_PIN, softwareValues[FAN_BOARD_PWM_INDEX] ? 1 : 0);
#endif
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
		WRITE(FAN_PIN, softwareValues[FAN_PWM_INDEX] ? 1 : 0);
#endif
#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
		WRITE(FAN2_PIN, softwareValues[FAN2_PWM_INDEX] ? 1 : 0);
#endif
	} else {
#if SPINDLE_PWM_PIN > -1
		if (softwareValues[SPINDLE_PWM_INDEX] == softwarePosition) WRITE(SPINDLE_PWM_PIN, 0);
#endif
#if FAN_BOARD_PIN > -1
		if (softwareValues[FAN_BOARD_PWM_INDEX] == softwarePosition) WRITE(FAN_BOARD_PIN, 0);
#endif
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
		if (deincrementKickStart && softwareKickstartValue[FAN_PWM_INDEX]) softwareKickstartValue[FAN_PWM_INDEX]--;
		if (softwareKickstartValue[FAN_PWM_INDEX] == 0 && softwareValues[FAN_PWM_INDEX] == softwarePosition) WRITE(FAN_PIN, 0);
#endif
#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
		if (deincrementKickStart && softwareKickstartValue[FAN2_PWM_INDEX]) softwareKickstartValue[FAN2_PWM_INDEX]--;
		if (softwareKickstartValue[FAN2_PWM_INDEX] == 0 && softwareValues[FAN2_PWM_INDEX] == softwarePosition) WRITE(FAN2_PIN, 0);
#endif
	}
}

void PWM::clear() {
	for (uint8_t i = 0; i < NUM_PWM; i++) {
		softwareValues[i]			= 0;
		softwareKickstartValue[i]	= 0;
	}

#if SPINDLE_PWM_PIN > -1
	WRITE(SPINDLE_PWM_PIN, 0);
#endif
#if FAN_BOARD_PIN > -1
	WRITE(FAN_BOARD_PIN, 0);
#endif
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
	WRITE(FAN_PIN, 0);
#endif
#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
	WRITE(FAN2_PIN, 0);
#endif
#if defined(SUPPORT_LASER) && SUPPORT_LASER
	OCR5B = 0;
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