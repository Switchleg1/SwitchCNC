#include "SwitchCNC.h"

const uint8_t PWM::softwareKickstart[NUM_PWM] PROGMEM = { SPINDLE_KICKSTART_TIME, FAN_KICKSTART_TIME, FAN2_KICKSTART_TIME, FAN_BOARD_KICKSTART_TIME };

PWM::PWM() {
	softwarePosition = 0;

#if SPINDLE_PWM_PIN > -1
	SET_OUTPUT(SPINDLE_PWM_PIN);
#endif
#if FAN_CONTROL_SUPPORT
#if FAN_PIN > -1
	SET_OUTPUT(FAN_PIN);
#endif
#if FAN2_PIN > -1
	SET_OUTPUT(FAN2_PIN);
#endif
#if FAN_BOARD_PIN > -1
	SET_OUTPUT(FAN_BOARD_PIN);
#endif
#endif

	clear();
}

void PWM::doPWM(bool deincrementKickStart) {
	if (softwarePosition++ == 0) {
#if SPINDLE_SUPPORT && SPINDLE_PWM_PIN > -1
		WRITE(SPINDLE_PWM_PIN, softwareValues[SPINDLE_PWM_INDEX] ? HIGH : LOW);
#endif
#if FAN_CONTROL_SUPPORT
#if FAN_PIN > -1
		WRITE(FAN_PIN, softwareValues[FAN_PWM_INDEX] ? HIGH : LOW);
#endif
#if FAN2_PIN > -1
		WRITE(FAN2_PIN, softwareValues[FAN2_PWM_INDEX] ? HIGH : LOW);
#endif
#if FAN_BOARD_PIN > -1
		WRITE(FAN_BOARD_PIN, softwareValues[FAN_BOARD_PWM_INDEX] ? HIGH : LOW);
#endif
#endif
	} else {
#if SPINDLE_SUPPORT && SPINDLE_PWM_PIN > -1
		if (deincrementKickStart && softwareKickstartValue[SPINDLE_PWM_INDEX]) softwareKickstartValue[SPINDLE_PWM_INDEX]--;
		if (softwareKickstartValue[SPINDLE_PWM_INDEX] == 0 && softwareValues[SPINDLE_PWM_INDEX] == softwarePosition) WRITE(SPINDLE_PWM_PIN, LOW);
#endif
#if FAN_CONTROL_SUPPORT
#if FAN_PIN > -1
		if (deincrementKickStart && softwareKickstartValue[FAN_PWM_INDEX]) softwareKickstartValue[FAN_PWM_INDEX]--;
		if (softwareKickstartValue[FAN_PWM_INDEX] == 0 && softwareValues[FAN_PWM_INDEX] == softwarePosition) WRITE(FAN_PIN, LOW);
#endif
#if FAN2_PIN > -1
		if (deincrementKickStart && softwareKickstartValue[FAN2_PWM_INDEX]) softwareKickstartValue[FAN2_PWM_INDEX]--;
		if (softwareKickstartValue[FAN2_PWM_INDEX] == 0 && softwareValues[FAN2_PWM_INDEX] == softwarePosition) WRITE(FAN2_PIN, LOW);
#endif
#if FAN_BOARD_PIN > -1
		if (deincrementKickStart && softwareKickstartValue[FAN_BOARD_PWM_INDEX]) softwareKickstartValue[FAN_BOARD_PWM_INDEX]--;
		if (softwareKickstartValue[FAN_BOARD_PWM_INDEX] == 0 && softwareValues[FAN_BOARD_PWM_INDEX] == softwarePosition) WRITE(FAN_BOARD_PIN, LOW);
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
	WRITE(SPINDLE_PWM_PIN, LOW);
#endif
#if FAN_CONTROL_SUPPORT
#if FAN_BOARD_PIN > -1
	WRITE(FAN_BOARD_PIN, LOW);
#endif
#if FAN_PIN > -1
	WRITE(FAN_PIN, LOW);
#endif
#if FAN2_PIN > -1
	WRITE(FAN2_PIN, LOW);
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