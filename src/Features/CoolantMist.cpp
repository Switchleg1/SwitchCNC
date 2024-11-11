#include "../../SwitchCNC.h"

#if COOLANT_SUPPORT && COOLANT_MIST_PIN > -1

uint8_t CoolantMist::nextState;
uint8_t CoolantMist::currentState;

void CoolantMist::initialize() {
	nextState = 0;
	currentState = 0;

	HAL::pinMode(COOLANT_MIST_PIN, OUTPUT);
	HAL::digitalWrite(COOLANT_MIST_PIN, LOW);
}

void CoolantMist::setNextState(uint8_t state, bool immediately) {
	if (nextState == state) {
		return;
	}

	nextState = state;
	if (MachineLine::linesCount == 0 || immediately) {
		for (fast8_t i = 0; i < MACHINELINE_CACHE_SIZE; i++) {
			if (state) MachineLine::lines[i].toolFlags |= FLAG_TOOL_MIST_ON;
			else MachineLine::lines[i].toolFlags &= ~FLAG_TOOL_MIST_ON;
		}

		setState(state);
	}

	Com::printFLN(Com::tVacuumState, state); // send only new values to break update loops!
}

void CoolantMist::setState(uint8_t state) {
	if (currentState == state) {
		return;
	}

	HAL::digitalWrite(COOLANT_MIST_PIN, state ? HIGH : LOW);

	currentState = state;
}

#endif