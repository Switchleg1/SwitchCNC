#include "../../SwitchCNC.h"

#if SUPPORT_COOLANT && COOLANT_MIST_PIN > -1

uint8_t CoolantMistDriver::nextState;
uint8_t CoolantMistDriver::currentState;

void CoolantMistDriver::initialize() {
	nextState = 0;
	currentState = 0;

	SET_OUTPUT(COOLANT_MIST_PIN);
	WRITE(COOLANT_MIST_PIN, LOW);
}

void CoolantMistDriver::setNextState(uint8_t state, bool immediately) {
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

void CoolantMistDriver::setState(uint8_t state) {
	if (currentState == state) {
		return;
	}

	WRITE(COOLANT_MIST_PIN, state ? HIGH : LOW);

	currentState = state;
}

#endif