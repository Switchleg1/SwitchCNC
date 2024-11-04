#include "../../SwitchCNC.h"

#if COOLANT_SUPPORT && COOLANT_FLOOD_PIN > -1

uint8_t CoolantFloodDriver::nextState;
uint8_t CoolantFloodDriver::currentState;

void CoolantFloodDriver::initialize() {
	nextState = 0;
	currentState = 0;

	SET_OUTPUT(COOLANT_FLOOD_PIN);
	WRITE(COOLANT_FLOOD_PIN, LOW);
}

void CoolantFloodDriver::setNextState(uint8_t state, bool immediately) {
	if (nextState == state) {
		return;
	}

	nextState = state;
	if (MachineLine::linesCount == 0 || immediately) {
		for (fast8_t i = 0; i < MACHINELINE_CACHE_SIZE; i++) {
			if (state) MachineLine::lines[i].toolFlags |= FLAG_TOOL_FLOOD_ON;
			else MachineLine::lines[i].toolFlags &= ~FLAG_TOOL_FLOOD_ON;
		}

		setState(state);
	}

	Com::printFLN(Com::tVacuumState, state); // send only new values to break update loops!
}

void CoolantFloodDriver::setState(uint8_t state) {
	if (currentState == state) {
		return;
	}

	WRITE(COOLANT_FLOOD_PIN, state ? HIGH : LOW);

	currentState = state;
}

#endif