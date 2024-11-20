#include "../../SwitchCNC.h"

#if COOLANT_SUPPORT && COOLANT_FLOOD_PIN > -1

uint8_t CoolantFlood::nextState;
uint8_t CoolantFlood::currentState;

void CoolantFlood::initialize() {
	nextState = 0;
	currentState = 0;

	HAL::pinMode(COOLANT_FLOOD_PIN, OUTPUT);
	HAL::digitalWrite(COOLANT_FLOOD_PIN, LOW);
}

void CoolantFlood::setNextState(uint8_t state, bool immediately) {
	if (nextState == state) {
		return;
	}

	nextState = state;
	if (MachineLine::linesCount == 0 || immediately) {
		for (uint8_t i = 0; i < MACHINELINE_CACHE_SIZE; i++) {
			if (state) MachineLine::lines[i].toolFlags |= FLAG_TOOL_FLOOD_ON;
			else MachineLine::lines[i].toolFlags &= ~FLAG_TOOL_FLOOD_ON;
		}

		setState(state);
	}

	Com::printFLN(Com::tVacuumState, state); // send only new values to break update loops!
}

void CoolantFlood::setState(uint8_t state) {
	if (currentState == state) {
		return;
	}

	HAL::digitalWrite(COOLANT_FLOOD_PIN, state ? HIGH : LOW);

	currentState = state;
}

#endif