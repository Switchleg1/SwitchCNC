#include "../../SwitchCNC.h"

#if VACUUM_SUPPORT

uint8_t Vacuum::nextState;
uint8_t Vacuum::currentState;

void Vacuum::initialize() {
	nextState = 0;
	currentState = 0;

#if VACUUM_PIN >- 1
	HAL::pinMode(VACUUM_PIN, OUTPUT);
	HAL::digitalWrite(VACUUM_PIN, LOW);
#endif
}

void Vacuum::setNextState(uint8_t state, bool immediately) {
	if (nextState == state) {
		return;
	}

	nextState = state;
	if (MachineLine::linesCount == 0 || immediately) {
		for (fast8_t i = 0; i < MACHINELINE_CACHE_SIZE; i++) {
			if (state) MachineLine::lines[i].toolFlags |= FLAG_TOOL_VACUUM_ON;
			else MachineLine::lines[i].toolFlags &= ~FLAG_TOOL_VACUUM_ON;
		}

		setState(state);
	}

	Com::printFLN(Com::tVacuumState, state); // send only new values to break update loops!
}

void Vacuum::setState(uint8_t state) {
	if (currentState == state) {
		return;
	}

#if VACUUM_PIN >- 1
	HAL::digitalWrite(VACUUM_PIN, state ? HIGH : LOW);
#endif

	currentState = state;
}

#endif