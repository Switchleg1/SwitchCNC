#ifndef COOLANTMISTDRIVER_H
#define COOLANTMISTDRIVER_H

#if COOLANT_SUPPORT && COOLANT_MIST_PIN > -1

class CoolantMistDriver {
public:
	static void initialize();
	static void setNextState(uint8_t state, bool immediately = false);
	static void setState(uint8_t state);

	static inline uint8_t isOn() {
		return currentState;
	}

	static inline uint8_t next() {
		return nextState;
	}

private:
	static uint8_t currentState;
	static uint8_t nextState;
};

#endif

#endif