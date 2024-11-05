#ifndef COOLANTFLOOD_H
#define COOLANTFLOOD_H

#if COOLANT_SUPPORT && COOLANT_FLOOD_PIN > -1

class CoolantFlood {
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