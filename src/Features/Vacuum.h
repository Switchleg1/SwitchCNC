#ifndef VACUUM_H
#define VACUUM_H

#if VACUUM_SUPPORT

class Vacuum {
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