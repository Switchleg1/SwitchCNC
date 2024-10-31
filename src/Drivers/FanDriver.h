#ifndef FANDRIVER_H
#define FANDRIVER_H

#if FAN_PIN >- 1 && FEATURE_FAN_CONTROL

class FanDriver {
public:
	static void initialize();
	static void setNextSpeed(uint8_t speed, bool immediately = false);
	static void setSpeed(uint8_t speed, uint8_t fan = 0);

	static inline uint8_t speed() {
		return currentSpeed;
	}

	static inline uint8_t next() {
		return nextSpeed;
	}

private:
	static uint8_t currentSpeed;
	static uint8_t nextSpeed;
};

#endif

#endif