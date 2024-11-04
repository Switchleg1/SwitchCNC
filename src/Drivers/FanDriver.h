#ifndef FANDRIVER_H
#define FANDRIVER_H

#define FAN_INDEX		0
#define FAN2_INDEX		1
#define FAN_BOARD_INDEX	2

#if FAN_CONTROL_SUPPORT

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