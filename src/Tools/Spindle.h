#ifndef SPINDLE_H
#define SPINDLE_H

#define SPINDLE_CW  1
#define SPINDLE_CCW -1
#define SPINDLE_OFF 0

#if SPINDLE_SUPPORT

/**
The CNC driver differs a bit from laser driver. Here only M3,M4,M5 have an influence on the spindle.
The motor also keeps running for G0 moves. M3 and M4 wait for old moves to be finished and then enables
the motor. It then waits SPINDLE_WAIT_ON_START milliseconds for the spindle to reach target speed.
*/
class Spindle {
public:
    /** Initialize cnc pins. EVENT_INITIALIZE_SPINDLE should return false to prevent default initialization.*/
    static void initialize();
    /** Turns off spindle. For event override implement
    EVENT_SPINDLE_OFF
    returning false.
    */
    static void turnOff(uint8_t quiet = false);
    /** Turns spindle on. Default implementation uses a enable pin SPINDLE_ON_PIN. If
    SPINDLE_DIRECTION_PIN is not -1 it sets direction to SPINDLE_DIRECTION_CW. rpm is ignored.
    To override with event system, return false for the event
    EVENT_SPINDLE_CW(rpm)
    */
    static void turnOn(int8_t direction, uint16_t rpm, uint8_t quiet = false);

    static void setRpmMultiplier(uint8_t multiplier);

    static inline int8_t direction() {
        return currentDirection;
    }

    static inline uint16_t spindleTargetRpm() {
        return targetSpindleRpm;
    }

    static inline uint8_t spindleSpeed() {
        return currentSpindleSpeed;
    }

    static inline uint16_t spindleRpm() {
        return currentSpindleRpm;
    }

private:
    static void setRpm();
    static void printState();

    static uint8_t rpmMultiplier;
    static uint16_t targetSpindleRpm;

    static int8_t currentDirection;
    static uint8_t currentSpindleSpeed;
    static uint16_t currentSpindleRpm;
};

#endif

#endif