#ifndef SPINDLEDRIVER_H
#define SPINGLEDRIVER_H

#if defined(SUPPORT_SPINDLE) && SUPPORT_SPINDLE

/**
The CNC driver differs a bit from laser driver. Here only M3,M4,M5 have an influence on the spindle.
The motor also keeps running for G0 moves. M3 and M4 wait for old moves to be finished and then enables
the motor. It then waits SPINDLE_WAIT_ON_START milliseconds for the spindle to reach target speed.
*/
class SpindleDriver {
public:
    static int8_t direction;
    static secondspeed_t spindleSpeed;
    static uint16_t spindleRpm;

    /** Initialize cnc pins. EVENT_INITIALIZE_SPINDLE should return false to prevent default initialization.*/
    static void initialize();
    /** Turns off spindle. For event override implement
    EVENT_SPINDLE_OFF
    returning false.
    */
    static void turnOff();
    /** Turns spindle on. Default implementation uses a enable pin SPINDLE_ON_PIN. If
    SPINDLE_DIRECTION_PIN is not -1 it sets direction to SPINDLE_DIRECTION_CW. rpm is ignored.
    To override with event system, return false for the event
    EVENT_SPINDLE_CW(rpm)
    */
    static void turnOnCW(int32_t rpm);
    /** Turns spindle on. Default implementation uses a enable pin SPINDLE_ON_PIN. If
    SPINDLE_DIRECTION_PIN is not -1 it sets direction to !SPINDLE_DIRECTION_CW. rpm is ignored.
    To override with event system, return false for the event
    EVENT_SPINDLE_CCW(rpm)
    */
    static void turnOnCCW(int32_t rpm);
};

#endif

#endif