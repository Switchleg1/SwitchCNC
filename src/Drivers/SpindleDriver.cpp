#include "../../SwitchCNC.h"

#if defined(SUPPORT_SPINDLE) && SUPPORT_SPINDLE

/**
The CNC driver differs a bit from laser driver. Here only M3,M4,M5 have an influence on the spindle.
The motor also keeps running for G0 moves. M3 and M4 wait for old moves to be finished and then enables
the motor. It then waits SPINDLE_WAIT_ON_START milliseconds for the spindle to reach target speed.
*/

int8_t SpindleDriver::direction = 0;
secondspeed_t SpindleDriver::spindleSpeed = 0;
uint16_t SpindleDriver::spindleRpm = 0;

/** Initialize cnc pins. EVENT_INITIALIZE_SPINDLE should return false to prevent default initialization.*/
void SpindleDriver::initialize()
{
    if (EVENT_INITIALIZE_SPINDLE) {
#if SPINDLE_ON_PIN > -1
        SET_OUTPUT(SPINDLE_ON_PIN);
        WRITE(SPINDLE_ON_PIN, !SPINDLE_ON_HIGH);
#endif
#if SPINDLE_DIRECTION_PIN > -1
        SET_OUTPUT(SPINDLE_DIRECTION_PIN);
#endif
    }
}
/** Turns off spindle. For event override implement
EVENT_SPINDLE_OFF
returning false.
*/
void SpindleDriver::turnOff()
{
    spindleRpm = 0;
    if (direction == 0) return; // already off
    if (EVENT_SPINDLE_OFF) {
#if SPINDLE_ON_PIN > -1
        WRITE(SPINDLE_ON_PIN, !SPINDLE_ON_HIGH);
#endif
#if SPINDLE_PWM_PIN > -1
        PWM::set(SPINDLE_PWM_INDEX, 0);
#endif
    }
    HAL::delayMilliseconds(SPINDLE_WAIT_ON_STOP);
    direction = 0;
}
/** Turns spindle on. Default implementation uses a enable pin SPINDLE_ON_PIN. If
SPINDLE_DIRECTION_PIN is not -1 it sets direction to SPINDLE_DIRECTION_CW. rpm is ignored.
To override with event system, return false for the event
EVENT_SPINDLE_CW(rpm)
*/
void SpindleDriver::turnOnCW(int32_t rpm)
{
    spindleSpeed = map(rpm, SPINDLE_RPM_MIN, SPINDLE_RPM_MAX, 0, 255);// linear interpolation

    if (direction == 1 && spindleRpm == rpm)
        return;
    if (direction == -1) {
        turnOff();
    }
    spindleRpm = rpm;// for display
    direction = 1;
    if (EVENT_SPINDLE_CW(rpm)) {
#if SPINDLE_DIRECTION_PIN > -1
        WRITE(SPINDLE_DIRECTION_PIN, SPINDLE_DIRECTION_CW);
#endif
#if SPINDLE_ON_PIN > -1
        WRITE(SPINDLE_ON_PIN, SPINDLE_ON_HIGH);
#endif
#if SPINDLE_PWM_PIN > -1
        PWM::set(SPINDLE_PWM_INDEX, spindleSpeed);
#endif
    }
    HAL::delaySeconds(SPINDLE_WAIT_ON_START);
}
/** Turns spindle on. Default implementation uses a enable pin SPINDLE_ON_PIN. If
SPINDLE_DIRECTION_PIN is not -1 it sets direction to !SPINDLE_DIRECTION_CW. rpm is ignored.
To override with event system, return false for the event
EVENT_SPINDLE_CCW(rpm)
*/
void SpindleDriver::turnOnCCW(int32_t rpm)
{
    spindleSpeed = map(rpm, SPINDLE_RPM_MIN, SPINDLE_RPM_MAX, 0, 255);// linear interpolation

    if (direction == -1 && spindleRpm == rpm)
        return;
    if (direction == 1) {
        turnOff();
    }
    spindleRpm = rpm;// for display
    direction = -1;
    if (EVENT_SPINDLE_CCW(rpm)) {
#if SPINDLE_DIRECTION_PIN > -1
        WRITE(SPINDLE_DIRECTION_PIN, !SPINDLE_DIRECTION_CW);
#endif
#if SPINDLE_ON_PIN > -1
        WRITE(SPINDLE_ON_PIN, SPINDLE_ON_HIGH);
#endif
#if SPINDLE_PWM_PIN > -1
        PWM::set(SPINDLE_PWM_INDEX, spindleSpeed);
#endif
    }
    HAL::delayMilliseconds(SPINDLE_WAIT_ON_START);
}

#endif