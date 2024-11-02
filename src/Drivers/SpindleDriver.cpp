#include "../../SwitchCNC.h"

#if SUPPORT_SPINDLE

/**
The CNC driver differs a bit from laser driver. Here only M3,M4,M5 have an influence on the spindle.
The motor also keeps running for G0 moves. M3 and M4 wait for old moves to be finished and then enables
the motor. It then waits SPINDLE_WAIT_ON_START milliseconds for the spindle to reach target speed.
*/
uint8_t SpindleDriver::rpmMultiplier;
uint16_t SpindleDriver::targetSpindleRpm;
int8_t SpindleDriver::currentDirection;
uint8_t SpindleDriver::currentSpindleSpeed;
uint16_t SpindleDriver::currentSpindleRpm;

/** Initialize cnc pins. EVENT_INITIALIZE_SPINDLE should return false to prevent default initialization.*/
void SpindleDriver::initialize() {
    rpmMultiplier = 100;
    targetSpindleRpm = 0;
    currentDirection = 0;
    currentSpindleSpeed = 0;
    currentSpindleRpm = 0;

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
void SpindleDriver::turnOff(uint8_t quiet) {
    if (currentDirection == SPINDLE_OFF) {
        return; // already off
    }

    if (EVENT_SPINDLE_OFF) {
        Commands::waitUntilEndOfAllMoves();

#if SPINDLE_ON_PIN > -1
        WRITE(SPINDLE_ON_PIN, !SPINDLE_ON_HIGH);
#endif

        targetSpindleRpm = 0;
        currentDirection = 0;

        setRpm();

        HAL::delaySeconds(SPINDLE_WAIT_ON_STOP);

        if (!quiet) {
            printState();
        }
    }
}
/** Turns spindle on. Default implementation uses a enable pin SPINDLE_ON_PIN. If
SPINDLE_DIRECTION_PIN is not -1 it sets direction to SPINDLE_DIRECTION_CW. rpm is ignored.
To override with event system, return false for the event
EVENT_SPINDLE_CW(rpm)
*/
void SpindleDriver::turnOn(int8_t direction, uint16_t rpm, uint8_t quiet) {
    if (direction == SPINDLE_OFF || (currentDirection == direction && targetSpindleRpm == rpm)) {
        return;
    }

    if (currentDirection != direction) {
        turnOff();
    }

    if (direction > 0 ? EVENT_SPINDLE_CW(rpm) : EVENT_SPINDLE_CW(rpm)) {
        Commands::waitUntilEndOfAllMoves();

        if (currentDirection == SPINDLE_OFF) {
            Endstops::update();
            Endstops::update();
            while (!Endstops::zProbe()) {
                Com::printFLN(PSTR("Unclip Zprobe from tool prior to starting spindle!"));
                millis_t wait = 1000 + HAL::timeInMilliseconds();

                while (wait - HAL::timeInMilliseconds() < 100000) {
                    Machine::defaultLoopActions();
                }
                Endstops::update();
                Endstops::update();
            }
        }

#if SPINDLE_DIRECTION_PIN > -1
        WRITE(SPINDLE_DIRECTION_PIN, direction > 0 ? SPINDLE_DIRECTION_CW : SPINDLE_DIRECTION_CCW);
#endif
#if SPINDLE_ON_PIN > -1
        WRITE(SPINDLE_ON_PIN, SPINDLE_ON_HIGH);
#endif

        targetSpindleRpm = rpm;
        currentDirection = direction;

        setRpm();

        HAL::delaySeconds(SPINDLE_WAIT_ON_START);

        if (!quiet) {
            printState();
        }
    }
}

void SpindleDriver::setRpmMultiplier(uint8_t multiplier) {
    rpmMultiplier = multiplier;

    setRpm();
}

void SpindleDriver::setRpm() {
    if (currentDirection) {
        currentSpindleRpm = (uint32_t)rpmMultiplier * targetSpindleRpm / 100;
        if (currentSpindleRpm > SPINDLE_RPM_MAX) {
            currentSpindleRpm = SPINDLE_RPM_MAX;
        }

        if (currentSpindleRpm < SPINDLE_RPM_MIN) {
            currentSpindleRpm = SPINDLE_RPM_MIN;
        }
    }
    else {
        currentSpindleRpm = 0;
    }

    currentSpindleSpeed = map(currentSpindleRpm, SPINDLE_RPM_MIN, SPINDLE_RPM_MAX, 0, 255);// linear interpolation
    Machine::pwm.set(SPINDLE_PWM_INDEX, currentSpindleSpeed);
}

void SpindleDriver::printState() {
    Com::printF(Com::tSpindleState, currentDirection);
    Com::printFLN(Com::tSpaceRpm, currentSpindleRpm);
}

#endif