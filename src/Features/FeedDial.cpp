#include "../../SwitchCNC.h"

#if FEED_DIAL_SUPPORT

uint8_t FeedDial::currentValue;

void FeedDial::initialize() {
    currentValue = FEED_DIAL_MIN_PERCENT;

#if PAUSE_PIN > -1
    SET_INPUT(PAUSE_PIN);
#if PAUSE_PULLUP && PAUSE_PIN > -1
    PULLUP(PAUSE_PIN, HIGH);
#endif
#endif
}

void FeedDial::checkPeriodic() {
#if FEED_DIAL_PIN > -1
    uint16_t minSpeedValue = (uint16_t)FEED_DIAL_MAX_VALUE * FEED_DIAL_MIN_PERCENT / 100;
    uint16_t speedDivisor = FEED_DIAL_DIV * 100 / (100 - FEED_DIAL_MIN_PERCENT);
#if FEED_DIAL_INVERT
    uint8_t value = (ANALOG_MAX_VALUE - Machine::analog.values[FEED_DIAL_ANALOG_INDEX] + speedDivisor / 2) / speedDivisor + minSpeedValue;
#else
    uint8_t value = (Machine::analog.values[FEED_DIAL_ANALOG_INDEX] + speedDivisor / 2) / speedDivisor + minSpeedValue;
#endif
    if (value > FEED_DIAL_MAX_VALUE) {
        value = FEED_DIAL_MAX_VALUE;
    }

    currentValue = value;
#else
    currentValue = FEED_DIAL_MAX_VALUE;
#endif
}

#endif