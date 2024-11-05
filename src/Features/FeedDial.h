#ifndef FEEDDIAL_H
#define FEEDDIAL_H

#define FEED_DIAL_MAX_VALUE    (1 << FEED_DIAL_BITS)
#define FEED_DIAL_DIV          (ANALOG_MAX_VALUE >> FEED_DIAL_BITS)


#if FEED_DIAL_SUPPORT

class FeedDial {
public:
    static void initialize();
    static void checkPeriodic();
    static inline uint8_t value() {
        return currentValue;
    }

private:
    static uint8_t currentValue;
};

#endif

#endif
