#ifndef PAUSE_H
#define PAUSE_H

#if PAUSE_SUPPORT

class Pause {
public:
    static void initialize();
    static void checkPeriodic();
    static uint8_t calculateSteps(uint8_t steps);

    static inline uint8_t isActive() {
        return active;
    }
#if PAUSE_STEPS > 245
    static inline uint16_t steps() {
        return pauseSteps;
    }
#else
    static inline uint8_t steps() {
        return pauseSteps;
}
#endif

private:
#if PAUSE_STEPS > 245
    static uint16_t pauseSteps;
#else
    static uint8_t pauseSteps;
#endif
    static uint8_t active;
};

#endif

#endif
