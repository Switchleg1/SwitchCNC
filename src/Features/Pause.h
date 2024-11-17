#ifndef PAUSE_H
#define PAUSE_H

#if PAUSE_SUPPORT

class Pause {
public:
    static void initialize();
    static void checkPeriodic();
    static uint8_t calculateSteps(uint8_t steps);
#if PAUSE_SUPPORT_CANCEL
    static inline uint8_t setCancel(uint8_t state) {
        cancel = state;
    }
    static inline uint8_t doCancel() {
        return cancel;
    }
#endif
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
#if PAUSE_SUPPORT_CANCEL
    static uint8_t cancel;
#endif
};

#endif

#endif
