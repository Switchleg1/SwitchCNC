#include "../../SwitchCNC.h"

#if PAUSE_SUPPORT

#if PAUSE_STEPS > 245
uint16_t Pause::pauseSteps;
#else
uint8_t Pause::pauseSteps;
#endif
uint8_t Pause::active;
#if PAUSE_SUPPORT_CANCEL
uint8_t Pause::cancel;
#endif

void Pause::initialize() {
    active = 0;
    pauseSteps = 0;

#if PAUSE_PIN > -1
#if PAUSE_PULLUP
    HAL::pinMode(PAUSE_PIN, INPUT_PULLUP);
#else
    HAL::pinMode(PAUSE_PIN, INPUT);
#endif
#endif
}

void Pause::checkPeriodic() {
#if PAUSE_PIN > -1
    bool pausePin = (HAL::digitalRead(PAUSE_PIN) != !PAUSE_INVERTING);
#else
    bool pausePin = false;
#endif
#if PAUSE_SUPPORT_CANCEL && PAUSE_CANCEL_PIN > -1
    cancel = (HAL::digitalRead(PAUSE_CANCEL_PIN) != !PAUSE_INVERTING);
#endif
    if (cancel) pausePin = true;
    if (active != pausePin) {
        if (!MachineLine::hasLines()) {
            if (pausePin) pauseSteps = PAUSE_STEPS;
            else pauseSteps = 0;
        }

        if (pausePin) Com::printFLN(Com::tPaused);
        else Com::printFLN(Com::tUnpaused);

        active = pausePin;
    }
}

uint8_t Pause::calculateSteps(uint8_t steps) {
    if (active) {
        pauseSteps += steps;
    }
    else {
        if (pauseSteps > steps) {
            pauseSteps -= steps;
        }
        else {
            if (!pauseSteps) {
                return 1;
            }

            pauseSteps = 0;
        }
    }

    return 0;
}

#endif