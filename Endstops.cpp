#include "SwitchCNC.h"

flag8_t Endstops::lastState = 0;
flag8_t Endstops::lastRead = 0;
flag8_t Endstops::accumulator = 0;
#ifdef EXTENDED_ENDSTOPS
flag8_t Endstops::lastState2 = 0;
flag8_t Endstops::lastRead2 = 0;
flag8_t Endstops::accumulator2 = 0;
#endif

void Endstops::update() {
    flag8_t newRead = 0;
#ifdef EXTENDED_ENDSTOPS
    flag8_t newRead2 = 0;
#endif
#if (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X
    if (HAL::digitalRead(X_MIN_PIN) != ENDSTOP_X_MIN_INVERTING) {
        newRead |= ENDSTOP_X_MIN_ID;
    }
#endif
#if (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X
    if (HAL::digitalRead(X_MAX_PIN) != ENDSTOP_X_MAX_INVERTING)
        newRead |= ENDSTOP_X_MAX_ID;
#endif
#if (Y_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Y
    if(HAL::digitalRead(Y_MIN_PIN) != ENDSTOP_Y_MIN_INVERTING)
        newRead |= ENDSTOP_Y_MIN_ID;
#endif
#if (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y
    if(HAL::digitalRead(Y_MAX_PIN) != ENDSTOP_Y_MAX_INVERTING)
        newRead |= ENDSTOP_Y_MAX_ID;
#endif
#if (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z
    if(HAL::digitalRead(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING)
        newRead |= ENDSTOP_Z_MIN_ID;
#endif
#if (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z
    if(HAL::digitalRead(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING)
        newRead |= ENDSTOP_Z_MAX_ID;
#endif
#if (Z2_MINMAX_PIN > -1) && MINMAX_HARDWARE_ENDSTOP_Z2
    if(HAL::digitalRead(Z2_MINMAX_PIN) != ENDSTOP_Z2_MINMAX_INVERTING)
        newRead |= ENDSTOP_Z2_MINMAX_ID;
#endif
#if Z_PROBE_SUPPORT
#if Z_PROBE_PIN == Z_MIN_PIN && MIN_HARDWARE_ENDSTOP_Z
    if(newRead & ENDSTOP_Z_MIN_ID) // prevent different results causing confusion
        newRead |= ENDSTOP_Z_PROBE_ID;
    if(!Machine::isHoming())
        newRead &= ~ENDSTOP_Z_MIN_ID; // could cause wrong signals depending on probe position
#else
    if(Z_PROBE_ON_HIGH ? HAL::digitalRead(Z_PROBE_PIN) : !HAL::digitalRead(Z_PROBE_PIN))
        newRead |= ENDSTOP_Z_PROBE_ID;
#endif
#endif
#ifdef EXTENDED_ENDSTOPS
#if HAS_PIN(Y2_MIN) && MIN_HARDWARE_ENDSTOP_Y2
    if(HAL::digitalRead(Y2_MIN_PIN) != ENDSTOP_Y2_MIN_INVERTING)
        newRead2 |= ENDSTOP_Y2_MIN_ID;
#endif
#if HAS_PIN(Y2_MAX) && MAX_HARDWARE_ENDSTOP_Y2
    if(HAL::digitalRead(Y2_MAX_PIN) != ENDSTOP_Y2_MAX_INVERTING)
        newRead2 |= ENDSTOP_Y2_MAX_ID;
#endif
#if HAS_PIN(X2_MIN) && MIN_HARDWARE_ENDSTOP_X2
    if(HAL::digitalRead(X2_MIN_PIN) != ENDSTOP_X2_MIN_INVERTING) {
        newRead2 |= ENDSTOP_X2_MIN_ID;
    }
#endif
#if HAS_PIN(X2_MAX) && MAX_HARDWARE_ENDSTOP_X2
    if(HAL::digitalRead(X2_MAX_PIN) != ENDSTOP_X2_MAX_INVERTING)
        newRead2 |= ENDSTOP_X2_MAX_ID;
#endif
#if HAS_PIN(Z2_MAX) && MAX_HARDWARE_ENDSTOP_Z2
    if(HAL::digitalRead(Z2_MAX_PIN) != ENDSTOP_Z2_MAX_INVERTING)
        newRead2 |= ENDSTOP_Z2_MAX_ID;
#endif
#if HAS_PIN(Z3_MAX) && MAX_HARDWARE_ENDSTOP_Z3
    if(HAL::digitalRead(Z3_MAX_PIN) != ENDSTOP_Z3_MAX_INVERTING)
        newRead2 |= ENDSTOP_Z3_MAX_ID;
#endif
#if HAS_PIN(Z3_MIN) && MIN_HARDWARE_ENDSTOP_Z3
    if(HAL::digitalRead(Z3_MIN_PIN) != ENDSTOP_Z3_MIN_INVERTING)
        newRead2 |= ENDSTOP_Z3_MIN_ID;
#endif

#endif
    InterruptProtectedBlock noInts; // bad idea to run this from different interrupts at once!
    lastRead &= newRead;
#ifdef EXTENDED_ENDSTOPS
    lastRead2 &= newRead2;
#endif // EXTENDED_ENDSTOPS
    if(lastRead != lastState
#ifdef EXTENDED_ENDSTOPS
            || (lastState2 != lastRead2)
#endif
      ) { // Report endstop hit changes
        lastState = lastRead;
        accumulator |= lastState;
#ifdef EXTENDED_ENDSTOPS
        lastState2 = lastRead2;
        accumulator2 |= lastState2;
#endif
        if (Machine::debugEndStop())  Endstops::report();
    } else {
        lastState = lastRead;
#ifdef EXTENDED_ENDSTOPS
        lastState2 = lastRead2;
#endif
    }
    lastRead = newRead;
#ifdef EXTENDED_ENDSTOPS
    lastRead2 = newRead2;
#endif
}

void Endstops::report() {
    Com::printF(PSTR("endstops hit: "));
#if (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X
    Com::printF(Com::tXMinColon);
    Com::printF(xMin() ? Com::tHSpace : Com::tLSpace);
#endif
#if HAS_PIN(X2_MIN) && IS_MAC_TRUE(MIN_HARDWARE_ENDSTOP_X2)
    Com::printF(PSTR("x2_min:"));
    Com::printF(x2Min() ? Com::tHSpace : Com::tLSpace);
#endif
#if (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X
    Com::printF(Com::tXMaxColon);
    Com::printF(xMax() ? Com::tHSpace : Com::tLSpace);
#endif
#if HAS_PIN(X2_MAX) && IS_MAC_TRUE(MAX_HARDWARE_ENDSTOP_X2)
    Com::printF(PSTR("x2_max:"));
    Com::printF(x2Max() ? Com::tHSpace : Com::tLSpace);
#endif
#if (Y_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Y
    Com::printF(Com::tYMinColon);
    Com::printF(yMin() ? Com::tHSpace : Com::tLSpace);
#endif
#if HAS_PIN(Y2_MIN) && IS_MAC_TRUE(MIN_HARDWARE_ENDSTOP_Y2)
    Com::printF(PSTR("y2_min:"));
    Com::printF(y2Min() ? Com::tHSpace : Com::tLSpace);
#endif
#if (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y
    Com::printF(Com::tYMaxColon);
    Com::printF(yMax() ? Com::tHSpace : Com::tLSpace);
#endif
#if HAS_PIN(Y2_MAX) && IS_MAC_TRUE(MAX_HARDWARE_ENDSTOP_Y2)
    Com::printF(PSTR("y2_max:"));
    Com::printF(y2Max() ? Com::tHSpace : Com::tLSpace);
#endif
#if (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z
    Com::printF(Com::tZMinColon);
    Com::printF(zMin() ? Com::tHSpace : Com::tLSpace);
#endif
#if (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z
    Com::printF(Com::tZMaxColon);
    Com::printF(zMax() ? Com::tHSpace : Com::tLSpace);
#endif
#if (Z2_MINMAX_PIN > -1) && MINMAX_HARDWARE_ENDSTOP_Z2
    Com::printF(PSTR("z2_minmax:"));
    Com::printF(z2MinMax() ? Com::tHSpace : Com::tLSpace);
#endif
#if Z_PROBE_SUPPORT
    Com::printF(Com::tZProbeState);
    Com::printF(zProbe() ? Com::tHSpace : Com::tLSpace);
#endif
    Com::println();
}

void Endstops::setup() {
    // Set end stops to input and enable pullup if required
#if MIN_HARDWARE_ENDSTOP_X
#if X_MIN_PIN > -1
    HAL::pinMode(X_MIN_PIN, ENDSTOP_PULLUP_X_MIN ? INPUT_PULLUP : INPUT);
#else
#error You have defined hardware x min endstop without pin assignment. Set pin number for X_MIN_PIN
#endif
#endif

#if MIN_HARDWARE_ENDSTOP_X2
#if X2_MIN_PIN > -1
    HAL::pinMode(X2_MIN_PIN, ENDSTOP_PULLUP_X2_MIN ? INPUT_PULLUP : INPUT);
#else
#error You have defined hardware 2 min endstop without pin assignment. Set pin number for X2_MIN_PIN
#endif
#endif

#if MIN_HARDWARE_ENDSTOP_Y
#if Y_MIN_PIN > -1
    HAL::pinMode(Y_MIN_PIN, ENDSTOP_PULLUP_Y_MIN ? INPUT_PULLUP : INPUT);
#else
#error You have defined hardware y min endstop without pin assignment. Set pin number for Y_MIN_PIN
#endif
#endif

#if MIN_HARDWARE_ENDSTOP_Y2
#if Y2_MIN_PIN > -1
    HAL::pinMode(Y2_MIN_PIN, ENDSTOP_PULLUP_Y2_MIN ? INPUT_PULLUP : INPUT);
#else
#error You have defined hardware y2 min endstop without pin assignment. Set pin number for Y2_MIN_PIN
#endif
#endif

#if MIN_HARDWARE_ENDSTOP_Z
#if Z_MIN_PIN > -1
    HAL::pinMode(Z_MIN_PIN, ENDSTOP_PULLUP_Z_MIN ? INPUT_PULLUP : INPUT);
#else
#error You have defined hardware z min endstop without pin assignment. Set pin number for Z_MIN_PIN
#endif
#endif

#if MINMAX_HARDWARE_ENDSTOP_Z2
#if Z2_MINMAX_PIN > -1
    HAL::pinMode(Z2_MINMAX_PIN, ENDSTOP_PULLUP_Z2_MINMAX ? INPUT_PULLUP : INPUT);
#else
#error You have defined hardware z2 minmax endstop without pin assignment. Set pin number for Z2_MINMAX_PIN
#endif
#endif

#if MAX_HARDWARE_ENDSTOP_X
#if X_MAX_PIN > -1
    HAL::pinMode(X_MAX_PIN, ENDSTOP_PULLUP_X_MAX ? INPUT_PULLUP : INPUT);
#else
#error You have defined hardware x max endstop without pin assignment. Set pin number for X_MAX_PIN
#endif
#endif

#if MAX_HARDWARE_ENDSTOP_X2
#if X2_MAX_PIN > -1
    HAL::pinMode(X2_MAX_PIN, ENDSTOP_PULLUP_X2_MAX ? INPUT_PULLUP : INPUT);
#else
#error You have defined hardware x2 max endstop without pin assignment. Set pin number for X2_MAX_PIN
#endif
#endif

#if MAX_HARDWARE_ENDSTOP_Y
#if Y_MAX_PIN > -1
    HAL::pinMode(Y_MAX_PIN, ENDSTOP_PULLUP_Y_MAX ? INPUT_PULLUP : INPUT);
#else
#error You have defined hardware y max endstop without pin assignment. Set pin number for Y_MAX_PIN
#endif
#endif

#if MAX_HARDWARE_ENDSTOP_Y2
#if Y2_MAX_PIN > -1
    HAL::pinMode(Y2_MAX_PIN, ENDSTOP_PULLUP_Y2_MAX ? INPUT_PULLUP : INPUT);
#else
#error You have defined hardware y2 max endstop without pin assignment. Set pin number for Y2_MAX_PIN
#endif
#endif

#if MAX_HARDWARE_ENDSTOP_Z
#if Z_MAX_PIN > -1
    HAL::pinMode(Z_MAX_PIN, ENDSTOP_PULLUP_Z_MAX ? INPUT_PULLUP : INPUT);
#else
#error You have defined hardware z max endstop without pin assignment. Set pin number for Z_MAX_PIN
#endif
#endif
}
