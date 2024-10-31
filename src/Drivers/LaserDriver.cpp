#include "../../SwitchCNC.h"

#if defined(SUPPORT_LASER) && SUPPORT_LASER

uint8_t LaserDriver::intensity = LASER_PWM_MAX; // Intensity to use for next move queued if we want lasers. This is NOT the current value!
uint8_t LaserDriver::minIntensity = 0; // Intensity to use for next move queued if we want lasers. This is NOT the current value!
bool LaserDriver::laserOn = false;
bool LaserDriver::firstMove = true;
float LaserDriver::temperature = 0;

void LaserDriver::initialize() {
    if (EVENT_INITIALIZE_LASER) {
#if defined(SUPPORT_LASER) && SUPPORT_LASER
        // Set up Timer5 in 8-bit Fast PWM mode, with a TOP value of 255
        TCCR5A |= (1 << COM5B1) | (1 << WGM50); // set up Fast PWM
        TCCR5B = (TCCR5B & 0xF0) | (1 << WGM52) | (1 << CS51);
        OCR5A = 0xFF;
        OCR5B = 0;  // Set PWM value to zero value
        SET_OUTPUT(45);
#endif
    }
}

void LaserDriver::changeIntensity(uint8_t newIntensity) {
#if defined(SUPPORT_LASER) && SUPPORT_LASER
    uint16_t addFlow = newIntensity;
    if (addFlow) {
        addFlow *= Machine::intensityMultiply;
        addFlow /= 100;
        if (addFlow > 0xFF) {
            addFlow = 0xFF;
        }
    }

    if (addFlow < minIntensity) {
        addFlow = minIntensity;
    }
    OCR5B = addFlow;
#endif
}

void LaserDriver::turnOn() {
    if (laserOn) {
        return;
    }

    laserOn = true;
    Com::printFLN(Com::tLaserOn, (int)intensity);
}

void LaserDriver::turnOff(bool instantOff) {
    if (!laserOn) {
        return;
    }

    laserOn = false;
    if (instantOff) OCR5B = 0;
    Com::printFLN(Com::tLaserOff);
}

void LaserDriver::updateTemperature() {
#if ANALOG_OUTPUT_BITS > TEMPERATURE_TABLE_BITS
    uint16_t tempRaw = Machine::analog.values[LASER_TEMP_ANALOG_INDEX] >> (ANALOG_OUTPUT_BITS - TEMPERATURE_TABLE_BITS);
#elif ANALOG_OUTPUT_BITS == TEMPERATURE_TABLE_BITS
    uint16_t tempRaw = Machine::analog.values[LASER_TEMP_ANALOG_INDEX];
#else
    uint16_t tempRaw = Machine::analog.values[LASER_TEMP_ANALOG_INDEX] << (TEMPERATURE_TABLE_BITS - ANALOG_OUTPUT_BITS);
#endif

    int16_t oldRaw = pgm_read_word(&temperatureTables[0][TEMPERATURE_RAW][LASER_TEMP_SENSOR_TYPE]);
    int16_t oldTemp = pgm_read_word(&temperatureTables[0][TEMERATURE_TEMP][LASER_TEMP_SENSOR_TYPE]);
    int16_t newRaw, newTemp = 0;
    uint8_t i = 1;
    while (i < TEMPERATURE_TABLE_ENTRIES) {
        newRaw = pgm_read_word(&temperatureTables[i][TEMPERATURE_RAW][LASER_TEMP_SENSOR_TYPE]);
        newTemp = pgm_read_word(&temperatureTables[i][TEMERATURE_TEMP][LASER_TEMP_SENSOR_TYPE]);
        if (newRaw > tempRaw) {
            temperature = oldTemp + ((float)(tempRaw - oldRaw) * (float)(newTemp - oldTemp) / (float)(newRaw - oldRaw));
            return;
        }
        oldTemp = newTemp;
        oldRaw = newRaw;
        i++;
    }

    temperature = newTemp;
}

#endif // SUPPORT_LASER