#include "../../SwitchCNC.h"

#if LASER_SUPPORT

uint8_t Laser::laserOn;
uint8_t Laser::intensityMultiplier;
uint8_t Laser::currentIntensity;
uint8_t Laser::minimumIntensity;
uint8_t Laser::nextIntensity;
float Laser::currentTemperature;

void Laser::initialize() {
    laserOn = false;
    intensityMultiplier = 100;
    currentIntensity = 0;
    minimumIntensity = 0;
    nextIntensity = 0;
    currentTemperature = 0;

    if (EVENT_INITIALIZE_LASER) {
        // Set up Timer5 in 8-bit Fast PWM mode, with a TOP value of 255
        TCCR5A |= (1 << COM5B1) | (1 << WGM50); // set up Fast PWM
        TCCR5B = (TCCR5B & 0xF0) | (1 << WGM52) | (1 << CS51);
        OCR5A = 0xFF;
        OCR5B = 0;  // Set PWM value to zero value

        HAL::pinMode(45, OUTPUT);
    }
}

void Laser::turnOn(uint8_t minIntensity, uint8_t quiet) {
    if (laserOn) {
        setMinimumIntensity(minIntensity, true);
        return;
    }
    Commands::waitUntilEndOfAllMoves();

    laserOn = true;

    setMinimumIntensity(minIntensity, true);

#if LASER_WARMUP_TIME > 0
    if (minIntensity) {
        Commands::waitUntilEndOfAllMoves();
        HAL::delaySeconds(LASER_WARMUP_TIME);
    }
#endif

    if (!quiet) {
        printState();
    }
}

void Laser::turnOff(uint8_t quiet) {
    if (!laserOn) {
        return;
    }

    Commands::waitUntilEndOfAllMoves();

    laserOn = false;

    currentIntensity = 0;
    OCR5B = 0;

    if (!quiet) {
        printState();
    }
}

void Laser::setIntensity(uint16_t intensity) {
    if (laserOn) {
        if (intensity) {
            intensity *= intensityMultiplier;
            intensity /= 100;
            if (intensity > LASER_PWM_MAX) {
                intensity = LASER_PWM_MAX;
            }
        }

        if (intensity < minimumIntensity) {
            intensity = minimumIntensity;
        }

        currentIntensity = intensity;

        OCR5B = currentIntensity;
    }
}

void Laser::setNextIntensity(uint8_t intensity, uint8_t immediately) {
    if (intensity > LASER_PWM_MAX) {
        intensity = LASER_PWM_MAX;
    }

    if (intensity < minimumIntensity) {
        intensity = minimumIntensity;
    }

    nextIntensity = intensity;

    if (MachineLine::linesCount == 0 || immediately) {
        for (uint8_t i = 0; i < MACHINELINE_CACHE_SIZE; i++) {
            MachineLine::lines[i].laserIntensity = intensity;
        }

        setIntensity(nextIntensity);
    }
}

void Laser::setIntensityMultiplier(uint8_t multiplier) {
    if (multiplier < 25) multiplier = 25;
    if (multiplier > 200) multiplier = 200;
    intensityMultiplier = multiplier;

    setIntensity(currentIntensity);

    Com::printFLN(Com::tIntensityMultiply, multiplier);
}

void Laser::setMinimumIntensity(uint8_t intensity, uint8_t quiet) {
    minimumIntensity = intensity;

    if (currentIntensity < minimumIntensity) {
        setIntensity(0);
    }

    if (!quiet) {
        printState();
    }
}

void Laser::updateTemperature() {
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
            currentTemperature = oldTemp + ((float)(tempRaw - oldRaw) * (float)(newTemp - oldTemp) / (float)(newRaw - oldRaw));
            return;
        }
        oldTemp = newTemp;
        oldRaw = newRaw;
        i++;
    }

    currentTemperature = newTemp;
}

void Laser::printState() {
    Com::printF(Com::tLaserState, laserOn);
    if (laserOn && minimumIntensity) {
        Com::printFLN(Com::tSpaceMinimumIntensity, minimumIntensity);
    }
}

#endif