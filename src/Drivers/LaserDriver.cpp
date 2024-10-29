#include "../../SwitchCNC.h"

#if defined(SUPPORT_LASER) && SUPPORT_LASER

secondspeed_t LaserDriver::intensity = LASER_PWM_MAX; // Intensity to use for next move queued if we want lasers. This is NOT the current value!
secondspeed_t LaserDriver::minIntensity = 0; // Intensity to use for next move queued if we want lasers. This is NOT the current value!
bool LaserDriver::laserOn = false;
bool LaserDriver::firstMove = true;
float LaserDriver::temperature = 0;
uint8_t LaserDriver::tempCount = 0;
uint16_t LaserDriver::tempRaw = 0;

void LaserDriver::initialize()
{
    if (EVENT_INITIALIZE_LASER)
    {
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

void LaserDriver::changeIntensity(secondspeed_t newIntensity)
{
#if defined(SUPPORT_LASER) && SUPPORT_LASER
    uint16_t addFlow = newIntensity;
    if (addFlow) {
        addFlow *= Machine::intensityMultiply;
        addFlow /= 100;
        if (addFlow > 0xFF) {
            addFlow = 0xFF;
        }
    }

    if (addFlow < LaserDriver::minIntensity) {
        addFlow = LaserDriver::minIntensity;
    }
    OCR5B = addFlow;
#endif
}

void LaserDriver::turnOn()
{
    LaserDriver::laserOn = true;
    Com::printFLN(Com::tLaserOn, (int)LaserDriver::intensity);
}

void LaserDriver::turnOff(bool instantOff)
{
    LaserDriver::laserOn = false;
    if (instantOff) OCR5B = 0;
    Com::printFLN(Com::tLaserOff);
}

#endif // SUPPORT_LASER