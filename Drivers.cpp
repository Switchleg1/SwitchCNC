#include "SwitchCNC.h"

#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
MOTOR_DRIVER_1(motorDriver1);
#if NUM_MOTOR_DRIVERS > 1
MOTOR_DRIVER_2(motorDriver2);
#endif
#if NUM_MOTOR_DRIVERS > 2
MOTOR_DRIVER_3(motorDriver3);
#endif
#if NUM_MOTOR_DRIVERS > 3
MOTOR_DRIVER_4(motorDriver4);
#endif
#if NUM_MOTOR_DRIVERS > 4
MOTOR_DRIVER_5(motorDriver5);
#endif
#if NUM_MOTOR_DRIVERS > 5
MOTOR_DRIVER_6(motorDriver6);
#endif

MotorDriverInterface *motorDrivers[NUM_MOTOR_DRIVERS] =
{
    &motorDriver1
#if NUM_MOTOR_DRIVERS > 1
    , &motorDriver2
#endif
#if NUM_MOTOR_DRIVERS > 2
    , &motorDriver3
#endif
#if NUM_MOTOR_DRIVERS > 3
    , &motorDriver4
#endif
#if NUM_MOTOR_DRIVERS > 4
    , &motorDriver5
#endif
#if NUM_MOTOR_DRIVERS > 5
    , &motorDriver6
#endif
};

MotorDriverInterface *getMotorDriver(int idx)
{
    return motorDrivers[idx];
}

/**
Run motor P until it is at position X
*/
void commandG201(GCode &code)
{
    int id = 0;
    if(code.hasP())
        id = code.P;
    if(id < 0) id = 0;
    if(id >= NUM_MOTOR_DRIVERS) id = 0;
    if(!code.hasX()) return;
    motorDrivers[id]->gotoPosition(code.X);
}

//G202 P<motorId> X<setpos>  - Mark current position as X
void commandG202(GCode &code)
{
    int id = 0;
    if(code.hasP())
        id = code.P;
    if(id < 0) id = 0;
    if(id >= NUM_MOTOR_DRIVERS) id = 0;
    if(!code.hasX()) return;
    motorDrivers[id]->setCurrentAs(code.X);
}
//G203 P<motorId>            - Report current motor position
void commandG203(GCode &code)
{
    int id = 0;
    if(code.hasP())
        id = code.P;
    if(id < 0) id = 0;
    if(id >= NUM_MOTOR_DRIVERS) id = 0;
    Com::printF(PSTR("Motor"),id);
    Com::printFLN(PSTR(" Pos:"),motorDrivers[id]->getPosition());
}
//G204 P<motorId> S<0/1>     - Enable/disable motor
void commandG204(GCode &code)
{
    int id = 0;
    if(code.hasP())
        id = code.P;
    if(id < 0) id = 0;
    if(id >= NUM_MOTOR_DRIVERS) id = 0;
    if(!code.hasS()) return;
    if(code.S)
        motorDrivers[id]->enable();
    else
        motorDrivers[id]->disable();
}
// G205 P<motorId> S<0/1> E<0/1> - Home motor, S1 = go back to stored position, E1 = home only if endstop was never met, meaning it was never homed with motor.
void commandG205(GCode &code)
{
	int id = 0;
	if(code.hasP())
		id = code.P;
	if(id < 0) id = 0;
	if(id >= NUM_MOTOR_DRIVERS) id = 0;
	motorDrivers[id]->home(code.hasS() && code.S != 0, code.hasE() && code.E != 0);
}

void disableAllMotorDrivers()
{
    for(int i = 0; i < NUM_MOTOR_DRIVERS; i++)
        motorDrivers[i]->disable();
}
void initializeAllMotorDrivers()
{
    for(int i = 0; i < NUM_MOTOR_DRIVERS; i++)
        motorDrivers[i]->initialize();
}

#endif // NUM_MOTOR_DRIVERS


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
    /*if (addFlow)
    {
        addFlow *= Machine::extrudeMultiply;
        addFlow /= 100;
        if (addFlow > 0xFF)
        {
            addFlow = 0xFF;
        }
    }*/

    if (addFlow < LaserDriver::minIntensity)
    {
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

void LaserDriver::turnOff(bool instantOff = false)
{
    LaserDriver::laserOn = false;
    if(instantOff) OCR5B = 0;
    Com::printFLN(Com::tLaserOff);
}
#endif // SUPPORT_LASER


/**
The CNC driver differs a bit from laser driver. Here only M3,M4,M5 have an influence on the spindle.
The motor also keeps running for G0 moves. M3 and M4 wait for old moves to be finished and then enables
the motor. It then waits SPINDLE_WAIT_ON_START milliseconds for the spindle to reach target speed.
*/

int8_t SpindleDriver::direction = 0;
secondspeed_t SpindleDriver::spindleSpeed= 0;
uint16_t SpindleDriver::spindleRpm= 0;


/** Initialize cnc pins. EVENT_INITIALIZE_SPINDLE should return false to prevent default initialization.*/
void SpindleDriver::initialize()
{
    if(EVENT_INITIALIZE_SPINDLE) {
#if SPINDLE_ON_PIN > -1
        SET_OUTPUT(SPINDLE_ON_PIN);
		WRITE(SPINDLE_ON_PIN,!SPINDLE_ON_HIGH);
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
void SpindleDriver::turnOff()
{
    spindleRpm=0;
    if(direction == 0) return; // already off
    if(EVENT_SPINDLE_OFF)
    {
#if SPINDLE_ON_PIN > -1
        WRITE(SPINDLE_ON_PIN,!SPINDLE_ON_HIGH);
#endif
    }
    HAL::delayMilliseconds(SPINDLE_WAIT_ON_STOP);
	direction = 0;
}
/** Turns spindle on. Default implementation uses a enable pin SPINDLE_ON_PIN. If
SPINDLE_DIRECTION_PIN is not -1 it sets direction to SPINDLE_DIRECTION_CW. rpm is ignored.
To override with event system, return false for the event
EVENT_SPINDLE_CW(rpm)
*/
void SpindleDriver::turnOnCW(int32_t rpm)
{
	spindleSpeed=map(rpm,SPINDLE_RPM_MIN,SPINDLE_RPM_MAX,0,255);// linear interpolation

    if(direction == 1 && spindleRpm == rpm)
		return;
    if(direction == -1) {
        turnOff();
    }
    spindleRpm = rpm;// for display
    direction = 1;
    if(EVENT_SPINDLE_CW(rpm)) {
#if SPINDLE_DIRECTION_PIN > -1
        WRITE(SPINDLE_DIRECTION_PIN, SPINDLE_DIRECTION_CW);
#endif
#if SPINDLE_ON_PIN > -1
        WRITE(SPINDLE_ON_PIN, SPINDLE_ON_HIGH);
#endif
#if SPINDLE_PWM_PIN > -1
		pwm_pos[PWM_SPINDLE] = spindleSpeed;
#endif
	}
	HAL::delaySeconds(SPINDLE_WAIT_ON_START);
}
/** Turns spindle on. Default implementation uses a enable pin SPINDLE_ON_PIN. If
SPINDLE_DIRECTION_PIN is not -1 it sets direction to !SPINDLE_DIRECTION_CW. rpm is ignored.
To override with event system, return false for the event
EVENT_SPINDLE_CCW(rpm)
*/
void SpindleDriver::turnOnCCW(int32_t rpm)
{
	spindleSpeed=map(rpm,SPINDLE_RPM_MIN,SPINDLE_RPM_MAX,0,255);// linear interpolation

    if(direction == -1 && spindleRpm == rpm)
		return;
    if(direction == 1) {
        turnOff();
	}
    spindleRpm = rpm;// for display
    direction = -1;
    if(EVENT_SPINDLE_CCW(rpm)) {
#if SPINDLE_DIRECTION_PIN > -1
        WRITE(SPINDLE_DIRECTION_PIN, !SPINDLE_DIRECTION_CW);
#endif
#if SPINDLE_ON_PIN > -1
        WRITE(SPINDLE_ON_PIN, SPINDLE_ON_HIGH);
#endif
#if SPINDLE_PWM_PIN > -1
		pwm_pos[PWM_SPINDLE] = spindleSpeed;
#endif
    }
    HAL::delayMilliseconds(SPINDLE_WAIT_ON_START);
}

void VacuumDriver::initialize()
{
#if VACUUM_PIN>-1
    SET_OUTPUT(VACUUM_PIN);
    WRITE(VACUUM_PIN, LOW);
#endif
}

void VacuumDriver::turnOn()
{
#if VACUUM_PIN>-1
	WRITE(VACUUM_PIN, HIGH);
#endif
}

void VacuumDriver::turnOff()
{
#if VACUUM_PIN>-1
	WRITE(VACUUM_PIN, LOW);
#endif
}
