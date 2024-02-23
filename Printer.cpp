#include "SwitchCNC.h"

unsigned int counterPeriodical = 0;
volatile uint8_t executePeriodical = 0;
uint8_t counter500ms = 5;
uint8_t Printer::unitIsInches = 0; ///< 0 = Units are mm, 1 = units are inches.
//Stepper Movement Variables
float Printer::axisStepsPerMM[A_AXIS_ARRAY] = {XAXIS_STEPS_PER_MM, YAXIS_STEPS_PER_MM, ZAXIS_STEPS_PER_MM, AAXIS_STEPS_PER_MM}; ///< Number of steps per mm needed.
float Printer::invAxisStepsPerMM[A_AXIS_ARRAY]; ///< Inverse of axisStepsPerMM for faster conversion
float Printer::maxFeedrate[A_AXIS_ARRAY] = {MAX_FEEDRATE_X, MAX_FEEDRATE_Y, MAX_FEEDRATE_Z, MAX_FEEDRATE_A}; ///< Maximum allowed feedrate.
float Printer::homingFeedrate[Z_AXIS_ARRAY] = {HOMING_FEEDRATE_X, HOMING_FEEDRATE_Y, HOMING_FEEDRATE_Z};
#if RAMP_ACCELERATION
//  float max_start_speed_units_per_second[A_AXIS_ARRAY] = MAX_START_SPEED_UNITS_PER_SECOND; ///< Speed we can use, without acceleration.
float Printer::maxAccelerationMMPerSquareSecond[A_AXIS_ARRAY] = {MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_A}; ///< X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
/** Acceleration in steps/s^3 in printing mode.*/
unsigned long Printer::maxAccelerationStepsPerSquareSecond[A_AXIS_ARRAY];
#endif
int32_t Printer::zCorrectionStepsIncluded = 0;
uint8_t Printer::relativeCoordinateMode = false;  ///< Determines absolute (false) or relative Coordinates (true).

long Printer::currentPositionSteps[A_AXIS_ARRAY];
float Printer::currentPosition[A_AXIS_ARRAY];
float Printer::lastCmdPos[A_AXIS_ARRAY];
long Printer::destinationSteps[A_AXIS_ARRAY];
float Printer::coordinateOffset[Z_AXIS_ARRAY] = {0, 0, 0};
uint8_t Printer::flag0 = 0;
uint8_t Printer::flag1 = 0;
uint8_t Printer::debugLevel = 6; ///< Bitfield defining debug output. 1 = echo, 2 = info, 4 = error, 8 = dry run., 16 = Only communication, 32 = No moves
fast8_t Printer::stepsPerTimerCall = 1;
uint8_t Printer::fanSpeed = 0; // Last fan speed set with M106/M107
uint8_t Printer::interruptEvent = 0;
#if DISTORTION_CORRECTION
int16_t Printer::distortionXMIN;
int16_t Printer::distortionXMAX;
int16_t Printer::distortionYMIN;
int16_t Printer::distortionYMAX;
uint8_t Printer::distortionPoints;
float Printer::distortionStart;
float Printer::distortionEnd;
uint8_t Printer::distortionUseOffset;
#endif
uint32_t Printer::interval = 30000;           ///< Last step duration in ticks.
uint32_t Printer::timer;              ///< used for acceleration/deceleration timing
uint32_t Printer::stepNumber;         ///< Step number in current move.
#if FEATURE_Z_PROBE || MAX_HARDWARE_ENDSTOP_Z
int32_t Printer::stepsRemainingAtZHit;
#endif
int32_t Printer::axisMaxSteps[Z_AXIS_ARRAY];      ///< For software endstops, limit of move in positive direction.
int32_t Printer::axisMinSteps[Z_AXIS_ARRAY];      ///< For software endstops, limit of move in negative direction.
float Printer::axisLength[Z_AXIS_ARRAY];
float Printer::axisMin[Z_AXIS_ARRAY];
float Printer::feedrate;                   ///< Last requested feedrate.
int Printer::feedrateMultiply;             ///< Multiplier for feedrate in percent (factor 1 = 100)
float Printer::maxJerk[A_AXIS_ARRAY];      ///< Maximum allowed jerk in mm/s
speed_t Printer::vMaxReached;               ///< Maximum reached speed
#if ENABLE_BACKLASH_COMPENSATION
float Printer::backlash[A_AXIS_ARRAY];
uint8_t Printer::backlashDir;
#endif
float Printer::memoryPosition[A_AXIS_ARRAY] = {IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE};
float Printer::memoryF = -1;
#ifdef DEBUG_SEGMENT_LENGTH
float Printer::maxRealSegmentLength = 0;
#endif
#ifdef DEBUG_REAL_JERK
float Printer::maxRealJerk = 0;
#endif
#if TMC_DRIVERS
#if TMC_X_TYPE==TMC_5160
TMC5160Stepper Printer::tmcStepperX(TMC_X_CS, TMC_X_RSENSE);
#endif
#if TMC_Y_TYPE==TMC_5160
TMC5160Stepper Printer::tmcStepperY(TMC_Y_CS, TMC_Y_RSENSE);
#endif
#if TMC_Z_TYPE==TMC_5160
TMC5160Stepper Printer::tmcStepperZ(TMC_Z_CS, TMC_Z_RSENSE);
#endif
#if TMC_2_TYPE==TMC_5160
TMC5160Stepper Printer::tmcStepper2(TMC_2_CS, TMC_2_RSENSE);
#endif
#endif
#if defined(PAUSE_PIN) && PAUSE_PIN>-1
uint8_t  Printer::isPaused = 0;
#if PAUSE_STEPS > 120
int16_t Printer::pauseSteps = 0;
#else
int8_t Printer::pauseSteps = 0;
#endif
#endif //PAUSE
#if SPEED_DIAL && SPEED_DIAL_PIN > -1
uint8_t Printer::speed_dial = SPEED_DIAL_MIN_PERCENT;
#endif
#if MULTI_XENDSTOP_HOMING
fast8_t Printer::multiXHomeFlags;  // 1 = move X0, 2 = move X1
#endif
#if MULTI_YENDSTOP_HOMING
fast8_t Printer::multiYHomeFlags;  // 1 = move Y0, 2 = move Y1
#endif
#if MULTI_ZENDSTOP_HOMING
fast8_t Printer::multiZHomeFlags;  // 1 = move Z0, 2 = move Z1
#endif
#ifdef DEBUG_PRINT
int debugWaitLoop = 0;
#endif

void Printer::constrainDestinationCoords() {
    if(isNoDestinationCheck() || isHoming()) return;
#if min_software_endstop_x
	if (destinationSteps[X_AXIS] < axisMinSteps[X_AXIS]) destinationSteps[X_AXIS] = axisMinSteps[X_AXIS];
#endif
#if min_software_endstop_y
	if (destinationSteps[Y_AXIS] < axisMinSteps[Y_AXIS]) destinationSteps[Y_AXIS] = axisMinSteps[Y_AXIS];
#endif
#if min_software_endstop_z
	if (destinationSteps[Z_AXIS] < axisMinSteps[Z_AXIS] && !isZProbingActive()) destinationSteps[Z_AXIS] = axisMinSteps[Z_AXIS];
#endif

#if max_software_endstop_x
	if (destinationSteps[X_AXIS] > axisMaxSteps[X_AXIS]) destinationSteps[X_AXIS] = axisMaxSteps[X_AXIS];
#endif
#if max_software_endstop_y
	if (destinationSteps[Y_AXIS] > axisMaxSteps[Y_AXIS]) destinationSteps[Y_AXIS] = axisMaxSteps[Y_AXIS];
#endif
#if max_software_endstop_z
	if (destinationSteps[Z_AXIS] > zMaxStepsAdj && !isZProbingActive()) destinationSteps[Z_AXIS] = axisMaxSteps[Z_AXIS];
#endif
	EVENT_CONTRAIN_DESTINATION_COORDINATES
}

void Printer::setDebugLevel(uint8_t newLevel) {
    if(newLevel != debugLevel) {
		debugLevel = newLevel;
    }
    Com::printFLN(PSTR("DebugLevel:"), (int)newLevel);
}

void Printer::toggleEcho() {
    setDebugLevel(debugLevel ^ 1);
}

void Printer::toggleInfo() {
    setDebugLevel(debugLevel ^ 2);
}

void Printer::toggleErrors() {
    setDebugLevel(debugLevel ^ 4);
}

void Printer::toggleCommunication() {
    setDebugLevel(debugLevel ^ 16);
}

void Printer::toggleNoMoves() {
    setDebugLevel(debugLevel ^ 32);
}

void Printer::toggleEndStop() {
    setDebugLevel(debugLevel ^ 64);
}

bool Printer::isPositionAllowed(float x, float y, float z) {
    if(isNoDestinationCheck()) return true;
	bool allowed = true;
    if(!isHoming()) {
		allowed = allowed && x >= axisMin[X_AXIS] - 0.01;
		allowed = allowed && x <= axisMin[X_AXIS] + axisLength[X_AXIS] + 0.01;
		allowed = allowed && y >= axisMin[Y_AXIS] - 0.01;
		allowed = allowed && y <= axisMin[Y_AXIS] + axisLength[Y_AXIS] + 0.01;
		allowed = allowed && z >= axisMin[Z_AXIS] - 0.01;
		allowed = allowed && z <= axisMin[Z_AXIS] + axisLength[Z_AXIS] + ENDSTOP_Z_BACK_ON_HOME + 0.01;
	}
    if(!allowed) {
        Printer::updateCurrentPosition(true);
        Commands::printCurrentPosition();
    }
    return allowed;
}

void Printer::setFanSpeedDirectly(uint8_t speed) {
	uint8_t trimmedSpeed = TRIM_FAN_PWM(speed);
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    if(pwm_pos[PWM_FAN1] == trimmedSpeed)
        return;
#if FAN_KICKSTART_TIME
    if(fanKickstart == 0 && speed > pwm_pos[PWM_FAN1] && speed < 85) {
        if(pwm_pos[PWM_FAN1]) fanKickstart = FAN_KICKSTART_TIME / 100;
        else                  fanKickstart = FAN_KICKSTART_TIME / 25;
    }
#endif
    pwm_pos[PWM_FAN1] = trimmedSpeed;
#endif
}
void Printer::setFan2SpeedDirectly(uint8_t speed) {
	uint8_t trimmedSpeed = TRIM_FAN_PWM(speed);
#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
    if(pwm_pos[PWM_FAN2] == trimmedSpeed)
        return;
#if FAN_KICKSTART_TIME
    if(fan2Kickstart == 0 && speed > pwm_pos[PWM_FAN2] && speed < 85) {
        if(pwm_pos[PWM_FAN2]) fan2Kickstart = FAN_KICKSTART_TIME / 100;
        else                  fan2Kickstart = FAN_KICKSTART_TIME / 25;
    }
#endif
    pwm_pos[PWM_FAN2] = trimmedSpeed;
#endif
}

void Printer::updateDerivedParameter() {
	for(uint8_t i = 0; i < Z_AXIS_ARRAY; i++) {
		axisMaxSteps[i] = static_cast<int32_t>(axisStepsPerMM[i] * (axisMin[i] + axisLength[i]));
		axisMinSteps[i] = static_cast<int32_t>(axisStepsPerMM[i] * axisMin[i]);
	}

    // For which directions do we need backlash compensation
#if ENABLE_BACKLASH_COMPENSATION
    backlashDir &= XYZA_DIRPOS;
	if(backlash[X_AXIS] != 0) backlashDir |= 16;
	if(backlash[Y_AXIS] != 0) backlashDir |= 32;
	if(backlash[Z_AXIS] != 0) backlashDir |= 64;
	if(backlash[A_AXIS] != 0) backlashDir |= 128;
#endif
	for(uint8_t i = 0; i < A_AXIS_ARRAY; i++) {
        invAxisStepsPerMM[i] = 1.0f / axisStepsPerMM[i];
#ifdef RAMP_ACCELERATION
		/** Acceleration in steps/s^3 in printing mode.*/
		maxAccelerationStepsPerSquareSecond[i] = maxAccelerationMMPerSquareSecond[i] * axisStepsPerMM[i];
#endif
    }
	// For numeric stability we need to start accelerations at a minimum speed and hence ensure that the
	// jerk is at least 2 * minimum speed.

	// For xy moves the minimum speed is multiplied with 1.41 to enforce the condition also for diagonals since the
	// driving axis is the problematic speed.
	float minimumSpeedX = 0.5 * maxAccelerationMMPerSquareSecond[X_AXIS] * sqrt(2.0f / (axisStepsPerMM[X_AXIS] * maxAccelerationMMPerSquareSecond[X_AXIS]));
	if(maxJerk[X_AXIS] < 2 * minimumSpeedX) {// Enforce minimum start speed if target is faster and jerk too low
		maxJerk[X_AXIS] = 2 * minimumSpeedX;
		Com::printFLN(PSTR("X jerk was too low, setting to "), maxJerk[X_AXIS]);
	}
	float minimumSpeedY = 0.5 * maxAccelerationMMPerSquareSecond[Y_AXIS] * sqrt(2.0f / (axisStepsPerMM[Y_AXIS] * maxAccelerationMMPerSquareSecond[Y_AXIS]));
	if(maxJerk[Y_AXIS] < 2 * minimumSpeedY) {// Enforce minimum start speed if target is faster and jerk too low
		maxJerk[Y_AXIS] = 2 * minimumSpeedY;
		Com::printFLN(PSTR("Y jerk was too low, setting to "), maxJerk[Y_AXIS]);
	}
	float minimumSpeedZ = 0.5 * maxAccelerationMMPerSquareSecond[Z_AXIS] * sqrt(2.0f / (axisStepsPerMM[Z_AXIS] * maxAccelerationMMPerSquareSecond[Z_AXIS]));
	if(maxJerk[Z_AXIS] < 2 * minimumSpeedZ) {// Enforce minimum start speed if target is faster and jerk too low
		maxJerk[Z_AXIS] = 2 * minimumSpeedZ;
		Com::printFLN(PSTR("Z jerk was too low, setting to "), maxJerk[Z_AXIS]);
	}
	float minimumSpeedA = 0.5 * maxAccelerationMMPerSquareSecond[A_AXIS] * sqrt(2.0f / (axisStepsPerMM[A_AXIS] * maxAccelerationMMPerSquareSecond[A_AXIS]));
	if(maxJerk[A_AXIS] < 2 * minimumSpeedA) {// Enforce minimum start speed if target is faster and jerk too low
		maxJerk[A_AXIS] = 2 * minimumSpeedA;
		Com::printFLN(PSTR("A jerk was too low, setting to "), maxJerk[A_AXIS]);
	}
#if DISTORTION_CORRECTION
    distortion.updateDerived();
#endif // DISTORTION_CORRECTION
    EVENT_UPDATE_DERIVED;
}
#if AUTOMATIC_POWERUP
void Printer::enablePowerIfNeeded() {
	if(Printer::isPowerOn())
		return;
    SET_OUTPUT(PS_ON_PIN); //GND
    Printer::setPowerOn(true);
    WRITE(PS_ON_PIN, (POWER_INVERTING ? HIGH : LOW));
	HAL::delayMilliseconds(500); // Just to ensure power is up and stable
}
#endif

/**
  \brief Stop stepper motors. Disable power,if possible.
*/
void Printer::kill(uint8_t onlySteppers) {
    EVENT_KILL(onlySteppers);
    if(areAllSteppersDisabled() && onlySteppers) return;
    if(Printer::isAllKilled()) return;
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
    disableAllMotorDrivers();
#endif // defined
    disableXStepper();
	disableYStepper();
#if !defined(PREVENT_Z_DISABLE_ON_STEPPER_TIMEOUT)
    disableZStepper();
#else
    if(!onlySteppers)
        disableZStepper();
#endif
	disableAStepper();
    setAllSteppersDiabled();
    unsetHomedAll();
	if(!onlySteppers) {
#if defined(PS_ON_PIN) && PS_ON_PIN>-1 && !defined(NO_POWER_TIMEOUT)
        //pinMode(PS_ON_PIN,INPUT);
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, (POWER_INVERTING ? LOW : HIGH));
        Printer::setPowerOn(false);
#endif
        Printer::setAllKilled(true);
	}
#if FAN_BOARD_PIN > -1
        pwm_pos[PWM_BOARD_FAN] = BOARD_FAN_MIN_SPEED;
#endif // FAN_BOARD_PIN
}

// This is for untransformed move to coordinates in printers absolute Cartesian space
uint8_t Printer::moveTo(float x, float y, float z, float a, float f) {
    if(x != IGNORE_COORDINATE)
		destinationSteps[X_AXIS] = x * axisStepsPerMM[X_AXIS];
    if(y != IGNORE_COORDINATE)
		destinationSteps[Y_AXIS] = y * axisStepsPerMM[Y_AXIS];
	if(z != IGNORE_COORDINATE)
		destinationSteps[Z_AXIS] = z * axisStepsPerMM[Z_AXIS];
	if(a != IGNORE_COORDINATE)
		destinationSteps[A_AXIS] = a * axisStepsPerMM[A_AXIS];
	if(f != IGNORE_COORDINATE)
		feedrate = f;
	PrintLine::queueCartesianMove(ALWAYS_CHECK_ENDSTOPS, true);
    updateCurrentPosition(false);
    return 1;
}

uint8_t Printer::moveToReal(float x, float y, float z, float a, float f, bool pathOptimize) {
    if(x == IGNORE_COORDINATE)
        x = currentPosition[X_AXIS];
    else
        currentPosition[X_AXIS] = x;
    if(y == IGNORE_COORDINATE)
		y = currentPosition[Y_AXIS];
    else
        currentPosition[Y_AXIS] = y;
	if(z == IGNORE_COORDINATE)
		z = currentPosition[Z_AXIS];
	else
		currentPosition[Z_AXIS] = z;
	if(a == IGNORE_COORDINATE)
		a = currentPosition[A_AXIS];
	else
		currentPosition[A_AXIS] = a;

    // There was conflicting use of IGNOR_COORDINATE
    destinationSteps[X_AXIS] = static_cast<int32_t>(floor(x * axisStepsPerMM[X_AXIS] + 0.5f));
    destinationSteps[Y_AXIS] = static_cast<int32_t>(floor(y * axisStepsPerMM[Y_AXIS] + 0.5f));
	destinationSteps[Z_AXIS] = static_cast<int32_t>(floor(z * axisStepsPerMM[Z_AXIS] + 0.5f));
	destinationSteps[A_AXIS] = static_cast<int32_t>(floor(a * axisStepsPerMM[A_AXIS] + 0.5f));
	if(f != IGNORE_COORDINATE)
        feedrate = f;

	PrintLine::queueCartesianMove(ALWAYS_CHECK_ENDSTOPS, pathOptimize);
    return 1;
}

void Printer::setOrigin(float xOff, float yOff, float zOff) {
	coordinateOffset[X_AXIS] = xOff;// offset from G92
	coordinateOffset[Y_AXIS] = yOff;
	coordinateOffset[Z_AXIS] = zOff;
}

/** Computes currentPosition from currentPositionSteps including correction for offset. */
void Printer::updateCurrentPosition(bool copyLastCmd) {
	currentPosition[X_AXIS] = static_cast<float>(currentPositionSteps[X_AXIS]) * invAxisStepsPerMM[X_AXIS];
	currentPosition[Y_AXIS] = static_cast<float>(currentPositionSteps[Y_AXIS]) * invAxisStepsPerMM[Y_AXIS];
	currentPosition[Z_AXIS] = static_cast<float>(currentPositionSteps[Z_AXIS] - zCorrectionStepsIncluded) * invAxisStepsPerMM[Z_AXIS];
	currentPosition[A_AXIS] = static_cast<float>(currentPositionSteps[A_AXIS]) * invAxisStepsPerMM[A_AXIS];
	if(copyLastCmd) {
        lastCmdPos[X_AXIS] = currentPosition[X_AXIS];
        lastCmdPos[Y_AXIS] = currentPosition[Y_AXIS];
		lastCmdPos[Z_AXIS] = currentPosition[Z_AXIS];
		lastCmdPos[A_AXIS] = currentPosition[A_AXIS];
    }
}

void Printer::updateCurrentPositionSteps() {
	currentPositionSteps[X_AXIS] = static_cast<int32_t>(floor(currentPosition[X_AXIS] * axisStepsPerMM[X_AXIS] + 0.5f));
	currentPositionSteps[Y_AXIS] = static_cast<int32_t>(floor(currentPosition[Y_AXIS] * axisStepsPerMM[Y_AXIS] + 0.5f));
	currentPositionSteps[Z_AXIS] = static_cast<int32_t>(floor(currentPosition[Z_AXIS] * axisStepsPerMM[Z_AXIS] + 0.5f));
	currentPositionSteps[A_AXIS] = static_cast<int32_t>(floor(currentPosition[A_AXIS] * axisStepsPerMM[A_AXIS] + 0.5f));
	zCorrectionStepsIncluded = 0;
}

/** \brief Sets the destination coordinates to values stored in com.

Extracts x,y,z,e,f from g-code considering active units. Converted result is stored in currentPosition and lastCmdPos. Converts
position to destinationSteps including rotation and offsets, excluding distortion correction (which gets added on move queuing).
\param com g-code with new destination position.
\return true if it is a move, false if no move results from coordinates.
 */

uint8_t Printer::setDestinationStepsFromGCode(GCode *com) {
	register int32_t p;
	bool posAllowed = true;
#if MOVE_X_WHEN_HOMED == 1 || MOVE_Y_WHEN_HOMED == 1 || MOVE_Z_WHEN_HOMED == 1
    if(!isNoDestinationCheck()) {
#if MOVE_X_WHEN_HOMED
        if(!isXHomed())
            com->unsetX();
#endif
#if MOVE_Y_WHEN_HOMED
        if(!isYHomed())
            com->unsetY();
#endif
#if MOVE_Z_WHEN_HOMED
        if(!isZHomed())
            com->unsetZ();
#endif
    }
#endif
#if DISTORTION_CORRECTION == 0
	if(!com->hasNoXYZA()) {
#endif
        if(!relativeCoordinateMode) {
			if(com->hasX()) lastCmdPos[X_AXIS] = currentPosition[X_AXIS] = convertToMM(com->X) - coordinateOffset[X_AXIS];
            if(com->hasY()) lastCmdPos[Y_AXIS] = currentPosition[Y_AXIS] = convertToMM(com->Y) - coordinateOffset[Y_AXIS];
			if(com->hasZ()) lastCmdPos[Z_AXIS] = currentPosition[Z_AXIS] = convertToMM(com->Z) - coordinateOffset[Z_AXIS];
			if(com->hasA()) lastCmdPos[A_AXIS] = currentPosition[A_AXIS] = convertToMM(com->A);
        } else {
            if(com->hasX()) currentPosition[X_AXIS] = (lastCmdPos[X_AXIS] += convertToMM(com->X));
			if(com->hasY()) currentPosition[Y_AXIS] = (lastCmdPos[Y_AXIS] += convertToMM(com->Y));
			if(com->hasZ()) currentPosition[Z_AXIS] = (lastCmdPos[Z_AXIS] += convertToMM(com->Z));
			if(com->hasA()) currentPosition[A_AXIS] = (lastCmdPos[A_AXIS] += convertToMM(com->A));
		}
		destinationSteps[X_AXIS] = static_cast<int32_t>(floor(lastCmdPos[X_AXIS] * axisStepsPerMM[X_AXIS] + 0.5f));
		destinationSteps[Y_AXIS] = static_cast<int32_t>(floor(lastCmdPos[Y_AXIS] * axisStepsPerMM[Y_AXIS] + 0.5f));
		destinationSteps[Z_AXIS] = static_cast<int32_t>(floor(lastCmdPos[Z_AXIS] * axisStepsPerMM[Z_AXIS] + 0.5f));
		destinationSteps[A_AXIS] = static_cast<int32_t>(floor(lastCmdPos[A_AXIS] * axisStepsPerMM[A_AXIS] + 0.5f));
		posAllowed = com->hasNoXYZ() || Printer::isPositionAllowed(lastCmdPos[X_AXIS], lastCmdPos[Y_AXIS], lastCmdPos[Z_AXIS]);
#if DISTORTION_CORRECTION == 0
    }
#endif
    if(com->hasF() && com->F > 0.1) {
        if(unitIsInches)
            feedrate = com->F * 0.0042333f * (float)feedrateMultiply;  // Factor is 25.5/60/100
        else
            feedrate = com->F * (float)feedrateMultiply * 0.00016666666f;
    }
    if(!posAllowed) {
		return false; // ignore move
    }
	return !com->hasNoXYZA(); // ignore unproductive moves
}

void Printer::setup() {
	//SPI fix
	pinMode(SS, OUTPUT);
	pinMode(SCK, OUTPUT);
	pinMode(MOSI, OUTPUT);

	digitalWrite(SS, LOW);

    HAL::stopWatchdog();
	for(uint8_t i = 0; i < NUM_PWM; i++) pwm_pos[i] = 0;
#if defined(MB_SETUP)
    MB_SETUP;
#endif
#if defined(EEPROM_AVAILABLE) && defined(EEPROM_SPI_ALLIGATOR) && EEPROM_AVAILABLE == EEPROM_SPI_ALLIGATOR
    HAL::spiBegin();
#endif
    HAL::hwSetup();
    EVENT_INITIALIZE_EARLY
#ifdef ANALYZER
// Channel->pin assignments
#if ANALYZER_CH0>=0
    SET_OUTPUT(ANALYZER_CH0);
#endif
#if ANALYZER_CH1>=0
    SET_OUTPUT(ANALYZER_CH1);
#endif
#if ANALYZER_CH2>=0
    SET_OUTPUT(ANALYZER_CH2);
#endif
#if ANALYZER_CH3>=0
    SET_OUTPUT(ANALYZER_CH3);
#endif
#if ANALYZER_CH4>=0
    SET_OUTPUT(ANALYZER_CH4);
#endif
#if ANALYZER_CH5>=0
    SET_OUTPUT(ANALYZER_CH5);
#endif
#if ANALYZER_CH6>=0
    SET_OUTPUT(ANALYZER_CH6);
#endif
#if ANALYZER_CH7>=0
    SET_OUTPUT(ANALYZER_CH7);
#endif
#endif

#if defined(ENABLE_POWER_ON_STARTUP) && ENABLE_POWER_ON_STARTUP && (PS_ON_PIN>-1)
    SET_OUTPUT(PS_ON_PIN); //GND
    WRITE(PS_ON_PIN, (POWER_INVERTING ? HIGH : LOW));
    Printer::setPowerOn(true);
#else
#if PS_ON_PIN > -1
    SET_OUTPUT(PS_ON_PIN); //GND
    WRITE(PS_ON_PIN, (POWER_INVERTING ? LOW : HIGH));
    Printer::setPowerOn(false);
#else
    Printer::setPowerOn(true);
#endif
#endif
#if SDSUPPORT
    //power to SD reader
#if SDPOWER > -1
    SET_OUTPUT(SDPOWER);
    WRITE(SDPOWER, HIGH);
#endif
#if defined(SDCARDDETECT) && SDCARDDETECT > -1
    SET_INPUT(SDCARDDETECT);
    PULLUP(SDCARDDETECT, HIGH);
#endif
#endif


    //Initialize Step Pins
    SET_OUTPUT(X_STEP_PIN);
    SET_OUTPUT(Y_STEP_PIN);
	SET_OUTPUT(Z_STEP_PIN);
	SET_OUTPUT(A_STEP_PIN);
	endXYZASteps();

	//Initialize Dir Pins
	SET_OUTPUT(X_DIR_PIN);
	SET_OUTPUT(Y_DIR_PIN);
	SET_OUTPUT(Z_DIR_PIN);
	SET_OUTPUT(A_DIR_PIN);

	//Steppers default to disabled.
#if X_ENABLE_PIN > -1
	SET_OUTPUT(X_ENABLE_PIN);
	WRITE(X_ENABLE_PIN, !X_ENABLE_ON);
#endif
#if Y_ENABLE_PIN > -1
    SET_OUTPUT(Y_ENABLE_PIN);
	WRITE(Y_ENABLE_PIN, !Y_ENABLE_ON);
#endif
#if Z_ENABLE_PIN > -1
	SET_OUTPUT(Z_ENABLE_PIN);
	WRITE(Z_ENABLE_PIN, !Z_ENABLE_ON);
#endif
#if A_ENABLE_PIN > -1
	SET_OUTPUT(A_ENABLE_PIN);
	WRITE(A_ENABLE_PIN, !A_ENABLE_ON);
#endif

#if FEATURE_TWO_XSTEPPER || DUAL_X_AXIS
    SET_OUTPUT(X2_STEP_PIN);
    SET_OUTPUT(X2_DIR_PIN);
#if X2_ENABLE_PIN > -1
    SET_OUTPUT(X2_ENABLE_PIN);
    WRITE(X2_ENABLE_PIN, !X_ENABLE_ON);
#endif
#endif

#if FEATURE_TWO_YSTEPPER
    SET_OUTPUT(Y2_STEP_PIN);
    SET_OUTPUT(Y2_DIR_PIN);
#if Y2_ENABLE_PIN > -1
    SET_OUTPUT(Y2_ENABLE_PIN);
    WRITE(Y2_ENABLE_PIN, !Y_ENABLE_ON);
#endif
#endif

#if FEATURE_TWO_ZSTEPPER
    SET_OUTPUT(Z2_STEP_PIN);
    SET_OUTPUT(Z2_DIR_PIN);
#if Z2_ENABLE_PIN > -1
    SET_OUTPUT(Z2_ENABLE_PIN);
    WRITE(Z2_ENABLE_PIN, !Z_ENABLE_ON);
#endif
#endif

	Endstops::setup();
#if FEATURE_Z_PROBE && Z_PROBE_PIN>-1
    SET_INPUT(Z_PROBE_PIN);
#if Z_PROBE_PULLUP
    PULLUP(Z_PROBE_PIN, HIGH);
#endif
#endif // FEATURE_FEATURE_Z_PROBE
#if defined(PAUSE_PIN) && PAUSE_PIN > -1
	SET_INPUT(PAUSE_PIN);
#if defined(PAUSE_PULLUP) && PAUSE_PULLUP
    PULLUP(PAUSE_PIN, HIGH);
#endif
#endif
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    SET_OUTPUT(FAN_PIN);
    WRITE(FAN_PIN, LOW);
#endif
#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
    SET_OUTPUT(FAN2_PIN);
    WRITE(FAN2_PIN, LOW);
#endif
#if FAN_BOARD_PIN>-1
    SET_OUTPUT(FAN_BOARD_PIN);
    WRITE(FAN_BOARD_PIN, LOW);
    pwm_pos[PWM_BOARD_FAN] = BOARD_FAN_MIN_SPEED;
#endif
    HAL::delayMilliseconds(1);
#if CASE_LIGHTS_PIN >= 0
    SET_OUTPUT(CASE_LIGHTS_PIN);
    WRITE(CASE_LIGHTS_PIN, CASE_LIGHT_DEFAULT_ON);
#endif // CASE_LIGHTS_PIN
	CNCDriver::initialize();

#ifdef RED_BLUE_STATUS_LEDS
    SET_OUTPUT(RED_STATUS_LED);
    SET_OUTPUT(BLUE_STATUS_LED);
    WRITE(BLUE_STATUS_LED, HIGH);
    WRITE(RED_STATUS_LED, LOW);
#endif // RED_BLUE_STATUS_LEDS
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
    initializeAllMotorDrivers();
#endif // defined
	feedrate = 10; ///< Current feedrate in mm/s.
    feedrateMultiply = 100;
	lastCmdPos[X_AXIS] = lastCmdPos[Y_AXIS] = lastCmdPos[Z_AXIS] = lastCmdPos[A_AXIS] = 0;
	maxJerk[X_AXIS] = MAX_XJERK;
	maxJerk[Y_AXIS] = MAX_YJERK;
	maxJerk[Z_AXIS] = MAX_ZJERK;
	maxJerk[A_AXIS] = MAX_AJERK;
	interval = 5000;
	stepsPerTimerCall = 1;
    flag0 = PRINTER_FLAG0_STEPPER_DISABLED;
	axisLength[X_AXIS] = X_MAX_LENGTH;
	axisLength[Y_AXIS] = Y_MAX_LENGTH;
	axisLength[Z_AXIS] = Z_MAX_LENGTH;
	axisMin[X_AXIS] = X_MIN_POS;
	axisMin[Y_AXIS] = Y_MIN_POS;
	axisMin[Z_AXIS] = Z_MIN_POS;
#if ENABLE_BACKLASH_COMPENSATION
	backlash[X_AXIS] = X_BACKLASH;
	backlash[Y_AXIS] = Y_BACKLASH;
	backlash[Z_AXIS] = Z_BACKLASH;
	backlash[A_AXIS] = A_BACKLASH;
    backlashDir = 0;
#endif
    EEPROM::initBaudrate();
    HAL::serialSetBaudrate(baudrate);
    Com::printFLN(Com::tStart);
    HAL::showStartReason();
	// sets auto leveling in eeprom init
    EEPROM::init(); // Read settings from eeprom if wanted
	for(uint8_t i = 0; i < A_AXIS_ARRAY; i++) {
		currentPositionSteps[i] = 0;
		currentPosition[i] = 0;
    }
    //Commands::printCurrentPosition();
#if DISTORTION_CORRECTION
	distortion.init();
#endif // DISTORTION_CORRECTION

    updateDerivedParameter();
    Commands::checkFreeMemory();
    Commands::writeLowestFreeRAM();
    HAL::setupTimer();

#if FEATURE_WATCHDOG
    HAL::startWatchdog();
#endif // FEATURE_WATCHDOG
#if SDSUPPORT
    sd.mount();
#endif

#if FEATURE_SERVO                   // set servos to neutral positions at power_up
#if defined(SERVO0_NEUTRAL_POS) && SERVO0_NEUTRAL_POS >= 500
    HAL::servoMicroseconds(0, SERVO0_NEUTRAL_POS, 1000);
#endif
#if defined(SERVO1_NEUTRAL_POS) && SERVO1_NEUTRAL_POS >= 500
    HAL::servoMicroseconds(1, SERVO1_NEUTRAL_POS, 1000);
#endif
#if defined(SERVO2_NEUTRAL_POS) && SERVO2_NEUTRAL_POS >= 500
    HAL::servoMicroseconds(2, SERVO2_NEUTRAL_POS, 1000);
#endif
#if defined(SERVO3_NEUTRAL_POS) && SERVO3_NEUTRAL_POS >= 500
    HAL::servoMicroseconds(3, SERVO3_NEUTRAL_POS, 1000);
#endif
#endif
    EVENT_INITIALIZE;
#ifdef STARTUP_GCODE
    GCode::executeFString(Com::tStartupGCode);
#endif
#if TMC_DRIVERS
#if TMC_X_TYPE==TMC_5160
	configTMC5160(&tmcStepperX, TMC_X_INTPOL, TMC_X_RMS, TMC_X_HOLD, TMC_X_HOLDDELAY, TMC_X_TPWRDOWN, TMC_X_HSTART, TMC_X_HEND, TMC_X_TOFF, TMC_X_TBL, TMC_X_TPFD, TMC_X_PWM_FREQ, TMC_X_TPWMTHRS, TMC_X_TCOOLTHRS, TMC_X_THIGHTHRS, TMC_X_SEMIN, TMC_X_SEMAX, TMC_X_SGT, TMC_X_S2VS, TMC_X_S2G, TMC_X_SFILTER, TMC_X_MICROSTEP, TMC_X_PWM_GRAD, TMC_X_PWM_OFS, TMC_X_PWM_LIM, TMC_X_MODE);
#endif
#if TMC_Y_TYPE==TMC_5160
	configTMC5160(&tmcStepperY, TMC_Y_INTPOL, TMC_Y_RMS, TMC_Y_HOLD, TMC_Y_HOLDDELAY, TMC_Y_TPWRDOWN, TMC_Y_HSTART, TMC_Y_HEND, TMC_Y_TOFF, TMC_Y_TBL, TMC_Y_TPFD, TMC_Y_PWM_FREQ, TMC_Y_TPWMTHRS, TMC_Y_TCOOLTHRS, TMC_Y_THIGHTHRS, TMC_Y_SEMIN, TMC_Y_SEMAX, TMC_Y_SGT, TMC_Y_S2VS, TMC_Y_S2G, TMC_Y_SFILTER, TMC_Y_MICROSTEP, TMC_Y_PWM_GRAD, TMC_Y_PWM_OFS, TMC_Y_PWM_LIM, TMC_Y_MODE);
#endif
#if TMC_Z_TYPE==TMC_5160
	configTMC5160(&tmcStepperZ, TMC_Z_INTPOL, TMC_Z_RMS, TMC_Z_HOLD, TMC_Z_HOLDDELAY, TMC_Z_TPWRDOWN, TMC_Z_HSTART, TMC_Z_HEND, TMC_Z_TOFF, TMC_Z_TBL, TMC_Z_TPFD, TMC_Z_PWM_FREQ, TMC_Z_TPWMTHRS, TMC_Z_TCOOLTHRS, TMC_Z_THIGHTHRS, TMC_Z_SEMIN, TMC_Z_SEMAX, TMC_Z_SGT, TMC_Z_S2VS, TMC_Z_S2G, TMC_Z_SFILTER, TMC_Z_MICROSTEP, TMC_Z_PWM_GRAD, TMC_Z_PWM_OFS, TMC_Z_PWM_LIM, TMC_Z_MODE);
#endif
#if TMC_2_TYPE==TMC_5160
	configTMC5160(&tmcStepper2, TMC_2_INTPOL, TMC_2_RMS, TMC_2_HOLD, TMC_2_HOLDDELAY, TMC_2_TPWRDOWN, TMC_2_HSTART, TMC_2_HEND, TMC_2_TOFF, TMC_2_TBL, TMC_2_TPFD, TMC_2_PWM_FREQ, TMC_2_TPWMTHRS, TMC_2_TCOOLTHRS, TMC_2_THIGHTHRS, TMC_2_SEMIN, TMC_2_SEMAX, TMC_2_SGT, TMC_2_S2VS, TMC_2_S2G, TMC_2_SFILTER, TMC_2_MICROSTEP, TMC_2_PWM_GRAD, TMC_2_PWM_OFS, TMC_2_PWM_LIM, TMC_2_MODE);
#endif
#endif
#if ANALOG_INPUTS > 0
    AnalogIn::start();
#endif
}

void Printer::defaultLoopActions() {
    Commands::checkForPeriodicalActions(true);
	millis_t curtime = HAL::timeInMilliseconds();
	if(PrintLine::hasLines())
        previousMillisCmd = curtime;
    else {
        curtime -= previousMillisCmd;
        if(maxInactiveTime != 0 && curtime >  maxInactiveTime )
            Printer::kill(false);
        else
            Printer::setAllKilled(false); // prevent repeated kills
        if(stepperInactiveTime != 0 && curtime >  stepperInactiveTime )
            Printer::kill(true);
    }
#if SDCARDDETECT > -1 && SDSUPPORT
    sd.automount();
#endif
#if defined(EEPROM_AVAILABLE) && EEPROM_AVAILABLE == EEPROM_SDCARD
    HAL::syncEEPROM();
#endif

    DEBUG_MEMORY;
}

void Printer::SetMemoryPosition() {
    Commands::waitUntilEndOfAllMoves();
    updateCurrentPosition(false);
	realPosition(memoryPosition[X_AXIS], memoryPosition[Y_AXIS], memoryPosition[Z_AXIS], memoryPosition[A_AXIS]);
	memoryF = feedrate;
}

void Printer::GoToMemoryPosition(bool x, bool y, bool z, bool a, float feed) {
    if(memoryF < 0) return; // Not stored before call, so we ignore it
	bool all = !(x || y || z || a);
	moveToReal((all || x ? (lastCmdPos[X_AXIS] = memoryPosition[X_AXIS]) : IGNORE_COORDINATE)
			   , (all || y ? (lastCmdPos[Y_AXIS] = memoryPosition[Y_AXIS]) : IGNORE_COORDINATE)
			   , (all || z ? (lastCmdPos[Z_AXIS] = memoryPosition[Z_AXIS]) : IGNORE_COORDINATE)
			   , (all || a ? (lastCmdPos[A_AXIS] = memoryPosition[A_AXIS]) : IGNORE_COORDINATE)
			   , feed);
	feedrate = memoryF;
    updateCurrentPosition(false);
}

void Printer::homeXAxis() {
    bool nocheck = isNoDestinationCheck();
    setNoDestinationCheck(true);
    long steps;
	Commands::waitUntilEndOfAllMoves();
	setHoming(true);
    if ((MIN_HARDWARE_ENDSTOP_X && X_MIN_PIN > -1 && X_HOME_DIR == -1) || (MAX_HARDWARE_ENDSTOP_X && X_MAX_PIN > -1 && X_HOME_DIR == 1)) {
        coordinateOffset[X_AXIS] = 0;
		long offX = 0;
		steps = (Printer::axisMaxSteps[X_AXIS] - Printer::axisMinSteps[X_AXIS]) * X_HOME_DIR;
		currentPositionSteps[X_AXIS] = -steps;
        PrintLine::moveRelativeDistanceInSteps(2 * steps, 0, 0, 0, homingFeedrate[X_AXIS], true, true);
		currentPositionSteps[X_AXIS] = (X_HOME_DIR == -1) ? axisMinSteps[X_AXIS] - offX : axisMaxSteps[X_AXIS] + offX;
        PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * -ENDSTOP_X_BACK_MOVE * X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, false);
        PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * 2 * ENDSTOP_X_BACK_MOVE * X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, true);
#if defined(ENDSTOP_X_BACK_ON_HOME)
        if(ENDSTOP_X_BACK_ON_HOME > 0)
            PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * -ENDSTOP_X_BACK_ON_HOME * X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS], true, true);
#endif
		currentPositionSteps[X_AXIS] = (X_HOME_DIR == -1) ? axisMinSteps[X_AXIS] - offX : axisMaxSteps[X_AXIS] + offX;
        setXHomed(true);
    }
    setNoDestinationCheck(nocheck);
    setHoming(false);
}

void Printer::homeYAxis() {
    long steps;
    if ((MIN_HARDWARE_ENDSTOP_Y && Y_MIN_PIN > -1 && Y_HOME_DIR == -1) || (MAX_HARDWARE_ENDSTOP_Y && Y_MAX_PIN > -1 && Y_HOME_DIR == 1)) {
        coordinateOffset[Y_AXIS] = 0;
		long offY = 0;
		steps = (axisMaxSteps[Y_AXIS] - Printer::axisMinSteps[Y_AXIS]) * Y_HOME_DIR;
        currentPositionSteps[Y_AXIS] = -steps;
		setHoming(true);
        PrintLine::moveRelativeDistanceInSteps(0, 2 * steps, 0, 0, homingFeedrate[Y_AXIS], true, true);
		currentPositionSteps[Y_AXIS] = (Y_HOME_DIR == -1) ? axisMinSteps[Y_AXIS] - offY : axisMaxSteps[Y_AXIS] + offY;
        PrintLine::moveRelativeDistanceInSteps(0, axisStepsPerMM[Y_AXIS] * -ENDSTOP_Y_BACK_MOVE * Y_HOME_DIR, 0, 0, homingFeedrate[Y_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, false);
        PrintLine::moveRelativeDistanceInSteps(0, axisStepsPerMM[Y_AXIS] * 2 * ENDSTOP_Y_BACK_MOVE * Y_HOME_DIR, 0, 0, homingFeedrate[Y_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, true);
        setHoming(false);
#if defined(ENDSTOP_Y_BACK_ON_HOME)
        if(ENDSTOP_Y_BACK_ON_HOME > 0)
            PrintLine::moveRelativeDistanceInSteps(0, axisStepsPerMM[Y_AXIS] * -ENDSTOP_Y_BACK_ON_HOME * Y_HOME_DIR, 0, 0, homingFeedrate[Y_AXIS], true, false);
#endif
		currentPositionSteps[Y_AXIS] = (Y_HOME_DIR == -1) ? axisMinSteps[Y_AXIS] - offY : axisMaxSteps[Y_AXIS] + offY;
        setYHomed(true);
	}
}

/** \brief homes z axis.

Homing z axis is the most complicated homing part as it needs to correct for several parameters depending on rotation correction,
distortion correction, position, homing direction, sensor type and bed coating. Because we get lots of support questions from "wrong"
behavior due to misunderstandings of all these dependencies, I will try to describe the function as detailed as possible.

## Step 1: Test if homing is possible at all

Test if homing in Z_HOME_DIR has a matching hardware endstop. If not, no z homing is possible at all.

## Step 2: Activate z-probe if required

If homing to z min using a z-probe, activate the probe. Requires a position where this is possible, since probe offset could prevent the
move. Hence in this case x and y must be homed first and a z homing position given and reached before calling this function.

## Step 3: Fast homing to sensor

A move of 2 * z_length is send to printer. The move will stop when the endstop triggers.

## Step 4: Untrigger sensor

Move in opposite direction ENDSTOP_Z_BACK_MOVE mm to disable the endstop signal safely.

## Step 5: Retest endstop slowly

For best precision we rerun step 3 with 1/ENDSTOP_Z_RETEST_REDUCTION_FACTOR factor for speed. This should give a very accurate trigge rposition.

## Step 6: Deactivate z-probe if it was activated in step 2

## Step 7: Correct Z position

zCorrection sums up several influences:
- -axisStepsPerMM[Z_AXIS] * EEPROM::zProbeHeight() to compensate for z-prove trigger offset if probe was used.
- -axisStepsPerMM[Z_AXIS] * ENDSTOP_Z_BACK_ON_HOME * Z_HOME_DIR to add wanted extra distance ENDSTOP_Z_BACK_ON_HOME.
- axisStepsPerMM[Z_AXIS] * zBedOffset to correct for bed coating if correction mode Z_PROBE_Z_OFFSET_MODE == 0 and z min homing.
and moves the z axis up that distance.

## Step 8: Set position in CMC

For z min homing set currentPositionSteps[Z_AXIS] to zMinSteps. For z max homing set it to zMaxSteps - bed coating.

Set z offset according to selected tool z offset, if extruder is not sensor.

If distortion correction is enabled, set zCorrectionStepsIncluded and add value to currentPositionSteps.

#step 9: Compute RWC and correct rotation influence

Compute real position from position in steps. If rotation is on we did measure with a z-probe,
this result is wrong and we need to correct by the z change between origin and current position.

    currentPositionSteps[Z_AXIS] -= (axisStepsPerMM[Z_AXIS] * currentPosition[Z_AXIS] - zMinSteps);
    currentPosition[Z_AXIS] = zMin;

## Step 10: Update NMC for nonlinear systems

*/
void Printer::homeZAxis() { // Cartesian homing
    long steps;
    if ((MIN_HARDWARE_ENDSTOP_Z && Z_MIN_PIN > -1 && Z_HOME_DIR == -1) || (MAX_HARDWARE_ENDSTOP_Z && Z_MAX_PIN > -1 && Z_HOME_DIR == 1)) {
#if Z_HOME_DIR < 0 && Z_PROBE_PIN == Z_MIN_PIN && FEATURE_Z_PROBE
        Printer::startProbing(true);
#endif
        coordinateOffset[Z_AXIS] = 0; // G92 Z offset
		steps = (axisMaxSteps[Z_AXIS] - axisMinSteps[Z_AXIS]) * Z_HOME_DIR;
        currentPositionSteps[Z_AXIS] = -steps;
        setHoming(true);
#if defined(Z_PROBE_DELAY) && Z_PROBE_DELAY > 0 && Z_MIN_PIN == Z_PROBE_PIN && Z_HOME_DIR == -1
        HAL::delayMilliseconds(Z_PROBE_DELAY);
#endif
        PrintLine::moveRelativeDistanceInSteps(0, 0, 2 * steps, 0, homingFeedrate[Z_AXIS], true, true);
		currentPositionSteps[Z_AXIS] = (Z_HOME_DIR == -1) ? axisMinSteps[Z_AXIS] : axisMaxSteps[Z_AXIS];
        PrintLine::moveRelativeDistanceInSteps(0, 0, axisStepsPerMM[Z_AXIS] * -ENDSTOP_Z_BACK_MOVE * Z_HOME_DIR, 0, homingFeedrate[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR, true, false);
#if defined(ZHOME_WAIT_UNSWING) && ZHOME_WAIT_UNSWING > 0
        HAL::delayMilliseconds(ZHOME_WAIT_UNSWING);
#endif
#if defined(Z_PROBE_DELAY) && Z_PROBE_DELAY > 0 && Z_MIN_PIN == Z_PROBE_PIN && Z_HOME_DIR == -1
        HAL::delayMilliseconds(Z_PROBE_DELAY);
#endif
#if Z_HOME_DIR < 0 && Z_PROBE_PIN == Z_MIN_PIN && FEATURE_Z_PROBE
#ifdef Z_PROBE_RUN_AFTER_EVERY_PROBE
GCode::executeFString(PSTR(Z_PROBE_RUN_AFTER_EVERY_PROBE));
#endif
#endif
        PrintLine::moveRelativeDistanceInSteps(0, 0, axisStepsPerMM[Z_AXIS] * 2 * ENDSTOP_Z_BACK_MOVE * Z_HOME_DIR, 0, homingFeedrate[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR, true, true);
#if Z_HOME_DIR < 0 && Z_PROBE_PIN == Z_MIN_PIN && FEATURE_Z_PROBE
        Printer::finishProbing();
#endif
        setHoming(false);
        int32_t zCorrection = 0;
#if Z_HOME_DIR < 0 && MIN_HARDWARE_ENDSTOP_Z && FEATURE_Z_PROBE && Z_PROBE_PIN == Z_MIN_PIN
        // Fix error from z probe testing
        zCorrection -= axisStepsPerMM[Z_AXIS] * EEPROM::zProbeHeight();
        // Correct from bed rotation
        //updateCurrentPosition(true);
        //float xt,yt,zt;
        //transformToPrinter(currentPosition[X_AXIS],currentPosition[Y_AXIS],0,xt,yt,zt);
        //zCorrection -= zt;
#endif
#if defined(ENDSTOP_Z_BACK_ON_HOME)
        // If we want to go up a bit more for some reason
        if(ENDSTOP_Z_BACK_ON_HOME > 0)
            zCorrection -= axisStepsPerMM[Z_AXIS] * ENDSTOP_Z_BACK_ON_HOME * Z_HOME_DIR;
#endif
        //Com::printFLN(PSTR("Z-Correction-Steps:"),zCorrection); // TEST
		PrintLine::moveRelativeDistanceInSteps(0, 0, zCorrection, 0, homingFeedrate[Z_AXIS], true, false);
		currentPositionSteps[Z_AXIS] = ((Z_HOME_DIR == -1) ? axisMinSteps[Z_AXIS] : axisMaxSteps[Z_AXIS]);
#if DISTORTION_CORRECTION && Z_HOME_DIR < 0 && Z_PROBE_PIN == Z_MIN_PIN && FEATURE_Z_PROBE
// Special case where z probe is z min end stop and distortion correction is enabled
        if(Printer::distortion.isEnabled()) {
            Printer::zCorrectionStepsIncluded = Printer::distortion.correct(Printer::currentPositionSteps[X_AXIS], currentPositionSteps[Y_AXIS], currentPositionSteps[Z_AXIS]);
            currentPositionSteps[Z_AXIS] += Printer::zCorrectionStepsIncluded;
        }
#endif
        updateCurrentPosition(true);
#if Z_HOME_DIR < 0 && Z_PROBE_PIN == Z_MIN_PIN && FEATURE_Z_PROBE
        // If we have software leveling enabled and are not at 0,0 z position is not zero, but we measured
        // for z = 0, so we need to correct for rotation.
        currentPositionSteps[Z_AXIS] -= (axisStepsPerMM[Z_AXIS] * currentPosition[Z_AXIS] - axisMinSteps[Z_AXIS]);
        currentPosition[Z_AXIS] = zMin;
#endif
		setZHomed(true);
	}
}

/** \brief Main function for all homing operations.

For homing operations only this function should be used. It calls Printer::homeXAxis, Printer::homeYAxis and Printer::homeZAxis
after doing some initialization work. The order of operation and some extra functions are controlled by HOMING_ORDER.

\param xaxis True if homing of x axis is wanted.
\param yaxis True if homing of y axis is wanted.
\param zaxis True if homing of z axis is wanted.
*/
void Printer::homeAxis(bool xaxis, bool yaxis, bool zaxis) { // home non-delta printer
    bool nocheck = isNoDestinationCheck();
    setNoDestinationCheck(true);

    float startX, startY, startZ, startA;
	realPosition(startX, startY, startZ, startA);
#if !defined(HOMING_ORDER)
#define HOMING_ORDER HOME_ORDER_XYZ
#endif
#if Z_HOME_DIR < 0
#if ZHOME_PRE_RAISE == 1
    if(zaxis && Endstops::zProbe())
        PrintLine::moveRelativeDistanceInSteps(0, 0, ZHOME_PRE_RAISE_DISTANCE * axisStepsPerMM[Z_AXIS], 0, homingFeedrate[Z_AXIS], true, true);
#elif ZHOME_PRE_RAISE == 2
    if(zaxis)
        PrintLine::moveRelativeDistanceInSteps(0, 0, ZHOME_PRE_RAISE_DISTANCE * axisStepsPerMM[Z_AXIS], 0, homingFeedrate[Z_AXIS], true, true);
#endif
#endif
#if Z_HOME_DIR < 0 && Z_PROBE_PIN == Z_MIN_PIN && FEATURE_Z_PROBE
#if HOMING_ORDER != HOME_ORDER_XYZ && HOMING_ORDER != HOME_ORDER_YXZ
#error Illegal homing order for z probe based homing!
#endif
    if(zaxis) { // we need to know xy position for z probe to work properly
        if(!xaxis && !isXHomed())
            xaxis = true;
        if(!yaxis && !isYHomed())
            yaxis = true;
    }
#endif
    if(zaxis) {
        EVENT_BEFORE_Z_HOME;
    }
#if HOMING_ORDER == HOME_ORDER_XYZ
    if(xaxis) homeXAxis();
    if(yaxis) homeYAxis();
    if(zaxis) homeZAxis();
#elif HOMING_ORDER == HOME_ORDER_XZY
    if(xaxis) homeXAxis();
    if(zaxis) homeZAxis();
    if(yaxis) homeYAxis();
#elif HOMING_ORDER == HOME_ORDER_YXZ
    if(yaxis) homeYAxis();
    if(xaxis) homeXAxis();
    if(zaxis) homeZAxis();
#elif HOMING_ORDER == HOME_ORDER_YZX
    if(yaxis) homeYAxis();
    if(zaxis) homeZAxis();
    if(xaxis) homeXAxis();
#elif HOMING_ORDER == HOME_ORDER_ZXY
    if(zaxis) homeZAxis();
    if(xaxis) homeXAxis();
    if(yaxis) homeYAxis();
#elif HOMING_ORDER == HOME_ORDER_ZYX
    if(zaxis) homeZAxis();
    if(yaxis) homeYAxis();
	if(xaxis) homeXAxis();
#endif
	if(xaxis) {
		if(X_HOME_DIR < 0) startX = Printer::axisMin[X_AXIS];
		else startX = Printer::axisMin[X_AXIS] + Printer::axisLength[X_AXIS];
    }
    if(yaxis) {
		if(Y_HOME_DIR < 0) startY = Printer::axisMin[Y_AXIS];
		else startY = Printer::axisMin[Y_AXIS] + Printer::axisLength[Y_AXIS];
	}
    if(zaxis) {
		if(Z_HOME_DIR < 0) startZ = Printer::axisMin[Z_AXIS];
		else startZ = Printer::axisMin[Z_AXIS] + Printer::axisLength[Z_AXIS];
	}
    updateCurrentPosition(true);
#if defined(Z_UP_AFTER_HOME) && Z_HOME_DIR < 0
#ifdef HOME_ZUP_FIRST
    PrintLine::moveRelativeDistanceInSteps(0, 0, axisStepsPerMM[Z_AXIS]*Z_UP_AFTER_HOME * Z_HOME_DIR, 0, homingFeedrate[Z_AXIS], true, false);
#endif
    if(zaxis)
        startZ = Z_UP_AFTER_HOME;
#endif
	moveToReal(startX, startY, startZ, IGNORE_COORDINATE, homingFeedrate[X_AXIS]);
	updateCurrentPosition(true);
    updateHomedAll();
	Commands::printCurrentPosition();
    setNoDestinationCheck(nocheck);
    Printer::updateCurrentPosition();
}

void Printer::setCaseLight(bool on) {
#if CASE_LIGHTS_PIN > -1
    WRITE(CASE_LIGHTS_PIN, on);
    reportCaseLightStatus();
#endif
}

void Printer::reportCaseLightStatus() {
#if CASE_LIGHTS_PIN > -1
    if(READ(CASE_LIGHTS_PIN))
        Com::printInfoFLN(PSTR("Case lights on"));
    else
        Com::printInfoFLN(PSTR("Case lights off"));
#else
    Com::printInfoFLN(PSTR("No case lights"));
#endif
}

void Printer::handleInterruptEvent() {
    if(interruptEvent == 0) return;
    int event = interruptEvent;
    interruptEvent = 0;
    switch(event) {

    }
}

void Printer::showConfiguration() {
	Com::config(PSTR("Baudrate:"), baudrate);
#ifndef EXTERNALSERIAL
    Com::config(PSTR("InputBuffer:"), SERIAL_BUFFER_SIZE - 1);
#endif
	Com::config(PSTR("SDCard:"), SDSUPPORT);
    Com::config(PSTR("Fan:"), FAN_PIN > -1 && FEATURE_FAN_CONTROL);
#if FEATURE_FAN2_CONTROL && defined(FAN2_PIN) && FAN2_PIN > -1
    Com::config(PSTR("Fan2:1"));
#else
    Com::config(PSTR("Fan2:0"));
#endif
	Com::config(PSTR("SoftwarePowerSwitch:"), PS_ON_PIN > -1);
    Com::config(PSTR("XHomeDir:"), X_HOME_DIR);
    Com::config(PSTR("YHomeDir:"), Y_HOME_DIR);
	Com::config(PSTR("ZHomeDir:"), Z_HOME_DIR);
	Com::config(PSTR("XHomePos:"), axisMin[X_AXIS] + (X_HOME_DIR > 0 ? axisLength[X_AXIS] : 0), 2);
	Com::config(PSTR("YHomePos:"), axisMin[Y_AXIS] + (Y_HOME_DIR > 0 ? axisLength[Y_AXIS] : 0), 2);
	Com::config(PSTR("ZHomePos:"), axisMin[Z_AXIS] + (Z_HOME_DIR > 0 ? axisLength[Z_AXIS] : 0), 3);
	Com::config(PSTR("CaseLights:"), CASE_LIGHTS_PIN > -1);
    Com::config(PSTR("ZProbe:"), FEATURE_Z_PROBE);
	Com::config(PSTR("EEPROM:"), EEPROM_MODE != 0);
    Com::config(PSTR("PrintlineCache:"), PRINTLINE_CACHE_SIZE);
	Com::config(PSTR("KeepAliveInterval:"), KEEP_ALIVE_INTERVAL);
	Com::config(PSTR("JerkX:"), maxJerk[X_AXIS]);
	Com::config(PSTR("JerkY:"), maxJerk[Y_AXIS]);
	Com::config(PSTR("JerkZ:"), maxJerk[Z_AXIS]);
	Com::config(PSTR("JerkA:"), maxJerk[A_AXIS]);
	Com::config(PSTR("XMin:"), axisMin[X_AXIS]);
	Com::config(PSTR("YMin:"), axisMin[Y_AXIS]);
	Com::config(PSTR("ZMin:"), axisMin[Z_AXIS]);
	Com::config(PSTR("XMax:"), axisMin[X_AXIS] + axisLength[X_AXIS]);
	Com::config(PSTR("YMax:"), axisMin[Y_AXIS] + axisLength[Y_AXIS]);
	Com::config(PSTR("ZMax:"), axisMin[Z_AXIS] + axisLength[Z_AXIS]);
	Com::config(PSTR("XSize:"), axisLength[X_AXIS]);
	Com::config(PSTR("YSize:"), axisLength[Y_AXIS]);
	Com::config(PSTR("ZSize:"), axisLength[Z_AXIS]);
	Com::config(PSTR("XPrintAccel:"), maxAccelerationMMPerSquareSecond[X_AXIS]);
	Com::config(PSTR("YPrintAccel:"), maxAccelerationMMPerSquareSecond[Y_AXIS]);
	Com::config(PSTR("ZPrintAccel:"), maxAccelerationMMPerSquareSecond[Z_AXIS]);
	Com::config(PSTR("APrintAccel:"), maxAccelerationMMPerSquareSecond[A_AXIS]);
	Com::config(PSTR("PrinterType:Cartesian"));
}

#if TMC_DRIVERS
void Printer::configTMC5160(TMC5160Stepper* driver, uint8_t intpol, uint16_t rms, float hold_mult, uint8_t hold_delay, uint8_t tpower_down,
							uint8_t hstart, uint8_t hend, uint8_t toff, uint8_t tbl, uint8_t tpfd, uint8_t pwm_freq, uint16_t tpwmthrs,
							uint16_t tcoolthrs, uint16_t thighthrs, uint8_t semin, uint8_t semax, int8_t sgt, uint8_t s2vs, uint8_t s2g,
							uint8_t sfilter, uint16_t microsteps, uint8_t pwm_grad, uint8_t pwm_ofs, uint8_t pwm_lim, uint8_t mode)
{
	wdt_reset();
	driver->begin();
	driver->intpol(intpol);
	driver->rms_current(rms, hold_mult);	// Set motor RMS current
	driver->iholddelay(hold_delay);
	driver->TPOWERDOWN(tpower_down);
	driver->hysteresis_start(hstart);
	driver->hysteresis_end(hend);
	driver->toff(toff);                 	// Enables driver in software
	driver->tbl(tbl);
	driver->tpfd(tpfd);
	driver->pwm_freq(pwm_freq);
	driver->TPWMTHRS(tpwmthrs);
	driver->TCOOLTHRS(tcoolthrs);
	driver->THIGH(thighthrs);
	driver->semin(semin);
	driver->semax(semax);
	driver->sgt(sgt);
	driver->s2vs_level(s2vs);
	driver->s2g_level(s2g);
	driver->shortfilter(sfilter);
	driver->microsteps(microsteps);
	if(mode == TMC_STEALTH)
	{
		driver->en_pwm_mode(true);       	// Toggle stealthChop on TMC2130/2160/5130/5160
		driver->pwm_grad(pwm_grad);
		driver->pwm_ofs(pwm_ofs);
		driver->pwm_lim(pwm_lim);
		driver->pwm_autoscale(true);     	// Needed for stealthChop
		driver->pwm_autograd(true);
	} else
	{
		driver->en_pwm_mode(false);       	// Toggle stealthChop on TMC2130/2160/5130/5160
	}
	wdt_reset();
}

void Printer::CheckTMCDrivers()
{
	if(Printer::tmcStepperX.DRV_STATUS() & 0b00011010000000000011000000000000)
	{
		Com::printErrorFLN(PSTR("X TMC Driver faulted."));
		Commands::emergencyStop();
	} else if(Printer::tmcStepperY.DRV_STATUS() & 0b00011010000000000011000000000000)
	{
		Com::printErrorFLN(PSTR("Y TMC Driver faulted."));
		Commands::emergencyStop();
	} else if(Printer::tmcStepperZ.DRV_STATUS() & 0b00011010000000000011000000000000)
	{
		Com::printErrorFLN(PSTR("Z TMC Driver faulted."));
		Commands::emergencyStop();
	} else if(Printer::tmcStepper2.DRV_STATUS() & 0b00011010000000000011000000000000)
	{
		Com::printErrorFLN(PSTR("2 TMC Driver faulted."));
		Commands::emergencyStop();
	}
}
#endif
