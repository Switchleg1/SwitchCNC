#include "SwitchCNC.h"

void EEPROM::update(GCode *com) {
#if EEPROM_MODE != 0
    if(com->hasT() && com->hasP()) switch(com->T)
        {
        case 0:
            if(com->hasS())
                HAL::eprSetByte(com->P, (uint8_t)com->S);
            break;
        case 1:
            if(com->hasS())
                HAL::eprSetInt16(com->P, (int16_t)com->S);
            break;
        case 2:
            if(com->hasS())
                HAL::eprSetInt32(com->P, (int32_t)com->S);
            break;
        case 3:
            if(com->hasX())
                HAL::eprSetFloat(com->P, com->X);
            break;
        }
	updateChecksum(EEPROM_MAIN_OFFSET, EEPROM_MAIN_LENGTH);
	readDataFromEEPROM();
#else
    Com::printErrorF(Com::tNoEEPROMSupport);
#endif
}

void EEPROM::restoreEEPROMSettingsFromConfiguration() {
	// can only be done right if we also update permanent values not cached!
#if EEPROM_MODE != 0
	Machine::baudrate = BAUDRATE;
	Machine::maxInactiveTime = MAX_INACTIVE_TIME * 1000L;
	Machine::stepperInactiveTime = STEPPER_INACTIVE_TIME * 1000L;
    Machine::axisStepsPerMM[X_AXIS] = XAXIS_STEPS_PER_MM;
	Machine::axisStepsPerMM[Y_AXIS] = YAXIS_STEPS_PER_MM;
	Machine::axisStepsPerMM[Z_AXIS] = ZAXIS_STEPS_PER_MM;
	Machine::axisStepsPerMM[A_AXIS] = AAXIS_STEPS_PER_MM;
    Machine::maxFeedrate[X_AXIS] = MAX_FEEDRATE_X;
    Machine::maxFeedrate[Y_AXIS] = MAX_FEEDRATE_Y;
	Machine::maxFeedrate[Z_AXIS] = MAX_FEEDRATE_Z;
	Machine::maxFeedrate[A_AXIS] = MAX_FEEDRATE_A;
	Machine::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X;
    Machine::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y;
    Machine::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z;
	Machine::maxJerk[X_AXIS] = MAX_XJERK;
	Machine::maxJerk[Y_AXIS] = MAX_YJERK;
	Machine::maxJerk[Z_AXIS] = MAX_ZJERK;
	Machine::maxJerk[A_AXIS] = MAX_AJERK;
#if RAMP_ACCELERATION
	Machine::maxAccelerationMMPerSquareSecond[X_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X;
    Machine::maxAccelerationMMPerSquareSecond[Y_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y;
	Machine::maxAccelerationMMPerSquareSecond[Z_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z;
	Machine::maxAccelerationMMPerSquareSecond[A_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_A;
#endif
	Machine::axisLength[X_AXIS] = X_MAX_LENGTH;
	Machine::axisLength[Y_AXIS] = Y_MAX_LENGTH;
	Machine::axisLength[Z_AXIS] = Z_MAX_LENGTH;
	Machine::axisMin[X_AXIS] = X_MIN_POS;
	Machine::axisMin[Y_AXIS] = Y_MIN_POS;
	Machine::axisMin[Z_AXIS] = Z_MIN_POS;
#if BACKLASH_COMPENSATION_SUPPORT
	Backlash::backlash[X_AXIS] = X_BACKLASH;
	Backlash::backlash[Y_AXIS] = Y_BACKLASH;
	Backlash::backlash[Z_AXIS] = Z_BACKLASH;
	Backlash::backlash[A_AXIS] = A_BACKLASH;
#endif
#if DISTORTION_CORRECTION_SUPPORT
	Distortion::XMIN		= DISTORTION_XMIN;
	Distortion::XMAX		= DISTORTION_XMAX;
	Distortion::YMIN		= DISTORTION_YMIN;
	Distortion::YMAX		= DISTORTION_YMAX;
	Distortion::setPoints(DISTORTION_CORRECTION_POINTS);
	Distortion::start		= DISTORTION_START;
	Distortion::end			= DISTORTION_END;
	Distortion::useOffset	= DISTORTION_USE_OFFSET;
	Distortion::resetCorrection();
#endif
#if ALLOW_PARTIAL_GCODE_AS_MOVE
	Commands::allowPartialGCode = ALLOW_PARTIAL_GCODE_DEFAULT;
#endif
	storeDataIntoEEPROM(true); // Store new fields for changed version
	Machine::updateDerivedParameter();
	Com::printInfoFLN(Com::tEPRConfigResetDefaults);
#else
    Com::printErrorFLN(Com::tNoEEPROMSupport);
#endif
}

void EEPROM::storeDataIntoEEPROM(uint8_t corrupted) {
#if EEPROM_MODE != 0
    HAL::eprSetInt32(EPR_BAUDRATE, Machine::baudrate);
    HAL::eprSetInt32(EPR_MAX_INACTIVE_TIME, Machine::maxInactiveTime);
    HAL::eprSetInt32(EPR_STEPPER_INACTIVE_TIME, Machine::stepperInactiveTime);
	HAL::eprSetFloat(EPR_XAXIS_STEPS_PER_MM,Machine::axisStepsPerMM[X_AXIS]);
    HAL::eprSetFloat(EPR_YAXIS_STEPS_PER_MM,Machine::axisStepsPerMM[Y_AXIS]);
	HAL::eprSetFloat(EPR_ZAXIS_STEPS_PER_MM,Machine::axisStepsPerMM[Z_AXIS]);
	HAL::eprSetFloat(EPR_AAXIS_STEPS_PER_MM,Machine::axisStepsPerMM[A_AXIS]);
	HAL::eprSetFloat(EPR_X_MAX_FEEDRATE,Machine::maxFeedrate[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_MAX_FEEDRATE,Machine::maxFeedrate[Y_AXIS]);
	HAL::eprSetFloat(EPR_Z_MAX_FEEDRATE,Machine::maxFeedrate[Z_AXIS]);
	HAL::eprSetFloat(EPR_A_MAX_FEEDRATE,Machine::maxFeedrate[A_AXIS]);
	HAL::eprSetFloat(EPR_X_HOMING_FEEDRATE,Machine::homingFeedrate[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_HOMING_FEEDRATE,Machine::homingFeedrate[Y_AXIS]);
	HAL::eprSetFloat(EPR_Z_HOMING_FEEDRATE,Machine::homingFeedrate[Z_AXIS]);
	HAL::eprSetFloat(EPR_MAX_XJERK,Machine::maxJerk[X_AXIS]);
	HAL::eprSetFloat(EPR_MAX_YJERK,Machine::maxJerk[Y_AXIS]);
	HAL::eprSetFloat(EPR_MAX_ZJERK,Machine::maxJerk[Z_AXIS]);
	HAL::eprSetFloat(EPR_MAX_AJERK,Machine::maxJerk[A_AXIS]);
#if RAMP_ACCELERATION
	HAL::eprSetFloat(EPR_X_MAX_ACCEL,Machine::maxAccelerationMMPerSquareSecond[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_MAX_ACCEL,Machine::maxAccelerationMMPerSquareSecond[Y_AXIS]);
	HAL::eprSetFloat(EPR_Z_MAX_ACCEL,Machine::maxAccelerationMMPerSquareSecond[Z_AXIS]);
	HAL::eprSetFloat(EPR_A_MAX_ACCEL,Machine::maxAccelerationMMPerSquareSecond[A_AXIS]);
#endif
	HAL::eprSetFloat(EPR_X_HOME_OFFSET,Machine::axisMin[X_AXIS]);
	HAL::eprSetFloat(EPR_Y_HOME_OFFSET,Machine::axisMin[Y_AXIS]);
	HAL::eprSetFloat(EPR_Z_HOME_OFFSET,Machine::axisMin[Z_AXIS]);
	HAL::eprSetFloat(EPR_X_LENGTH,Machine::axisLength[X_AXIS]);
	HAL::eprSetFloat(EPR_Y_LENGTH,Machine::axisLength[Y_AXIS]);
	HAL::eprSetFloat(EPR_Z_LENGTH,Machine::axisLength[Z_AXIS]);
#if BACKLASH_COMPENSATION_SUPPORT
	HAL::eprSetFloat(EPR_BACKLASH_X, Backlash::backlash[X_AXIS]);
	HAL::eprSetFloat(EPR_BACKLASH_Y, Backlash::backlash[Y_AXIS]);
	HAL::eprSetFloat(EPR_BACKLASH_Z, Backlash::backlash[Z_AXIS]);
	HAL::eprSetFloat(EPR_BACKLASH_Z, Backlash::backlash[A_AXIS]);
#endif
#if DISTORTION_CORRECTION_SUPPORT
	HAL::eprSetInt16(EPR_DISTORTION_XMIN, Distortion::XMIN);
	HAL::eprSetInt16(EPR_DISTORTION_XMAX, Distortion::XMAX);
	HAL::eprSetInt16(EPR_DISTORTION_YMIN, Distortion::YMIN);
	HAL::eprSetInt16(EPR_DISTORTION_YMAX, Distortion::YMAX);
	HAL::eprSetByte(EPR_DISTORTION_POINTS, Distortion::getPoints());
	HAL::eprSetFloat(EPR_DISTORTION_START, Distortion::start);
	HAL::eprSetFloat(EPR_DISTORTION_END, Distortion::end);
	HAL::eprSetByte(EPR_DISTORTION_USE_OFFSET, Distortion::useOffset);
#endif
#if ALLOW_PARTIAL_GCODE_AS_MOVE
	HAL::eprSetByte(EPR_ALLOW_PARTIAL_GCODE_AS_MOVE, Commands::allowPartialGCode);
#endif
    if(corrupted) {
		initalizeUncached();
    }
    // Save version and build checksum
    HAL::eprSetByte(EPR_VERSION, EEPROM_VERSION);
	updateChecksum(EEPROM_MAIN_OFFSET, EEPROM_MAIN_LENGTH);
#endif
}

void EEPROM::initalizeUncached() {
    HAL::eprSetFloat(EPR_Z_PROBE_HEIGHT, Z_PROBE_HEIGHT);
    HAL::eprSetFloat(EPR_Z_PROBE_SPEED, Z_PROBE_SPEED);
	HAL::eprSetFloat(EPR_Z_PROBE_XY_SPEED, Z_PROBE_XY_SPEED);
	HAL::eprSetByte(EPR_DISTORTION_CORRECTION_ENABLED, 0);
	HAL::eprSetByte(EPR_ALLOW_PARTIAL_GCODE_AS_MOVE, ALLOW_PARTIAL_GCODE_DEFAULT);
}

void EEPROM::readDataFromEEPROM() {
#if EEPROM_MODE != 0
    uint8_t version = HAL::eprGetByte(EPR_VERSION); // This is the saved version. Don't copy data nor set it to older versions!
    Com::printFLN(PSTR("Detected EEPROM version:"),(int)version);

	if (version != EEPROM_VERSION) {
		restoreEEPROMSettingsFromConfiguration();
		Com::printFLN(PSTR("Restored values from configuration"));
	}

	Machine::baudrate = HAL::eprGetInt32(EPR_BAUDRATE);
	Machine::maxInactiveTime = HAL::eprGetInt32(EPR_MAX_INACTIVE_TIME);
	Machine::stepperInactiveTime = HAL::eprGetInt32(EPR_STEPPER_INACTIVE_TIME);
	Machine::axisStepsPerMM[X_AXIS] = HAL::eprGetFloat(EPR_XAXIS_STEPS_PER_MM);
    Machine::axisStepsPerMM[Y_AXIS] = HAL::eprGetFloat(EPR_YAXIS_STEPS_PER_MM);
	Machine::axisStepsPerMM[Z_AXIS] = HAL::eprGetFloat(EPR_ZAXIS_STEPS_PER_MM);
	Machine::axisStepsPerMM[A_AXIS] = HAL::eprGetFloat(EPR_AAXIS_STEPS_PER_MM);
	Machine::maxFeedrate[X_AXIS] = HAL::eprGetFloat(EPR_X_MAX_FEEDRATE);
	Machine::maxFeedrate[Y_AXIS] = HAL::eprGetFloat(EPR_Y_MAX_FEEDRATE);
	Machine::maxFeedrate[Z_AXIS] = HAL::eprGetFloat(EPR_Z_MAX_FEEDRATE);
	Machine::maxFeedrate[A_AXIS] = HAL::eprGetFloat(EPR_A_MAX_FEEDRATE);
	Machine::homingFeedrate[X_AXIS] = HAL::eprGetFloat(EPR_X_HOMING_FEEDRATE);
	Machine::homingFeedrate[Y_AXIS] = HAL::eprGetFloat(EPR_Y_HOMING_FEEDRATE);
    Machine::homingFeedrate[Z_AXIS] = HAL::eprGetFloat(EPR_Z_HOMING_FEEDRATE);
	Machine::maxJerk[X_AXIS] = HAL::eprGetFloat(EPR_MAX_XJERK);
	Machine::maxJerk[Y_AXIS] = HAL::eprGetFloat(EPR_MAX_YJERK);
	Machine::maxJerk[Z_AXIS] = HAL::eprGetFloat(EPR_MAX_ZJERK);
	Machine::maxJerk[A_AXIS] = HAL::eprGetFloat(EPR_MAX_AJERK);
#if RAMP_ACCELERATION
	Machine::maxAccelerationMMPerSquareSecond[X_AXIS] = HAL::eprGetFloat(EPR_X_MAX_ACCEL);
	Machine::maxAccelerationMMPerSquareSecond[Y_AXIS] = HAL::eprGetFloat(EPR_Y_MAX_ACCEL);
	Machine::maxAccelerationMMPerSquareSecond[Z_AXIS] = HAL::eprGetFloat(EPR_Z_MAX_ACCEL);
	Machine::maxAccelerationMMPerSquareSecond[A_AXIS] = HAL::eprGetFloat(EPR_A_MAX_ACCEL);
#endif
	Machine::axisMin[X_AXIS] = HAL::eprGetFloat(EPR_X_HOME_OFFSET);
	Machine::axisMin[Y_AXIS] = HAL::eprGetFloat(EPR_Y_HOME_OFFSET);
	Machine::axisMin[Z_AXIS] = HAL::eprGetFloat(EPR_Z_HOME_OFFSET);
	Machine::axisLength[X_AXIS] = HAL::eprGetFloat(EPR_X_LENGTH);
	Machine::axisLength[Y_AXIS] = HAL::eprGetFloat(EPR_Y_LENGTH);
	Machine::axisLength[Z_AXIS] = HAL::eprGetFloat(EPR_Z_LENGTH);
#if BACKLASH_COMPENSATION_SUPPORT
	Backlash::backlash[X_AXIS] = HAL::eprGetFloat(EPR_BACKLASH_X);
	Backlash::backlash[Y_AXIS] = HAL::eprGetFloat(EPR_BACKLASH_Y);
	Backlash::backlash[Z_AXIS] = HAL::eprGetFloat(EPR_BACKLASH_Z);
	Backlash::backlash[A_AXIS] = HAL::eprGetFloat(EPR_BACKLASH_A);
#endif
#if DISTORTION_CORRECTION_SUPPORT
	Distortion::XMIN		= HAL::eprGetInt16(EPR_DISTORTION_XMIN);
	Distortion::XMAX		= HAL::eprGetInt16(EPR_DISTORTION_XMAX);
	Distortion::YMIN		= HAL::eprGetInt16(EPR_DISTORTION_YMIN);
	Distortion::YMAX		= HAL::eprGetInt16(EPR_DISTORTION_YMAX);
	Distortion::setPoints(HAL::eprGetByte(EPR_DISTORTION_POINTS));
	Distortion::start		= HAL::eprGetFloat(EPR_DISTORTION_START);
	Distortion::end			= HAL::eprGetFloat(EPR_DISTORTION_END);
	Distortion::useOffset	= HAL::eprGetByte(EPR_DISTORTION_USE_OFFSET);
	Distortion::SetStartEnd(Distortion::start, Distortion::end);
#endif
#if ALLOW_PARTIAL_GCODE_AS_MOVE
	Commands::allowPartialGCode = HAL::eprGetByte(EPR_ALLOW_PARTIAL_GCODE_AS_MOVE);
#endif

	Machine::updateDerivedParameter();
#endif
}

void EEPROM::initBaudrate() {
    // Invariant - baudrate is initialized with or without eeprom!
	Machine::baudrate = BAUDRATE;
#if EEPROM_MODE != 0
    if(HAL::eprGetByte(EPR_MAGIC_BYTE) == EEPROM_MODE) {
		Machine::baudrate = HAL::eprGetInt32(EPR_BAUDRATE);
    }
#endif
}

void EEPROM::init() {
#if EEPROM_MODE != 0
    uint8_t check = computeChecksum(EEPROM_MAIN_OFFSET, EEPROM_MAIN_LENGTH);
    uint8_t storedcheck = HAL::eprGetByte(EEPROM_MAIN_OFFSET);
	if (HAL::eprGetByte(EPR_MAGIC_BYTE) != EEPROM_MODE || storedcheck != check) {
		HAL::eprSetByte(EPR_MAGIC_BYTE, EEPROM_MODE); // Make data change permanent
		restoreEEPROMSettingsFromConfiguration();
	}

    readDataFromEEPROM();
    if (USE_CONFIGURATION_BAUD_RATE) {
        // Used if eeprom gets unusable baud rate set and communication wont work at all.
        if(HAL::eprGetInt32(EPR_BAUDRATE) != BAUDRATE) {
            HAL::eprSetInt32(EPR_BAUDRATE,BAUDRATE);
			Machine::baudrate = BAUDRATE;
			updateChecksum(EEPROM_MAIN_OFFSET, EEPROM_MAIN_LENGTH);
        }

        Com::printFLN(PSTR("EEPROM baud rate restored from configuration."));
        Com::printFLN(PSTR("RECOMPILE WITH USE_CONFIGURATION_BAUD_RATE == 0 to alter baud rate via EEPROM"));
    }
#endif
}

/** \brief Writes all eeprom settings to serial console.

For each value stored, this function generates one line with syntax

EPR: pos type value description

With
- pos = Position in EEPROM, the data starts.
- type = Value type: 0 = byte, 1 = int, 2 = long, 3 = float
- value = The value currently stored
- description = Definition of the value
*/
void EEPROM::writeSettings() {
#if EEPROM_MODE != 0
	writeLong(EPR_BAUDRATE, Com::tEPRBaudrate);
	writeLong(EPR_MAX_INACTIVE_TIME, Com::tEPRMaxInactiveTime);
	writeLong(EPR_STEPPER_INACTIVE_TIME, Com::tEPRStopAfterInactivty);
    writeFloat(EPR_XAXIS_STEPS_PER_MM, Com::tEPRXStepsPerMM, 4);
	writeFloat(EPR_YAXIS_STEPS_PER_MM, Com::tEPRYStepsPerMM, 4);
	writeFloat(EPR_ZAXIS_STEPS_PER_MM, Com::tEPRZStepsPerMM, 4);
	writeFloat(EPR_AAXIS_STEPS_PER_MM, Com::tEPRAStepsPerMM, 4);
	writeFloat(EPR_X_MAX_FEEDRATE, Com::tEPRXMaxFeedrate);
	writeFloat(EPR_Y_MAX_FEEDRATE, Com::tEPRYMaxFeedrate);
	writeFloat(EPR_Z_MAX_FEEDRATE, Com::tEPRZMaxFeedrate);
	writeFloat(EPR_A_MAX_FEEDRATE, Com::tEPRAMaxFeedrate);
	writeFloat(EPR_X_HOMING_FEEDRATE, Com::tEPRXHomingFeedrate);
	writeFloat(EPR_Y_HOMING_FEEDRATE, Com::tEPRYHomingFeedrate);
	writeFloat(EPR_Z_HOMING_FEEDRATE, Com::tEPRZHomingFeedrate);
	writeFloat(EPR_MAX_XJERK, Com::tEPRMaxXJerk);
	writeFloat(EPR_MAX_YJERK, Com::tEPRMaxYJerk);
	writeFloat(EPR_MAX_ZJERK, Com::tEPRMaxZJerk);
	writeFloat(EPR_MAX_AJERK, Com::tEPRMaxAJerk);
	writeFloat(EPR_X_HOME_OFFSET, Com::tEPRXHomePos);
	writeFloat(EPR_Y_HOME_OFFSET, Com::tEPRYHomePos);
    writeFloat(EPR_Z_HOME_OFFSET, Com::tEPRZHomePos);
	writeFloat(EPR_X_LENGTH, Com::tEPRXMaxLength);
	writeFloat(EPR_Y_LENGTH, Com::tEPRYMaxLength);
	writeFloat(EPR_Z_LENGTH, Com::tEPRZMaxLength);
#if BACKLASH_COMPENSATION_SUPPORT
    writeFloat(EPR_BACKLASH_X, Com::tEPRXBacklash);
    writeFloat(EPR_BACKLASH_Y, Com::tEPRYBacklash);
	writeFloat(EPR_BACKLASH_Z, Com::tEPRZBacklash);
	writeFloat(EPR_BACKLASH_A, Com::tEPRABacklash);
#endif
#if RAMP_ACCELERATION
	writeFloat(EPR_X_MAX_ACCEL, Com::tEPRXAcceleration);
	writeFloat(EPR_Y_MAX_ACCEL, Com::tEPRYAcceleration);
	writeFloat(EPR_Z_MAX_ACCEL, Com::tEPRZAcceleration);
	writeFloat(EPR_A_MAX_ACCEL, Com::tEPRAAcceleration);
#endif
#if Z_PROBE_SUPPORT
    writeFloat(EPR_Z_PROBE_HEIGHT, Com::tZProbeHeight);
	writeFloat(EPR_Z_PROBE_SPEED, Com::tZProbeSpeed);
	writeFloat(EPR_Z_PROBE_XY_SPEED, Com::tZProbeSpeedXY);
#endif
#if DISTORTION_CORRECTION_SUPPORT
	writeInt(EPR_DISTORTION_XMIN, Com::tDistortionXMIN);
	writeInt(EPR_DISTORTION_XMAX, Com::tDistortionXMAX);
	writeInt(EPR_DISTORTION_YMIN, Com::tDistortionYMIN);
	writeInt(EPR_DISTORTION_YMAX, Com::tDistortionYMAX);
	writeByte(EPR_DISTORTION_POINTS, Com::tDistortionPoints);
	writeFloat(EPR_DISTORTION_START, Com::tDistortionStart);
	writeFloat(EPR_DISTORTION_END, Com::tDistortionEnd);
	writeByte(EPR_DISTORTION_USE_OFFSET, Com::tDistortionUseOffset);
#endif
#else
    Com::printErrorF(Com::tNoEEPROMSupport);
#endif
}

void EEPROM::setVersion(uint8_t version) {
	if (version != getVersion()) {
		HAL::eprSetByte(EPR_VERSION, version);

		updateChecksum(EEPROM_MAIN_OFFSET, EEPROM_MAIN_LENGTH);
	}
}

uint8_t EEPROM::getVersion() {
	return HAL::eprGetByte(EPR_VERSION);
}

#if Z_PROBE_SUPPORT
void EEPROM::setZProbeHeight(float mm) {
#if EEPROM_MODE != 0
	HAL::eprSetFloat(EPR_Z_PROBE_HEIGHT, mm);
	Com::printFLN(PSTR("Z-Probe height set to: "), mm, 3);

	updateChecksum(EEPROM_MAIN_OFFSET, EEPROM_MAIN_LENGTH);
#endif
}

float EEPROM::zProbeSpeed() {
#if EEPROM_MODE != 0
	return HAL::eprGetFloat(EPR_Z_PROBE_SPEED);
#else
	return Z_PROBE_SPEED;
#endif
}

float EEPROM::zProbeXYSpeed() {
#if EEPROM_MODE != 0
	return HAL::eprGetFloat(EPR_Z_PROBE_XY_SPEED);
#else
	return Z_PROBE_XY_SPEED;
#endif
}

float EEPROM::zProbeHeight() {
#if EEPROM_MODE != 0
	return HAL::eprGetFloat(EPR_Z_PROBE_HEIGHT);
#else
	return Z_PROBE_HEIGHT;
#endif
}
#endif

#if DISTORTION_CORRECTION_SUPPORT && EEPROM_MODE
void EEPROM::setZCorrectionPoints(uint8_t count) {
	HAL::eprSetInt16(EPR_DISTORTION_POINTS, count);

	updateChecksum(EEPROM_MAIN_OFFSET, EEPROM_MAIN_LENGTH);
}

void EEPROM::setZCorrectionMinMax(int16_t xMin, int16_t yMin, int16_t xMax, int16_t yMax) {
	HAL::eprSetInt16(EPR_DISTORTION_XMIN, xMin);
	HAL::eprSetInt16(EPR_DISTORTION_YMIN, yMin);
	HAL::eprSetInt16(EPR_DISTORTION_XMAX, xMax);
	HAL::eprSetInt16(EPR_DISTORTION_YMAX, yMax);

	updateChecksum(EEPROM_MAIN_OFFSET, EEPROM_MAIN_LENGTH);
}

void EEPROM::setZCorrection(int32_t* data, uint16_t count) {
	for (uint16_t i = 0; i < count; i++) {
		HAL::eprSetInt32(EPR_DISTORTION_POINT_DATA + (i << 2), data[i]);
	}
}

void EEPROM::getZCorrection(int32_t* data, uint16_t count) {
	for (uint16_t i = 0; i < count; i++) {
		data[i] = HAL::eprGetInt32(EPR_DISTORTION_POINT_DATA + (i << 2));
	}
}

void EEPROM::setZCorrectionEnabled(int8_t on) {
	if (isZCorrectionEnabled() == on) return;
	HAL::eprSetByte(EPR_DISTORTION_CORRECTION_ENABLED, on);

	updateChecksum(EEPROM_MAIN_OFFSET, EEPROM_MAIN_LENGTH);
}

int8_t EEPROM::isZCorrectionEnabled() {
	return HAL::eprGetByte(EPR_DISTORTION_CORRECTION_ENABLED);
}
#endif

#if ALLOW_PARTIAL_GCODE_AS_MOVE && EEPROM_MODE

void EEPROM::setAllowPartialGCode(uint8_t allow) {
	if (getAllowPartialGCode() != allow) {
		HAL::eprSetByte(EPR_ALLOW_PARTIAL_GCODE_AS_MOVE, allow);

		updateChecksum(EEPROM_MAIN_OFFSET, EEPROM_MAIN_LENGTH);
	}
}

uint8_t EEPROM::getAllowPartialGCode() {
	return HAL::eprGetByte(EPR_ALLOW_PARTIAL_GCODE_AS_MOVE);
}

#endif

#if AUTO_SAVE_RESTORE_STATE

void EEPROM::setCurrentValidState(uint8_t isValid) {
	if (getCurrentValidState() != isValid) {
		HAL::eprSetByte(EPR_STATE_VALID, isValid);

		updateChecksum(EEPROM_STATE_OFFSET, EEPROM_STATE_LENGTH);
	}
}

void EEPROM::setCurrentSteps(int32_t* steps) {
	int32_t currentSteps[A_AXIS_ARRAY];
	getCurrentSteps(currentSteps);

	uint8_t doChecksum = false;
	for (uint8_t i = 0; i < A_AXIS_ARRAY; i++) {
		if (steps[i] != currentSteps[i]) {
			HAL::eprSetInt32(EPR_STATE_X_STEPS + (i * 4), steps[i]);
			doChecksum = true;
		}
	}

	if (doChecksum) {
		updateChecksum(EEPROM_STATE_OFFSET, EEPROM_STATE_LENGTH);
	}
}

void EEPROM::setCurrentOffset(float* offset) {
	float currentOffset[Z_AXIS_ARRAY];
	getCurrentOffset(currentOffset);

	uint8_t doChecksum = false;
	for (uint8_t i = 0; i < Z_AXIS_ARRAY; i++) {
		if (offset[i] != currentOffset[i]) {
			HAL::eprSetFloat(EPR_STATE_X_OFFSET + (i * 4), offset[i]);
			doChecksum = true;
		}
	}

	if (doChecksum) {
		updateChecksum(EEPROM_STATE_OFFSET, EEPROM_STATE_LENGTH);
	}
}

void EEPROM::setCurrentFlags(uint16_t flags) {
	if (flags != getCurrentFlags()) {
		HAL::eprSetInt16(EPR_STATE_FLAGS, flags);

		updateChecksum(EEPROM_STATE_OFFSET, EEPROM_STATE_LENGTH);
	}
}

uint8_t EEPROM::isCurrentStateChecksumValid() {
	return computeChecksum(EEPROM_STATE_OFFSET, EEPROM_STATE_LENGTH) == HAL::eprGetByte(EEPROM_STATE_OFFSET);
}

uint8_t EEPROM::getCurrentValidState() {
	return HAL::eprGetByte(EPR_STATE_VALID);
}

void EEPROM::getCurrentSteps(int32_t* steps) {
	for (uint8_t i = 0; i < A_AXIS_ARRAY; i++) {
		steps[i] = HAL::eprGetInt32(EPR_STATE_X_STEPS + (i * 4));
	}
}

void EEPROM::getCurrentOffset(float* offset) {
	for (uint8_t i = 0; i < Z_AXIS_ARRAY; i++) {
		offset[i] = HAL::eprGetFloat(EPR_STATE_X_OFFSET + (i * 4));
	}
}

uint16_t EEPROM::getCurrentFlags() {
	return HAL::eprGetInt16(EPR_STATE_FLAGS);
}

#endif

#if EEPROM_MODE != 0

uint8_t EEPROM::computeChecksum(uint16_t offset, uint16_t length) {
    uint8_t checksum = 0;
    for(uint16_t i = offset + 1; i < offset + length; i++) {
        checksum += HAL::eprGetByte(i);
    }
    return checksum;
}

void EEPROM::updateChecksum(uint16_t offset, uint16_t length) {
	uint8_t newcheck = computeChecksum(offset, length);
	if (newcheck != HAL::eprGetByte(offset)) {
		HAL::eprSetByte(offset, newcheck);
	}
}

void EEPROM::writeFloat(uint pos,PGM_P text,uint8_t digits) {
    Com::printF(Com::tEPR3, static_cast<int>(pos));
    Com::print(' ');
    Com::printFloat(HAL::eprGetFloat(pos),digits);
	Com::print(' ');
    Com::printFLN(text);
	HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

void EEPROM::writeLong(uint pos,PGM_P text) {
    Com::printF(Com::tEPR2, static_cast<int>(pos));
    Com::print(' ');
    Com::print(HAL::eprGetInt32(pos));
    Com::print(' ');
	Com::printFLN(text);
	HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

void EEPROM::writeInt(uint pos,PGM_P text) {
    Com::printF(Com::tEPR1, static_cast<int>(pos));
    Com::print(' ');
    Com::print(HAL::eprGetInt16(pos));
	Com::print(' ');
	Com::printFLN(text);
	HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

void EEPROM::writeByte(uint pos,PGM_P text) {
	Com::printF(Com::tEPR0, static_cast<int>(pos));
	Com::print(' ');
	Com::print((int)HAL::eprGetByte(pos));
	Com::print(' ');
	Com::printFLN(text);
	HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

#endif