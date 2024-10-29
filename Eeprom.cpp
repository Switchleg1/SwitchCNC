#include "SwitchCNC.h"

void EEPROM::update(GCode *com)
{
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
    uint8_t newcheck = computeChecksum();
    if(newcheck != HAL::eprGetByte(EPR_INTEGRITY_BYTE))
        HAL::eprSetByte(EPR_INTEGRITY_BYTE, newcheck);
	readDataFromEEPROM();
#else
    Com::printErrorF(Com::tNoEEPROMSupport);
#endif
}

void EEPROM::restoreEEPROMSettingsFromConfiguration()
{
	// can only be done right if we also update permanent values not cached!
#if EEPROM_MODE != 0
	EEPROM::initalizeUncached();
    uint8_t newcheck = computeChecksum();
    if(newcheck != HAL::eprGetByte(EPR_INTEGRITY_BYTE))
		HAL::eprSetByte(EPR_INTEGRITY_BYTE, newcheck);	
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
#if ENABLE_BACKLASH_COMPENSATION
	Machine::backlash[X_AXIS] = X_BACKLASH;
	Machine::backlash[Y_AXIS] = Y_BACKLASH;
	Machine::backlash[Z_AXIS] = Z_BACKLASH;
	Machine::backlash[A_AXIS] = A_BACKLASH;
#endif
#if DISTORTION_CORRECTION
	Distortion::XMIN		= DISTORTION_XMIN;					//SL
	Distortion::XMAX		= DISTORTION_XMAX;					//SL
	Distortion::YMIN		= DISTORTION_YMIN;					//SL
	Distortion::YMAX		= DISTORTION_YMAX;					//SL
	Distortion::setPoints(DISTORTION_CORRECTION_POINTS);		//SL
	Distortion::start		= DISTORTION_START;					//SL
	Distortion::end			= DISTORTION_END;					//SL
	Distortion::useOffset	= DISTORTION_USE_OFFSET;			//SL
#endif
    initalizeUncached();
	Machine::updateDerivedParameter();
	Com::printInfoFLN(Com::tEPRConfigResetDefaults);
#else
    Com::printErrorFLN(Com::tNoEEPROMSupport);
#endif
}

void EEPROM::storeDataIntoEEPROM(uint8_t corrupted)
{
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
#if ENABLE_BACKLASH_COMPENSATION
	HAL::eprSetFloat(EPR_BACKLASH_X,Machine::backlash[X_AXIS]);
	HAL::eprSetFloat(EPR_BACKLASH_Y,Machine::backlash[Y_AXIS]);
	HAL::eprSetFloat(EPR_BACKLASH_Z,Machine::backlash[Z_AXIS]);
	HAL::eprSetFloat(EPR_BACKLASH_Z,Machine::backlash[A_AXIS]);
#endif
#if DISTORTION_CORRECTION
	HAL::eprSetInt16(EPR_DISTORTION_XMIN, Distortion::XMIN); //SL
	HAL::eprSetInt16(EPR_DISTORTION_XMAX, Distortion::XMAX); //SL
	HAL::eprSetInt16(EPR_DISTORTION_YMIN, Distortion::YMIN); //SL
	HAL::eprSetInt16(EPR_DISTORTION_YMAX, Distortion::YMAX); //SL
	HAL::eprSetByte(EPR_DISTORTION_POINTS, Distortion::getPoints()); //SL
	HAL::eprSetFloat(EPR_DISTORTION_START, Distortion::start); //SL
	HAL::eprSetFloat(EPR_DISTORTION_END, Distortion::end); //SL
	HAL::eprSetByte(EPR_DISTORTION_USE_OFFSET, Distortion::useOffset); //SL
#endif
    if(corrupted)
    {
		initalizeUncached();
    }
    // Save version and build checksum
    HAL::eprSetByte(EPR_VERSION,EEPROM_PROTOCOL_VERSION);
    HAL::eprSetByte(EPR_INTEGRITY_BYTE,computeChecksum());
#endif
}

void EEPROM::initalizeUncached()
{
    HAL::eprSetFloat(EPR_Z_PROBE_HEIGHT,Z_PROBE_HEIGHT);
    HAL::eprSetFloat(EPR_Z_PROBE_SPEED,Z_PROBE_SPEED);
	HAL::eprSetFloat(EPR_Z_PROBE_XY_SPEED,Z_PROBE_XY_SPEED);
	HAL::eprSetByte(EPR_DISTORTION_CORRECTION_ENABLED,0);
}

void EEPROM::readDataFromEEPROM()
{
#if EEPROM_MODE != 0
    uint8_t version = HAL::eprGetByte(EPR_VERSION); // This is the saved version. Don't copy data nor set it to older versions!
    Com::printFLN(PSTR("Detected EEPROM version:"),(int)version);
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
#if ENABLE_BACKLASH_COMPENSATION
	Machine::backlash[X_AXIS] = HAL::eprGetFloat(EPR_BACKLASH_X);
	Machine::backlash[Y_AXIS] = HAL::eprGetFloat(EPR_BACKLASH_Y);
	Machine::backlash[Z_AXIS] = HAL::eprGetFloat(EPR_BACKLASH_Z);
	Machine::backlash[A_AXIS] = HAL::eprGetFloat(EPR_BACKLASH_A);
#endif
#if DISTORTION_CORRECTION
	Distortion::XMIN		= HAL::eprGetInt16(EPR_DISTORTION_XMIN); //SL
	Distortion::XMAX		= HAL::eprGetInt16(EPR_DISTORTION_XMAX); //SL
	Distortion::YMIN		= HAL::eprGetInt16(EPR_DISTORTION_YMIN); //SL
	Distortion::YMAX		= HAL::eprGetInt16(EPR_DISTORTION_YMAX); //SL
	Distortion::setPoints(HAL::eprGetByte(EPR_DISTORTION_POINTS)); //SL
	Distortion::start		= HAL::eprGetFloat(EPR_DISTORTION_START); //SL
	Distortion::end			= HAL::eprGetFloat(EPR_DISTORTION_END); //SL
	Distortion::useOffset	= HAL::eprGetByte(EPR_DISTORTION_USE_OFFSET); //SL
	Distortion::SetStartEnd(Distortion::start, Distortion::end);
#endif
	if(version != EEPROM_PROTOCOL_VERSION)
    {
        Com::printInfoFLN(Com::tEPRProtocolChanged);
		if(version < 1)
		{
			HAL::eprSetFloat(EPR_Z_PROBE_HEIGHT,Z_PROBE_HEIGHT);
			HAL::eprSetFloat(EPR_Z_PROBE_SPEED,Z_PROBE_SPEED);
			HAL::eprSetFloat(EPR_Z_PROBE_XY_SPEED,Z_PROBE_XY_SPEED);
			HAL::eprSetByte(EPR_DISTORTION_CORRECTION_ENABLED, 0);
		}

        storeDataIntoEEPROM(false); // Store new fields for changed version
	}
	Machine::updateDerivedParameter();
#endif
}

void EEPROM::initBaudrate()
{
    // Invariant - baudrate is initialized with or without eeprom!
	Machine::baudrate = BAUDRATE;
#if EEPROM_MODE != 0
    if(HAL::eprGetByte(EPR_MAGIC_BYTE) == EEPROM_MODE) {
		Machine::baudrate = HAL::eprGetInt32(EPR_BAUDRATE);
    }
#endif
}

#ifndef USE_CONFIGURATION_BAUD_RATE
#define USE_CONFIGURATION_BAUD_RATE 0
#endif // USE_CONFIGURATION_BAUD_RATE
void EEPROM::init()
{
#if EEPROM_MODE != 0
    uint8_t check = computeChecksum();
    uint8_t storedcheck = HAL::eprGetByte(EPR_INTEGRITY_BYTE);
    if(HAL::eprGetByte(EPR_MAGIC_BYTE) == EEPROM_MODE && storedcheck == check) {
        readDataFromEEPROM();
        if (USE_CONFIGURATION_BAUD_RATE) {
            // Used if eeprom gets unusable baud rate set and communication wont work at all.
            if(HAL::eprGetInt32(EPR_BAUDRATE) != BAUDRATE) {
                HAL::eprSetInt32(EPR_BAUDRATE,BAUDRATE);
				Machine::baudrate = BAUDRATE;
                uint8_t newcheck = computeChecksum();
                if(newcheck != HAL::eprGetByte(EPR_INTEGRITY_BYTE))
                    HAL::eprSetByte(EPR_INTEGRITY_BYTE,newcheck);
            }
            Com::printFLN(PSTR("EEPROM baud rate restored from configuration."));
            Com::printFLN(PSTR("RECOMPILE WITH USE_CONFIGURATION_BAUD_RATE == 0 to alter baud rate via EEPROM"));
        }
    }
    else
    {
        HAL::eprSetByte(EPR_MAGIC_BYTE,EEPROM_MODE); // Make data change permanent
        initalizeUncached();
        storeDataIntoEEPROM(storedcheck != check);
    }
#endif
}

void EEPROM::updatePrinterUsage()
{
#if EEPROM_MODE != 0
	
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
void EEPROM::writeSettings()
{
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
#if ENABLE_BACKLASH_COMPENSATION
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
#if FEATURE_Z_PROBE
    writeFloat(EPR_Z_PROBE_HEIGHT, Com::tZProbeHeight);
	writeFloat(EPR_Z_PROBE_SPEED, Com::tZProbeSpeed);
	writeFloat(EPR_Z_PROBE_XY_SPEED, Com::tZProbeSpeedXY);
#endif
#if DISTORTION_CORRECTION
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

#if EEPROM_MODE != 0

uint8_t EEPROM::computeChecksum()
{
    unsigned int i;
    uint8_t checksum = 0;
    for(i = 0; i < 2048; i++)
    {
        if(i == EEPROM_OFFSET + EPR_INTEGRITY_BYTE) continue;
        checksum += HAL::eprGetByte(i);
    }
    return checksum;
}

void EEPROM::updateChecksum()
{
    uint8_t newcheck = computeChecksum();
    if(newcheck!=HAL::eprGetByte(EPR_INTEGRITY_BYTE))
        HAL::eprSetByte(EPR_INTEGRITY_BYTE,newcheck);
}


void EEPROM::writeFloat(uint pos,PGM_P text,uint8_t digits)
{
    Com::printF(Com::tEPR3, static_cast<int>(pos));
    Com::print(' ');
    Com::printFloat(HAL::eprGetFloat(pos),digits);
	Com::print(' ');
    Com::printFLN(text);
	HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

void EEPROM::writeLong(uint pos,PGM_P text)
{
    Com::printF(Com::tEPR2, static_cast<int>(pos));
    Com::print(' ');
    Com::print(HAL::eprGetInt32(pos));
    Com::print(' ');
	Com::printFLN(text);
	HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

void EEPROM::writeInt(uint pos,PGM_P text)
{
    Com::printF(Com::tEPR1, static_cast<int>(pos));
    Com::print(' ');
    Com::print(HAL::eprGetInt16(pos));
	Com::print(' ');
	Com::printFLN(text);
	HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

void EEPROM::writeByte(uint pos,PGM_P text)
{
	Com::printF(Com::tEPR0, static_cast<int>(pos));
	Com::print(' ');
	Com::print((int)HAL::eprGetByte(pos));
	Com::print(' ');
	Com::printFLN(text);
	HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

void EEPROM::setZCorrection(int32_t c,int index)
{
    HAL::eprSetInt32(2048 + (index << 2), c);
}

#endif
