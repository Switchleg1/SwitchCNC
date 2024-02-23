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
    baudrate = BAUDRATE;
    maxInactiveTime = MAX_INACTIVE_TIME * 1000L;
    stepperInactiveTime = STEPPER_INACTIVE_TIME * 1000L;
    Printer::axisStepsPerMM[X_AXIS] = XAXIS_STEPS_PER_MM;
	Printer::axisStepsPerMM[Y_AXIS] = YAXIS_STEPS_PER_MM;
	Printer::axisStepsPerMM[Z_AXIS] = ZAXIS_STEPS_PER_MM;
	Printer::axisStepsPerMM[A_AXIS] = AAXIS_STEPS_PER_MM;
    Printer::maxFeedrate[X_AXIS] = MAX_FEEDRATE_X;
    Printer::maxFeedrate[Y_AXIS] = MAX_FEEDRATE_Y;
	Printer::maxFeedrate[Z_AXIS] = MAX_FEEDRATE_Z;
	Printer::maxFeedrate[A_AXIS] = MAX_FEEDRATE_A;
	Printer::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X;
    Printer::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y;
    Printer::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z;
	Printer::maxJerk[X_AXIS] = MAX_XJERK;
	Printer::maxJerk[Y_AXIS] = MAX_YJERK;
	Printer::maxJerk[Z_AXIS] = MAX_ZJERK;
	Printer::maxJerk[A_AXIS] = MAX_AJERK;
#if RAMP_ACCELERATION
	Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X;
    Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y;
	Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z;
	Printer::maxAccelerationMMPerSquareSecond[A_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_A;
#endif
	Printer::axisLength[X_AXIS] = X_MAX_LENGTH;
	Printer::axisLength[Y_AXIS] = Y_MAX_LENGTH;
	Printer::axisLength[Z_AXIS] = Z_MAX_LENGTH;
	Printer::axisMin[X_AXIS] = X_MIN_POS;
	Printer::axisMin[Y_AXIS] = Y_MIN_POS;
	Printer::axisMin[Z_AXIS] = Z_MIN_POS;
#if ENABLE_BACKLASH_COMPENSATION
	Printer::backlash[X_AXIS] = X_BACKLASH;
	Printer::backlash[Y_AXIS] = Y_BACKLASH;
	Printer::backlash[Z_AXIS] = Z_BACKLASH;
	Printer::backlash[A_AXIS] = A_BACKLASH;
#endif
#if DISTORTION_CORRECTION
	Printer::distortionXMIN	= DISTORTION_XMIN; //SL
	Printer::distortionXMAX = DISTORTION_XMAX; //SL
	Printer::distortionYMIN = DISTORTION_YMIN; //SL
	Printer::distortionYMAX = DISTORTION_YMAX; //SL
	Printer::distortionPoints = DISTORTION_CORRECTION_POINTS; //SL
	Printer::distortionStart = DISTORTION_START; //SL
	Printer::distortionEnd = DISTORTION_END; //SL
	Printer::distortionUseOffset = DISTORTION_USE_OFFSET; //SL
#endif
    initalizeUncached();
	Printer::updateDerivedParameter();
	Com::printInfoFLN(Com::tEPRConfigResetDefaults);
#else
    Com::printErrorFLN(Com::tNoEEPROMSupport);
#endif
}

void EEPROM::storeDataIntoEEPROM(uint8_t corrupted)
{
#if EEPROM_MODE != 0
    HAL::eprSetInt32(EPR_BAUDRATE,baudrate);
    HAL::eprSetInt32(EPR_MAX_INACTIVE_TIME,maxInactiveTime);
    HAL::eprSetInt32(EPR_STEPPER_INACTIVE_TIME,stepperInactiveTime);
	HAL::eprSetFloat(EPR_XAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[X_AXIS]);
    HAL::eprSetFloat(EPR_YAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[Y_AXIS]);
	HAL::eprSetFloat(EPR_ZAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[Z_AXIS]);
	HAL::eprSetFloat(EPR_AAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[A_AXIS]);
	HAL::eprSetFloat(EPR_X_MAX_FEEDRATE,Printer::maxFeedrate[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_MAX_FEEDRATE,Printer::maxFeedrate[Y_AXIS]);
	HAL::eprSetFloat(EPR_Z_MAX_FEEDRATE,Printer::maxFeedrate[Z_AXIS]);
	HAL::eprSetFloat(EPR_A_MAX_FEEDRATE,Printer::maxFeedrate[A_AXIS]);
	HAL::eprSetFloat(EPR_X_HOMING_FEEDRATE,Printer::homingFeedrate[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_HOMING_FEEDRATE,Printer::homingFeedrate[Y_AXIS]);
	HAL::eprSetFloat(EPR_Z_HOMING_FEEDRATE,Printer::homingFeedrate[Z_AXIS]);
	HAL::eprSetFloat(EPR_MAX_XJERK,Printer::maxJerk[X_AXIS]);
	HAL::eprSetFloat(EPR_MAX_YJERK,Printer::maxJerk[Y_AXIS]);
	HAL::eprSetFloat(EPR_MAX_ZJERK,Printer::maxJerk[Z_AXIS]);
	HAL::eprSetFloat(EPR_MAX_AJERK,Printer::maxJerk[A_AXIS]);
#if RAMP_ACCELERATION
	HAL::eprSetFloat(EPR_X_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[Y_AXIS]);
	HAL::eprSetFloat(EPR_Z_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[Z_AXIS]);
	HAL::eprSetFloat(EPR_A_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[A_AXIS]);
#endif
	HAL::eprSetFloat(EPR_X_HOME_OFFSET,Printer::axisMin[X_AXIS]);
	HAL::eprSetFloat(EPR_Y_HOME_OFFSET,Printer::axisMin[Y_AXIS]);
	HAL::eprSetFloat(EPR_Z_HOME_OFFSET,Printer::axisMin[Z_AXIS]);
	HAL::eprSetFloat(EPR_X_LENGTH,Printer::axisLength[X_AXIS]);
	HAL::eprSetFloat(EPR_Y_LENGTH,Printer::axisLength[Y_AXIS]);
	HAL::eprSetFloat(EPR_Z_LENGTH,Printer::axisLength[Z_AXIS]);
#if ENABLE_BACKLASH_COMPENSATION
	HAL::eprSetFloat(EPR_BACKLASH_X,Printer::backlash[X_AXIS]);
	HAL::eprSetFloat(EPR_BACKLASH_Y,Printer::backlash[Y_AXIS]);
	HAL::eprSetFloat(EPR_BACKLASH_Z,Printer::backlash[Z_AXIS]);
	HAL::eprSetFloat(EPR_BACKLASH_Z,Printer::backlash[A_AXIS]);
#endif
#if DISTORTION_CORRECTION
	HAL::eprSetInt16(EPR_DISTORTION_XMIN, Printer::distortionXMIN); //SL
	HAL::eprSetInt16(EPR_DISTORTION_XMAX, Printer::distortionXMAX); //SL
	HAL::eprSetInt16(EPR_DISTORTION_YMIN, Printer::distortionYMIN); //SL
	HAL::eprSetInt16(EPR_DISTORTION_YMAX, Printer::distortionYMAX); //SL
	HAL::eprSetByte(EPR_DISTORTION_POINTS, Printer::distortionPoints); //SL
	HAL::eprSetFloat(EPR_DISTORTION_START, Printer::distortionStart); //SL
	HAL::eprSetFloat(EPR_DISTORTION_END, Printer::distortionEnd); //SL
	HAL::eprSetByte(EPR_DISTORTION_USE_OFFSET, Printer::distortionUseOffset); //SL
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
    baudrate = HAL::eprGetInt32(EPR_BAUDRATE);
    maxInactiveTime = HAL::eprGetInt32(EPR_MAX_INACTIVE_TIME);
    stepperInactiveTime = HAL::eprGetInt32(EPR_STEPPER_INACTIVE_TIME);
	Printer::axisStepsPerMM[X_AXIS] = HAL::eprGetFloat(EPR_XAXIS_STEPS_PER_MM);
    Printer::axisStepsPerMM[Y_AXIS] = HAL::eprGetFloat(EPR_YAXIS_STEPS_PER_MM);
	Printer::axisStepsPerMM[Z_AXIS] = HAL::eprGetFloat(EPR_ZAXIS_STEPS_PER_MM);
	Printer::axisStepsPerMM[A_AXIS] = HAL::eprGetFloat(EPR_AAXIS_STEPS_PER_MM);
	Printer::maxFeedrate[X_AXIS] = HAL::eprGetFloat(EPR_X_MAX_FEEDRATE);
	Printer::maxFeedrate[Y_AXIS] = HAL::eprGetFloat(EPR_Y_MAX_FEEDRATE);
	Printer::maxFeedrate[Z_AXIS] = HAL::eprGetFloat(EPR_Z_MAX_FEEDRATE);
	Printer::maxFeedrate[A_AXIS] = HAL::eprGetFloat(EPR_A_MAX_FEEDRATE);
	Printer::homingFeedrate[X_AXIS] = HAL::eprGetFloat(EPR_X_HOMING_FEEDRATE);
	Printer::homingFeedrate[Y_AXIS] = HAL::eprGetFloat(EPR_Y_HOMING_FEEDRATE);
    Printer::homingFeedrate[Z_AXIS] = HAL::eprGetFloat(EPR_Z_HOMING_FEEDRATE);
	Printer::maxJerk[X_AXIS] = HAL::eprGetFloat(EPR_MAX_XJERK);
	Printer::maxJerk[Y_AXIS] = HAL::eprGetFloat(EPR_MAX_YJERK);
	Printer::maxJerk[Z_AXIS] = HAL::eprGetFloat(EPR_MAX_ZJERK);
	Printer::maxJerk[A_AXIS] = HAL::eprGetFloat(EPR_MAX_AJERK);
#if RAMP_ACCELERATION
	Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = HAL::eprGetFloat(EPR_X_MAX_ACCEL);
	Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = HAL::eprGetFloat(EPR_Y_MAX_ACCEL);
	Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = HAL::eprGetFloat(EPR_Z_MAX_ACCEL);
	Printer::maxAccelerationMMPerSquareSecond[A_AXIS] = HAL::eprGetFloat(EPR_A_MAX_ACCEL);
#endif
	Printer::axisMin[X_AXIS] = HAL::eprGetFloat(EPR_X_HOME_OFFSET);
	Printer::axisMin[Y_AXIS] = HAL::eprGetFloat(EPR_Y_HOME_OFFSET);
	Printer::axisMin[Z_AXIS] = HAL::eprGetFloat(EPR_Z_HOME_OFFSET);
	Printer::axisLength[X_AXIS] = HAL::eprGetFloat(EPR_X_LENGTH);
	Printer::axisLength[Y_AXIS] = HAL::eprGetFloat(EPR_Y_LENGTH);
	Printer::axisLength[Z_AXIS] = HAL::eprGetFloat(EPR_Z_LENGTH);
#if ENABLE_BACKLASH_COMPENSATION
	Printer::backlash[X_AXIS] = HAL::eprGetFloat(EPR_BACKLASH_X);
	Printer::backlash[Y_AXIS] = HAL::eprGetFloat(EPR_BACKLASH_Y);
	Printer::backlash[Z_AXIS] = HAL::eprGetFloat(EPR_BACKLASH_Z);
	Printer::backlash[A_AXIS] = HAL::eprGetFloat(EPR_BACKLASH_A);
#endif
#if DISTORTION_CORRECTION
	Printer::distortionXMIN = HAL::eprGetInt16(EPR_DISTORTION_XMIN); //SL
	Printer::distortionXMAX = HAL::eprGetInt16(EPR_DISTORTION_XMAX); //SL
	Printer::distortionYMIN = HAL::eprGetInt16(EPR_DISTORTION_YMIN); //SL
	Printer::distortionYMAX = HAL::eprGetInt16(EPR_DISTORTION_YMAX); //SL
	Printer::distortionPoints = HAL::eprGetByte(EPR_DISTORTION_POINTS); //SL
	if(Printer::distortionPoints > DISTORTION_CORRECTION_POINTS)
		Printer::distortionPoints = DISTORTION_CORRECTION_POINTS;
	if(Printer::distortionPoints < 2)
		Printer::distortionPoints = 2;
	Printer::distortionStart = HAL::eprGetFloat(EPR_DISTORTION_START); //SL
	Printer::distortionEnd = HAL::eprGetFloat(EPR_DISTORTION_END); //SL
	Printer::distortionUseOffset = HAL::eprGetByte(EPR_DISTORTION_USE_OFFSET); //SL
	Printer::distortion.SetStartEnd(Printer::distortionStart, Printer::distortionEnd);
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
	Printer::updateDerivedParameter();
#endif
}

void EEPROM::initBaudrate()
{
    // Invariant - baudrate is initialized with or without eeprom!
    baudrate = BAUDRATE;
#if EEPROM_MODE != 0
    if(HAL::eprGetByte(EPR_MAGIC_BYTE) == EEPROM_MODE)
    {
        baudrate = HAL::eprGetInt32(EPR_BAUDRATE);
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
    if(HAL::eprGetByte(EPR_MAGIC_BYTE) == EEPROM_MODE && storedcheck == check)
    {
        readDataFromEEPROM();
        if (USE_CONFIGURATION_BAUD_RATE)
        {
            // Used if eeprom gets unusable baud rate set and communication wont work at all.
            if(HAL::eprGetInt32(EPR_BAUDRATE) != BAUDRATE)
            {
                HAL::eprSetInt32(EPR_BAUDRATE,BAUDRATE);
                baudrate = BAUDRATE;
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
