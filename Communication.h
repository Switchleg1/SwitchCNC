#ifndef COMMUNICATION_H
#define COMMUNICATION_H

class Com {
public:
    FSTRINGVAR(tDebug)
    FSTRINGVAR(tFirmware)
    FSTRINGVAR(tOk)
    FSTRINGVAR(tNewline)
    FSTRINGVAR(tNAN)
    FSTRINGVAR(tINF)
    FSTRINGVAR(tError)
    FSTRINGVAR(tInfo)
    FSTRINGVAR(tWarning)
    FSTRINGVAR(tResend)
    FSTRINGVAR(tEcho)
    FSTRINGVAR(tCap)
    FSTRINGVAR(tCapEeprom)
    FSTRINGVAR(tCapZProbe)
    FSTRINGVAR(tCapSoftwarePower)
    FSTRINGVAR(tCapToggleLights)
    FSTRINGVAR(tCapCoolantMist)
    FSTRINGVAR(tCapCoolantFlood)
    FSTRINGVAR(tCapDistortionCorrection)
    FSTRINGVAR(tCapFanControl)
    FSTRINGVAR(tCapFan)
    FSTRINGVAR(tCapFan2)
    FSTRINGVAR(tCapVacuum)
    FSTRINGVAR(tCapSDCard)
    FSTRINGVAR(tCfgBaudrate)
    FSTRINGVAR(tCfgInputBuffer)
    FSTRINGVAR(tCfgXHomeDir)
    FSTRINGVAR(tCfgYHomeDir)
    FSTRINGVAR(tCfgZHomeDir)
    FSTRINGVAR(tCfgXHomePos)
    FSTRINGVAR(tCfgYHomePos)
    FSTRINGVAR(tCfgZHomePos)
    FSTRINGVAR(tCfgMachinelineCache)
    FSTRINGVAR(tCfgKeepAliveInterval)
    FSTRINGVAR(tCfgJerkX)
    FSTRINGVAR(tCfgJerkY)
    FSTRINGVAR(tCfgJerkZ)
    FSTRINGVAR(tCfgJerkA)
    FSTRINGVAR(tCfgXMin)
    FSTRINGVAR(tCfgYMin)
    FSTRINGVAR(tCfgZMin)
    FSTRINGVAR(tCfgXMax)
    FSTRINGVAR(tCfgYMax)
    FSTRINGVAR(tCfgZMax)
    FSTRINGVAR(tCfgXSize)
    FSTRINGVAR(tCfgYSize)
    FSTRINGVAR(tCfgZSize)
    FSTRINGVAR(tCfgXAccel)
    FSTRINGVAR(tCfgYAccel)
    FSTRINGVAR(tCfgZAccel)
    FSTRINGVAR(tCfgAAccel)
    FSTRINGVAR(tOkSpace)
    FSTRINGVAR(tWrongChecksum)
    FSTRINGVAR(tFormatError)
    FSTRINGVAR(tDoneMilling)
    FSTRINGVAR(tX)
    FSTRINGVAR(tY)
    FSTRINGVAR(tZ)
    FSTRINGVAR(tE)
    FSTRINGVAR(tF)
    FSTRINGVAR(tS)
    FSTRINGVAR(tP)
    FSTRINGVAR(tI)
    FSTRINGVAR(tJ)
    FSTRINGVAR(tR)
    FSTRINGVAR(tD)
    FSTRINGVAR(tC)
    FSTRINGVAR(tH)
    FSTRINGVAR(tA)
    FSTRINGVAR(tB)
    FSTRINGVAR(tK)
    FSTRINGVAR(tL)
    FSTRINGVAR(tO)
    FSTRINGVAR(tSDReadError)
    FSTRINGVAR(tExpectedLine)
    FSTRINGVAR(tGot)
    FSTRINGVAR(tSkip)
    FSTRINGVAR(tBLK)
    FSTRINGVAR(tStart)
    FSTRINGVAR(tPowerUp)
    FSTRINGVAR(tExternalReset)
    FSTRINGVAR(tBrownOut)
    FSTRINGVAR(tWatchdog)
    FSTRINGVAR(tSoftwareReset)
    FSTRINGVAR(tUnknownCommand)
    FSTRINGVAR(tFreeRAM)
    FSTRINGVAR(tXColon)
    FSTRINGVAR(tSlash)
    FSTRINGVAR(tSpaceSlash)
    FSTRINGVAR(tFatal)
    FSTRINGVAR(tSpaceXColon)
    FSTRINGVAR(tSpaceYColon)
    FSTRINGVAR(tSpaceZColon)
    FSTRINGVAR(tSpaceAColon)
    FSTRINGVAR(tTColon)
    FSTRINGVAR(tSpaceBColon)
    FSTRINGVAR(tSpaceAtColon)
    FSTRINGVAR(tSpaceT)
    FSTRINGVAR(tSpaceRaw)
    FSTRINGVAR(tSpaceAt)
    FSTRINGVAR(tSpaceBAtColon)
    FSTRINGVAR(tColon)
    FSTRINGVAR(tSpeedMultiply)
    FSTRINGVAR(tIntensityMultiply)
    FSTRINGVAR(tFanspeed)
    FSTRINGVAR(tFan2speed)
    FSTRINGVAR(tVacuumState)
    FSTRINGVAR(tInvalidArc)
    FSTRINGVAR(tComma)
    FSTRINGVAR(tSpace)
    FSTRINGVAR(tYColon)
    FSTRINGVAR(tZColon)
    FSTRINGVAR(tMS1MS2Pins)
    FSTRINGVAR(tSetOutputSpace)
    FSTRINGVAR(tGetInputSpace)
    FSTRINGVAR(tSpaceToSpace)
    FSTRINGVAR(tSpaceIsSpace)
    FSTRINGVAR(tHSpace)
    FSTRINGVAR(tLSpace)
    FSTRINGVAR(tXMinColon)
    FSTRINGVAR(tXMaxColon)
    FSTRINGVAR(tYMinColon)
    FSTRINGVAR(tYMaxColon)
    FSTRINGVAR(tZMinColon)
    FSTRINGVAR(tZ2MinMaxColon)
    FSTRINGVAR(tZMaxColon)
    FSTRINGVAR(tXJerkColon)
    FSTRINGVAR(tYJerkColon)
    FSTRINGVAR(tZJerkColon)
    FSTRINGVAR(tAJerkColon)
    FSTRINGVAR(tEEPROMUpdated)
    FSTRINGVAR(tMachineHeight)
#ifdef DEBUG_QUEUE_MOVE
    FSTRINGVAR(tDBGId)
    FSTRINGVAR(tDBGVStartEnd)
    FSTRINGVAR(tDBAccelSteps)
    FSTRINGVAR(tDBGStartEndSpeed)
    FSTRINGVAR(tDBGFlags)
    FSTRINGVAR(tDBGJoinFlags)
    FSTRINGVAR(tDBGDelta)
    FSTRINGVAR(tDBGDir)
    FSTRINGVAR(tDBGFullSpeed)
    FSTRINGVAR(tDBGVMax)
    FSTRINGVAR(tDBGAcceleration)
    FSTRINGVAR(tDBGAccelerationPrim)
    FSTRINGVAR(tDBGRemainingSteps)
    FSTRINGVAR(tDBGLimitInterval)
    FSTRINGVAR(tDBGMoveDistance)
    FSTRINGVAR(tDBGCommandedFeedrate)
    FSTRINGVAR(tDBGConstFullSpeedMoveTime)
#endif // DEBUG_QUEUE_MOVEFSTRINGVALUE(Com::,"")
#ifdef DEBUG_STEPCOUNT
    FSTRINGVAR(tDBGMissedSteps)
#endif
#if FEATURE_Z_PROBE
    FSTRINGVAR(tZProbe)
    FSTRINGVAR(tZProbeState)
    FSTRINGVAR(tZProbeZReset)
    FSTRINGVAR(tZProbeFailed)
#endif
#if DISTORTION_CORRECTION
    FSTRINGVAR(tDistortionXMIN) //SL
    FSTRINGVAR(tDistortionXMAX) //SL
    FSTRINGVAR(tDistortionYMIN) //SL
    FSTRINGVAR(tDistortionYMAX) //SL
    FSTRINGVAR(tDistortionPoints) //SL
    FSTRINGVAR(tDistortionStart) //SL
    FSTRINGVAR(tDistortionEnd) //SL
    FSTRINGVAR(tDistortionUseOffset) //SL
#endif
#if defined(PAUSE_PIN) && PAUSE_PIN>-1
    FSTRINGVAR(tPaused)
    FSTRINGVAR(tUnpaused)
#endif
#ifdef WAITING_IDENTIFIER
    FSTRINGVAR(tWait)
#endif // WAITING_IDENTIFIER

#if EEPROM_MODE==0
    FSTRINGVAR(tNoEEPROMSupport)
#else
#if FEATURE_Z_PROBE
    FSTRINGVAR(tZProbeHeight)
    FSTRINGVAR(tZProbeSpeed)
    FSTRINGVAR(tZProbeSpeedXY)
#endif
    FSTRINGVAR(tConfigStoredEEPROM)
    FSTRINGVAR(tConfigLoadedEEPROM)
    FSTRINGVAR(tEPRConfigResetDefaults)
    FSTRINGVAR(tEPRProtocolChanged)
    FSTRINGVAR(tEPR0)
    FSTRINGVAR(tEPR1)
    FSTRINGVAR(tEPR2)
    FSTRINGVAR(tEPR3)
    FSTRINGVAR(tEPRBaudrate)
    FSTRINGVAR(tEPRMaxInactiveTime)
    FSTRINGVAR(tEPRStopAfterInactivty)
    FSTRINGVAR(tEPRXHomePos)
    FSTRINGVAR(tEPRYHomePos)
    FSTRINGVAR(tEPRZHomePos)
    FSTRINGVAR(tEPRXMaxLength)
    FSTRINGVAR(tEPRYMaxLength)
    FSTRINGVAR(tEPRZMaxLength)
    FSTRINGVAR(tEPRXBacklash)
    FSTRINGVAR(tEPRYBacklash)
    FSTRINGVAR(tEPRZBacklash)
    FSTRINGVAR(tEPRABacklash)
    FSTRINGVAR(tEPRAccelerationFactorAtTop)
    FSTRINGVAR(tEPRMaxXJerk)
    FSTRINGVAR(tEPRMaxYJerk)
    FSTRINGVAR(tEPRMaxZJerk)
    FSTRINGVAR(tEPRMaxAJerk)
    FSTRINGVAR(tEPRXStepsPerMM)
    FSTRINGVAR(tEPRYStepsPerMM)
    FSTRINGVAR(tEPRZStepsPerMM)
    FSTRINGVAR(tEPRAStepsPerMM)
    FSTRINGVAR(tEPRXMaxFeedrate)
    FSTRINGVAR(tEPRYMaxFeedrate)
    FSTRINGVAR(tEPRZMaxFeedrate)
    FSTRINGVAR(tEPRAMaxFeedrate)
    FSTRINGVAR(tEPRXHomingFeedrate)
    FSTRINGVAR(tEPRYHomingFeedrate)
    FSTRINGVAR(tEPRZHomingFeedrate)
    FSTRINGVAR(tEPRXAcceleration)
    FSTRINGVAR(tEPRYAcceleration)
    FSTRINGVAR(tEPRZAcceleration)
    FSTRINGVAR(tEPRAAcceleration)
    FSTRINGVAR(tEPROPSMode)
    FSTRINGVAR(tEPROPSMoveAfter)
    FSTRINGVAR(tEPROPSMinDistance)
#endif
#if SDSUPPORT
    //FSTRINGVAR(tSDRemoved)
    //FSTRINGVAR(tSDInserted)
    FSTRINGVAR(tSDInitFail)
    FSTRINGVAR(tErrorWritingToFile)
    FSTRINGVAR(tBeginFileList)
    FSTRINGVAR(tEndFileList)
    FSTRINGVAR(tFileOpened)
    FSTRINGVAR(tSpaceSizeColon)
    FSTRINGVAR(tFileSelected)
    FSTRINGVAR(tFileOpenFailed)
    FSTRINGVAR(tSDPrintingByte)
    FSTRINGVAR(tNotSDPrinting)
    FSTRINGVAR(tOpenFailedFile)
    FSTRINGVAR(tWritingToFile)
    FSTRINGVAR(tDoneSavingFile)
    FSTRINGVAR(tFileDeleted)
    FSTRINGVAR(tDeletionFailed)
    FSTRINGVAR(tDirectoryCreated)
    FSTRINGVAR(tCreationFailed)
    FSTRINGVAR(tSDErrorCode)
#endif // SDSUPPORT
#if DISTORTION_CORRECTION
    FSTRINGVAR(tZCorrectionEnabled)
    FSTRINGVAR(tZCorrectionDisabled)
#endif
    FSTRINGVAR(tConfig)
    FSTRINGVAR(tExtrDot)
    FSTRINGVAR(tMachineModeLaser)
    FSTRINGVAR(tMachineModeCNC)
#if SUPPORT_SPINDLE
    FSTRINGVAR(tSpindleState)
    FSTRINGVAR(tSpaceRpm)
#endif
#if SUPPORT_LASER
    FSTRINGVAR(tLaserState)
    FSTRINGVAR(tSpaceMinimumIntensity)
#endif
#ifdef STARTUP_GCODE
    FSTRINGVAR(tStartupGCode)
#endif

    static void cap(FSTRINGPARAM(text), int value);
    static void config(FSTRINGPARAM(text));
    static void config(FSTRINGPARAM(text), int value);
    static void config(FSTRINGPARAM(text), const char *msg);
    static void config(FSTRINGPARAM(text), int32_t value);
    static void config(FSTRINGPARAM(text), uint32_t value);
    static void config(FSTRINGPARAM(text), float value, uint8_t digits=2);
    static void printNumber(uint32_t n);
    static void printWarningF(FSTRINGPARAM(text));
    static void printInfoF(FSTRINGPARAM(text));
    static void printErrorF(FSTRINGPARAM(text));
    static void printWarningFLN(FSTRINGPARAM(text));
    static void printInfoFLN(FSTRINGPARAM(text));
    static void printErrorFLN(FSTRINGPARAM(text));
    static void printFLN(FSTRINGPARAM(text));
    static void printF(FSTRINGPARAM(text));
    static void printF(FSTRINGPARAM(text), int16_t value);
    static void printF(FSTRINGPARAM(text), uint16_t value);
    static void printF(FSTRINGPARAM(text), const char *msg);
    static void printF(FSTRINGPARAM(text), int32_t value);
    static void printF(FSTRINGPARAM(text), uint32_t value);
    static void printF(FSTRINGPARAM(text), float value, uint8_t digits=2);
    static void printFLN(FSTRINGPARAM(text), int16_t value);
    static void printFLN(FSTRINGPARAM(text), uint16_t value);
    static void printFLN(FSTRINGPARAM(text), int32_t value);
    static void printFLN(FSTRINGPARAM(text), uint32_t value);
    static void printFLN(FSTRINGPARAM(text), const char *msg);
    static void printFLN(FSTRINGPARAM(text), float value,uint8_t digits=2);
    static void printArrayFLN(FSTRINGPARAM(text), float *arr, uint8_t n=4, uint8_t digits=2);
    static void printArrayFLN(FSTRINGPARAM(text), long *arr, uint8_t n=4);
    static void print(int32_t value);
    static inline void print(uint32_t value) { printNumber(value); }
    static inline void print(int16_t value) { printNumber((int32_t)value); }
    static inline void print(uint16_t value) { printNumber((uint32_t)value); }
    static void print(const char *text);
    static inline void print(char c) { GCodeSource::writeToAll(c); }
    static void printFloat(float number, uint8_t digits);
    static inline void print(float number) { printFloat(number, 6); }
    static inline void println() { GCodeSource::writeToAll('\r'); GCodeSource::writeToAll('\n'); }
    static bool writeToAll;

protected:
private:
};

#ifdef DEBUG
#define SHOW(x)                 {Com::printF(PSTR(" " #x "=")); Com::print(x); Com::println();}
#define SHOWS(x)                {Com::printF(PSTR(" " #x "=")); Com::print(x); Com::print(" steps  "); Com::print(x/80); Com::printFLN(PSTR(" mm"));}
#define SHOWM(x)                {Com::printF(PSTR(" " #x "=")); Com::print((long)x*80); Com::print(" steps  "); Com::print(x); Com::printFLN(PSTR(" mm"));}
#define SHOT(x)                 Com::printF(PSTR(x " "))
#define SHOWA(t,a,n)            {SHOT(t); for (int i=0;i<n;i++) SHOWS(a[i]);}
#define SHOWAM(t,a,n)           {SHOT(t); for (int i=0;i<n;i++) SHOWM(a[i]);}
#else
#define SHOW(x)
#define SHOT(x)
#define SHOWS(x)
#define SHOWM(x)
#define SHOWA(t,a,n)
#define SHOWAM(t,a,n)
#endif

#endif // COMMUNICATION_H
