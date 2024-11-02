#include "SwitchCNC.h"

FSTRINGVALUE(Com::tDebug, "Debug:")
FSTRINGVALUE(Com::tFirmware, "FIRMWARE_NAME:Repetier_" SWITCHCNC_VERSION " COMPILED:" __DATE__ " FIRMWARE_URL:" FIRMWARE_URL " PROTOCOL_VERSION:1.0 MACHINE_TYPE:" MACHINE_TYPE " REPETIER_PROTOCOL:3")
FSTRINGVALUE(Com::tOk, "ok")
FSTRINGVALUE(Com::tNewline, "\r\n")
FSTRINGVALUE(Com::tNAN, "NAN")
FSTRINGVALUE(Com::tINF, "INF")
FSTRINGVALUE(Com::tError, "Error:")
FSTRINGVALUE(Com::tInfo, "Info:")
FSTRINGVALUE(Com::tWarning, "Warning:")
FSTRINGVALUE(Com::tResend, "Resend:")
FSTRINGVALUE(Com::tEcho, "Echo:")
FSTRINGVALUE(Com::tCap, "Cap:")
FSTRINGVALUE(Com::tCapEeprom, "EEPROM:")
FSTRINGVALUE(Com::tCapZProbe, "Z_PROBE:")
FSTRINGVALUE(Com::tCapSoftwarePower, "SOFTWARE_POWER:")
FSTRINGVALUE(Com::tCapToggleLights, "TOGGLE_LIGHTS:")
FSTRINGVALUE(Com::tCapCoolantMist, "COOLANT_MIST:")
FSTRINGVALUE(Com::tCapCoolantFlood, "COOLANT_FLOOD:")
FSTRINGVALUE(Com::tCapDistortionCorrection, "DISTORTION_CORRECTION:")
FSTRINGVALUE(Com::tCapFanControl, "FAN_CONTROL:")
FSTRINGVALUE(Com::tCapFan, "FAN:")
FSTRINGVALUE(Com::tCapFan2, "FAN2:")
FSTRINGVALUE(Com::tCapVacuum, "VACUUM:")
FSTRINGVALUE(Com::tCapSDCard, "SDCARD:")
FSTRINGVALUE(Com::tCfgBaudrate, "Baudrate:")
FSTRINGVALUE(Com::tCfgInputBuffer, "InputBuffer:")
FSTRINGVALUE(Com::tCfgXHomeDir, "XHomeDir:")
FSTRINGVALUE(Com::tCfgYHomeDir, "YHomeDir:")
FSTRINGVALUE(Com::tCfgZHomeDir, "ZHomeDir:")
FSTRINGVALUE(Com::tCfgXHomePos, "XHomePos:")
FSTRINGVALUE(Com::tCfgYHomePos, "YHomePos:")
FSTRINGVALUE(Com::tCfgZHomePos, "ZHomePos:")
FSTRINGVALUE(Com::tCfgMachinelineCache, "MachinelineCache:")
FSTRINGVALUE(Com::tCfgKeepAliveInterval, "KeepAliveInterval:")
FSTRINGVALUE(Com::tCfgJerkX, "JerkX:")
FSTRINGVALUE(Com::tCfgJerkY, "JerkY:")
FSTRINGVALUE(Com::tCfgJerkZ, "JerkZ:")
FSTRINGVALUE(Com::tCfgJerkA, "JerkA:")
FSTRINGVALUE(Com::tCfgXMin, "XMin:")
FSTRINGVALUE(Com::tCfgYMin, "YMin:")
FSTRINGVALUE(Com::tCfgZMin, "ZMin:")
FSTRINGVALUE(Com::tCfgXMax, "XMax:")
FSTRINGVALUE(Com::tCfgYMax, "YMax:")
FSTRINGVALUE(Com::tCfgZMax, "ZMax:")
FSTRINGVALUE(Com::tCfgXSize, "XSize:")
FSTRINGVALUE(Com::tCfgYSize, "YSize:")
FSTRINGVALUE(Com::tCfgZSize, "ZSize:")
FSTRINGVALUE(Com::tCfgXAccel, "XAccel:")
FSTRINGVALUE(Com::tCfgYAccel, "YAccel:")
FSTRINGVALUE(Com::tCfgZAccel, "ZAccel:")
FSTRINGVALUE(Com::tCfgAAccel, "AAccel:")
FSTRINGVALUE(Com::tOkSpace, "ok ")
FSTRINGVALUE(Com::tWrongChecksum, "Wrong checksum")
FSTRINGVALUE(Com::tFormatError, "Format error")
FSTRINGVALUE(Com::tDoneMilling, "Done milling part")
FSTRINGVALUE(Com::tX, " X")
FSTRINGVALUE(Com::tY, " Y")
FSTRINGVALUE(Com::tZ, " Z")
FSTRINGVALUE(Com::tE, " E")
FSTRINGVALUE(Com::tF, " F")
FSTRINGVALUE(Com::tS, " S")
FSTRINGVALUE(Com::tP, " P")
FSTRINGVALUE(Com::tI, " I")
FSTRINGVALUE(Com::tJ, " J")
FSTRINGVALUE(Com::tR, " R")
FSTRINGVALUE(Com::tD, " D")
FSTRINGVALUE(Com::tC, " C")
FSTRINGVALUE(Com::tH, " H")
FSTRINGVALUE(Com::tA, " A")
FSTRINGVALUE(Com::tB, " B")
FSTRINGVALUE(Com::tK, " K")
FSTRINGVALUE(Com::tL, " L")
FSTRINGVALUE(Com::tO, " O")
FSTRINGVALUE(Com::tSDReadError, "SD read error")
FSTRINGVALUE(Com::tExpectedLine, "Error:expected line ")
FSTRINGVALUE(Com::tGot, " got ")
FSTRINGVALUE(Com::tSkip, "skip ")
FSTRINGVALUE(Com::tBLK, "BLK ")
FSTRINGVALUE(Com::tStart, "start")
FSTRINGVALUE(Com::tPowerUp, "PowerUp")
FSTRINGVALUE(Com::tExternalReset, "External Reset")
FSTRINGVALUE(Com::tBrownOut, "Brown out Reset")
FSTRINGVALUE(Com::tWatchdog, "Watchdog Reset")
FSTRINGVALUE(Com::tSoftwareReset, "Software Reset")
FSTRINGVALUE(Com::tUnknownCommand, "Unknown command:")
FSTRINGVALUE(Com::tFreeRAM, "Free RAM:")
FSTRINGVALUE(Com::tXColon, "X:")
FSTRINGVALUE(Com::tSpaceXColon, " X:")
FSTRINGVALUE(Com::tSpaceYColon, " Y:")
FSTRINGVALUE(Com::tSpaceZColon, " Z:")
FSTRINGVALUE(Com::tSpaceAColon, " A:")
FSTRINGVALUE(Com::tTColon, "T:")
FSTRINGVALUE(Com::tSpaceBColon, " B:")
FSTRINGVALUE(Com::tSpaceAtColon, " @:")
FSTRINGVALUE(Com::tSpaceT, " T")
FSTRINGVALUE(Com::tSpaceAt, " @")
FSTRINGVALUE(Com::tSpaceBAtColon, " B@:")
FSTRINGVALUE(Com::tSpaceRaw, " RAW")
FSTRINGVALUE(Com::tColon, ":")
FSTRINGVALUE(Com::tSlash, "/")
FSTRINGVALUE(Com::tSpaceSlash, " /")
FSTRINGVALUE(Com::tFatal, "fatal:")
FSTRINGVALUE(Com::tSpeedMultiply, "SpeedMultiply:")
FSTRINGVALUE(Com::tIntensityMultiply, "FlowMultiply:")
FSTRINGVALUE(Com::tFanspeed, "Fanspeed:")
FSTRINGVALUE(Com::tFan2speed, "Fanspeed2:")
FSTRINGVALUE(Com::tVacuumState, "Vacuum:")
FSTRINGVALUE(Com::tInvalidArc, "Invalid arc")
FSTRINGVALUE(Com::tComma, ",")
FSTRINGVALUE(Com::tSpace, " ")
FSTRINGVALUE(Com::tYColon, "Y:")
FSTRINGVALUE(Com::tZColon, "Z:")
FSTRINGVALUE(Com::tMS1MS2Pins, "MS1,MS2 Pins")
FSTRINGVALUE(Com::tSetOutputSpace, "Set output: ")
FSTRINGVALUE(Com::tGetInputSpace, "Get Input: ")
FSTRINGVALUE(Com::tSpaceToSpace, " to ")
FSTRINGVALUE(Com::tSpaceIsSpace, " is ")
FSTRINGVALUE(Com::tHSpace, "H ")
FSTRINGVALUE(Com::tLSpace, "L ")
FSTRINGVALUE(Com::tXMinColon, "x_min:")
FSTRINGVALUE(Com::tXMaxColon, "x_max:")
FSTRINGVALUE(Com::tYMinColon, "y_min:")
FSTRINGVALUE(Com::tYMaxColon, "y_max:")
FSTRINGVALUE(Com::tZMinColon, "z_min:")
FSTRINGVALUE(Com::tZMaxColon, "z_max:")
FSTRINGVALUE(Com::tZ2MinMaxColon, "z2_minmax:")
FSTRINGVALUE(Com::tXJerkColon, "XJerk:")
FSTRINGVALUE(Com::tYJerkColon, " YJerk:")
FSTRINGVALUE(Com::tZJerkColon, " ZJerk:")
FSTRINGVALUE(Com::tAJerkColon, " AJerk:")
FSTRINGVALUE(Com::tEEPROMUpdated, "EEPROM updated")
FSTRINGVALUE(Com::tMachineHeight, "Machine height:")
#ifdef DEBUG_QUEUE_MOVE
FSTRINGVALUE(Com::tDBGId, "ID:")
FSTRINGVALUE(Com::tDBGVStartEnd, "vStart/End:")
FSTRINGVALUE(Com::tDBAccelSteps, "accel/decel steps:")
FSTRINGVALUE(Com::tDBGStartEndSpeed, "st./end speed:")
FSTRINGVALUE(Com::tDBGFlags, "Flags:")
FSTRINGVALUE(Com::tDBGJoinFlags, "joinFlags:")
FSTRINGVALUE(Com::tDBGDelta, "Delta")
FSTRINGVALUE(Com::tDBGDir, "Dir:")
FSTRINGVALUE(Com::tDBGFullSpeed, "fullSpeed:")
FSTRINGVALUE(Com::tDBGVMax, "vMax:")
FSTRINGVALUE(Com::tDBGAcceleration, "Acceleration:")
FSTRINGVALUE(Com::tDBGAccelerationPrim, "Acceleration Prim:")
FSTRINGVALUE(Com::tDBGRemainingSteps, "Remaining steps:")
FSTRINGVALUE(Com::tDBGLimitInterval, "LimitInterval:")
FSTRINGVALUE(Com::tDBGMoveDistance, "Move distance on the XYZ space:")
FSTRINGVALUE(Com::tDBGCommandedFeedrate, "Commanded feedrate:")
FSTRINGVALUE(Com::tDBGConstFullSpeedMoveTime, "Constant full speed move time:")
#endif // DEBUG_QUEUE_MOVEFSTRINGVALUE(Com::,"")
#ifdef DEBUG_STEPCOUNT
FSTRINGVALUE(Com::tDBGMissedSteps, "Missed steps:")
#endif // DEBUG_STEPCOUNT
#if FEATURE_Z_PROBE
FSTRINGVALUE(Com::tZProbe, "Z-probe:")
FSTRINGVALUE(Com::tZProbeState, "Z-probe state:")
FSTRINGVALUE(Com::tZProbeZReset, "Reset Z height")
FSTRINGVALUE(Com::tZProbeFailed, "Z-probe failed")
#endif
#if DISTORTION_CORRECTION
FSTRINGVALUE(Com::tDistortionXMIN, "Distortion XMIN:") //SL
FSTRINGVALUE(Com::tDistortionXMAX, "Distortion XMAX:") //SL
FSTRINGVALUE(Com::tDistortionYMIN, "Distortion YMIN:") //SL
FSTRINGVALUE(Com::tDistortionYMAX, "Distortion YMAX:") //SL
FSTRINGVALUE(Com::tDistortionPoints, "Distortion Grid Points:") //SL
FSTRINGVALUE(Com::tDistortionStart, "Distortion Degrade Start:") //SL
FSTRINGVALUE(Com::tDistortionEnd, "Distortion Degrade Length:") //SL
FSTRINGVALUE(Com::tDistortionUseOffset, "Distortion Use Z Offset:") //SL
#endif
#if defined(PAUSE_PIN) && PAUSE_PIN>-1
FSTRINGVALUE(Com::tPaused, "-PAUSED-") //SL
FSTRINGVALUE(Com::tUnpaused, "-UNPAUSED-")
#endif
//FSTRINGVALUE(Com::,"")
#ifdef WAITING_IDENTIFIER
FSTRINGVALUE(Com::tWait, WAITING_IDENTIFIER)
#endif // WAITING_IDENTIFIER
#if EEPROM_MODE == 0
FSTRINGVALUE(Com::tNoEEPROMSupport, "No EEPROM support compiled.\r\n")
#else
#if FEATURE_Z_PROBE
FSTRINGVALUE(Com::tZProbeHeight, "Z-probe height [mm]")
FSTRINGVALUE(Com::tZProbeSpeed, "Z-probe speed [mm/s]")
FSTRINGVALUE(Com::tZProbeSpeedXY, "Z-probe x-y-speed [mm/s]")
#endif
FSTRINGVALUE(Com::tConfigStoredEEPROM, "Configuration stored to EEPROM.")
FSTRINGVALUE(Com::tConfigLoadedEEPROM, "Configuration loaded from EEPROM.")
FSTRINGVALUE(Com::tEPRConfigResetDefaults, "Configuration reset to defaults.")
FSTRINGVALUE(Com::tEPRProtocolChanged, "Protocol version changed, upgrading")
FSTRINGVALUE(Com::tEPR0, "EPR:0 ")
FSTRINGVALUE(Com::tEPR1, "EPR:1 ")
FSTRINGVALUE(Com::tEPR2, "EPR:2 ")
FSTRINGVALUE(Com::tEPR3, "EPR:3 ")
FSTRINGVALUE(Com::tEPRBaudrate, "Baudrate")
FSTRINGVALUE(Com::tEPRMaxInactiveTime, "Max. inactive time [ms,0=off]")
FSTRINGVALUE(Com::tEPRStopAfterInactivty, "Stop stepper after inactivity [ms,0=off]")
FSTRINGVALUE(Com::tEPRXHomePos, "X min pos [mm]")
FSTRINGVALUE(Com::tEPRYHomePos, "Y min pos [mm]")
FSTRINGVALUE(Com::tEPRZHomePos, "Z min pos [mm]")
FSTRINGVALUE(Com::tEPRXMaxLength, "X max length [mm]")
FSTRINGVALUE(Com::tEPRYMaxLength, "Y max length [mm]")
FSTRINGVALUE(Com::tEPRZMaxLength, "Z max length [mm]")
FSTRINGVALUE(Com::tEPRXBacklash, "X backlash [mm]")
FSTRINGVALUE(Com::tEPRYBacklash, "Y backlash [mm]")
FSTRINGVALUE(Com::tEPRZBacklash, "Z backlash [mm]")
FSTRINGVALUE(Com::tEPRABacklash, "A backlash [mm]")
FSTRINGVALUE(Com::tEPRAccelerationFactorAtTop, "Acceleration factor at top [%,100=like bottom]")
FSTRINGVALUE(Com::tEPRMaxXJerk, "Max. X-jerk [mm/s]")
FSTRINGVALUE(Com::tEPRMaxYJerk, "Max. Y-jerk [mm/s]")
FSTRINGVALUE(Com::tEPRMaxZJerk, "Max. Z-jerk [mm/s]")
FSTRINGVALUE(Com::tEPRMaxAJerk, "Max. A-jerk [mm/s]")
FSTRINGVALUE(Com::tEPRXStepsPerMM, "X-axis steps per mm")
FSTRINGVALUE(Com::tEPRYStepsPerMM, "Y-axis steps per mm")
FSTRINGVALUE(Com::tEPRZStepsPerMM, "Z-axis steps per mm")
FSTRINGVALUE(Com::tEPRAStepsPerMM, "A-axis steps per mm")
FSTRINGVALUE(Com::tEPRXMaxFeedrate, "X-axis max. feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRYMaxFeedrate, "Y-axis max. feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRZMaxFeedrate, "Z-axis max. feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRAMaxFeedrate, "A-axis max. feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRXHomingFeedrate, "X-axis homing feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRYHomingFeedrate, "Y-axis homing feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRZHomingFeedrate, "Z-axis homing feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRXAcceleration, "X-axis acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRYAcceleration, "Y-axis acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRZAcceleration, "Z-axis acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRAAcceleration, "A-axis acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPROPSMode, "OPS operation mode [0=Off,1=Classic,2=Fast]")
#endif
#if SDSUPPORT
FSTRINGVALUE(Com::tSDInitFail, "SD init fail")
FSTRINGVALUE(Com::tErrorWritingToFile, "error writing to file")
FSTRINGVALUE(Com::tBeginFileList, "Begin file list")
FSTRINGVALUE(Com::tEndFileList, "End file list")
FSTRINGVALUE(Com::tFileOpened, "File opened:")
FSTRINGVALUE(Com::tSpaceSizeColon, " Size:")
FSTRINGVALUE(Com::tFileSelected, "File selected")
FSTRINGVALUE(Com::tFileOpenFailed, "file.open failed")
FSTRINGVALUE(Com::tSDPrintingByte, "SD printing byte ")
FSTRINGVALUE(Com::tNotSDPrinting, "Not SD printing")
FSTRINGVALUE(Com::tOpenFailedFile, "open failed, File: ")
FSTRINGVALUE(Com::tWritingToFile, "Writing to file: ")
FSTRINGVALUE(Com::tDoneSavingFile, "Done saving file.")
FSTRINGVALUE(Com::tFileDeleted, "File deleted")
FSTRINGVALUE(Com::tDeletionFailed, "Deletion failed")
FSTRINGVALUE(Com::tDirectoryCreated, "Directory created")
FSTRINGVALUE(Com::tCreationFailed, "Creation failed")
FSTRINGVALUE(Com::tSDErrorCode, "SD errorCode:")
#endif // SDSUPPORT
#if DISTORTION_CORRECTION
FSTRINGVALUE(Com::tZCorrectionEnabled, "Z correction enabled")
FSTRINGVALUE(Com::tZCorrectionDisabled, "Z correction disabled")
#endif
FSTRINGVALUE(Com::tConfig, "Config:")
FSTRINGVALUE(Com::tExtrDot, "Extr.")
FSTRINGVALUE(Com::tMachineModeLaser, "MachineMode: Laser")
FSTRINGVALUE(Com::tMachineModeCNC, "MachineMode: Spindle")
#if SUPPORT_SPINDLE
FSTRINGVALUE(Com::tSpindleState, "Spindle:");
FSTRINGVALUE(Com::tSpaceRpm, " RPM:");
#endif
#if SUPPORT_LASER
FSTRINGVALUE(Com::tLaserState, "Laser:");
FSTRINGVALUE(Com::tSpaceMinimumIntensity, " Minimum Intensity:");
#endif
#ifdef STARTUP_GCODE
FSTRINGVALUE(Com::tStartupGCode, STARTUP_GCODE)
#endif
bool Com::writeToAll = true; // transmit start messages to all devices!

void Com::cap(FSTRINGPARAM(text), int value) {
    printF(tCap);
    printFLN(text, value);
}
void Com::config(FSTRINGPARAM(text)) {
    printF(tConfig);
    printFLN(text);
}
void Com::config(FSTRINGPARAM(text), int value) {
    printF(tConfig);
    printFLN(text, value);
}
void Com::config(FSTRINGPARAM(text), const char *msg) {
    printF(tConfig);
    printF(text);
    print(msg);
    println();
}
void Com::config(FSTRINGPARAM(text), int32_t value) {
    printF(tConfig);
    printFLN(text, value);
}
void Com::config(FSTRINGPARAM(text), uint32_t value) {
    printF(tConfig);
    printFLN(text, value);
}
void Com::config(FSTRINGPARAM(text), float value, uint8_t digits) {
    printF(tConfig);
    printFLN(text, value, digits);
}
void Com::printWarningF(FSTRINGPARAM(text)) {
    printF(tWarning);
    printF(text);
}
void Com::printWarningFLN(FSTRINGPARAM(text)) {
    printF(tWarning);
    printFLN(text);
}
void Com::printInfoF(FSTRINGPARAM(text)) {
    printF(tInfo);
    printF(text);
}
void Com::printInfoFLN(FSTRINGPARAM(text)) {
    printF(tInfo);
    printFLN(text);
}

void Com::printErrorF(FSTRINGPARAM(text)) {
    printF(tError);
    printF(text);
}
void Com::printErrorFLN(FSTRINGPARAM(text)) {
    printF(tError);
    printFLN(text);
}
void Com::printFLN(FSTRINGPARAM(text)) {
    printF(text);
    println();
}
void Com::printFLN(FSTRINGPARAM(text), const char *msg) {
    printF(text);
    print(msg);
    println();
}

void Com::printF(FSTRINGPARAM(ptr)) {
    char c;
    while ((c = HAL::readFlashByte(ptr++)) != 0) {
        GCodeSource::writeToAll(c);
    }
}
void Com::printF(FSTRINGPARAM(text), const char *msg) {
    printF(text);
    print(msg);
}
void Com::printF(FSTRINGPARAM(text), int16_t value) {
    printF(text);
    print(value);
}
void Com::printF(FSTRINGPARAM(text), uint16_t value) {
    printF(text);
    print(value);
}
void Com::printF(FSTRINGPARAM(text), int32_t value) {
    printF(text);
    print(value);
}
void Com::printF(FSTRINGPARAM(text), uint32_t value) {
    printF(text);
    printNumber(value);
}
void Com::printFLN(FSTRINGPARAM(text), int16_t value) {
    printF(text);
    print(value);
    println();
}
void Com::printFLN(FSTRINGPARAM(text), uint16_t value) {
    printF(text);
    print(value);
    println();
}
void Com::printFLN(FSTRINGPARAM(text), int32_t value) {
    printF(text);
    print(value);
    println();
}
void Com::printFLN(FSTRINGPARAM(text), uint32_t value) {
    printF(text);
    printNumber(value);
    println();
}
void Com::printFLN(FSTRINGPARAM(text), float value, uint8_t digits) {
    printF(text);
    printFloat(value, digits);
    println();
}
void Com::printF(FSTRINGPARAM(text), float value, uint8_t digits) {
    printF(text);
    printFloat(value, digits);
}

void Com::print(const char *text) {
    while(*text) {
        GCodeSource::writeToAll(*text++);
    }
}
void Com::print(int32_t value) {
    if(value < 0) {
        GCodeSource::writeToAll('-');
        value = -value;
    }
    printNumber(value);
}

void Com::printNumber(uint32_t n) {
    char buf[11]; // Assumes 8-bit chars plus zero byte.
    char *str = &buf[10];
    *str = '\0';
    do {
        unsigned long m = n;
        n /= 10;
        *--str = '0' + (m - 10 * n);
    } while(n);

    print(str);
}
void Com::printArrayFLN(FSTRINGPARAM(text), float *arr, uint8_t n, uint8_t digits) {
    printF(text);
    for(uint8_t i = 0; i < n; i++)
        printF(Com::tSpace, arr[i], digits);
    println();
}
void Com::printArrayFLN(FSTRINGPARAM(text), int32_t *arr, uint8_t n) {
    printF(text);
    for(uint8_t i = 0; i < n; i++)
        printF(Com::tSpace, arr[i]);
    println();
}

void Com::printFloat(float number, uint8_t digits) {
    if (isnanf(number)) {
        printF(tNAN);
        return;
    }
    if (isinff(number)) {
        printF(tINF);
        return;
    }
    // Handle negative numbers
    if (number < 0.0) {
        print('-');
        number = -number;
    }
    // Round correctly so that print(1.999, 2) prints as "2.00"
    float rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i)
        rounding /= 10.0;

    number += rounding;

    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)number;
    float remainder = number - (float)int_part;
    printNumber(int_part);

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0)
        print('.');

    // Extract digits from the remainder one at a time
    while (digits-- > 0) {
        remainder *= 10.0;
        int toPrint = int(remainder);
        print(toPrint);
        remainder -= toPrint;
    }
}

