#ifndef _EEPROM_H
#define _EEPROM_H

#define EEPROM_VERSION                      2

#define EEPROM_MAIN_OFFSET                  0
#define EEPROM_MAIN_LENGTH                  2048

#define EEPROM_DISTORTION_OFFSET            2048
#define EEPROM_DISTORTION_LENGTH            1024

#define EEPROM_STATE_OFFSET                 4000
#define EEPROM_STATE_LENGTH                 96

/** Where to start with our data block in memory. Can be moved if you
have problems with other modules using the eeprom */

#define EPR_CHECKSUM              		    EEPROM_MAIN_OFFSET + 0   // Here the xored sum over eeprom is stored
#define EPR_MAGIC_BYTE         			    EEPROM_MAIN_OFFSET + 1   
#define EPR_VERSION                			EEPROM_MAIN_OFFSET + 2   // Version id for updates in EEPROM storage
#define EPR_ALLOW_PARTIAL_GCODE_AS_MOVE     EEPROM_MAIN_OFFSET + 3
#define EPR_XAXIS_STEPS_PER_MM      		EEPROM_MAIN_OFFSET + 4
#define EPR_YAXIS_STEPS_PER_MM      		EEPROM_MAIN_OFFSET + 8
#define EPR_ZAXIS_STEPS_PER_MM      		EEPROM_MAIN_OFFSET + 12
#define EPR_X_MAX_FEEDRATE         			EEPROM_MAIN_OFFSET + 16
#define EPR_Y_MAX_FEEDRATE         			EEPROM_MAIN_OFFSET + 20
#define EPR_Z_MAX_FEEDRATE         			EEPROM_MAIN_OFFSET + 24
#define EPR_X_HOMING_FEEDRATE      			EEPROM_MAIN_OFFSET + 28
#define EPR_Y_HOMING_FEEDRATE      			EEPROM_MAIN_OFFSET + 32
#define EPR_Z_HOMING_FEEDRATE      			EEPROM_MAIN_OFFSET + 36
#define EPR_MAX_XJERK               		EEPROM_MAIN_OFFSET + 40
#define EPR_MAX_ZJERK              			EEPROM_MAIN_OFFSET + 44
#define EPR_STEPPER_INACTIVE_TIME  			EEPROM_MAIN_OFFSET + 48
#define EPR_X_MAX_ACCEL            			EEPROM_MAIN_OFFSET + 52
#define EPR_Y_MAX_ACCEL            			EEPROM_MAIN_OFFSET + 56
#define EPR_Z_MAX_ACCEL            			EEPROM_MAIN_OFFSET + 60
#define EPR_BAUDRATE               			EEPROM_MAIN_OFFSET + 64
#define EPR_MAX_INACTIVE_TIME      			EEPROM_MAIN_OFFSET + 68
#define EPR_X_HOME_OFFSET          			EEPROM_MAIN_OFFSET + 72
#define EPR_Y_HOME_OFFSET          			EEPROM_MAIN_OFFSET + 76
#define EPR_Z_HOME_OFFSET          			EEPROM_MAIN_OFFSET + 80
#define EPR_X_LENGTH               			EEPROM_MAIN_OFFSET + 84
#define EPR_Y_LENGTH               			EEPROM_MAIN_OFFSET + 88
#define EPR_Z_LENGTH              			EEPROM_MAIN_OFFSET + 92
#define EPR_BACKLASH_X            			EEPROM_MAIN_OFFSET + 96
#define EPR_BACKLASH_Y            			EEPROM_MAIN_OFFSET + 100
#define EPR_BACKLASH_Z            			EEPROM_MAIN_OFFSET + 104
#define EPR_Z_PROBE_HEIGHT        			EEPROM_MAIN_OFFSET + 108
#define EPR_Z_PROBE_SPEED         			EEPROM_MAIN_OFFSET + 112
#define EPR_Z_PROBE_XY_SPEED      			EEPROM_MAIN_OFFSET + 116
#define EPR_MAX_AJERK              			EEPROM_MAIN_OFFSET + 120
#define EPR_A_MAX_ACCEL            			EEPROM_MAIN_OFFSET + 124
#define EPR_AAXIS_STEPS_PER_MM      		EEPROM_MAIN_OFFSET + 128
#define EPR_A_MAX_FEEDRATE         			EEPROM_MAIN_OFFSET + 132
#define EPR_BACKLASH_A            			EEPROM_MAIN_OFFSET + 136
#define EPR_MAX_YJERK               		EEPROM_MAIN_OFFSET + 140
#define EPR_DISTORTION_CORRECTION_ENABLED  	EEPROM_MAIN_OFFSET + 144
#define EPR_DISTORTION_USE_OFFSET			EEPROM_MAIN_OFFSET + 145
#define EPR_DISTORTION_XMIN					EEPROM_MAIN_OFFSET + 146
#define EPR_DISTORTION_XMAX					EEPROM_MAIN_OFFSET + 148
#define EPR_DISTORTION_YMIN					EEPROM_MAIN_OFFSET + 150
#define EPR_DISTORTION_YMAX					EEPROM_MAIN_OFFSET + 152
#define EPR_DISTORTION_POINTS				EEPROM_MAIN_OFFSET + 154
#define EPR_DISTORTION_START				EEPROM_MAIN_OFFSET + 158
#define EPR_DISTORTION_END					EEPROM_MAIN_OFFSET + 162


#define EPR_DISTORTION_POINT_DATA           EEPROM_DISTORTION_OFFSET

#define EPR_STATE_CHECKSUM                  EEPROM_STATE_OFFSET + 0
#define EPR_STATE_VALID                     EEPROM_STATE_OFFSET + 1
#define EPR_STATE_FLAGS                     EEPROM_STATE_OFFSET + 2
#define EPR_STATE_X_STEPS                   EEPROM_STATE_OFFSET + 4
#define EPR_STATE_Y_STEPS                   EEPROM_STATE_OFFSET + 8
#define EPR_STATE_Z_STEPS                   EEPROM_STATE_OFFSET + 12
#define EPR_STATE_A_STEPS                   EEPROM_STATE_OFFSET + 16
#define EPR_STATE_X_OFFSET                  EEPROM_STATE_OFFSET + 20
#define EPR_STATE_Y_OFFSET                  EEPROM_STATE_OFFSET + 24
#define EPR_STATE_Z_OFFSET                  EEPROM_STATE_OFFSET + 28

#if EEPROM_MODE != 0
#define EEPROM_FLOAT(x) HAL::eprGetFloat(EPR_##x)
#define EEPROM_INT32(x) HAL::eprGetInt32(EPR_##x)
#define EEPROM_BYTE(x) HAL::eprGetByte(EPR_##x)
#define EEPROM_SET_BYTE(x,val) HAL::eprSetByte(EPR_##x,val)
#else
#define EEPROM_FLOAT(x) (float)(x)
#define EEPROM_INT32(x) (int32_t)(x)
#define EEPROM_BYTE(x) (uint8_t)(x)
#define EEPROM_SET_BYTE(x,val)
#endif

class EEPROM {
public:
    static void init();
    static void initBaudrate();
    static void storeDataIntoEEPROM(uint8_t corrupted = 0);
	static void readDataFromEEPROM();
    static void restoreEEPROMSettingsFromConfiguration();
    static void writeSettings();
    static void update(GCode *com);

    static void setVersion(uint8_t version);
    static uint8_t getVersion();
#if Z_PROBE_SUPPORT
    static void setZProbeHeight(float mm);
    static float zProbeSpeed();
    static float zProbeXYSpeed();
    static float zProbeHeight();
#endif
#if DISTORTION_CORRECTION_SUPPORT && EEPROM_MODE
    static void setZCorrectionPoints(uint8_t count);
    static void setZCorrectionMinMax(int16_t xMin, int16_t yMin, int16_t xMax, int16_t yMax);
    static void setZCorrection(int32_t* data, uint16_t count);
    static void getZCorrection(int32_t* data, uint16_t count);
    static void setZCorrectionEnabled(int8_t on);
    static int8_t isZCorrectionEnabled();
#endif
#if ALLOW_PARTIAL_GCODE_AS_MOVE && EEPROM_MODE
    static void setAllowPartialGCode(uint8_t allow);
    static uint8_t getAllowPartialGCode();
#endif
#if AUTO_SAVE_RESTORE_STATE
    static void setCurrentValidState(uint8_t isValid);
    static void setCurrentSteps(int32_t* steps);
    static void setCurrentOffset(float* offset);
    static void setCurrentFlags(uint16_t flags);
    static uint8_t isCurrentStateChecksumValid();
    static uint8_t getCurrentValidState();
    static void getCurrentSteps(int32_t* steps);
    static void getCurrentOffset(float* offset);
    static uint16_t getCurrentFlags();
#endif

private:
#if EEPROM_MODE != 0
    static uint8_t computeChecksum(uint16_t offset, uint16_t length);
    static void updateChecksum(uint16_t offset, uint16_t length);
    static void initalizeUncached();
    static void writeFloat(uint pos, PGM_P text, uint8_t digits = 3);
    static void writeLong(uint pos, PGM_P text);
    static void writeInt(uint pos, PGM_P text);
    static void writeByte(uint pos, PGM_P text);
#endif
};
#endif
