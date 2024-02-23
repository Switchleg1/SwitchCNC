#ifndef _EEPROM_H
#define _EEPROM_H

// Id to distinguish version changes
#define EEPROM_PROTOCOL_VERSION 2

/** Where to start with our data block in memory. Can be moved if you
have problems with other modules using the eeprom */

#define EPR_MAGIC_BYTE              		0
#define EPR_XAXIS_STEPS_PER_MM      		1
#define EPR_YAXIS_STEPS_PER_MM      		5
#define EPR_ZAXIS_STEPS_PER_MM      		9
#define EPR_X_MAX_FEEDRATE         			13
#define EPR_Y_MAX_FEEDRATE         			17
#define EPR_Z_MAX_FEEDRATE         			21
#define EPR_X_HOMING_FEEDRATE      			25
#define EPR_Y_HOMING_FEEDRATE      			29
#define EPR_Z_HOMING_FEEDRATE      			33
#define EPR_MAX_XJERK               		37
#define EPR_MAX_ZJERK              			41
#define EPR_STEPPER_INACTIVE_TIME  			45
#define EPR_X_MAX_ACCEL            			49
#define EPR_Y_MAX_ACCEL            			53
#define EPR_Z_MAX_ACCEL            			57
#define EPR_BAUDRATE               			61
#define EPR_MAX_INACTIVE_TIME      			65
#define EPR_INTEGRITY_BYTE         			69   // Here the xored sum over eeprom is stored
#define EPR_VERSION                			73   // Version id for updates in EEPROM storage
#define EPR_X_HOME_OFFSET          			81
#define EPR_Y_HOME_OFFSET          			85
#define EPR_Z_HOME_OFFSET          			89
#define EPR_X_LENGTH               			93
#define EPR_Y_LENGTH               			97
#define EPR_Z_LENGTH              			101
#define EPR_BACKLASH_X            			105
#define EPR_BACKLASH_Y            			109
#define EPR_BACKLASH_Z            			113
#define EPR_Z_PROBE_HEIGHT        			117
#define EPR_Z_PROBE_SPEED         			121
#define EPR_Z_PROBE_XY_SPEED      			125
#define EPR_MAX_AJERK              			129  	//SL
#define EPR_A_MAX_ACCEL            			133		//SL
#define EPR_AAXIS_STEPS_PER_MM      		137		//SL
#define EPR_A_MAX_FEEDRATE         			141		//SL
#define EPR_BACKLASH_A            			145		//SL
#define EPR_MAX_YJERK               		149		//SL
#define EPR_DISTORTION_CORRECTION_ENABLED  	165
#define EPR_DISTORTION_USE_OFFSET			166		 //SL
#define EPR_DISTORTION_XMIN					167       //SL
#define EPR_DISTORTION_XMAX					169       //SL
#define EPR_DISTORTION_YMIN					171       //SL
#define EPR_DISTORTION_YMAX					173       //SL
#define EPR_DISTORTION_POINTS				175       //SL
#define EPR_DISTORTION_START				176		 //SL
#define EPR_DISTORTION_END					180		 //SL

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

class EEPROM
{
#if EEPROM_MODE != 0
	static void writeFloat(uint pos,PGM_P text,uint8_t digits = 3);
    static void writeLong(uint pos,PGM_P text);
    static void writeInt(uint pos,PGM_P text);
    static void writeByte(uint pos,PGM_P text);
public:
    static uint8_t computeChecksum();
    static void updateChecksum();
#endif
public:

    static void init();
    static void initBaudrate();
    static void storeDataIntoEEPROM(uint8_t corrupted = 0);
	static void readDataFromEEPROM();
    static void restoreEEPROMSettingsFromConfiguration();
    static void writeSettings();
    static void update(GCode *com);
    static void updatePrinterUsage();
    static inline void setVersion(uint8_t v) {
#if EEPROM_MODE != 0
        HAL::eprSetByte(EPR_VERSION,v);
        HAL::eprSetByte(EPR_INTEGRITY_BYTE,computeChecksum());
#endif
    }
#if FEATURE_Z_PROBE
	static inline void setZProbeHeight(float mm) {
#if EEPROM_MODE != 0
		HAL::eprSetFloat(EPR_Z_PROBE_HEIGHT, mm);
		Com::printFLN(PSTR("Z-Probe height set to: "),mm,3);
		EEPROM::updateChecksum();
#endif
	}
#endif

    static inline float zProbeSpeed() {
#if EEPROM_MODE != 0
        return HAL::eprGetFloat(EPR_Z_PROBE_SPEED);
#else
        return Z_PROBE_SPEED;
#endif
    }
    static inline float zProbeXYSpeed() {
#if EEPROM_MODE != 0
        return HAL::eprGetFloat(EPR_Z_PROBE_XY_SPEED);
#else
        return Z_PROBE_XY_SPEED;
#endif
	}
	static inline float zProbeHeight() {
#if EEPROM_MODE != 0
		return HAL::eprGetFloat(EPR_Z_PROBE_HEIGHT);
#else
        return Z_PROBE_HEIGHT;
#endif
	}
	static void initalizeUncached();

    static void setZCorrection(int32_t c,int index);
    static inline int32_t getZCorrection(int index) {
        return HAL::eprGetInt32(2048 + (index << 2));
    }
    static inline void setZCorrectionEnabled(int8_t on) {
#if EEPROM_MODE != 0
        if(isZCorrectionEnabled() == on) return;
		HAL::eprSetByte(EPR_DISTORTION_CORRECTION_ENABLED, on);
        EEPROM::updateChecksum();
#endif
    }
    static inline int8_t isZCorrectionEnabled() {
#if EEPROM_MODE != 0
        return HAL::eprGetByte(EPR_DISTORTION_CORRECTION_ENABLED);
#else
        return 0;
#endif
	}
};
#endif
