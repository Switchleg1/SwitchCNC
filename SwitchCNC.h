#ifndef SWITCHCNC_H
#define SWITCHCNC_H

#include <math.h>
#include <stdint.h>
#include "Configuration.h"

#define SWITCHCNC_VERSION       "1.0.3"
#if SUPPORT_SPINDLE
#if SUPPORT_LASER
#define MACHINE_TYPE            "SPINDLE/LASER"
#else
#define MACHINE_TYPE            "SPINDLE"
#endif
#else
#define MACHINE_TYPE            "LASER"
#endif

#define FIRMWARE_URL            "https://github.com/switchleg1/SwitchCNC"

#define MACHINE_MODE_SPINDLE    0
#define MACHINE_MODE_LASER      1

//error check default machine mode
#if DEFAULT_MACHINE_MODE == 0 && (!SUPPORT_SPINDLE)
#error DEFAULT_MACHINE_MODE cannot be 0 when SUPPORT_SPINDLE is not defined
#endif
#if DEFAULT_MACHINE_MODE == 1 && (!SUPPORT_LASER)
#error DEFAULT_MACHINE_MODE cannot be 1 when SUPPORT_LASER is not defined
#endif
#if DEFAULT_MACHINE_MODE > 1
#error DEFAULT_MACHINE_MODE cannot be greater than 1
#endif

// Use new communication model for multiple channels - only until stable, then old version gets deleted
#define NEW_COMMUNICATION 1

/** Uncomment, to see detailed data for every move. Only for debugging purposes! */
//#define DEBUG_QUEUE_MOVE
/** write infos about path planner changes */
//#define DEBUG_PLANNER
/** Allows M111 to set bit 5 (16) which disables all commands except M111. This can be used
to test your data throughput or search for communication problems. */
#define INCLUDE_DEBUG_COMMUNICATION
// Echo all ascii commands after receiving
//#define DEBUG_ECHO_ASCII
/** Allows M111 so set bit 6 (32) which disables moves, at the first tried step. In combination
with a dry run, you can test the speed of path computations, which are still performed. */
#define INCLUDE_DEBUG_NO_MOVE
/** Writes the free RAM to output, if it is less then at the last test. Should always return
values >500 for safety, since it doesn't catch every function call. Nice to tweak cache
usage or for searching for memory induced errors. Switch it off for production, it costs execution time. */
//#define DEBUG_FREE_MEMORY
/** If enabled, steps to move and moved steps are compared. */
//#define DEBUG_STEPCOUNT
/** This enables code to make M666 drop an ok, so you get problems with communication. It is to test host robustness. */
//#define DEBUG_COM_ERRORS
/** Adds a menu point in quick settings to write debug informations to the host in case of hangs where the ui still works. */
//#define DEBUG_MACHINE
// Find the longest segment length during a print
//#define DEBUG_SEGMENT_LENGTH
// Find the maximum real jerk during a print
//#define DEBUG_REAL_JERK
// Uncomment the following line to enable debugging. You can better control debugging below the following line
//#define DEBUG

#define DEBUG_MSG(x)				{if(Machine::debugEcho()) {Com::printFLN(PSTR(x));HAL::delayMilliseconds(20);}}
#define DEBUG_MSG2(x,y)				{if(Machine::debugEcho()) {Com::printFLN(PSTR(x),y);HAL::delayMilliseconds(20);}}
#define DEBUG_MSG_FAST(x)			{if(Machine::debugEcho()) {Com::printFLN(PSTR(x));}}
#define DEBUG_MSG2_FAST(x,y)		{if(Machine::debugEcho()) {Com::printFLN(PSTR(x),y);}}

#define IGNORE_COORDINATE			999999

#define IS_MAC_TRUE(x)				(x!=0)
#define IS_MAC_FALSE(x)				(x==0)
#define HAS_PIN(x)					(defined( x ## _PIN) && x > -1)

// Uncomment if no analyzer is connected
//#define ANALYZER
// Channel->pin assignments
#define ANALYZER_CH0				63 // New move
#define ANALYZER_CH1				40 // Step loop
#define ANALYZER_CH2				53 // X Step
#define ANALYZER_CH3				65 // Y Step
#define ANALYZER_CH4				59 // X Direction
#define ANALYZER_CH5				64 // Y Direction
#define ANALYZER_CH6				58 // xsig
#define ANALYZER_CH7				57 // ysig

#ifdef ANALYZER
#define ANALYZER_ON(a)              {WRITE(a,HIGH);}
#define ANALYZER_OFF(a)             {WRITE(a,LOW);}
#else
#define ANALYZER_ON(a)
#define ANALYZER_OFF(a)
#endif

#define X_AXIS						0
#define Y_AXIS						1
#define Z_AXIS						2
#define A_AXIS						3
// How big an array to hold X_AXIS..<MAX_AXIS>
#define Z_AXIS_ARRAY				3
#define A_AXIS_ARRAY				4


#define HOME_ORDER_XYZ				1
#define HOME_ORDER_XZY				2
#define HOME_ORDER_YXZ				3
#define HOME_ORDER_YZX				4
#define HOME_ORDER_ZXY				5
#define HOME_ORDER_ZYX				6

//direction flags
#define X_DIRPOS					1
#define Y_DIRPOS					2
#define Z_DIRPOS					4
#define A_DIRPOS					8
#define XYZ_DIRPOS					7
#define XYZA_DIRPOS					15

//step flags
#define XSTEP						16
#define YSTEP						32
#define ZSTEP						64
#define ASTEP						128

//combo's
#define XY_STEP						48
#define XYZ_STEP					112
#define XYZA_STEP					240
#define X_STEP_DIRPOS				17
#define Y_STEP_DIRPOS				34
#define Z_STEP_DIRPOS				68
#define A_STEP_DIRPOS				136

#define ILLEGAL_Z_PROBE				-888

// we can not prevent this as some configurations need a parameter and others not
#pragma GCC diagnostic ignored		"-Wunused-parameter"
#pragma GCC diagnostic ignored		"-Wunused-variable"

#ifndef MOVE_X_WHEN_HOMED
#define MOVE_X_WHEN_HOMED           0
#endif
#ifndef MOVE_Y_WHEN_HOMED
#define MOVE_Y_WHEN_HOMED           0
#endif
#ifndef MOVE_Z_WHEN_HOMED
#define MOVE_Z_WHEN_HOMED           0
#endif

#ifndef BOARD_FAN_SPEED
#define BOARD_FAN_SPEED
#endif

#if FEATURE_Z_PROBE && Z_PROBE_PIN < 0
#error You need to define Z_PROBE_PIN to use z probe!
#endif

#if DISTORTION_CORRECTION
#if !FEATURE_Z_PROBE
#error Distortion correction requires the z probe feature to be enabled and configured!
#endif
#endif

#ifndef ZHOME_X_POS
#define ZHOME_X_POS IGNORE_COORDINATE
#endif
#ifndef ZHOME_Y_POS
#define ZHOME_Y_POS IGNORE_COORDINATE
#endif

#define GCODE_BUFFER_SIZE           1

#if !defined(Z_PROBE_REPETITIONS) || Z_PROBE_REPETITIONS < 1
#define Z_PROBE_SWITCHING_DISTANCE  0.5 // Distance to safely untrigger probe
#define Z_PROBE_REPETITIONS         1
#endif

#ifndef MINMAX_HARDWARE_ENDSTOP_Z2
#define MINMAX_HARDWARE_ENDSTOP_Z2  0
#define Z2_MINMAX_PIN               -1
#endif

#if MINMAX_HARDWARE_ENDSTOP_Z2 && Z2_MINMAX_PIN > -1
#undef MULTI_ZENDSTOP_HOMING 
#define MULTI_ZENDSTOP_HOMING       1
#define MULTI_ZENDSTOP_ALL          3
#else
#define MULTI_ZENDSTOP_HOMING       0
#endif

#if (X_HOME_DIR < 0 && HAS_PIN(X2_MIN) && MIN_HARDWARE_ENDSTOP_X2) || (X_HOME_DIR > 0 && HAS_PIN(X2_MAX) && MAX_HARDWARE_ENDSTOP_X2)
#define MULTI_XENDSTOP_HOMING       1
#define MULTI_XENDSTOP_ALL          3
#else
#define MULTI_XENDSTOP_HOMING       0
#endif

#if (Y_HOME_DIR < 0 && HAS_PIN(Y2_MIN) && MIN_HARDWARE_ENDSTOP_Y2) || (Y_HOME_DIR > 0 && HAS_PIN(Y2_MAX) && MAX_HARDWARE_ENDSTOP_Y2)
#define MULTI_YENDSTOP_HOMING       1
#define MULTI_YENDSTOP_ALL          3
#else
#define MULTI_YENDSTOP_HOMING       0
#endif

#define SPEED_MIN_MILLIS            400
#define SPEED_MAX_MILLIS            60
#define SPEED_MAGNIFICATION         100.0f

/**  \brief Horizontal distance bridged by the diagonal push rod when the end effector is in the center. It is pretty close to 50% of the push rod length (250 mm).
*/

#ifdef FEATURE_Z_PROBE
#define MANUAL_CONTROL              1
#endif

//Step to split a circle in small Lines
#ifndef MM_PER_ARC_SEGMENT
#define MM_PER_ARC_SEGMENT          0.5
#define MM_PER_ARC_SEGMENT_BIG      2
#else
#define MM_PER_ARC_SEGMENT_BIG      MM_PER_ARC_SEGMENT
#endif
//After this count of steps a new SIN / COS calculation is started to correct the circle interpolation
#define N_ARC_CORRECTION            25

#ifndef START_STEP_WITH_HIGH
#define START_STEP_WITH_HIGH        1
#endif


#ifndef DEBUG_FREE_MEMORY
#define DEBUG_MEMORY
#else
#define DEBUG_MEMORY                Commands::checkFreeMemory();
#endif

#ifndef KEEP_ALIVE_INTERVAL
#define KEEP_ALIVE_INTERVAL         2000
#endif

#ifndef MAX_VFAT_ENTRIES
#ifdef AVR_BOARD
#define MAX_VFAT_ENTRIES            (2)
#else
#define MAX_VFAT_ENTRIES            (3)
#endif
#endif
/** Total size of the buffer used to store the long filenames */
#define LONG_FILENAME_LENGTH        (13 * MAX_VFAT_ENTRIES + 1)
#define SD_MAX_FOLDER_DEPTH         2

#ifndef SDCARDDETECT
#define SDCARDDETECT                -1
#endif

#ifndef SDSUPPORT
#define SDSUPPORT                   0
#endif

#define DELAY1MICROSECOND           __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t")
#define DELAY2MICROSECOND           __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\tnop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t")

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#define COMPAT_PRE1
#endif
#include "fastio.h"
#include "TypeDefs.h"
#include "InterruptProtectedBlock.h"
#include "HAL.h"
#include "src/GCode/GcodeSource.h"
#include "src/GCode/SerialGcodeSource.h"
#include "src/GCode/SDCardGCodeSource.h"
#include "src/GCode/FlashGCodeSource.h"
#include "src/GCode/gcode.h"
#include "Communication.h"
#include "RMath.h"
#include "Machine.h"
#include "motion.h"
#include "SDCard.h"
#include "Commands.h"
#include "Eeprom.h"
#include "src/Drivers/MotorDrivers.h"
#include "src/Drivers/LaserDriver.h"
#include "src/Drivers/SpindleDriver.h"
#include "src/Drivers/VacuumDriver.h"
#include "Events.h"

#if SDSUPPORT
#include "src/SdFat/SdFat.h"
#endif

#if defined(CUSTOM_EVENTS)
#include "CustomEvents.h"
#endif

#endif
