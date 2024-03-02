#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// ################# Misc. settings ##################
#define DEFAULT_MACHINE_MODE					0				//0: SPINDLE 1: LASER
#define BAUDRATE								250000
#define KILL_METHOD								1
#define ACK_WITH_LINENUMBER						1
#define KEEP_ALIVE_INTERVAL						2000
#define WAITING_IDENTIFIER						"wait"
#define ECHO_ON_EXECUTE							1
#define EEPROM_MODE								1
#define FEATURE_WATCHDOG						1
#define BLUETOOTH_SERIAL						-1
#define BLUETOOTH_BAUD							250000
#define PAUSE_START_COMMANDS					""
#define PAUSE_END_COMMANDS						""
#define ARC_SUPPORT								1
#define FEATURE_MEMORY_POSITION					1
#define FEATURE_CHECKSUM_FORCED					0
#define CASE_LIGHTS_PIN							-1
#define CASE_LIGHT_DEFAULT_ON					1
#define VACUUM_PIN								9
#define INTERRUPT_FREQUENCY_DIVISOR				8

//Power settings
#define AUTOMATIC_POWERUP						1
#define ENABLE_POWER_ON_STARTUP					0
#define POWER_INVERTING							1
#define PS_ON_PIN								10

//Spindle settings
#define SUPPORT_SPINDLE							1
#define SPINDLE_WAIT_ON_START					5
#define SPINDLE_WAIT_ON_STOP					3
#define SPINDLE_ON_PIN							12
#define SPINDLE_ON_HIGH							1
#define SPINDLE_DIRECTION_PIN					-1
#define SPINDLE_DIRECTION_CW					1
#define SPINDLE_PWM_PIN							-1
#define SPINDLE_KICKSTART_TIME					200
#define SPINDLE_RPM_MIN							6000
#define SPINDLE_RPM_MAX							24000

//Laser settings
#define SUPPORT_LASER							0
#define LASER_ON_HIGH							1
#define LASER_WARMUP_TIME						0
#define LASER_PWM_MAX							255
#define LASER_WATT								0
#define LASER_TEMP_PIN							-1
#define LASER_TEMP_SENSOR_TYPE					0

//Fan settings
#define FEATURE_FAN_CONTROL						1
#define FAN_KICKSTART_TIME						200
#define FAN_PIN									-1

#define FEATURE_FAN2_CONTROL					0
#define FAN2_KICKSTART_TIME						200
#define FAN2_PIN								-1

//Fanboard settings
#define BOARD_FAN_SPEED							255
#define BOARD_FAN_MIN_SPEED						0
#define BOARD_FAN_KICKSTART_TIME				200
#define FAN_BOARD_PIN 							11

//Pause settings
#define PAUSE_STEPS								400
#define PAUSE_SLOPE								100
#define PAUSE_PIN								5
#define PAUSE_PULLUP							1
#define PAUSE_INVERTING							1

//speed dial
#define SPEED_DIAL								1
#define SPEED_DIAL_PIN							14
#define SPEED_DIAL_INVERT						0
#define SPEED_DIAL_MIN_PERCENT					50
#define SPEED_DIAL_BITS							6

//Servo support
#define FEATURE_SERVO							0
#define SERVO0_PIN								-1
#define SERVO1_PIN								-1
#define SERVO2_PIN								-1
#define SERVO3_PIN								-1
#define SERVO0_NEUTRAL_POS						-1
#define SERVO1_NEUTRAL_POS						-1
#define SERVO2_NEUTRAL_POS						-1
#define SERVO3_NEUTRAL_POS						-1

//SD Support
#define SDSUPPORT								0
#define SDPOWER									-1
#define SDSS									53
#define SDCARDDETECT							49
#define SDSSORIG								-1
#define SDCARDDETECTINVERTED					0
#define SD_EXTENDED_DIR							1 /** Show extended directory including file length. Don't use this with Pronterface! */
#define SD_RUN_ON_STOP							""
#define SD_STOP_MOTORS_ON_STOP					1

//Distortion settings
#define DISTORTION_CORRECTION					1 //SL
#define DISTORTION_CORRECTION_POINTS			10
#define DISTORTION_LIMIT_TO						0
#define DISTORTION_CORRECTION_R					100
#define DISTORTION_PERMANENT					0 // SL
#define DISTORTION_UPDATE_FREQUENCY				15
#define DISTORTION_START						0
#define DISTORTION_END							1
#define DISTORTION_USE_OFFSET					1
#define DISTORTION_EXTRAPOLATE_CORNERS			0
#define DISTORTION_XMIN							10
#define DISTORTION_YMIN							10
#define DISTORTION_XMAX							290
#define DISTORTION_YMAX							290

//Z probe settings
#define FEATURE_Z_PROBE							1 //SL
#define Z_PROBE_PIN								Z_MIN_PIN
#define Z_PROBE_PULLUP							1
#define Z_PROBE_ON_HIGH							0
#define Z_PROBE_WAIT_BEFORE_TEST				0
#define Z_PROBE_SPEED							2
#define Z_PROBE_XY_SPEED						1600
#define Z_PROBE_SWITCHING_DISTANCE				1
#define Z_PROBE_REPETITIONS						1
#define Z_PROBE_USE_MEDIAN						0
#define Z_PROBE_HEIGHT							6.1
#define Z_PROBE_DELAY							0
#define Z_PROBE_START_SCRIPT					""
#define Z_PROBE_FINISHED_SCRIPT					""
#define Z_PROBE_RUN_AFTER_EVERY_PROBE			""
#define Z_PROBE_X1								30
#define Z_PROBE_Y1								30
#define Z_PROBE_X2								275
#define Z_PROBE_Y2								30
#define Z_PROBE_X3								30
#define Z_PROBE_Y3								275

// ################# XYZA movements ###################
#define STEPPER_INACTIVE_TIME					0
#define MAX_INACTIVE_TIME						0L
#define X_ENABLE_ON								0
#define Y_ENABLE_ON								0
#define Z_ENABLE_ON								0
#define A_ENABLE_ON								0
#define DISABLE_X								0
#define DISABLE_Y								0
#define DISABLE_Z								0
#define DISABLE_A								0
#define INVERT_X_DIR							0
#define INVERT_X2_DIR							0
#define INVERT_Y_DIR							0
#define INVERT_Y2_DIR							0
#define INVERT_Z_DIR							0
#define INVERT_Z2_DIR							0
#define INVERT_A_DIR							0
#define X_HOME_DIR								-1
#define Y_HOME_DIR								-1
#define Z_HOME_DIR								1
#define X_MAX_LENGTH							305
#define Y_MAX_LENGTH							305
#define Z_MAX_LENGTH							290
#define X_MIN_POS								0
#define Y_MIN_POS								0
#define Z_MIN_POS								0
#define MOVE_X_WHEN_HOMED						0
#define MOVE_Y_WHEN_HOMED						0
#define MOVE_Z_WHEN_HOMED						0
#define XAXIS_STEPS_PER_MM						800
#define YAXIS_STEPS_PER_MM						800
#define ZAXIS_STEPS_PER_MM						800
#define AAXIS_STEPS_PER_MM						40.269
#define MAX_FEEDRATE_X							40
#define MAX_FEEDRATE_Y							40
#define MAX_FEEDRATE_Z							20
#define MAX_FEEDRATE_A							30
#define HOMING_FEEDRATE_X						20
#define HOMING_FEEDRATE_Y						20
#define HOMING_FEEDRATE_Z						10
#define HOMING_ORDER							HOME_ORDER_ZXY
#define ZHOME_PRE_RAISE							0
#define ZHOME_PRE_RAISE_DISTANCE				10
#define ZHOME_X_POS								999999
#define ZHOME_Y_POS								999999
#define RAMP_ACCELERATION						1
#define QUICK_STEP								0
#define STEPPER_HIGH_DELAY						0
#define STEP_DOUBLER_FREQUENCY					4000
#define MAX_STEPS_PER_CALL						8
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X	150
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y	150
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z	75
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_A	75
#define MAX_XJERK								4
#define MAX_YJERK								4
#define MAX_ZJERK								2
#define MAX_AJERK								2
#define MACHINELINE_CACHE_SIZE					32
#define ALWAYS_SPLIT_LINES						0
#define SEGMENT_SIZE							5
#define MOVE_CACHE_LOW							8
#define LOW_TICKS_PER_MOVE						250000

//Backlash settings
#define ENABLE_BACKLASH_COMPENSATION			0
#define X_BACKLASH								0
#define Y_BACKLASH								0
#define Z_BACKLASH								0
#define A_BACKLASH								0

// ################ Endstop configuration #####################
#define MULTI_ZENDSTOP_HOMING					0
#define ENDSTOP_PULLUP_X_MIN					true
#define ENDSTOP_X_MIN_INVERTING					true
#define MIN_HARDWARE_ENDSTOP_X					true
#define ENDSTOP_PULLUP_Y_MIN					true
#define ENDSTOP_Y_MIN_INVERTING					true
#define MIN_HARDWARE_ENDSTOP_Y					true
#define ENDSTOP_PULLUP_Z_MIN					false
#define ENDSTOP_Z_MIN_INVERTING					false
#define MIN_HARDWARE_ENDSTOP_Z					false
#define ENDSTOP_PULLUP_Z2_MINMAX				false
#define ENDSTOP_Z2_MINMAX_INVERTING				false
#define MINMAX_HARDWARE_ENDSTOP_Z2				false
#define ENDSTOP_PULLUP_X_MAX					false
#define ENDSTOP_X_MAX_INVERTING					false
#define MAX_HARDWARE_ENDSTOP_X					false
#define ENDSTOP_PULLUP_Y_MAX					false
#define ENDSTOP_Y_MAX_INVERTING					false
#define MAX_HARDWARE_ENDSTOP_Y					false
#define ENDSTOP_PULLUP_Z_MAX					true
#define ENDSTOP_Z_MAX_INVERTING					true
#define MAX_HARDWARE_ENDSTOP_Z					true
#define ENDSTOP_PULLUP_X2_MIN					false
#define ENDSTOP_PULLUP_Y2_MIN					false
#define ENDSTOP_PULLUP_Z2_MINMAX				false
#define ENDSTOP_PULLUP_X2_MAX					false
#define ENDSTOP_PULLUP_Y2_MAX					false
#define ENDSTOP_X2_MIN_INVERTING				false
#define ENDSTOP_Y2_MIN_INVERTING				false
#define ENDSTOP_X2_MAX_INVERTING				false
#define ENDSTOP_Y2_MAX_INVERTING				false
#define MIN_HARDWARE_ENDSTOP_X2					false
#define MIN_HARDWARE_ENDSTOP_Y2					false
#define MAX_HARDWARE_ENDSTOP_X2					false
#define MAX_HARDWARE_ENDSTOP_Y2					false
#define max_software_endstop_r					false
#define min_software_endstop_x					false
#define min_software_endstop_y					false
#define min_software_endstop_z					true
#define max_software_endstop_x					true
#define max_software_endstop_y					true
#define max_software_endstop_z					false
#define ENDSTOP_X_BACK_MOVE						2
#define ENDSTOP_Y_BACK_MOVE						2
#define ENDSTOP_Z_BACK_MOVE						2
#define ENDSTOP_X_RETEST_REDUCTION_FACTOR		3
#define ENDSTOP_Y_RETEST_REDUCTION_FACTOR		3
#define ENDSTOP_Z_RETEST_REDUCTION_FACTOR		2
#define ENDSTOP_X_BACK_ON_HOME					0
#define ENDSTOP_Y_BACK_ON_HOME					0
#define ENDSTOP_Z_BACK_ON_HOME					0
#define ALWAYS_CHECK_ENDSTOPS					0

//Stepper Configuration
#define X_STEP_PIN								40
#define X_DIR_PIN								38
#define X_ENABLE_PIN							42
#define X_MIN_PIN								22
#define X_MAX_PIN								4

#define Y_STEP_PIN								34
#define Y_DIR_PIN								32
#define Y_ENABLE_PIN							36
#define Y_MIN_PIN								2
#define Y_MAX_PIN								5

#define Z_STEP_PIN								64
#define Z_DIR_PIN								63
#define Z_ENABLE_PIN							69
#define Z_MIN_PIN								3
#define Z_MAX_PIN								6

#define A_STEP_PIN								39
#define A_DIR_PIN								37
#define A_ENABLE_PIN							41

#define FEATURE_TWO_XSTEPPER					0
#define X2_MIN_PIN								-1
#define X2_MAX_PIN								-1
#define X2_STEP_PIN								-1
#define X2_DIR_PIN								-1
#define X2_ENABLE_PIN							-1

#define FEATURE_TWO_YSTEPPER					0
#define Y2_MIN_PIN								-1
#define Y2_MAX_PIN								-1
#define Y2_STEP_PIN								-1
#define Y2_DIR_PIN								-1
#define Y2_ENABLE_PIN							-1

#define FEATURE_TWO_ZSTEPPER					1
#define Z2_STEP_PIN								55
#define Z2_DIR_PIN								54
#define Z2_ENABLE_PIN							59
#define Z2_MINMAX_PIN							-1

//TMC support
#define TMC_DRIVERS								1

#define TMC_X_TYPE								TMC_5160
#define TMC_X_CS								49
#define	TMC_X_RSENSE							0.075
#define TMC_X_INTPOL							1
#define TMC_X_RMS								1900
#define TMC_X_HOLD								0.3
#define TMC_X_HOLDDELAY							1
#define TMC_X_TPWRDOWN							2
#define TMC_X_HSTART							4  //4
#define TMC_X_HEND								6  //0
#define TMC_X_TOFF								5  //3
#define TMC_X_TBL								2
#define TMC_X_TPFD								0
#define TMC_X_PWM_FREQ							0
#define TMC_X_TPWMTHRS							0
#define TMC_X_TCOOLTHRS							0
#define TMC_X_THIGHTHRS							0
#define TMC_X_SEMIN								0
#define TMC_X_SEMAX								0
#define TMC_X_SGT								0
#define TMC_X_S2VS								12
#define TMC_X_S2G								12
#define TMC_X_SFILTER							2
#define TMC_X_MICROSTEP							16
#define TMC_X_PWM_GRAD							45
#define TMC_X_PWM_OFS							65
#define TMC_X_PWM_LIM							12
#define TMC_X_MODE								TMC_SPREAD

#define TMC_Y_TYPE								TMC_5160
#define TMC_Y_CS								47
#define	TMC_Y_RSENSE							0.075
#define TMC_Y_INTPOL							1
#define TMC_Y_RMS								1900
#define TMC_Y_HOLD								0.3
#define TMC_Y_HOLDDELAY							1
#define TMC_Y_TPWRDOWN							2
#define TMC_Y_HSTART							4
#define TMC_Y_HEND								6
#define TMC_Y_TOFF								5
#define TMC_Y_TBL								2
#define TMC_Y_TPFD								0
#define TMC_Y_PWM_FREQ							0
#define TMC_Y_TPWMTHRS							0
#define TMC_Y_TCOOLTHRS							0
#define TMC_Y_THIGHTHRS							0
#define TMC_Y_SEMIN								0
#define TMC_Y_SEMAX								0
#define TMC_Y_SGT								0
#define TMC_Y_S2VS								12
#define TMC_Y_S2G								12
#define TMC_Y_SFILTER							2
#define TMC_Y_MICROSTEP							16
#define TMC_Y_PWM_GRAD							45
#define TMC_Y_PWM_OFS							65
#define TMC_Y_PWM_LIM							12
#define TMC_Y_MODE								TMC_SPREAD

#define TMC_Z_TYPE								TMC_5160
#define TMC_Z_CS								45
#define	TMC_Z_RSENSE							0.075
#define TMC_Z_INTPOL							1
#define TMC_Z_RMS								2200
#define TMC_Z_HOLD								0.3
#define TMC_Z_HOLDDELAY							1
#define TMC_Z_TPWRDOWN							0
#define TMC_Z_HSTART							4
#define TMC_Z_HEND								6
#define TMC_Z_TOFF								5
#define TMC_Z_TBL								2
#define TMC_Z_TPFD								0
#define TMC_Z_PWM_FREQ							0
#define TMC_Z_TPWMTHRS							0
#define TMC_Z_TCOOLTHRS							0
#define TMC_Z_THIGHTHRS							0
#define TMC_Z_SEMIN								0
#define TMC_Z_SEMAX								0
#define TMC_Z_SGT								0
#define TMC_Z_S2VS								12
#define TMC_Z_S2G								12
#define TMC_Z_SFILTER							2
#define TMC_Z_MICROSTEP							16
#define TMC_Z_PWM_GRAD							45
#define TMC_Z_PWM_OFS							80
#define TMC_Z_PWM_LIM							12
#define TMC_Z_MODE								TMC_SPREAD

#define TMC_2_TYPE								TMC_5160
#define TMC_2_CS								43
#define	TMC_2_RSENSE							0.075
#define TMC_2_INTPOL							1
#define TMC_2_RMS								2200
#define TMC_2_HOLD								0.3
#define TMC_2_HOLDDELAY							1
#define TMC_2_TPWRDOWN							0
#define TMC_2_HSTART							4
#define TMC_2_HEND								6
#define TMC_2_TOFF								5
#define TMC_2_TBL								2
#define TMC_2_TPFD								0
#define TMC_2_PWM_FREQ							0
#define TMC_2_TPWMTHRS							0
#define TMC_2_TCOOLTHRS							0
#define TMC_2_THIGHTHRS							0
#define TMC_2_SEMIN								0
#define TMC_2_SEMAX								0
#define TMC_2_SGT								0
#define TMC_2_S2VS								12
#define TMC_2_S2G								12
#define TMC_2_SFILTER							2
#define TMC_2_MICROSTEP							16
#define TMC_2_PWM_GRAD							45
#define TMC_2_PWM_OFS							80
#define TMC_2_PWM_LIM							12
#define TMC_2_MODE								TMC_SPREAD


//Poorly written stepper driver support
#define NUM_MOTOR_DRIVERS						0

//SPI pins
#define SCK_PIN									52
#define MISO_PIN								50
#define MOSI_PIN								51

//Pins that cannot be changed with M42
#define SENSITIVE_PINS {0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, \
		Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN,\
		Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, PS_ON_PIN, SDSS }

#endif
