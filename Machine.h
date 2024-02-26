/**

Coordinate system transformations:

Level 1: G-code => Coordinates like send via g-codes.

Level 2: Real coordinates => Coordinates corrected by coordinate shift via G92
         currentPosition and lastCmdPos are from this level.
Level 3: Transformed and shifter => Include extruder offset and bed rotation.
         These variables are only stored temporary.

Level 4: Step position => Level 3 converted into steps for motor position
        currentPositionSteps and destinationPositionSteps are from this level.

Level 5: Nonlinear motor step position, only for nonlinear drive systems
         destinationDeltaSteps


*/

#ifndef MACHINE_H_INCLUDED
#define MACHINE_H_INCLUDED

#if TMC_DRIVERS
#include <TMCStepper.h>
#endif

//TMC Driver modes
#define TMC_STEALTH	1
#define TMC_SPREAD	2

//TMC Driver types
#define TMC_5160	1


#if defined(AUTOMATIC_POWERUP) && AUTOMATIC_POWERUP && PS_ON_PIN > -1
#define ENSURE_POWER {Machine::enablePowerIfNeeded();}
#else
#undef AUTOMATIC_POWERUP
#define AUTOMATIC_POWERUP 0
#define ENSURE_POWER {}
#endif

union floatLong {
    float f;
    uint32_t l;
#ifdef SUPPORT_64_BIT_MATH
    uint64_t L;
#endif
};

#define MACHINE_FLAG0_STEPPER_DISABLED      1
#define MACHINE_FLAG0_FORCE_CHECKSUM        2
#define MACHINE_FLAG0_MANUAL_MOVE_MODE      4
#define MACHINE_FLAG0_ZPROBEING             8
#define MACHINE_FLAG0_PAUSED         		16
#define MACHINE_FLAG0_AUTOMOUNT             32
#define MACHINE_FLAG0_ANIMATION             64
#define MACHINE_FLAG0_ALLKILLED             128
#define MACHINE_FLAG1_HOMED_ALL             1
#define MACHINE_FLAG1_X_HOMED               2
#define MACHINE_FLAG1_Y_HOMED               4
#define MACHINE_FLAG1_Z_HOMED               8
#define MACHINE_FLAG1_NO_DESTINATION_CHECK  16
#define MACHINE_FLAG1_POWER_ON              32
#define MACHINE_FLAG1_HOMING                64
#define MACHINE_FLAG1_IGNORE_M106_COMMAND   128


// define an integer number of steps more than large enough to get to end stop from anywhere
#define HOME_DISTANCE_STEPS (Machine::zMaxSteps-Machine::axisMinSteps[Z_AXIS]+1000)
#define HOME_DISTANCE_MM (HOME_DISTANCE_STEPS * invAxisStepsPerMM[Z_AXIS])

#include "Distortion.h"

#include "Endstops.h"

/**
The Printer class is the main class for the control of the 3d printer. Here all
movement related key variables are stored like positions, accelerations.

## Coordinates

The firmware works with 4 different coordinate systems and understanding the
dependencies between them is crucial to a good understanding on how positions
are handled.

### Real world floating coordinates (RWC)

These coordinates are the real floating positions with any offsets subtracted,
which might be set with G92. This is used to show coordinates or for computations
based on real positions. Any correction coming from rotation or distortion is
not included in these coordinates. currentPosition and lastCmdPos use this coordinate
system.

When these coordinates need to be used for computation, the value of offsetX, offsetY and offsetZ
is always added. These are the offsets of the currently active tool to virtual tool center
(normally first extruder).

### Rotated floating coordinates (ROTC)

If auto leveling is active, printing to the official coordinates is not possible. We have to assume
that the bed is somehow rotated against the Cartesian mechanics from the printer. Applying
_transformToPrinter_ to the real world coordinates, rotates them around the origin to
be equal to the rotated bed. _transformFromPrinter_ would apply the opposite transformation.

### Cartesian motor position coordinates (CMC)

The position of motors is stored as steps from 0. The reason for this is that it is crucial that
no rounding errors ever cause addition of any steps. These positions are simply computed by
multiplying the ROTC coordinates with the axisStepsPerMM.

If distortion correction is enabled, there is an additional factor for the z position that
gets added: _zCorrectionStepsIncluded_ This value is recalculated by every move added to
reflect the distortion at any given xyz position.

### Nonlinear motor position coordinates (NMC)

In case of a nonlinear mechanic like a delta printer, the CMC does not define the motor positions.
An additional transformation converts the CMC coordinates into NMC.

### Transformations from RWC to CMC

Given:
- Target position for tool: x_rwc, y_rwc, z_rwc
- Tool offsets: offsetX, offsetY, offsetZ
- Offset from bed leveling: offsetZ2

Step 1: Convert to ROTC

    transformToPrinter(x_rwc + Machine::offsetX, y_rwc + Machine::offsetY, z_rwc +  Machine::offsetZ, x_rotc, y_rotc, z_rotc);
    z_rotc += offsetZ2

Step 2: Compute CMC

    x_cmc = static_cast<int32_t>(floor(x_rotc * axisStepsPerMM[X_AXIS] + 0.5f));
    y_cmc = static_cast<int32_t>(floor(y_rotc * axisStepsPerMM[Y_AXIS] + 0.5f));
    z_cmc = static_cast<int32_t>(floor(z_rotc * axisStepsPerMM[Z_AXIS] + 0.5f));

### Transformation from CMC to RWC

Note: _zCorrectionStepsIncluded_ comes from distortion correction and gets set when a move is queued by the queuing function.
Therefore it is not visible in the inverse transformation above. When transforming back, consider if the value was set or not!

Step 1: Convert to ROTC

    x_rotc = static_cast<float>(x_cmc) * invAxisStepsPerMM[X_AXIS];
	y_rotc = static_cast<float>(y_cmc) * invAxisStepsPerMM[Y_AXIS];

Step 2: Convert to RWC

    transformFromPrinter(x_rotc, y_rotc, z_rotc,x_rwc, y_rwc, z_rwc);
	x_rwc -= Machine::offsetX; // Offset from active extruder or z probe
    y_rwc -= Machine::offsetY;
    z_rwc -= Machine::offsetZ;
*/
class Machine {
    static uint8_t debugLevel;
public:
	static float axisStepsPerMM[]; ///< Resolution of each axis in steps per mm.
    static float invAxisStepsPerMM[]; ///< 1/axisStepsPerMM for faster computation.
    static float maxFeedrate[]; ///< Maximum feedrate in mm/s per axis.
    static float homingFeedrate[]; // Feedrate in mm/s for homing.
	static float maxAccelerationMMPerSquareSecond[];
	static unsigned long maxAccelerationStepsPerSquareSecond[];
	static uint8_t relativeCoordinateMode;    ///< Determines absolute (false) or relative Coordinates (true).

	static uint8_t unitIsInches;
    static uint8_t fanSpeed; // Last fan speed set with M106/M107
    static fast8_t stepsTillNextCalc;
    static fast8_t stepsSinceLastCalc;
	static uint8_t flag0, flag1;
    static uint32_t interval;    ///< Last step duration in ticks.
    static uint32_t timer;              ///< used for acceleration/deceleration timing
    static uint32_t stepNumber;         ///< Step number in current move.
	static float coordinateOffset[Z_AXIS_ARRAY];
	static int32_t currentPositionSteps[A_AXIS_ARRAY];     ///< Position in steps from origin.
	static float currentPosition[A_AXIS_ARRAY]; ///< Position in global coordinates
	static float lastCmdPos[A_AXIS_ARRAY]; ///< Last coordinates (global coordinates) send by g-codes
	static int32_t destinationSteps[A_AXIS_ARRAY];         ///< Target position in steps.
	static int32_t zCorrectionStepsIncluded;
#if FEATURE_Z_PROBE || MAX_HARDWARE_ENDSTOP_Z
    static int32_t stepsRemainingAtZHit;
#endif
#if DISTORTION_CORRECTION
	static int16_t distortionXMIN;
	static int16_t distortionXMAX;
	static int16_t distortionYMIN;
	static int16_t distortionYMAX;
	static uint8_t distortionPoints;
	static float distortionStart;
	static float distortionEnd;
	static uint8_t distortionUseOffset;
#endif
	static int32_t axisMaxSteps[Z_AXIS_ARRAY];         ///< For software endstops, limit of move in positive direction.
	static int32_t axisMinSteps[Z_AXIS_ARRAY];         ///< For software endstops, limit of move in negative direction.
	static float axisLength[Z_AXIS_ARRAY];
	static float axisMin[Z_AXIS_ARRAY];
    static float feedrate;                   ///< Last requested feedrate.
    static int feedrateMultiply;             ///< Multiplier for feedrate in percent (factor 1 = 100)
	static float maxJerk[A_AXIS_ARRAY];      ///< Maximum allowed jerk in mm/s
	static uint8_t interruptEvent;           ///< Event generated in interrupts that should/could be handled in main thread
	static speed_t vMaxReached;               ///< Maximum reached speed
#if ENABLE_BACKLASH_COMPENSATION
	static float backlash[A_AXIS_ARRAY];
    static uint8_t backlashDir;
#endif
#if MULTI_XENDSTOP_HOMING
    static fast8_t multiXHomeFlags;  // 1 = move X0, 2 = move X1
#endif
#if MULTI_YENDSTOP_HOMING
    static fast8_t multiYHomeFlags;  // 1 = move Y0, 2 = move Y1
#endif
#if MULTI_ZENDSTOP_HOMING
	static fast8_t multiZHomeFlags;  // 1 = move Z0, 2 = move Z1
#endif
	static float memoryPosition[A_AXIS_ARRAY];
	static float memoryF;
#ifdef DEBUG_SEGMENT_LENGTH
    static float maxRealSegmentLength;
#endif
#ifdef DEBUG_REAL_JERK
    static float maxRealJerk;
#endif
#if TMC_DRIVERS
#if TMC_X_TYPE==TMC_5160
	static TMC5160Stepper tmcStepperX;
#endif
#if TMC_Y_TYPE==TMC_5160
	static TMC5160Stepper tmcStepperY;
#endif
#if TMC_Z_TYPE==TMC_5160
	static TMC5160Stepper tmcStepperZ;
#endif
#if TMC_2_TYPE==TMC_5160
	static TMC5160Stepper tmcStepper2;
#endif
#endif
#if defined(PAUSE_PIN) && PAUSE_PIN>-1
	static uint8_t isPaused;
#if PAUSE_STEPS > 120
	static int16_t pauseSteps;
#else
	static int8_t pauseSteps;
#endif
#endif //PAUSE
#if SPEED_DIAL && SPEED_DIAL_PIN > -1
    static uint8_t speed_dial;
#endif

    static void handleInterruptEvent();

    static INLINE void setInterruptEvent(uint8_t evt, bool highPriority) {
        if(highPriority || interruptEvent == 0)
            interruptEvent = evt;
    }

    static void setDebugLevel(uint8_t newLevel);
    static void toggleEcho();
    static void toggleInfo();
	static void toggleErrors();
    static void toggleCommunication();
    static void toggleNoMoves();
    static void toggleEndStop();
    static INLINE uint8_t getDebugLevel() {
        return debugLevel;
    }
    static INLINE bool debugEcho() {
        return ((debugLevel & 1) != 0);
    }

    static INLINE bool debugInfo() {
        return ((debugLevel & 2) != 0);
    }

    static INLINE bool debugErrors() {
        return ((debugLevel & 4) != 0);
    }

    static INLINE bool debugCommunication() {
        return ((debugLevel & 16) != 0);
    }

    static INLINE bool debugNoMoves() {
        return ((debugLevel & 32) != 0);
    }

    static INLINE bool debugEndStop() {
        return ((debugLevel & 64) != 0);
    }

    static INLINE bool debugFlag(uint8_t flags) {
        return (debugLevel & flags);
    }

    static INLINE void debugSet(uint8_t flags) {
        setDebugLevel(debugLevel | flags);
    }

    static INLINE void debugReset(uint8_t flags) {
        setDebugLevel(debugLevel & ~flags);
    }
#if AUTOMATIC_POWERUP
    static void enablePowerIfNeeded();
#endif
    /** Sets the pwm for the fan speed. Gets called by motion control or Commands::setFanSpeed. */
    static void setFanSpeedDirectly(uint8_t speed);
    /** Sets the pwm for the fan 2 speed. Gets called by motion control or Commands::setFan2Speed. */
    static void setFan2SpeedDirectly(uint8_t speed);
    /** \brief Disable stepper motor for x direction. */
    static INLINE void disableXStepper() {
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN, !X_ENABLE_ON);
#endif
#if FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)
        WRITE(X2_ENABLE_PIN, !X_ENABLE_ON);
#endif
	}
    /** \brief Disable stepper motor for y direction. */
    static INLINE void disableYStepper() {
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN, !Y_ENABLE_ON);
#endif
#if FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)
        WRITE(Y2_ENABLE_PIN, !Y_ENABLE_ON);
#endif
    }
    /** \brief Disable stepper motor for z direction. */
    static INLINE void disableZStepper() {
#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN, !Z_ENABLE_ON);
#endif
#if FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)
        WRITE(Z2_ENABLE_PIN, !Z_ENABLE_ON);
#endif
    }
    /** \brief Disable stepper motor for y direction. */
	static INLINE void disableAStepper() {
#if (Y_ENABLE_PIN > -1)
		WRITE(A_ENABLE_PIN, !A_ENABLE_ON);
#endif
#if FEATURE_TWO_ASTEPPER && (A2_ENABLE_PIN > -1)
		WRITE(A2_ENABLE_PIN, !A_ENABLE_ON);
#endif
	}
    /** \brief Enable stepper motor for x direction. */
    static INLINE void  enableXStepper() {
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN, X_ENABLE_ON);
#endif
#if FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)
        WRITE(X2_ENABLE_PIN, X_ENABLE_ON);
#endif
    }

    /** \brief Enable stepper motor for y direction. */
    static INLINE void  enableYStepper() {
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN, Y_ENABLE_ON);
#endif
#if FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)
        WRITE(Y2_ENABLE_PIN, Y_ENABLE_ON);
#endif
    }
    /** \brief Enable stepper motor for z direction. */
    static INLINE void  enableZStepper() {
#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);
#endif
#if FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)
        WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON);
#endif
    }
	/** \brief Enable stepper motor for z direction. */
	static INLINE void  enableAStepper() {
#if (A_ENABLE_PIN > -1)
		WRITE(A_ENABLE_PIN, A_ENABLE_ON);
#endif
	}
    static INLINE void setXDirection(bool positive) {
        if(positive) {
            WRITE(X_DIR_PIN, !INVERT_X_DIR);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_DIR_PIN, !INVERT_X2_DIR);
#endif
        } else {
            WRITE(X_DIR_PIN, INVERT_X_DIR);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_DIR_PIN, INVERT_X2_DIR);
#endif
        }
    }

    static INLINE void setYDirection(bool positive) {
        if(positive) {
            WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_DIR_PIN, !INVERT_Y2_DIR);
#endif
        } else {
            WRITE(Y_DIR_PIN, INVERT_Y_DIR);
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_DIR_PIN, INVERT_Y2_DIR);
#endif
        }
    }
    static INLINE void setZDirection(bool positive) {
        if(positive) {
			WRITE(Z_DIR_PIN, !INVERT_Z_DIR);
#if FEATURE_TWO_ZSTEPPER
			WRITE(Z2_DIR_PIN, !INVERT_Z2_DIR);
#endif
        } else {
			WRITE(Z_DIR_PIN, INVERT_Z_DIR);
#if FEATURE_TWO_ZSTEPPER
			WRITE(Z2_DIR_PIN, INVERT_Z2_DIR);
#endif
        }
	}
	static INLINE void setADirection(bool positive) {
        if(positive) {
			WRITE(A_DIR_PIN, !INVERT_Z_DIR);
        } else {
			WRITE(A_DIR_PIN, INVERT_Z_DIR);
        }
	}

    static INLINE bool getXDirection() {
        return((READ(X_DIR_PIN) != 0) ^ INVERT_X_DIR);
	}

	static INLINE bool getYDirection() {
		return((READ(Y_DIR_PIN) != 0) ^ INVERT_Y_DIR);
	}

	static INLINE bool getZDirection() {
		return ((READ(Z_DIR_PIN) != 0) ^ INVERT_Z_DIR);
	}

	static INLINE bool getADirection() {
		return ((READ(A_DIR_PIN) != 0) ^ INVERT_A_DIR);
	}

    static INLINE uint8_t isHomedAll() {
        return flag1 & MACHINE_FLAG1_HOMED_ALL;
    }

    static INLINE void unsetHomedAll() {
        flag1 &= ~MACHINE_FLAG1_HOMED_ALL;
		flag1 &= ~(MACHINE_FLAG1_X_HOMED | MACHINE_FLAG1_Y_HOMED | MACHINE_FLAG1_Z_HOMED);
    }

    static INLINE void updateHomedAll() {
        bool b = isXHomed() && isYHomed() && isZHomed();
        flag1 = (b ? flag1 | MACHINE_FLAG1_HOMED_ALL : flag1 & ~MACHINE_FLAG1_HOMED_ALL);
    }

    static INLINE uint8_t isXHomed() {
		return flag1 & MACHINE_FLAG1_X_HOMED;
    }

    static INLINE void setXHomed(uint8_t b) {
		flag1 = (b ? flag1 | MACHINE_FLAG1_X_HOMED : flag1 & ~MACHINE_FLAG1_X_HOMED);
		updateHomedAll();
    }

    static INLINE uint8_t isYHomed() {
		return flag1 & MACHINE_FLAG1_Y_HOMED;
    }

    static INLINE void setYHomed(uint8_t b) {
		flag1 = (b ? flag1 | MACHINE_FLAG1_Y_HOMED : flag1 & ~MACHINE_FLAG1_Y_HOMED);
        updateHomedAll();
    }

    static INLINE uint8_t isZHomed() {
		return flag1 & MACHINE_FLAG1_Z_HOMED;
    }

    static INLINE void setZHomed(uint8_t b) {
        flag1 = (b ? flag1 | MACHINE_FLAG1_Z_HOMED : flag1 & ~MACHINE_FLAG1_Z_HOMED);
        updateHomedAll();
    }

    static INLINE uint8_t isAllKilled() {
		return flag0 & MACHINE_FLAG0_ALLKILLED;
    }

	static INLINE void setAllKilled(uint8_t b) {
		flag0 = (b ? flag0 | MACHINE_FLAG0_ALLKILLED : flag0 & ~MACHINE_FLAG0_ALLKILLED);
    }

    static INLINE uint8_t isAutomount() {
		return flag0 & MACHINE_FLAG0_AUTOMOUNT;
    }

    static INLINE void setAutomount(uint8_t b) {
		flag0 = (b ? flag0 | MACHINE_FLAG0_AUTOMOUNT : flag1 & ~MACHINE_FLAG0_AUTOMOUNT);
    }

    static INLINE uint8_t isAnimation() {
		return flag1 & MACHINE_FLAG0_ANIMATION;
    }

    static INLINE void setAnimation(uint8_t b) {
		flag0 = (b ? flag0 | MACHINE_FLAG0_ANIMATION : flag0 & ~MACHINE_FLAG0_ANIMATION);
    }

    static INLINE uint8_t isNoDestinationCheck() {
		return flag1 & MACHINE_FLAG1_NO_DESTINATION_CHECK;
    }

    static INLINE void setNoDestinationCheck(uint8_t b) {
        flag1 = (b ? flag1 | MACHINE_FLAG1_NO_DESTINATION_CHECK : flag1 & ~MACHINE_FLAG1_NO_DESTINATION_CHECK);
    }

    static INLINE uint8_t isPowerOn() {
        return flag1 & MACHINE_FLAG1_POWER_ON;
    }

    static INLINE void setPowerOn(uint8_t b) {
        flag1 = (b ? flag1 | MACHINE_FLAG1_POWER_ON : flag1 & ~MACHINE_FLAG1_POWER_ON);
	}

    static INLINE uint8_t isHoming() {
		return flag1 & MACHINE_FLAG1_HOMING;
    }

    static INLINE void setHoming(uint8_t b) {
        flag1 = (b ? flag1 | MACHINE_FLAG1_HOMING : flag1 & ~MACHINE_FLAG1_HOMING);
	}

    static INLINE void toggleAnimation() {
        setAnimation(!isAnimation());
    }
    static INLINE float convertToMM(float x) {
        return (unitIsInches ? x * 25.4 : x);
    }
	static INLINE bool areAllSteppersDisabled() {
		return flag0 & MACHINE_FLAG0_STEPPER_DISABLED;
    }
	static INLINE void setAllSteppersDiabled() {
		flag0 |= MACHINE_FLAG0_STEPPER_DISABLED;
#if FAN_BOARD_PIN > -1
		pwm_pos[PWM_BOARD_FAN] = BOARD_FAN_MIN_SPEED;
#endif // FAN_BOARD_PIN
    }
	static INLINE void unsetAllSteppersDisabled() {
        flag0 &= ~MACHINE_FLAG0_STEPPER_DISABLED;
#if FAN_BOARD_PIN > -1
		pwm_pos[PWM_BOARD_FAN] = BOARD_FAN_SPEED;
#endif // FAN_BOARD_PIN
	}
    static INLINE bool isManualMoveMode() {
        return (flag0 & MACHINE_FLAG0_MANUAL_MOVE_MODE);
    }
    static INLINE void setManualMoveMode(bool on) {
        flag0 = (on ? flag0 | MACHINE_FLAG0_MANUAL_MOVE_MODE : flag0 & ~MACHINE_FLAG0_MANUAL_MOVE_MODE);
	}
    static INLINE void setZProbingActive(bool on) {
        flag0 = (on ? flag0 | MACHINE_FLAG0_ZPROBEING : flag0 & ~MACHINE_FLAG0_ZPROBEING);
    }
    static INLINE bool isZProbingActive() {
        return (flag0 & MACHINE_FLAG0_ZPROBEING);
	}
	static INLINE void startXStep() {
#if MULTI_XENDSTOP_HOMING
        if(Machine::multiXHomeFlags & 1) {
            WRITE(X_STEP_PIN, START_STEP_WITH_HIGH);
        }
#if FEATURE_TWO_XSTEPPER
        if(Machine::multiXHomeFlags & 2) {
            WRITE(X2_STEP_PIN, START_STEP_WITH_HIGH);
        }
#endif
#else // MULTI_XENDSTOP_HOMING
        WRITE(X_STEP_PIN, START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER
        WRITE(X2_STEP_PIN, START_STEP_WITH_HIGH);
#endif
#endif
    }
    static INLINE void startYStep() {
#if MULTI_YENDSTOP_HOMING
        if(Machine::multiYHomeFlags & 1) {
            WRITE(Y_STEP_PIN, START_STEP_WITH_HIGH);
        }
#if FEATURE_TWO_YSTEPPER
        if(Machine::multiYHomeFlags & 2) {
            WRITE(Y2_STEP_PIN, START_STEP_WITH_HIGH);
        }
#endif
#else
        WRITE(Y_STEP_PIN, START_STEP_WITH_HIGH);
#if FEATURE_TWO_YSTEPPER
        WRITE(Y2_STEP_PIN, START_STEP_WITH_HIGH);
#endif
#endif
    }
    static INLINE void startZStep() {
#if MULTI_ZENDSTOP_HOMING
        if(Machine::multiZHomeFlags & 1) {
            WRITE(Z_STEP_PIN, START_STEP_WITH_HIGH);
        }
#if FEATURE_TWO_ZSTEPPER
        if(Machine::multiZHomeFlags & 2) {
            WRITE(Z2_STEP_PIN, START_STEP_WITH_HIGH);
        }
#endif
#else
        WRITE(Z_STEP_PIN, START_STEP_WITH_HIGH);
#if FEATURE_TWO_ZSTEPPER
        WRITE(Z2_STEP_PIN, START_STEP_WITH_HIGH);
#endif
#endif
	}
	static INLINE void startAStep() {
		WRITE(A_STEP_PIN, START_STEP_WITH_HIGH);
	}
	static INLINE void endXYZASteps() {
        WRITE(X_STEP_PIN, !START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER
        WRITE(X2_STEP_PIN, !START_STEP_WITH_HIGH);
#endif
        WRITE(Y_STEP_PIN, !START_STEP_WITH_HIGH);
#if FEATURE_TWO_YSTEPPER
        WRITE(Y2_STEP_PIN, !START_STEP_WITH_HIGH);
#endif
        WRITE(Z_STEP_PIN, !START_STEP_WITH_HIGH);
#if FEATURE_TWO_ZSTEPPER
        WRITE(Z2_STEP_PIN, !START_STEP_WITH_HIGH);
#endif
		WRITE(A_STEP_PIN, !START_STEP_WITH_HIGH);
    }
    static INLINE speed_t updateStepsPerTimerCall(speed_t vbase) {
#if MAX_STEPS_PER_CALL >= 8
        if (vbase > STEP_DOUBLER_FREQUENCY * 4) {
            Machine::stepsTillNextCalc = 8;
#if QUICK_STEP
            return vbase >> 3;
#endif
        } else
#endif
#if MAX_STEPS_PER_CALL >= 4
        if(vbase > STEP_DOUBLER_FREQUENCY * 2) {
            Machine::stepsTillNextCalc = 4;
#if QUICK_STEP
            return vbase >> 2;
#endif
        } else
#endif
#if MAX_STEPS_PER_CALL >= 2
        if (vbase > STEP_DOUBLER_FREQUENCY) {
            Machine::stepsTillNextCalc = 2;
#if QUICK_STEP
            return vbase >> 1;
#endif
        } else
#endif
        {
            Machine::stepsTillNextCalc = 1;
        }

        return vbase;
    }
    static INLINE void updateStepsPerTimerCall(speed_t vbase, ticks_t fullInterval) {
#if MAX_STEPS_PER_CALL >= 8
        if (vbase > STEP_DOUBLER_FREQUENCY * 4) {
            Machine::stepsTillNextCalc = 8;
#if QUICK_STEP
            interval = fullInterval << 3;
#else
            interval = fullInterval;
#endif
        } else
#endif
#if MAX_STEPS_PER_CALL >= 4
        if (vbase > STEP_DOUBLER_FREQUENCY * 2) {
            Machine::stepsTillNextCalc = 4;
#if QUICK_STEP
            interval = fullInterval << 2;
#else
            interval = fullInterval;
#endif
        } else
#endif
#if MAX_STEPS_PER_CALL >= 2
        if (vbase > STEP_DOUBLER_FREQUENCY) {
            Machine::stepsTillNextCalc = 2;
#if QUICK_STEP
            interval = fullInterval << 1;
#else
            interval = fullInterval;
#endif
        } else
#endif
        {
            stepsTillNextCalc = 1;
            interval = fullInterval;
        }
    }
    static INLINE void disableAllowedStepper() {
		if(DISABLE_X) disableXStepper();
		if(DISABLE_Y) disableYStepper();
		if(DISABLE_Z) disableZStepper();
		if(DISABLE_A) disableAStepper();
	}
    static INLINE float realXPosition() {
        return currentPosition[X_AXIS];
    }

    static INLINE float realYPosition() {
        return currentPosition[Y_AXIS];
    }

	static INLINE float realZPosition() {
		return currentPosition[Z_AXIS];
	}

	static INLINE float realAPosition() {
		return currentPosition[A_AXIS];
	}
    /** \brief copies currentPosition to parameter. */
	static INLINE void realPosition(float &xp, float &yp, float &zp, float &ap) {
        xp = currentPosition[X_AXIS];
        yp = currentPosition[Y_AXIS];
		zp = currentPosition[Z_AXIS];
		ap = currentPosition[A_AXIS];
    }
	static void updateDerivedParameter();
    /** If we are not homing or destination check being disabled, this will reduce _destinationSteps_ to a
    valid value. In other words this works as software endstop. */
    static void constrainDestinationCoords();
    /** Computes _currentposition_ from _currentPositionSteps_ considering all active transformations. If the _copyLastCmd_ flag is true, the
    result is also copied to _lastCmdPos_ . */
    static void updateCurrentPosition(bool copyLastCmd = false);
    static void updateCurrentPositionSteps();
    /** \brief Sets the destination coordinates to values stored in com.

    Extracts x,y,z,e,f from g-code considering active units. Converted result is stored in currentPosition and lastCmdPos. Converts
    position to destinationSteps including rotation and offsets, excluding distortion correction (which gets added on move queuing).
    \param com g-code with new destination position.
    \return true if it is a move, false if no move results from coordinates.
     */
    static uint8_t setDestinationStepsFromGCode(GCode *com);
    /** \brief Move to position without considering transformations.

    Computes the destinationSteps without rotating but including active offsets!
    The coordinates are in printer coordinates with no G92 offset.

    \param x Target x coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target y coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target z coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target e coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target f feedrate or IGNORE_COORDINATE if it should use latest feedrate.
    \return true if queuing was successful.
    */
    static uint8_t moveTo(float x, float y, float z, float a, float f);
    /** \brief Move to position considering transformations.

    Computes the destinationSteps including rotating and active offsets.
    The coordinates are in printer coordinates with no G92 offset.

    \param x Target x coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target y coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target z coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target e coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target f feedrate or IGNORE_COORDINATE if it should use latest feedrate.
    \param pathOptimize true if path planner should include it in calculation, otherwise default start/end speed is enforced.
    \return true if queuing was successful.
    */
	static uint8_t moveToReal(float x, float y, float z, float a, float f, bool pathOptimize = true);
	static void kill(uint8_t only_steppers);
    static void setup();
    static void defaultLoopActions();
    static void homeAxis(bool xaxis, bool yaxis, bool zaxis); /// Home axis
    static void setOrigin(float xOff, float yOff, float zOff);
    /** \brief Tests if the target position is allowed.

    Tests if the test position lies inside the defined geometry. For Cartesian
    printers this is the defined cube defined by x,y,z min and length. For
    delta printers the cylindrical shape is tested.

    \param x X position in mm.
    \param x Y position in mm.
    \param x Z position in mm.
    \return true if position is valid and can be reached. */
    static bool isPositionAllowed(float x, float y, float z);
    static INLINE int getFanSpeed() {
        return (int)pwm_pos[PWM_FAN1];
    }
    static INLINE int getFan2Speed() {
        return (int)pwm_pos[PWM_FAN2];
    }
#if MAX_HARDWARE_ENDSTOP_Z
    static float runZMaxProbe();
#endif
#if FEATURE_Z_PROBE
    static bool startProbing(bool runScript, bool enforceStartHeight = true);
    static void finishProbing();
	static float runZProbe(bool first, bool last, uint8_t repeat = Z_PROBE_REPETITIONS, bool runStartScript = true, bool enforceStartHeight = true);
	static float runProbe(uint8_t axisDirection, float maxDistance, uint8_t repeat);
	static void measureZProbeHeight(float curHeight);
    static void waitForZProbeStart();
#endif
    // Moved outside FEATURE_Z_PROBE to allow auto-level functional test on
    // system without Z-probe
#if DISTORTION_CORRECTION
	static void measureDistortion(float maxDistance, int repetitions);
	static Distortion distortion;
#endif
	static void SetMemoryPosition();
    static void GoToMemoryPosition(bool x, bool y, bool z, bool a, float feed);

    static void showConfiguration();
    static void setCaseLight(bool on);
    static void reportCaseLightStatus();
    static void homeXAxis();
    static void homeYAxis();
    static void homeZAxis();
#if FEATURE_Z_PROBE
    /** \brief Prepares printer for probing commands.

    Probing can not start under all conditions. This command therefore makes sure,
    a probing command can be executed by:
    - Ensuring all axes are homed.
    - Going to a low z position for fast measuring.
    - Go to a position, where enabling the z-probe is possible without leaving the valid print area.
    */
	static void prepareForProbing();
#endif
#if TMC_DRIVERS
	static void configTMC5160(TMC5160Stepper* driver, uint8_t intpol, uint16_t rms, float hold_mult, uint8_t hold_delay, uint8_t tpower_down, uint8_t hstart, uint8_t hend, uint8_t toff, uint8_t tbl, uint8_t tpfd, uint8_t pwm_freq, uint16_t tpwmthrs, uint16_t tcoolthrs, uint16_t thighthrs, uint8_t semin, uint8_t semax, int8_t sgt, uint8_t s2vs, uint8_t s2g, uint8_t sfilter, uint16_t microsteps, uint8_t pwm_grad, uint8_t pwm_ofs, uint8_t pwm_lim, uint8_t mode);
	static void CheckTMCDrivers();
#endif
};

#endif // MACHINE_H_INCLUDED
