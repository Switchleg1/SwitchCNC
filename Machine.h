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
#define MACHINE_FLAG0_HAS_LINES             2
#define MACHINE_FLAG0_ALLKILLED             4
#define MACHINE_FLAG0_RELATIVE_COORD        8
#define MACHINE_FLAG0_UNIT_IS_INCH          16

#define MACHINE_FLAG1_HOMED_ALL             1
#define MACHINE_FLAG1_X_HOMED               2
#define MACHINE_FLAG1_Y_HOMED               4
#define MACHINE_FLAG1_Z_HOMED               8
#define MACHINE_FLAG1_NO_DESTINATION_CHECK  16
#define MACHINE_FLAG1_POWER_ON              32
#define MACHINE_FLAG1_HOMING                64
#define MACHINE_FLAG1_IGNORE_FAN_COMMAND    128


// define an integer number of steps more than large enough to get to end stop from anywhere
#define HOME_DISTANCE_STEPS (Machine::zMaxSteps - Machine::axisMinSteps[Z_AXIS] + 1000)
#define HOME_DISTANCE_MM (HOME_DISTANCE_STEPS * invAxisStepsPerMM[Z_AXIS])

#include "Endstops.h"
#include "Analog.h"
#include "PWM.h"
#include "Temperature.h"
#include "src/Features/FanControl.h"

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
public:
    static long baudrate;                                       ///< Communication speed rate.
    static millis_t previousMillisCmd;
    static millis_t maxInactiveTime;
    static millis_t stepperInactiveTime;

	static float axisStepsPerMM[];                              ///< Resolution of each axis in steps per mm.
    static float invAxisStepsPerMM[];                           ///< 1/axisStepsPerMM for faster computation.
    static float maxFeedrate[];                                 ///< Maximum feedrate in mm/s per axis.
    static float homingFeedrate[];                              // Feedrate in mm/s for homing.
	static float maxAccelerationMMPerSquareSecond[];
	static unsigned long maxAccelerationStepsPerSquareSecond[];

    static fast8_t stepsTillNextCalc;
    static fast8_t stepsSinceLastCalc;
    static uint8_t mode;
    static uint32_t interval;                                   ///< Last step duration in ticks.
    static uint32_t timer;                                      ///< used for acceleration/deceleration timing
    static uint32_t stepNumber;                                 ///< Step number in current move.
	static float coordinateOffset[Z_AXIS_ARRAY];
	static int32_t currentPositionSteps[A_AXIS_ARRAY];          ///< Position in steps from origin.
	static float currentPosition[A_AXIS_ARRAY];                 ///< Position in global coordinates
	static float lastCmdPos[A_AXIS_ARRAY];                      ///< Last coordinates (global coordinates) send by g-codes
	static int32_t zCorrectionStepsIncluded;
#if Z_PROBE_SUPPORT || MAX_HARDWARE_ENDSTOP_Z
    static int32_t stepsRemainingAtZHit;
#endif
	static int32_t axisMaxSteps[Z_AXIS_ARRAY];                  ///< For software endstops, limit of move in positive direction.
	static int32_t axisMinSteps[Z_AXIS_ARRAY];                  ///< For software endstops, limit of move in negative direction.
	static float axisLength[Z_AXIS_ARRAY];
	static float axisMin[Z_AXIS_ARRAY];
    static float feedrate;                                      ///< Last requested feedrate.
    static int feedrateMultiply;                                ///< Multiplier for feedrate in percent (factor 1 = 100)
	static float maxJerk[A_AXIS_ARRAY];                         ///< Maximum allowed jerk in mm/s
	static speed_t vMaxReached;                                 ///< Maximum reached speed
#if MULTI_XENDSTOP_HOMING
    static fast8_t multiXHomeFlags;                             // 1 = move X0, 2 = move X1
#endif
#if MULTI_YENDSTOP_HOMING
    static fast8_t multiYHomeFlags;                             // 1 = move Y0, 2 = move Y1
#endif
#if MULTI_ZENDSTOP_HOMING
	static fast8_t multiZHomeFlags;                             // 1 = move Z0, 2 = move Z1
#endif
	static float memoryPosition[A_AXIS_ARRAY];
	static float memoryF;
#ifdef DEBUG_REAL_JERK
    static float maxRealJerk;
#endif
#ifdef DEBUG_MACHINE
    static int debugWaitLoop;
#endif
#if ANALOG_INPUTS > 0
    static Analog analog;
#endif
    static PWM pwm;

    static void checkForPeriodicalActions(bool allowNewMoves);
    static void timerInterrupt();
    static void reportPrinterMode();
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
    /** \brief Disable stepper motor for x direction. */
    static INLINE void disableXStepper() {
#if (X_ENABLE_PIN > -1)
        HAL::digitalWrite(X_ENABLE_PIN, !X_ENABLE_ON);
#endif
#if X2_XSTEPPER_SUPPORT && (X2_ENABLE_PIN > -1)
        HAL::digitalWrite(X2_ENABLE_PIN, !X_ENABLE_ON);
#endif
	}
    /** \brief Disable stepper motor for y direction. */
    static INLINE void disableYStepper() {
#if (Y_ENABLE_PIN > -1)
        HAL::digitalWrite(Y_ENABLE_PIN, !Y_ENABLE_ON);
#endif
#if Y2_YSTEPPER_SUPPORT && (Y2_ENABLE_PIN > -1)
        HAL::digitalWrite(Y2_ENABLE_PIN, !Y_ENABLE_ON);
#endif
    }
    /** \brief Disable stepper motor for z direction. */
    static INLINE void disableZStepper() {
#if (Z_ENABLE_PIN > -1)
        HAL::digitalWrite(Z_ENABLE_PIN, !Z_ENABLE_ON);
#endif
#if Z2_ZSTEPPER_SUPPORT && (Z2_ENABLE_PIN > -1)
        HAL::digitalWrite(Z2_ENABLE_PIN, !Z_ENABLE_ON);
#endif
    }
    /** \brief Disable stepper motor for y direction. */
	static INLINE void disableAStepper() {
#if (Y_ENABLE_PIN > -1)
        HAL::digitalWrite(A_ENABLE_PIN, !A_ENABLE_ON);
#endif
#if A2_ASTEPPER_SUPPORT && (A2_ENABLE_PIN > -1)
        HAL::digitalWrite(A2_ENABLE_PIN, !A_ENABLE_ON);
#endif
	}
    /** \brief Enable stepper motor for x direction. */
    static INLINE void  enableXStepper(uint8_t safe = true) {
#if (X_ENABLE_PIN > -1)
        HAL::digitalWrite(X_ENABLE_PIN, X_ENABLE_ON, safe);
#endif
#if X2_XSTEPPER_SUPPORT && (X2_ENABLE_PIN > -1)
        HAL::digitalWrite(X2_ENABLE_PIN, X_ENABLE_ON, safe);
#endif
    }

    /** \brief Enable stepper motor for y direction. */
    static INLINE void  enableYStepper(uint8_t safe = true) {
#if (Y_ENABLE_PIN > -1)
        HAL::digitalWrite(Y_ENABLE_PIN, Y_ENABLE_ON, safe);
#endif
#if Y2_YSTEPPER_SUPPORT && (Y2_ENABLE_PIN > -1)
        HAL::digitalWrite(Y2_ENABLE_PIN, Y_ENABLE_ON, safe);
#endif
    }
    /** \brief Enable stepper motor for z direction. */
    static INLINE void  enableZStepper(uint8_t safe = true) {
#if (Z_ENABLE_PIN > -1)
        HAL::digitalWrite(Z_ENABLE_PIN, Z_ENABLE_ON, safe);
#endif
#if Z2_ZSTEPPER_SUPPORT && (Z2_ENABLE_PIN > -1)
        HAL::digitalWrite(Z2_ENABLE_PIN, Z_ENABLE_ON, safe);
#endif
    }
	/** \brief Enable stepper motor for z direction. */
	static INLINE void  enableAStepper(uint8_t safe = true) {
#if (A_ENABLE_PIN > -1)
        HAL::digitalWrite(A_ENABLE_PIN, A_ENABLE_ON, safe);
#endif
#if A2_ASTEPPER_SUPPORT && (A2_ENABLE_PIN > -1)
        HAL::digitalWrite(A2_ENABLE_PIN, A_ENABLE_ON, safe);
#endif
	}
    static INLINE void setXDirection(bool positive, uint8_t safe = true) {
        if(positive) {
            HAL::digitalWrite(X_DIR_PIN, !INVERT_X_DIR, safe);
#if X2_XSTEPPER_SUPPORT
            HAL::digitalWrite(X2_DIR_PIN, !INVERT_X2_DIR, safe);
#endif
        } else {
            HAL::digitalWrite(X_DIR_PIN, INVERT_X_DIR, safe);
#if X2_XSTEPPER_SUPPORT
            HAL::digitalWrite(X2_DIR_PIN, INVERT_X2_DIR, safe);
#endif
        }
    }

    static INLINE void setYDirection(bool positive, uint8_t safe = true) {
        if(positive) {
            HAL::digitalWrite(Y_DIR_PIN, !INVERT_Y_DIR, safe);
#if Y2_YSTEPPER_SUPPORT
            HAL::digitalWrite(Y2_DIR_PIN, !INVERT_Y2_DIR, safe);
#endif
        } else {
            HAL::digitalWrite(Y_DIR_PIN, INVERT_Y_DIR, safe);
#if Y2_YSTEPPER_SUPPORT
            HAL::digitalWrite(Y2_DIR_PIN, INVERT_Y2_DIR, safe);
#endif
        }
    }
    static INLINE void setZDirection(bool positive, uint8_t safe = true) {
        if(positive) {
            HAL::digitalWrite(Z_DIR_PIN, !INVERT_Z_DIR, safe);
#if Z2_ZSTEPPER_SUPPORT
            HAL::digitalWrite(Z2_DIR_PIN, !INVERT_Z2_DIR, safe);
#endif
        } else {
            HAL::digitalWrite(Z_DIR_PIN, INVERT_Z_DIR, safe);
#if Z2_ZSTEPPER_SUPPORT
            HAL::digitalWrite(Z2_DIR_PIN, INVERT_Z2_DIR, safe);
#endif
        }
	}
	static INLINE void setADirection(bool positive, uint8_t safe = true) {
        if(positive) {
            HAL::digitalWrite(A_DIR_PIN, !INVERT_A_DIR, safe);
#if A2_ASTEPPER_SUPPORT
            HAL::digitalWrite(A2_DIR_PIN, !INVERT_A2_DIR, safe);
#endif
        } else {
            HAL::digitalWrite(A_DIR_PIN, INVERT_A_DIR, safe);
#if A2_ASTEPPER_SUPPORT
            HAL::digitalWrite(A2_DIR_PIN, INVERT_A2_DIR, safe);
#endif
        }
	}

    static INLINE bool getXDirection() {
        return((HAL::digitalRead(X_DIR_PIN) != 0) ^ INVERT_X_DIR);
	}

	static INLINE bool getYDirection() {
		return((HAL::digitalRead(Y_DIR_PIN) != 0) ^ INVERT_Y_DIR);
	}

	static INLINE bool getZDirection() {
		return ((HAL::digitalRead(Z_DIR_PIN) != 0) ^ INVERT_Z_DIR);
	}

	static INLINE bool getADirection() {
		return ((HAL::digitalRead(A_DIR_PIN) != 0) ^ INVERT_A_DIR);
	}

    static INLINE bool areAllSteppersDisabled() {
        return flag0 & MACHINE_FLAG0_STEPPER_DISABLED;
    }

    static INLINE void setAllSteppersDiabled() {
        flag0 |= MACHINE_FLAG0_STEPPER_DISABLED;
#if FAN_CONTROL_SUPPORT
        FanControl::setSpeed(0, FAN_BOARD_INDEX);
#endif
    }

    static INLINE void unsetAllSteppersDisabled() {
        flag0 &= ~MACHINE_FLAG0_STEPPER_DISABLED;
#if FAN_CONTROL_SUPPORT
        FanControl::setSpeed(255, FAN_BOARD_INDEX);
#endif
    }

    static INLINE void setHasLines(bool lines) {
        flag0 = (lines ? flag0 | MACHINE_FLAG0_HAS_LINES : flag0 & ~MACHINE_FLAG0_HAS_LINES);
    }

    static INLINE bool lastHasLines() {
        return (flag0 & MACHINE_FLAG0_HAS_LINES);
    }

    static INLINE uint8_t isAllKilled() {
        return flag0 & MACHINE_FLAG0_ALLKILLED;
    }

    static INLINE void setAllKilled(uint8_t b) {
        flag0 = (b ? flag0 | MACHINE_FLAG0_ALLKILLED : flag0 & ~MACHINE_FLAG0_ALLKILLED);
    }

    static INLINE uint8_t isRelativeCoordinateMode() {
        return flag0 & MACHINE_FLAG0_RELATIVE_COORD;
    }

    static INLINE void setRelativeCoorinateMode(uint8_t coord) {
        flag0 = (coord ? flag0 | MACHINE_FLAG0_RELATIVE_COORD : flag0 & ~MACHINE_FLAG0_RELATIVE_COORD);
    }

    static INLINE uint8_t isUnitInches() {
        return flag0 & MACHINE_FLAG0_UNIT_IS_INCH;
    }

    static INLINE void setUnitInches(uint8_t inch) {
        flag0 = (inch ? flag0 | MACHINE_FLAG0_UNIT_IS_INCH : flag0 & ~MACHINE_FLAG0_UNIT_IS_INCH);
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

    static INLINE float convertToMM(float x) {
        return (isUnitInches() ? x * 25.4f : x);
    }

    static INLINE void setIgnoreFanCommand(bool enable) {
        flag1 = (enable ? flag1 | MACHINE_FLAG1_IGNORE_FAN_COMMAND : flag1 & ~MACHINE_FLAG1_IGNORE_FAN_COMMAND);
    }

    static INLINE bool isIgnoreFanCommand() {
        return (flag1 & MACHINE_FLAG1_IGNORE_FAN_COMMAND);
    }

	static INLINE void startXStep(uint8_t safe = true) {
#if MULTI_XENDSTOP_HOMING
        if(Machine::multiXHomeFlags & 1) {
            HAL::digitalWrite(X_STEP_PIN, START_STEP_WITH_HIGH, safe);
        }
#if X2_XSTEPPER_SUPPORT
        if(Machine::multiXHomeFlags & 2) {
            HAL::digitalWrite(X2_STEP_PIN, START_STEP_WITH_HIGH, safe);
        }
#endif
#else // MULTI_XENDSTOP_HOMING
        HAL::digitalWrite(X_STEP_PIN, START_STEP_WITH_HIGH, safe);
#if X2_XSTEPPER_SUPPORT
        HAL::digitalWrite(X2_STEP_PIN, START_STEP_WITH_HIGH, safe);
#endif
#endif
    }
    static INLINE void startYStep(uint8_t safe = true) {
#if MULTI_YENDSTOP_HOMING
        if(Machine::multiYHomeFlags & 1) {
            HAL::digitalWrite(Y_STEP_PIN, START_STEP_WITH_HIGH, safe);
        }
#if Y2_YSTEPPER_SUPPORT
        if(Machine::multiYHomeFlags & 2) {
            HAL::digitalWrite(Y2_STEP_PIN, START_STEP_WITH_HIGH, safe);
        }
#endif
#else
        HAL::digitalWrite(Y_STEP_PIN, START_STEP_WITH_HIGH, safe);
#if Y2_YSTEPPER_SUPPORT
        HAL::digitalWrite(Y2_STEP_PIN, START_STEP_WITH_HIGH, safe);
#endif
#endif
    }
    static INLINE void startZStep(uint8_t safe = true) {
#if MULTI_ZENDSTOP_HOMING
        if(Machine::multiZHomeFlags & 1) {
            HAL::digitalWrite(Z_STEP_PIN, START_STEP_WITH_HIGH, safe);
        }
#if Z2_ZSTEPPER_SUPPORT
        if(Machine::multiZHomeFlags & 2) {
            HAL::digitalWrite(Z2_STEP_PIN, START_STEP_WITH_HIGH, safe);
        }
#endif
#else
        HAL::digitalWrite(Z_STEP_PIN, START_STEP_WITH_HIGH, safe);
#if Z2_ZSTEPPER_SUPPORT
        HAL::digitalWrite(Z2_STEP_PIN, START_STEP_WITH_HIGH, safe);
#endif
#endif
	}
	static INLINE void startAStep(uint8_t safe = true) {
        HAL::digitalWrite(A_STEP_PIN, START_STEP_WITH_HIGH, safe);
#if A2_ASTEPPER_SUPPORT
        HAL::digitalWrite(A2_STEP_PIN, START_STEP_WITH_HIGH, safe);
#endif
	}
	static INLINE void endXYZASteps(uint8_t safe = true) {
        HAL::digitalWrite(X_STEP_PIN, !START_STEP_WITH_HIGH, safe);
#if X2_XSTEPPER_SUPPORT
        HAL::digitalWrite(X2_STEP_PIN, !START_STEP_WITH_HIGH, safe);
#endif
        HAL::digitalWrite(Y_STEP_PIN, !START_STEP_WITH_HIGH, safe);
#if Y2_YSTEPPER_SUPPORT
        HAL::digitalWrite(Y2_STEP_PIN, !START_STEP_WITH_HIGH, safe);
#endif
        HAL::digitalWrite(Z_STEP_PIN, !START_STEP_WITH_HIGH, safe);
#if Z2_ZSTEPPER_SUPPORT
        HAL::digitalWrite(Z2_STEP_PIN, !START_STEP_WITH_HIGH, safe);
#endif
        HAL::digitalWrite(A_STEP_PIN, !START_STEP_WITH_HIGH, safe);
#if A2_ASTEPPER_SUPPORT
        HAL::digitalWrite(A2_STEP_PIN, !START_STEP_WITH_HIGH, safe);
#endif
    }
    static INLINE speed_t updateStepsPerTimerCall(speed_t vbase) {
#if MAX_STEPS_PER_CALL >= 8
        if (vbase > STEP_DOUBLER_FREQUENCY * 4) {
            stepsTillNextCalc = 8;
#if QUICK_STEP
            return vbase >> 3;
#endif
        } else
#endif
#if MAX_STEPS_PER_CALL >= 4
        if(vbase > STEP_DOUBLER_FREQUENCY * 2) {
            stepsTillNextCalc = 4;
#if QUICK_STEP
            return vbase >> 2;
#endif
        } else
#endif
#if MAX_STEPS_PER_CALL >= 2
        if (vbase > STEP_DOUBLER_FREQUENCY) {
            stepsTillNextCalc = 2;
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
            stepsTillNextCalc = 8;
#if QUICK_STEP
            interval = fullInterval << 3;
#else
            interval = fullInterval;
#endif
        } else
#endif
#if MAX_STEPS_PER_CALL >= 4
        if (vbase > STEP_DOUBLER_FREQUENCY * 2) {
            stepsTillNextCalc = 4;
#if QUICK_STEP
            interval = fullInterval << 2;
#else
            interval = fullInterval;
#endif
        } else
#endif
#if MAX_STEPS_PER_CALL >= 2
        if (vbase > STEP_DOUBLER_FREQUENCY) {
            stepsTillNextCalc = 2;
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
	static INLINE void realPosition(float* positionArray) {
        positionArray[X_AXIS] = currentPosition[X_AXIS];
        positionArray[Y_AXIS] = currentPosition[Y_AXIS];
        positionArray[Z_AXIS] = currentPosition[Z_AXIS];
        positionArray[A_AXIS] = currentPosition[A_AXIS];
    }
    static INLINE void lastCmdPosSteps(int32_t *positionArray) {
        positionArray[X_AXIS] = static_cast<int32_t>(floor(lastCmdPos[X_AXIS] * axisStepsPerMM[X_AXIS] + 0.5f));
        positionArray[Y_AXIS] = static_cast<int32_t>(floor(lastCmdPos[Y_AXIS] * axisStepsPerMM[Y_AXIS] + 0.5f));
        positionArray[Z_AXIS] = static_cast<int32_t>(floor(lastCmdPos[Z_AXIS] * axisStepsPerMM[Z_AXIS] + 0.5f));
        positionArray[A_AXIS] = static_cast<int32_t>(floor(lastCmdPos[A_AXIS] * axisStepsPerMM[A_AXIS] + 0.5f));
    }
	static void updateDerivedParameter();
    /** If we are not homing or destination check being disabled, this will reduce _destinationSteps_ to a
    valid value. In other words this works as software endstop. */
    static void constrainDestinationCoords(int32_t* destinationSteps);
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
	static void moveToReal(float x, float y, float z, float a, float f, bool pathOptimize = true);
	static void kill(uint8_t only_steppers);
    static void setup();
    static void defaultLoopActions();
    static void homeAxis(bool xaxis, bool yaxis, bool zaxis); /// Home axis
    /** \brief Tests if the target position is allowed.

    Tests if the test position lies inside the defined geometry. For Cartesian
    printers this is the defined cube defined by x,y,z min and length. For
    delta printers the cylindrical shape is tested.

    \param x X position in mm.
    \param x Y position in mm.
    \param x Z position in mm.
    \return true if position is valid and can be reached. */
    static bool isPositionAllowed(float x, float y, float z);
	static void SetMemoryPosition();
    static void GoToMemoryPosition(bool x, bool y, bool z, bool a, float feed);

    static void changeFeedrateMultiply(uint16_t factor);
    static void showCapabilities();
    static void showConfiguration();
    static void setCaseLight(bool on);
    static void reportCaseLightStatus();
    static void homeXAxis();
    static void homeYAxis();
    static void homeZAxis();

private:
    static uint8_t          debugLevel;
    static uint8_t          flag0, flag1;

    static uint16_t         counterPeriodical;
    static volatile uint8_t executePeriodical;
    static uint8_t          counter500ms;
};

#endif // MACHINE_H_INCLUDED