/*   This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
#
  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#ifndef MOTION_H_INCLUDED
#define MOTION_H_INCLUDED

/** Marks the first step of a new move */
#define FLAG_WARMUP 1
#define FLAG_NOMINAL 2
#define FLAG_DECELERATING 4
#define FLAG_ACCELERATION_ENABLED 8 // unused
#define FLAG_CHECK_ENDSTOPS 16
#define FLAG_SKIP_DEACCELERATING 64 // unused
#define FLAG_BLOCKED 128

/** Are the step parameter computed */
#define FLAG_JOIN_STEPPARAMS_COMPUTED 1
/** The right speed is fixed. Don't check this block or any block to the left. */
#define FLAG_JOIN_END_FIXED 2
/** The left speed is fixed. Don't check left block. */
#define FLAG_JOIN_START_FIXED 4

class PrintLine { // RAM usage: 24*4+15 = 113 Byte
public:
    static ufast8_t linesPos; // Position for executing line movement
    static PrintLine lines[];
    static ufast8_t linesWritePos; // Position where we write the next cached line move
    ufast8_t joinFlags;
    volatile ufast8_t flags;
	secondspeed_t secondSpeed; // fan control
private:
    fast8_t primaryAxis;
    ufast8_t dir;                       ///< Direction of movement. 1 = X+, 2 = Y+, 4= Z+, values can be combined.
    int32_t timeInTicks;
	int32_t delta[A_AXIS_ARRAY];                  ///< Steps we want to move.
	int32_t error[A_AXIS_ARRAY];                  ///< Error calculation for Bresenham algorithm
	float speed[A_AXIS_ARRAY];                   ///< Speed in x direction at fullInterval in mm/s
    float fullSpeed;                ///< Desired speed mm/s
    float invFullSpeed;             ///< 1.0/fullSpeed for faster computation
    float accelerationDistance2;    ///< Real 2.0*distance*acceleration mm²/s²
    float maxJunctionSpeed;         ///< Max. junction speed between this and next segment
    float startSpeed;               ///< Starting speed in mm/s
    float endSpeed;                 ///< Exit speed in mm/s
    float minSpeed;
	float distance;
    ticks_t fullInterval;     ///< interval at full speed in ticks/step.
    uint32_t accelSteps;        ///< How much steps does it take, to reach the plateau.
    uint32_t decelSteps;        ///< How much steps does it take, to reach the end speed.
    uint32_t accelerationPrim; ///< Acceleration along primary axis
    uint32_t fAcceleration;    ///< accelerationPrim*262144/F_CPU
    speed_t vMax;              ///< Maximum reached speed in steps/s.
    speed_t vStart;            ///< Starting speed in steps/s.
	speed_t vEnd;              ///< End speed in steps/s
#ifdef DEBUG_STEPCOUNT
    int32_t totalStepsRemaining;
#endif
public:
    int32_t stepsRemaining;            ///< Remaining steps, until move is finished
    static PrintLine *cur;
    static volatile ufast8_t linesCount; // Number of lines cached 0 = nothing to do
    inline bool areParameterUpToDate() {
        return joinFlags & FLAG_JOIN_STEPPARAMS_COMPUTED;
    }
    inline void invalidateParameter() {
        joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED;
    }
    inline void setParameterUpToDate() {
        joinFlags |= FLAG_JOIN_STEPPARAMS_COMPUTED;
    }
    inline bool isStartSpeedFixed() {
        return joinFlags & FLAG_JOIN_START_FIXED;
    }
    inline void setStartSpeedFixed(bool newState) {
        joinFlags = (newState ? joinFlags | FLAG_JOIN_START_FIXED : joinFlags & ~FLAG_JOIN_START_FIXED);
    }
    inline void fixStartAndEndSpeed() {
        joinFlags |= FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
    }
    inline bool isEndSpeedFixed() {
        return joinFlags & FLAG_JOIN_END_FIXED;
    }
    inline void setEndSpeedFixed(bool newState) {
        joinFlags = (newState ? joinFlags | FLAG_JOIN_END_FIXED : joinFlags & ~FLAG_JOIN_END_FIXED);
    }
    inline bool isWarmUp() {
        return flags & FLAG_WARMUP;
    }
    inline uint8_t getWaitForXLinesFilled() {
        return primaryAxis;
    }
    inline void setWaitForXLinesFilled(uint8_t b) {
        primaryAxis = b;
    }
    inline void block() {
        flags |= FLAG_BLOCKED;
    }
    inline void unblock() {
        flags &= ~FLAG_BLOCKED;
    }
    inline bool isBlocked() {
        return flags & FLAG_BLOCKED;
    }
    inline bool isCheckEndstops() {
        return flags & FLAG_CHECK_ENDSTOPS;
    }
    inline bool isNominalMove() {
        return flags & FLAG_NOMINAL;
    }
    inline void setNominalMove() {
        flags |= FLAG_NOMINAL;
    }
    inline void checkEndstops() {
        if(isCheckEndstops()) {
            Endstops::update();
            if(Endstops::anyEndstopHit()) {
#if MULTI_XENDSTOP_HOMING
                {
                    if(Printer::isHoming()) {
                        if(isXNegativeMove()) {
                            if(Endstops::xMin())
                                Printer::multiXHomeFlags &= ~1;
                            if(Endstops::x2Min())
                                Printer::multiXHomeFlags &= ~2;
                            if(Printer::multiXHomeFlags == 0)
                                setXMoveFinished();
                        } else if(isXPositiveMove()) {
                            if(Endstops::xMax())
                                Printer::multiXHomeFlags &= ~1;
                            if(Endstops::x2Max())
                                Printer::multiXHomeFlags &= ~2;
                            if(Printer::multiXHomeFlags == 0) {
                                setXMoveFinished();
                            }
                        }
                    } else {
                        if(isXNegativeMove() && Endstops::xMin()) {
                            setXMoveFinished();
                        } else if(isXPositiveMove() && Endstops::xMax()) {
                            setXMoveFinished();
                        }
                    }
                }
#else  // Multi endstop homing
                if(isXNegativeMove() && Endstops::xMin())
                    setXMoveFinished();
                else if(isXPositiveMove() && Endstops::xMax())
                    setXMoveFinished();
#endif
#if MULTI_YENDSTOP_HOMING
                {
                    if(Printer::isHoming()) {
                        if(isYNegativeMove()) {
                            if(Endstops::yMin())
                                Printer::multiYHomeFlags &= ~1;
                            if(Endstops::y2Min())
                                Printer::multiYHomeFlags &= ~2;
                            if(Printer::multiYHomeFlags == 0)
                                setYMoveFinished();
                        } else if(isYPositiveMove()) {
                            if(Endstops::yMax())
                                Printer::multiYHomeFlags &= ~1;
                            if(Endstops::y2Max())
                                Printer::multiYHomeFlags &= ~2;
                            if(Printer::multiYHomeFlags == 0) {
                                setYMoveFinished();
                            }
                        }
                    } else {
                        if(isYNegativeMove() && Endstops::yMin()) {
                            setYMoveFinished();
                        } else if(isYPositiveMove() && Endstops::yMax()) {
                            setYMoveFinished();
                        }
                    }
                }
#else  // Multi endstop homing
                if(isYNegativeMove() && Endstops::yMin())
                    setYMoveFinished();
                else if(isYPositiveMove() && Endstops::yMax())
                    setYMoveFinished();
#endif
#if FEATURE_Z_PROBE
				if(Printer::isZProbingActive() /*&& isZNegativeMove()*/ && Endstops::zProbe()) {
					if(isXMove())
						setXMoveFinished();
					if(isYMove())
						setYMoveFinished();
					if(isZMove())
						setZMoveFinished();
                    Printer::stepsRemainingAtZHit = stepsRemaining;
                } else
#endif
#if MULTI_ZENDSTOP_HOMING
                {
                    if(Printer::isHoming()) {
                        if(isZNegativeMove()) {
                            if(Endstops::zMin())
                                Printer::multiZHomeFlags &= ~1;
                            if(Endstops::z2MinMax())
                                Printer::multiZHomeFlags &= ~2;
                            if(Printer::multiZHomeFlags == 0)
                                setZMoveFinished();
                        } else if(isZPositiveMove()) {
                            if(Endstops::zMax())
                                Printer::multiZHomeFlags &= ~1;
                            if(Endstops::z2MinMax())
                                Printer::multiZHomeFlags &= ~2;
                            if(Printer::multiZHomeFlags == 0) {
#if MAX_HARDWARE_ENDSTOP_Z
                                Printer::stepsRemainingAtZHit = stepsRemaining;
#endif
                                setZMoveFinished();
                            }
                        }
                    } else {
#if !(Z_MIN_PIN == Z_PROBE_PIN && FEATURE_Z_PROBE)
                        if(isZNegativeMove() && Endstops::zMin()) {
                            setZMoveFinished();
                        } else
#endif
						
						if(isZPositiveMove() && Endstops::zMax()) {
#if MAX_HARDWARE_ENDSTOP_Z
                            Printer::stepsRemainingAtZHit = stepsRemaining;
#endif
                            setZMoveFinished();
                        }
                    }
                }
#else  // Multi endstop homing
                    if(isZNegativeMove() && Endstops::zMin()
#if Z_MIN_PIN == Z_PROBE_PIN && FEATURE_Z_PROBE
						&& Printer::isHoming()
#endif
					) {
                        setZMoveFinished();
                    } else if(isZPositiveMove() && Endstops::zMax()) {
#if MAX_HARDWARE_ENDSTOP_Z
                        Printer::stepsRemainingAtZHit = stepsRemaining;
#endif
                        setZMoveFinished();
                    }
#endif
            }
#if FEATURE_Z_PROBE
			else if(Printer::isZProbingActive()/* && isZNegativeMove()*/) {
                Endstops::update();
                if(Endstops::zProbe()) {
                    if(isXMove())
						setXMoveFinished();
					if(isYMove())
						setYMoveFinished();
					if(isZMove())
						setZMoveFinished();
                    Printer::stepsRemainingAtZHit = stepsRemaining;
                }
            }
#endif
        }
    }

	inline void setXMoveFinished() {
		dir &= ~16;
    }
	inline void setYMoveFinished() {
		dir &= ~32;
	}
	inline void setZMoveFinished() {
		dir &= ~64;
    }
    inline void setXYMoveFinished() {
        dir &= ~48;
    }
    inline bool isXPositiveMove() {
        return (dir & X_STEP_DIRPOS) == X_STEP_DIRPOS;
    }
    inline bool isXNegativeMove() {
        return (dir & X_STEP_DIRPOS) == XSTEP;
    }
    inline bool isYPositiveMove() {
        return (dir & Y_STEP_DIRPOS) == Y_STEP_DIRPOS;
    }
    inline bool isYNegativeMove() {
        return (dir & Y_STEP_DIRPOS) == YSTEP;
    }
    inline bool isZPositiveMove() {
        return (dir & Z_STEP_DIRPOS) == Z_STEP_DIRPOS;
    }
    inline bool isZNegativeMove() {
        return (dir & Z_STEP_DIRPOS) == ZSTEP;
	}
	inline bool isAPositiveMove() {
		return (dir & A_STEP_DIRPOS) == A_STEP_DIRPOS;
    }
	inline bool isANegativeMove() {
		return (dir & A_STEP_DIRPOS) == ASTEP;
	}
    inline bool isXMove() {
        return (dir & XSTEP);
    }
    inline bool isYMove() {
        return (dir & YSTEP);
    }
    inline bool isXOrYMove() {
        return dir & XY_STEP;
    }
    inline bool isXOrZMove() {
        return dir & (XSTEP | ZSTEP);
    }
    inline bool isZMove() {
        return (dir & ZSTEP);
	}
	inline bool isAMove() {
		return (dir & ASTEP);
	}
    inline bool isNoMove() {
		return (dir & XYZA_STEP) == 0;
    }
	inline bool isXYZMove() {
		return dir & XYZ_STEP;
	}
	inline bool isXYZAMove() {
		return dir & XYZA_STEP;
	}
    inline bool isMoveOfAxis(uint8_t axis) {
        return (dir & (XSTEP << axis));
    }
    inline void setMoveOfAxis(uint8_t axis) {
        dir |= XSTEP << axis;
    }
    inline void setPositiveDirectionForAxis(uint8_t axis) {
        dir |= X_DIRPOS << axis;
    }
    inline static void resetPathPlanner() {
        linesCount = 0;
		linesPos = linesWritePos;
	}
    INLINE bool moveDecelerating() {
        if(stepsRemaining <= static_cast<int32_t>(decelSteps)) {
            if (!(flags & FLAG_DECELERATING)) {
                Printer::timer = 0;
                flags |= FLAG_DECELERATING;
            }
            return true;
        } else return false;
    }
    INLINE bool moveAccelerating() {
        return Printer::stepNumber <= accelSteps;
    }
	INLINE void startXStep() {
		Printer::startXStep();
#ifdef DEBUG_STEPCOUNT
        totalStepsRemaining--;
#endif
    }
	INLINE void startYStep() {
		Printer::startYStep();
#ifdef DEBUG_STEPCOUNT
        totalStepsRemaining--;
#endif
    }
	INLINE void startZStep() {
		Printer::startZStep();
#ifdef DEBUG_STEPCOUNT
        totalStepsRemaining--;
#endif
	}
	INLINE void startAStep() {
		Printer::startAStep();
#ifdef DEBUG_STEPCOUNT
        totalStepsRemaining--;
#endif
	}
    void updateStepsParameter();
    float safeSpeed(fast8_t drivingAxis);
    void calculateMove(float axis_diff[], uint8_t pathOptimize, fast8_t distanceBase);
    void logLine();
    INLINE long getWaitTicks() {
        return timeInTicks;
    }
    INLINE void setWaitTicks(long wait) {
        timeInTicks = wait;
    }

    static INLINE bool hasLines() {
        return linesCount;
    }
    static INLINE void setCurrentLine() {
		cur = &lines[linesPos];
    }
    // Only called from within interrupts
    static INLINE void removeCurrentLineForbidInterrupt() {
        nextPlannerIndex(linesPos);
        cur = NULL;
        HAL::forbidInterrupts();
		--linesCount;	}
    static INLINE void pushLine() {
        nextPlannerIndex(linesWritePos);
        InterruptProtectedBlock noInts;
        linesCount++;
    }
    static uint8_t getLinesCount() {
        InterruptProtectedBlock noInts;
        return linesCount;
    }
    static PrintLine *getNextWriteLine() {
        return &lines[linesWritePos];
    }
    static inline void computeMaxJunctionSpeed(PrintLine *previous, PrintLine *current);
    static int32_t bresenhamStep();
    static void waitForXFreeLines(uint8_t b = 1, bool allowMoves = false);
    static inline void forwardPlanner(ufast8_t p);
    static inline void backwardPlanner(ufast8_t p, ufast8_t last);
    static void updateTrapezoids();
    static uint8_t insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines);
	static void queueCartesianMove(uint8_t check_endstops, uint8_t pathOptimize);
#if DISTORTION_CORRECTION
    static void queueCartesianSegmentTo(uint8_t check_endstops, uint8_t pathOptimize);
#endif
	static void moveRelativeDistanceInSteps(int32_t x, int32_t y, int32_t z, int32_t a, float feedrate, bool waitEnd, bool check_endstop, bool pathOptimize = true);
	static void moveRelativeDistanceInStepsReal(int32_t x, int32_t y, int32_t z, int32_t a, float feedrate, bool waitEnd, bool pathOptimize = true);
#if ARC_SUPPORT
    static void arc(float *position, float *target, float *offset, float radius, uint8_t isclockwise);
#endif
    static INLINE void previousPlannerIndex(ufast8_t &p) {
        p = (p ? p - 1 : PRINTLINE_CACHE_SIZE - 1);
    }
    static INLINE void nextPlannerIndex(ufast8_t& p) {
        p = (p >= PRINTLINE_CACHE_SIZE - 1 ? 0 : p + 1);
    }
};



#endif // MOTION_H_INCLUDED
