#ifndef MOTION_H_INCLUDED
#define MOTION_H_INCLUDED

/** Marks the first step of a new move */
#define FLAG_WARMUP             1
#define FLAG_NOMINAL            2
#define FLAG_DECELERATING       4
#define FLAG_ACCELERATING       8
#define FLAG_CHECK_ENDSTOPS     16
#define FLAG_BLOCKED            128

#define FLAG_TOOL_VACUUM_ON     1
#define FLAG_TOOL_MIST_ON       2
#define FLAG_TOOL_FLOOD_ON      4


/** Are the step parameter computed */
#define FLAG_JOIN_STEPPARAMS_COMPUTED 1
/** The right speed is fixed. Don't check this block or any block to the left. */
#define FLAG_JOIN_END_FIXED 2
/** The left speed is fixed. Don't check left block. */
#define FLAG_JOIN_START_FIXED 4

class MachineLine { // RAM usage: 24*4+15 = 113 Byte
public:
    static ufast8_t linesPos; // Position for executing line movement
    static MachineLine lines[];
    static ufast8_t linesWritePos; // Position where we write the next cached line move
    ufast8_t joinFlags;
    volatile ufast8_t flags;
    
    uint8_t toolFlags;
#if SUPPORT_LASER
    uint8_t laserIntensity;
#endif
#if FEATURE_FAN_CONTROL
    uint8_t fanSpeed;
#endif
private:
    static int32_t cur_errupd;
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
    static MachineLine *cur;
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
                    if(Machine::isHoming()) {
                        if(isXNegativeMove()) {
                            if(Endstops::xMin())
                                Machine::multiXHomeFlags &= ~1;
                            if(Endstops::x2Min())
                                Machine::multiXHomeFlags &= ~2;
                            if(Machine::multiXHomeFlags == 0)
                                setXMoveFinished();
                        } else if(isXPositiveMove()) {
                            if(Endstops::xMax())
                                Machine::multiXHomeFlags &= ~1;
                            if(Endstops::x2Max())
                                Machine::multiXHomeFlags &= ~2;
                            if(Machine::multiXHomeFlags == 0) {
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
                    if(Machine::isHoming()) {
                        if(isYNegativeMove()) {
                            if(Endstops::yMin())
                                Machine::multiYHomeFlags &= ~1;
                            if(Endstops::y2Min())
                                Machine::multiYHomeFlags &= ~2;
                            if(Machine::multiYHomeFlags == 0)
                                setYMoveFinished();
                        } else if(isYPositiveMove()) {
                            if(Endstops::yMax())
                                Machine::multiYHomeFlags &= ~1;
                            if(Endstops::y2Max())
                                Machine::multiYHomeFlags &= ~2;
                            if(Machine::multiYHomeFlags == 0) {
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
				if(Machine::isZProbingActive() /*&& isZNegativeMove()*/ && Endstops::zProbe()) {
					if(isXMove())
						setXMoveFinished();
					if(isYMove())
						setYMoveFinished();
					if(isZMove())
						setZMoveFinished();
                    Machine::stepsRemainingAtZHit = stepsRemaining;
                } else
#endif
#if MULTI_ZENDSTOP_HOMING
                {
                    if(Machine::isHoming()) {
                        if(isZNegativeMove()) {
                            if(Endstops::zMin())
                                Machine::multiZHomeFlags &= ~1;
                            if(Endstops::z2MinMax())
                                Machine::multiZHomeFlags &= ~2;
                            if(Machine::multiZHomeFlags == 0)
                                setZMoveFinished();
                        } else if(isZPositiveMove()) {
                            if(Endstops::zMax())
                                Machine::multiZHomeFlags &= ~1;
                            if(Endstops::z2MinMax())
                                Machine::multiZHomeFlags &= ~2;
                            if(Machine::multiZHomeFlags == 0) {
#if MAX_HARDWARE_ENDSTOP_Z
                                Machine::stepsRemainingAtZHit = stepsRemaining;
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
                            Machine::stepsRemainingAtZHit = stepsRemaining;
#endif
                            setZMoveFinished();
                        }
                    }
                }
#else  // Multi endstop homing
                    if(isZNegativeMove() && Endstops::zMin()
#if Z_MIN_PIN == Z_PROBE_PIN && FEATURE_Z_PROBE
						&& Machine::isHoming()
#endif
					) {
                        setZMoveFinished();
                    } else if(isZPositiveMove() && Endstops::zMax()) {
#if MAX_HARDWARE_ENDSTOP_Z
                        Machine::stepsRemainingAtZHit = stepsRemaining;
#endif
                        setZMoveFinished();
                    }
#endif
            }
#if FEATURE_Z_PROBE
			else if(Machine::isZProbingActive()/* && isZNegativeMove()*/) {
                Endstops::update();
                if(Endstops::zProbe()) {
                    if(isXMove())
						setXMoveFinished();
					if(isYMove())
						setYMoveFinished();
					if(isZMove())
						setZMoveFinished();
                    Machine::stepsRemainingAtZHit = stepsRemaining;
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
        if (flags & FLAG_DECELERATING) {
            return true;
        }

        if(stepsRemaining <= static_cast<int32_t>(decelSteps)) {
            if (!(flags & FLAG_DECELERATING)) {
                Machine::timer = 0;
                flags |= FLAG_DECELERATING;
            }
            return true;
        } else return false;
    }
    INLINE bool moveAccelerating() {
        if (flags & FLAG_ACCELERATING) {
            if (Machine::stepNumber <= accelSteps) {
                return true;
            }

            flags &= ~FLAG_ACCELERATING;
        }

        return false;
    }

    void updateStepsParameter();
    float safeSpeed(fast8_t drivingAxis);
    void calculateMove(float* axisDistanceMM, fast8_t drivingAxis);
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
		--linesCount;
    }
    static INLINE void pushLine() {
        nextPlannerIndex(linesWritePos);
        InterruptProtectedBlock noInts;
        linesCount++;
    }
    static uint8_t getLinesCount() {
        InterruptProtectedBlock noInts;
        return linesCount;
    }
    static MachineLine *getNextWriteLine() {
        return &lines[linesWritePos];
    }
    static inline void computeMaxJunctionSpeed(MachineLine *previous, MachineLine *current);
    static uint32_t bresenhamStep();
    static void waitForXFreeLines(uint8_t b = 1, bool allowMoves = false);
    static inline void forwardPlanner(ufast8_t p);
    static inline void backwardPlanner(ufast8_t p, ufast8_t last);
    static void updateTrapezoids();
    static uint8_t insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines);
	static void queueCartesianMove(int32_t *destinationSteps, uint8_t checkEndstops, uint8_t pathOptimize);
    static void queueCartesianSegmentTo(int32_t *segmentSteps, uint8_t addDistortion, uint8_t checkEndstops, uint8_t pathOptimize);
	static void moveRelativeDistanceInSteps(int32_t x, int32_t y, int32_t z, int32_t a, float feedrate, bool waitEnd, bool check_endstop, bool pathOptimize = true);
	static void moveRelativeDistanceInStepsReal(int32_t x, int32_t y, int32_t z, int32_t a, float feedrate, bool waitEnd, bool pathOptimize = true);
#if ARC_SUPPORT
    static void queueArc(float *position, float *target, float *offset, float radius, uint8_t isclockwise);
#endif
    static INLINE void previousPlannerIndex(ufast8_t &p) {
        p = (p ? p - 1 : MACHINELINE_CACHE_SIZE - 1);
    }
    static INLINE void nextPlannerIndex(ufast8_t& p) {
        p = (p >= MACHINELINE_CACHE_SIZE - 1 ? 0 : p + 1);
    }
};



#endif // MOTION_H_INCLUDED
