#include "SwitchCNC.h"

// ================ Sanity checks ================
#ifndef STEP_DOUBLER_FREQUENCY
#error Please add new parameter STEP_DOUBLER_FREQUENCY to your configuration.
#else
#if STEP_DOUBLER_FREQUENCY < 4000 || STEP_DOUBLER_FREQUENCY > 10000
#error STEP_DOUBLER_FREQUENCY should be in range 4000-10000.
#endif
#endif
// ####################################################################################
// #          No configuration below this line - just some error checking             #
// ####################################################################################
#ifdef SUPPORT_MAX6675
#if !defined SCK_PIN || !defined MOSI_PIN || !defined MISO_PIN
#error For MAX6675 support, you need to define SCK_PIN, MISO_PIN and MOSI_PIN in pins.h
#endif
#endif
#if X_STEP_PIN < 0 || Y_STEP_PIN < 0 || Z_STEP_PIN < 0
#error One of the following pins is not assigned: X_STEP_PIN,Y_STEP_PIN,Z_STEP_PIN
#endif
#if MACHINELINE_CACHE_SIZE < 4
#error MACHINELINE_CACHE_SIZE must be at least 5
#endif

MachineLine MachineLine::lines[MACHINELINE_CACHE_SIZE]; ///< Cache for print moves.
MachineLine *MachineLine::cur = NULL;               ///< Current printing line
ufast8_t MachineLine::linesWritePos = 0;            ///< Position where we write the next cached line move.
volatile ufast8_t MachineLine::linesCount = 0;      ///< Number of lines cached 0 = nothing to do.
ufast8_t MachineLine::linesPos = 0;                 ///< Position for executing line movement.
int32_t MachineLine::cur_errupd = 0;
uint8_t toolFlags;

/**
Move printer the given number of steps. Puts the move into the queue. Used by e.g. homing commands.
Does not consider rotation but updates position correctly considering rotation. This can be used to
correct positions when changing tools.

\param x Distance in x direction in steps
\param y Distance in y direction in steps
\param z Distance in z direction in steps
\param e Distance in e direction in steps
\param feedrate Feed rate to be used in mm/s. Gets new active feedrate.
\param waitEnd If true will block until move is finished.
\param checkEndstop True if triggering endstop should stop move.
\param pathOptimize If false start and end speeds get fixed to minimum values.
*/
void MachineLine::moveRelativeDistanceInSteps(int32_t x, int32_t y, int32_t z, int32_t a, float feedrate, bool waitEnd, bool checkEndstop, bool pathOptimize) {
#if MOVE_X_WHEN_HOMED == 1 || MOVE_Y_WHEN_HOMED == 1 || MOVE_Z_WHEN_HOMED == 1
    if(!Machine::isHoming() && !Machine::isNoDestinationCheck()) {
#if MOVE_X_WHEN_HOMED
        if(!Machine::isXHomed())
            x = 0;
#endif
#if MOVE_Y_WHEN_HOMED
        if(!Machine::isYHomed())
            y = 0;
#endif
#if MOVE_Z_WHEN_HOMED
        if(!Machine::isZHomed() && !ZProbe::isActive())
            z = 0;
#endif
    }
#endif //  MOVE_X_WHEN_HOMED == 1 || MOVE_Y_WHEN_HOMED == 1 || MOVE_Z_WHEN_HOMED == 1
    float savedFeedrate = Machine::feedrate;
    int32_t destinationSteps[A_AXIS_ARRAY] = {
        Machine::currentPositionSteps[X_AXIS] + x,
        Machine::currentPositionSteps[Y_AXIS] + y,
        Machine::currentPositionSteps[Z_AXIS] + z,
        Machine::currentPositionSteps[A_AXIS] + a
    };
	Machine::feedrate = feedrate;
#if DISTORTION_CORRECTION_SUPPORT
    destinationSteps[Z_AXIS] -= Machine::zCorrectionStepsIncluded; // correct as it will be added later in Cartesian move computation
#endif
	queueCartesianMove(destinationSteps, checkEndstop, pathOptimize);
    Machine::feedrate = savedFeedrate;
    Machine::updateCurrentPosition(false);

    if (waitEnd) {
        Commands::waitUntilEndOfAllMoves();
    }

    Machine::previousMillisCmd = HAL::timeInMilliseconds();
}

/** Adds the steps converted to mm to the lastCmdPos position and moves to that position using Machine::moveToReal.
Will use Machine::isPositionAllowed to prevent illegal moves.

\param x Distance in x direction in steps
\param y Distance in y direction in steps
\param z Distance in z direction in steps
\param e Distance in e direction in steps
\param feedrate Feed rate to be used in mm/s. Gets new active feedrate.
\param waitEnd If true will block until move is finished.
\param pathOptimize If false start and end speeds get fixed to minimum values.
*/
void MachineLine::moveRelativeDistanceInStepsReal(int32_t x, int32_t y, int32_t z, int32_t a, float feedrate, bool waitEnd, bool pathOptimize) {
#if MOVE_X_WHEN_HOMED == 1 || MOVE_Y_WHEN_HOMED == 1 || MOVE_Z_WHEN_HOMED == 1
    if(!Machine::isHoming() && !Machine::isNoDestinationCheck()) { // prevent movements when not homed
#if MOVE_X_WHEN_HOMED
        if(!Machine::isXHomed())
            x = 0;
#endif
#if MOVE_Y_WHEN_HOMED
        if(!Machine::isYHomed())
            y = 0;
#endif
#if MOVE_Z_WHEN_HOMED
        if(!Machine::isZHomed() && !ZProbe::isActive())
            z = 0;
#endif
    }
#endif // MOVE_X_WHEN_HOMED == 1 || MOVE_Y_WHEN_HOMED == 1 || MOVE_Z_WHEN_HOMED == 1
    Machine::lastCmdPos[X_AXIS] += x * Machine::invAxisStepsPerMM[X_AXIS];
    Machine::lastCmdPos[Y_AXIS] += y * Machine::invAxisStepsPerMM[Y_AXIS];
	Machine::lastCmdPos[Z_AXIS] += z * Machine::invAxisStepsPerMM[Z_AXIS];
	Machine::lastCmdPos[A_AXIS] += a * Machine::invAxisStepsPerMM[A_AXIS];

    if (Machine::isPositionAllowed(Machine::lastCmdPos[X_AXIS], Machine::lastCmdPos[Y_AXIS], Machine::lastCmdPos[Z_AXIS])) {
        Machine::moveToReal(Machine::lastCmdPos[X_AXIS], Machine::lastCmdPos[Y_AXIS], Machine::lastCmdPos[Z_AXIS], Machine::lastCmdPos[A_AXIS], feedrate, pathOptimize);
        Machine::updateCurrentPosition();

        if (waitEnd) {
            Commands::waitUntilEndOfAllMoves();
        }

        Machine::previousMillisCmd = HAL::timeInMilliseconds();
    }
}

void MachineLine::queueCartesianSegmentTo(int32_t *segmentSteps, uint8_t addDistortion, uint8_t checkEndstops, uint8_t pathOptimize) {
#if DISTORTION_CORRECTION_SUPPORT
    if (addDistortion) {
        // Correct the bumps
        Machine::zCorrectionStepsIncluded = Distortion::correct(segmentSteps[X_AXIS], segmentSteps[Y_AXIS], segmentSteps[Z_AXIS]);
        segmentSteps[Z_AXIS] += Machine::zCorrectionStepsIncluded;
#if DEBUG_DISTORTION
        Com::printF(PSTR("zCorr:"), Machine::zCorrectionStepsIncluded * Machine::invAxisStepsPerMM[Z_AXIS], 3);
        Com::printF(PSTR(" atX:"), segmentSteps[X_AXIS] * Machine::invAxisStepsPerMM[X_AXIS]);
        Com::printFLN(PSTR(" atY:"), segmentSteps[Y_AXIS] * Machine::invAxisStepsPerMM[Y_AXIS]);
#endif
    }
#endif

    MachineLine::waitForXFreeLines(1);
    uint8_t newPath = MachineLine::insertWaitMovesIfNeeded(pathOptimize, 0);
    MachineLine *p = MachineLine::getNextWriteLine();

	float axisDistanceMM[A_AXIS_ARRAY]; // Axis movement in mm
	p->flags = (checkEndstops ? FLAG_CHECK_ENDSTOPS : 0) | FLAG_ACCELERATING;
    p->joinFlags = 0;
    if(!pathOptimize) p->setEndSpeedFixed(true);
    p->dir = 0;
    //Find direction
    //Machine::zCorrectionStepsIncluded = 0;
	for(uint8_t axis = 0; axis < A_AXIS_ARRAY; axis++) {
		p->delta[axis] = segmentSteps[axis] - Machine::currentPositionSteps[axis];
        if(p->delta[axis] >= 0) p->setPositiveDirectionForAxis(axis);
        else p->delta[axis] = -p->delta[axis];
        axisDistanceMM[axis] = p->delta[axis] * Machine::invAxisStepsPerMM[axis];
        if(p->delta[axis]) p->setMoveOfAxis(axis);
        Machine::currentPositionSteps[axis] = segmentSteps[axis];
    }

#if LASER_SUPPORT
    p->laserIntensity = Laser::next();
#endif
#if FAN_CONTROL_SUPPORT
    p->fanSpeed = FanControl::next();
#endif

    //build toolFlags
    p->toolFlags = 0;
#if VACUUM_SUPPORT
    if (Vacuum::next()) {
        p->toolFlags |= FLAG_TOOL_VACUUM_ON;
    }
#endif
#if COOLANT_SUPPORT && COOLANT_MIST_PIN > -1
    if (CoolantMist::next()) {
        p->toolFlags |= FLAG_TOOL_MIST_ON;
    }
#endif
#if COOLANT_SUPPORT && COOLANT_FLOOD_PIN > -1
    if (CoolantFlood::next()) {
        p->toolFlags |= FLAG_TOOL_FLOOD_ON;
    }
#endif

    if(p->isNoMove()) {
        if(newPath)   // need to delete dummy elements, otherwise commands can get locked.
            MachineLine::resetPathPlanner();
        return; // No steps included
    }

	float xydist2 = 0;
#if BACKLASH_COMPENSATION_SUPPORT
    if((p->isXYZAMove()) && ((p->dir & XYZA_DIRPOS) ^ (Backlash::dir & XYZA_DIRPOS)) & (Backlash::dir >> 4)) { // We need to compensate backlash, add a move
        MachineLine::waitForXFreeLines(2);
        uint8_t wpos2 = MachineLine::linesWritePos + 1;
        if(wpos2 >= MACHINELINE_CACHE_SIZE) wpos2 = 0;
        MachineLine *p2 = &MachineLine::lines[wpos2];
        memcpy(p2, p, sizeof(MachineLine)); // Move current data to p2
		uint8_t changed = (p->dir & XYZA_DIRPOS) ^ (Backlash::dir & XYZA_DIRPOS);
		float back_diff[A_AXIS_ARRAY]; // Axis movement in mm
        Backlash::buildDiff(back_diff, changed, p);
		p->dir &= XYZA_DIRPOS; // x,y,z and a are already correct
		for(uint8_t i = 0; i < A_AXIS_ARRAY; i++) {
            float f = back_diff[i] * Machine::axisStepsPerMM[i];
            p->delta[i] = abs((long)f);
            if(p->delta[i]) p->dir |= XSTEP << i;
		}
        Backlash::dir = (Backlash::dir & 240) | (p2->dir & XYZA_DIRPOS);

		//Define variables that are needed for the Bresenham algorithm.
		if(p->delta[Y_AXIS] > p->delta[X_AXIS] && p->delta[Y_AXIS] > p->delta[Z_AXIS] && p->delta[Y_AXIS] > p->delta[A_AXIS]) p->primaryAxis = Y_AXIS;
		else if(p->delta[X_AXIS] > p->delta[Z_AXIS] && p->delta[X_AXIS] > p->delta[A_AXIS]) p->primaryAxis = X_AXIS;
		else if(p->delta[Z_AXIS] > p->delta[A_AXIS]) p->primaryAxis = Z_AXIS;
		else p->primaryAxis = A_AXIS;
		p->stepsRemaining = p->delta[p->primaryAxis];

		//Feedrate calc based on XYZA travel distance
		if(p->isXMove()) xydist2 += back_diff[X_AXIS] * back_diff[X_AXIS];
		if(p->isYMove()) xydist2 += back_diff[Y_AXIS] * back_diff[Y_AXIS];
		if(p->isZMove()) xydist2 += back_diff[Z_AXIS] * back_diff[Z_AXIS];
		if(p->isAMove()) xydist2 += back_diff[A_AXIS] * back_diff[A_AXIS];

		p->distance = sqrtf(xydist2);
		p->calculateMove(back_diff, p->primaryAxis);

        p = p2; // use saved instance for the real move
    }
#endif

    //Define variables that are needed for the Bresenham algorithm.
	if(p->delta[Y_AXIS] > p->delta[X_AXIS] && p->delta[Y_AXIS] > p->delta[Z_AXIS] && p->delta[Y_AXIS] > p->delta[A_AXIS]) p->primaryAxis = Y_AXIS;
	else if(p->delta[X_AXIS] > p->delta[Z_AXIS] && p->delta[X_AXIS] > p->delta[A_AXIS]) p->primaryAxis = X_AXIS;
	else if(p->delta[Z_AXIS] > p->delta[A_AXIS]) p->primaryAxis = Z_AXIS;
	else p->primaryAxis = A_AXIS;
	p->stepsRemaining = p->delta[p->primaryAxis];

	if(p->isXMove()) xydist2 += axisDistanceMM[X_AXIS] * axisDistanceMM[X_AXIS];
	if(p->isYMove()) xydist2 += axisDistanceMM[Y_AXIS] * axisDistanceMM[Y_AXIS];
	if(p->isZMove()) xydist2 += axisDistanceMM[Z_AXIS] * axisDistanceMM[Z_AXIS];
	if(p->isAMove()) xydist2 += axisDistanceMM[A_AXIS] * axisDistanceMM[A_AXIS];

	p->distance = sqrtf(xydist2);
    p->calculateMove(axisDistanceMM, p->primaryAxis);
}

/**
  Put a move to the current destination coordinates into the movement cache.
  If the cache is full, the method will wait, until a place gets free. During
  wait communication and temperature control is enabled.

  destinationSteps must be excluding any z correction! We will add that if required here.

  @param checkEndstops Read end stop during move.
*/
void MachineLine::queueCartesianMove(int32_t *destinationSteps, uint8_t checkEndstops, uint8_t pathOptimize) {
	ENSURE_POWER
    Machine::constrainDestinationCoords(destinationSteps);
	Machine::unsetAllSteppersDisabled();

#if DISTORTION_CORRECTION_SUPPORT || ALWAYS_SPLIT_LINES
    float moveLen = 0;
    bool distortionCorrection = false;
#if ALWAYS_SPLIT_LINES
    int32_t deltaSteps[A_AXIS_ARRAY] = {
        destinationSteps[X_AXIS] - Machine::currentPositionSteps[X_AXIS],
        destinationSteps[Y_AXIS] - Machine::currentPositionSteps[Y_AXIS],
        destinationSteps[Z_AXIS] - Machine::currentPositionSteps[Z_AXIS],
        destinationSteps[A_AXIS] - Machine::currentPositionSteps[A_AXIS]
    };
#else
    int32_t deltaSteps[A_AXIS_ARRAY];
#endif
#endif

#if ALWAYS_SPLIT_LINES
	// we are inside correction height so we split all moves in lines of max. 10 mm and add them
	// including a z correction
	for(fast8_t i = 0; i < A_AXIS_ARRAY; i++) {
		float lenAxis = Machine::invAxisStepsPerMM[i] * deltaSteps[i];
        moveLen += lenAxis * lenAxis;
	}
#endif

#if DISTORTION_CORRECTION_SUPPORT
	if(Distortion::isEnabled() && (destinationSteps[Z_AXIS] < Distortion::zMaxSteps()) && !ZProbe::isActive() && !Machine::isHoming()) {
        Machine::currentPositionSteps[Z_AXIS] -= Machine::zCorrectionStepsIncluded;

#if ALWAYS_SPLIT_LINES == 0
        deltaSteps[X_AXIS] = destinationSteps[X_AXIS] - Machine::currentPositionSteps[X_AXIS];
        deltaSteps[Y_AXIS] = destinationSteps[Y_AXIS] - Machine::currentPositionSteps[Y_AXIS];
        deltaSteps[Z_AXIS] = destinationSteps[Z_AXIS] - Machine::currentPositionSteps[Z_AXIS];
        deltaSteps[A_AXIS] = destinationSteps[A_AXIS] - Machine::currentPositionSteps[A_AXIS];

        for (fast8_t i = 0; i < XY_AXIS_ARRAY; i++) {
            float lenAxis = Machine::invAxisStepsPerMM[i] * deltaSteps[i];
            moveLen += lenAxis * lenAxis;
        }
#else
        deltaSteps[Z_AXIS] += Machine::zCorrectionStepsIncluded;
#endif

        distortionCorrection = true;
	}
#endif

#if DISTORTION_CORRECTION_SUPPORT || ALWAYS_SPLIT_LINES
    // Split if length is larger than the allowed segment size
    if (moveLen > LINE_SEGMENT_SIZE * LINE_SEGMENT_SIZE) {
        // we need to split longer lines to follow bed curvature
        moveLen = sqrtf(moveLen);
        uint16_t segments = (static_cast<uint16_t>(moveLen) + LINE_SEGMENT_SIZE - 1) / LINE_SEGMENT_SIZE;
        int32_t deltaStepSegment[A_AXIS_ARRAY] = {
            (deltaSteps[X_AXIS] + (segments / 2)) / segments,
            (deltaSteps[Y_AXIS] + (segments / 2)) / segments,
            (deltaSteps[Z_AXIS] + (segments / 2)) / segments,
            (deltaSteps[A_AXIS] + (segments / 2)) / segments,
        };

        int32_t startSteps[A_AXIS_ARRAY] = {
            Machine::currentPositionSteps[X_AXIS],
            Machine::currentPositionSteps[Y_AXIS],
            Machine::currentPositionSteps[Z_AXIS],
            Machine::currentPositionSteps[A_AXIS]
        };

        int32_t segmentSteps[A_AXIS_ARRAY] = {
            startSteps[X_AXIS],
            startSteps[Y_AXIS],
            startSteps[Z_AXIS],
            startSteps[A_AXIS]
        };

#if DEBUG_DISTORTION
        Com::printF(PSTR("Split line len:"), moveLen);
        Com::printFLN(PSTR(" segments:"), segments);
#endif
        uint8_t segmentUpdate = LINE_N_SEGMENT_CORRECT;
        for (uint16_t i = 1; i < segments; i++) {
            if (!segmentUpdate--) {
                segmentSteps[X_AXIS] = startSteps[X_AXIS] + (i * deltaSteps[X_AXIS]) / segments;
                segmentSteps[Y_AXIS] = startSteps[Y_AXIS] + (i * deltaSteps[Y_AXIS]) / segments;
                segmentSteps[Z_AXIS] = startSteps[Z_AXIS] + (i * deltaSteps[Z_AXIS]) / segments;
                segmentSteps[A_AXIS] = startSteps[A_AXIS] + (i * deltaSteps[A_AXIS]) / segments;

                segmentUpdate = LINE_N_SEGMENT_CORRECT;
            }
            else {
                segmentSteps[X_AXIS] += deltaStepSegment[X_AXIS];
                segmentSteps[Y_AXIS] += deltaStepSegment[Y_AXIS];
                segmentSteps[Z_AXIS] += deltaStepSegment[Z_AXIS];
                segmentSteps[A_AXIS] += deltaStepSegment[A_AXIS];
            }

            queueCartesianSegmentTo(segmentSteps, distortionCorrection, checkEndstops, pathOptimize);
        }

        queueCartesianSegmentTo(destinationSteps, distortionCorrection, checkEndstops, pathOptimize);
    }
    else {
        queueCartesianSegmentTo(destinationSteps, distortionCorrection, checkEndstops, pathOptimize);
    }
#else
    queueCartesianSegmentTo(destinationSteps, false, checkEndstops, pathOptimize);
#endif
}

void MachineLine::calculateMove(float* axisDistanceMM, fast8_t drivingAxis) {
	long axisInterval[A_AXIS_ARRAY];
    //float timeForMove = (float)(F_CPU)*distance / (isXOrYMove() ? RMath::max(Machine::minimumSpeed, Machine::feedrate) : Machine::feedrate); // time is in ticks
    float timeForMove = (float)(F_CPU) * distance / Machine::feedrate; // time is in ticks
    //bool critical = ZProbe::isActive();
    if(linesCount < MOVE_CACHE_LOW && timeForMove < LOW_TICKS_PER_MOVE) { // Limit speed to keep cache full.
        //Com::printF(PSTR("L:"),(int)linesCount);
        //Com::printF(PSTR(" Old "),timeForMove);
        timeForMove += (3 * (LOW_TICKS_PER_MOVE - timeForMove)) / (linesCount + 1); // Increase time if queue gets empty. Add more time if queue gets smaller.
        //Com::printFLN(PSTR("Slow "),timeForMove);
        //critical = true;
    }
    timeInTicks = timeForMove;
	// Compute the slowest allowed interval (ticks/step), so maximum feedrate is not violated
	int32_t limitInterval0;
    int32_t limitInterval = limitInterval0 = timeForMove / stepsRemaining; // until not violated by other constraints it is your target speed
    float toTicks = static_cast<float>(F_CPU) / stepsRemaining;
    if(isXMove()) {
		axisInterval[X_AXIS] = axisDistanceMM[X_AXIS] * toTicks / (Machine::maxFeedrate[X_AXIS]); // mm*ticks/s/(mm/s*steps) = ticks/step
		limitInterval = RMath::max(axisInterval[X_AXIS], limitInterval);
    } else axisInterval[X_AXIS] = 0;

    if(isYMove()) {
		axisInterval[Y_AXIS] = axisDistanceMM[Y_AXIS] * toTicks / Machine::maxFeedrate[Y_AXIS];
		limitInterval = RMath::max(axisInterval[Y_AXIS], limitInterval);
    } else axisInterval[Y_AXIS] = 0;

	if(isZMove()) { // normally no move in z direction
		axisInterval[Z_AXIS] = axisDistanceMM[Z_AXIS] * toTicks / Machine::maxFeedrate[Z_AXIS]; // must prevent overflow!
		limitInterval = RMath::max(axisInterval[Z_AXIS], limitInterval);
	} else axisInterval[Z_AXIS] = 0;

	if(isAMove()) { // normally no move in a direction
		axisInterval[A_AXIS] = axisDistanceMM[A_AXIS] * toTicks / Machine::maxFeedrate[A_AXIS]; // must prevent overflow!
		limitInterval = RMath::max(axisInterval[A_AXIS], limitInterval);
	} else axisInterval[A_AXIS] = 0;

	fullInterval = limitInterval = limitInterval > LIMIT_INTERVAL ? limitInterval : LIMIT_INTERVAL; // This is our target speed

	if(limitInterval != limitInterval0) {
		// new time at full speed = limitInterval*p->stepsRemaining [ticks]
        timeForMove = (float)limitInterval * (float)stepsRemaining; // for large z-distance this overflows with long computation
    }

    float inverseTimeS = (float)F_CPU / timeForMove;
    if(isXMove()) {
		axisInterval[X_AXIS] = timeForMove / delta[X_AXIS];
		speed[X_AXIS] = axisDistanceMM[X_AXIS] * inverseTimeS;
		if(isXNegativeMove()) speed[X_AXIS] = -speed[X_AXIS];
	} else speed[X_AXIS] = 0;

    if(isYMove()) {
        axisInterval[Y_AXIS] = timeForMove / delta[Y_AXIS];
		speed[Y_AXIS] = axisDistanceMM[Y_AXIS] * inverseTimeS;
		if(isYNegativeMove()) speed[Y_AXIS] = -speed[Y_AXIS];
	} else speed[Y_AXIS] = 0;

	if(isZMove()) {
		axisInterval[Z_AXIS] = timeForMove / delta[Z_AXIS];
		speed[Z_AXIS] = axisDistanceMM[Z_AXIS] * inverseTimeS;
		if(isZNegativeMove()) speed[Z_AXIS] = -speed[Z_AXIS];
	} else speed[Z_AXIS] = 0;

	if(isAMove()) {
		axisInterval[A_AXIS] = timeForMove / delta[A_AXIS];
		speed[A_AXIS] = axisDistanceMM[A_AXIS] * inverseTimeS;
		if(isANegativeMove()) speed[A_AXIS] = -speed[A_AXIS];
	} else speed[A_AXIS] = 0;

    fullSpeed = distance * inverseTimeS;

    //long interval = axis_interval[primary_axis]; // time for every step in ticks with full speed
    //If acceleration is enabled, do some Bresenham calculations depending on which axis will lead it.
#if RAMP_ACCELERATION
    // slowest time to accelerate from v0 to limitInterval determines used acceleration
    // t = (v_end-v_start)/a
    float slowestAxisPlateauTimeRepro = 1e15; // 1/time to reduce division Unit: 1/s
	uint32_t *accel = Machine::maxAccelerationStepsPerSquareSecond;
	for(fast8_t i = 0; i < A_AXIS_ARRAY ; i++) {
        if (isMoveOfAxis(i)) {
            // v = a * t => t = v/a = F_CPU/(c*a) => 1/t = c*a/F_CPU
            slowestAxisPlateauTimeRepro = RMath::min(slowestAxisPlateauTimeRepro, (float)axisInterval[i] * (float)accel[i]); //  steps/s^2 * step/tick  Ticks/s^2
        }
	}

	// Errors for delta move are initialized in timer (except extruder)
	error[X_AXIS] = error[Y_AXIS] = error[Z_AXIS] = error[A_AXIS] = delta[primaryAxis] >> 1;
    invFullSpeed = 1.0 / fullSpeed;
	accelerationPrim = slowestAxisPlateauTimeRepro / axisInterval[primaryAxis]; // a = v/t = F_CPU/(c*t): Steps/s^2
	//Now we can calculate the new primary axis acceleration, so that the slowest axis max acceleration is not violated
	fAcceleration = 262144.0f * (float)accelerationPrim / F_CPU; // will overflow without float!
	accelerationDistance2 = 2.0f * distance * slowestAxisPlateauTimeRepro * fullSpeed / ((float)F_CPU); // mm^2/s^2
    startSpeed = endSpeed = minSpeed = safeSpeed(drivingAxis);
    if(startSpeed > Machine::feedrate)
        startSpeed = endSpeed = minSpeed = Machine::feedrate;
    // Can accelerate to full speed within the line
    if (startSpeed * startSpeed + accelerationDistance2 >= fullSpeed * fullSpeed) {
        setNominalMove();
    }

	vMax = F_CPU / fullInterval; // maximum steps per second, we can reach
    // if(p->vMax>46000)  // gets overflow in N computation
    //   p->vMax = 46000;
	//p->plateauN = (p->vMax*p->vMax/p->accelerationPrim)>>1;

	updateTrapezoids();
    // how much steps on primary axis do we need to reach target feedrate
	//p->plateauSteps = (long) (((float)p->acceleration *0.5f / slowest_axis_plateau_time_repro + p->vMin) *1.01f/slowest_axis_plateau_time_repro);
#endif

#ifdef DEBUG_STEPCOUNT
// Set in delta move calculation
	totalStepsRemaining = delta[X_AXIS] + delta[Y_AXIS] + delta[Z_AXIS] + delta[A_AXIS];
#endif
#ifdef DEBUG_QUEUE_MOVE
    if(Machine::debugEcho()) {
        logLine();
        Com::printFLN(Com::tDBGLimitInterval, limitInterval);
        Com::printFLN(Com::tDBGMoveDistance, distance);
        Com::printFLN(Com::tDBGCommandedFeedrate, Machine::feedrate);
        Com::printFLN(Com::tDBGConstFullSpeedMoveTime, timeForMove);
    }
#endif
	// Make result permanent
    pushLine();
    DEBUG_MEMORY;
}

/**
This is the path planner.

It goes from the last entry and tries to increase the end speed of previous moves in a fashion that the maximum jerk
is never exceeded. If a segment with reached maximum speed is met, the planner stops. Everything left from this
is already optimal from previous updates.
The first 2 entries in the queue are not checked. The first is the one that is already in print and the following will likely to become active.

The method is called before lines_count is increased!
*/
void MachineLine::updateTrapezoids() {
    ufast8_t first = linesWritePos;
    MachineLine *firstLine;
    MachineLine *act = &lines[linesWritePos];
    InterruptProtectedBlock noInts;

    // First we find out how far back we could go with optimization.
    ufast8_t maxfirst = linesPos; // first non fixed segment we might change
    if(maxfirst != linesWritePos)
        nextPlannerIndex(maxfirst); // don't touch the line printing
    // Now ignore enough segments to gain enough time for path planning
    millis_t timeleft = 0;
    // Skip as many stored moves as needed to gain enough time for computation
#if MACHINELINE_CACHE_SIZE < 10
#define minTime 4500L * MACHINELINE_CACHE_SIZE
#else
#define minTime 45000L
#endif
    while(timeleft < minTime && maxfirst != linesWritePos) {
        timeleft += lines[maxfirst].timeInTicks;
        nextPlannerIndex(maxfirst);
    }

    // Search last fixed element
    while(first != maxfirst && !lines[first].isEndSpeedFixed())
        previousPlannerIndex(first);
    if(first != linesWritePos && lines[first].isEndSpeedFixed())
        nextPlannerIndex(first);

    // now first points to last segment before the end speed is fixed
    // so start speed is also fixed.
    if(first == linesWritePos) { // Nothing to plan, only new element present
        act->block(); // Prevent stepper interrupt from using this
        noInts.unprotect();
        act->setStartSpeedFixed(true);
        act->updateStepsParameter();
        act->unblock();
        return;
    }
    // now we have at least one additional move for optimization
    // that is not a wait move
    // First is now the new element or the first element with non fixed end speed.
    // anyhow, the start speed of first is fixed
    firstLine = &lines[first];
    firstLine->block(); // don't let printer touch this or following segments during update
    noInts.unprotect();
    ufast8_t previousIndex = linesWritePos;
    previousPlannerIndex(previousIndex);
    MachineLine *previous = &lines[previousIndex]; // segment before the one we are inserting
	computeMaxJunctionSpeed(previous, act); // Set maximum junction speed if we have a real move before
    // Increase speed if possible neglecting current speed
    backwardPlanner(linesWritePos, first);
    // Reduce speed to reachable speeds
    forwardPlanner(first);

#ifdef DEBUG_PLANNER
    if(Machine::debugEcho()) {
        Com::printF(PSTR("Planner: "), (int)linesCount);
        previousPlannerIndex(first);
        Com::printF(PSTR(" F "), lines[first].startSpeed, 1);
        Com::printF(PSTR(" - "), lines[first].endSpeed, 1);
        Com::printF(PSTR("("), lines[first].maxJunctionSpeed, 1);
        Com::printF(PSTR(","), (int)lines[first].joinFlags);
        nextPlannerIndex(first);
    }
#endif
    // Update precomputed data
    do {
        lines[first].updateStepsParameter();
#ifdef DEBUG_PLANNER
        if(Machine::debugEcho()) {
            Com::printF(PSTR(" / "), lines[first].startSpeed, 1);
            Com::printF(PSTR(" - "), lines[first].endSpeed, 1);
            Com::printF(PSTR("("), lines[first].maxJunctionSpeed, 1);
            Com::printF(PSTR(","), (int)lines[first].joinFlags);
#ifdef DEBUG_QUEUE_MOVE
            Com::println();
#endif
        }
#endif
        //noInts.protect();
        lines[first].unblock();  // start with first block to release next used segment as early as possible
        nextPlannerIndex(first);
        lines[first].block();
        //noInts.unprotect();
    } while(first != linesWritePos);
    act->updateStepsParameter();
    act->unblock();
#ifdef DEBUG_PLANNER
    if(Machine::debugEcho()) {
        Com::printF(PSTR(" / "), lines[first].startSpeed, 1);
        Com::printF(PSTR(" - "), lines[first].endSpeed, 1);
        Com::printF(PSTR("("), lines[first].maxJunctionSpeed, 1);
        Com::printFLN(PSTR(","), (int)lines[first].joinFlags);
    }
#endif
}

/* Computes the maximum junction speed of the newly added segment under
optimal conditions. There is no guarantee that the previous move will be able to reach the
speed at all, but if it could exceed it will never exceed this theoretical limit.
*/
inline void MachineLine::computeMaxJunctionSpeed(MachineLine *previous, MachineLine *current) {
    // if we are here we have to identical move types
    // either pure extrusion -> pure extrusion or
	// move -> move (with or without extrusion)
    // First we compute the normalized jerk for speed 1
	float factor = 1.0;
	float maxJoinSpeed = RMath::min(current->fullSpeed, previous->fullSpeed);

    for (uint8_t i = 0; i < A_AXIS_ARRAY; i++) {
        float delta = fabs(current->speed[i] - previous->speed[i]);
        if (delta > Machine::maxJerk[i]) {
            factor = RMath::min(factor, Machine::maxJerk[i] / delta);
        }
    }

	previous->maxJunctionSpeed = maxJoinSpeed * factor; // set speed limit
#ifdef DEBUG_QUEUE_MOVE
    if(Machine::debugEcho()) {
        Com::printF(PSTR("ID:"), (int)previous);
        Com::printFLN(PSTR(" MJ:"), previous->maxJunctionSpeed);
    }
#endif // DEBUG_QUEUE_MOVE
}

/** Update parameter used by updateTrapezoids

Computes the acceleration/deceleration steps and advanced parameter associated.
*/
void MachineLine::updateStepsParameter() {
    if(areParameterUpToDate() || isWarmUp()) return;
    float startFactor = startSpeed * invFullSpeed;
    float endFactor   = endSpeed   * invFullSpeed;
    vStart = vMax * startFactor; //starting speed
    vEnd   = vMax * endFactor;

	uint32_t vmax2 = HAL::U16SquaredToU32(vMax);
	accelSteps = ((vmax2 - HAL::U16SquaredToU32(vStart)) / (accelerationPrim << 1)) + 1; // Always add 1 for missing precision
	decelSteps = ((vmax2 - HAL::U16SquaredToU32(vEnd))  / (accelerationPrim << 1)) + 1;

    if(static_cast<int32_t>(accelSteps + decelSteps) >= stepsRemaining) { // can't reach limit speed
        uint32_t red = (accelSteps + decelSteps - stepsRemaining) >> 1;
        accelSteps = accelSteps - RMath::min(static_cast<int32_t>(accelSteps), static_cast<int32_t>(red));
        decelSteps = decelSteps - RMath::min(static_cast<int32_t>(decelSteps), static_cast<int32_t>(red));
    }
    setParameterUpToDate();
#ifdef DEBUG_QUEUE_MOVE
    if(Machine::debugEcho()) {
        Com::printFLN(Com::tDBGId, (int)this);
        Com::printF(Com::tDBGVStartEnd, (long)vStart);
        Com::printFLN(Com::tSlash, (long)vEnd);
        Com::printF(Com::tDBAccelSteps, (long)accelSteps);
        Com::printF(Com::tSlash, (long)decelSteps);
        Com::printFLN(Com::tSlash, (long)stepsRemaining);
        Com::printF(Com::tDBGStartEndSpeed, startSpeed, 1);
        Com::printFLN(Com::tSlash, endSpeed, 1);
        Com::printFLN(Com::tDBGFlags, (uint32_t)flags);
        Com::printFLN(Com::tDBGJoinFlags, (uint32_t)joinFlags);
    }
#endif
}

/**
Compute the maximum speed from the last entered move.
The backwards planner traverses the moves from last to first looking at deceleration. The RHS of the accelerate/decelerate ramp.

start = last line inserted
last = last element until we check
*/
inline void MachineLine::backwardPlanner(ufast8_t start, ufast8_t last) {
    MachineLine *act = &lines[start], *previous;
    float lastJunctionSpeed = act->endSpeed; // Start always with safe speed

    //PREVIOUS_PLANNER_INDEX(last); // Last element is already fixed in start speed
    while(start != last) {
        previousPlannerIndex(start);
        previous = &lines[start];
        previous->block();
		// Avoid speed calculation once cruising in split delta move

        // Avoid speed calculations if we know we can accelerate within the line
        lastJunctionSpeed = (act->isNominalMove() ? act->fullSpeed : sqrtf(lastJunctionSpeed * lastJunctionSpeed + act->accelerationDistance2)); // acceleration is acceleration*distance*2! What can be reached if we try?
        // If that speed is more that the maximum junction speed allowed then ...
        if(lastJunctionSpeed >= previous->maxJunctionSpeed) { // Limit is reached
            // If the previous line's end speed has not been updated to maximum speed then do it now
            if(previous->endSpeed != previous->maxJunctionSpeed) {
                previous->invalidateParameter(); // Needs recomputation
                previous->endSpeed = RMath::max(previous->minSpeed, previous->maxJunctionSpeed); // possibly unneeded???
            }
            // If actual line start speed has not been updated to maximum speed then do it now
            if(act->startSpeed != previous->maxJunctionSpeed) {
                act->startSpeed = RMath::max(act->minSpeed, previous->maxJunctionSpeed); // possibly unneeded???
                act->invalidateParameter();
            }
            lastJunctionSpeed = previous->endSpeed;
        } else {
            // Block previous end and act start as calculated speed and recalculate plateau speeds (which could move the speed higher again)
            act->startSpeed = RMath::max(act->minSpeed, lastJunctionSpeed);
            lastJunctionSpeed = previous->endSpeed = RMath::max(lastJunctionSpeed, previous->minSpeed);
            previous->invalidateParameter();
            act->invalidateParameter();
        }
        act = previous;
    } // while loop
}

void MachineLine::forwardPlanner(ufast8_t first) {
    MachineLine *act;
    MachineLine *next = &lines[first];
    float vmaxRight;
    float leftSpeed = next->startSpeed;
    while(first != linesWritePos) { // All except last segment, which has fixed end speed
        act = next;
        nextPlannerIndex(first);
        next = &lines[first];
        /* if(act->isEndSpeedFixed())
         {
             leftSpeed = act->endSpeed;
             continue; // Nothing to do here
         }*/
		// Avoid speed calculate once cruising in split delta move
		// Avoid speed calculates if we know we can accelerate within the line.
		vmaxRight = (act->isNominalMove() ? act->fullSpeed : sqrtf(leftSpeed * leftSpeed + act->accelerationDistance2));
        if(vmaxRight > act->endSpeed) { // Could be higher next run?
            if(leftSpeed < act->minSpeed) {
                leftSpeed = act->minSpeed;
                act->endSpeed = sqrtf(leftSpeed * leftSpeed + act->accelerationDistance2);
            }
            act->startSpeed = leftSpeed;
            next->startSpeed = leftSpeed = RMath::max(RMath::min(act->endSpeed, act->maxJunctionSpeed), next->minSpeed);
            if(act->endSpeed == act->maxJunctionSpeed) { // Full speed reached, don't compute again!
                act->setEndSpeedFixed(true);
                next->setStartSpeedFixed(true);
            }
            act->invalidateParameter();
        } else { // We can accelerate full speed without reaching limit, which is as fast as possible. Fix it!
            act->fixStartAndEndSpeed();
            act->invalidateParameter();
            if(act->minSpeed > leftSpeed) {
                leftSpeed = act->minSpeed;
                vmaxRight = sqrtf(leftSpeed * leftSpeed + act->accelerationDistance2);
            }
            act->startSpeed = leftSpeed;
            act->endSpeed = RMath::max(act->minSpeed, vmaxRight);
            next->startSpeed = leftSpeed = RMath::max(RMath::min(act->endSpeed, act->maxJunctionSpeed), next->minSpeed);
            next->setStartSpeedFixed(true);
        }
    } // While
    next->startSpeed = RMath::max(next->minSpeed, leftSpeed); // This is the new segment, which is updated anyway, no extra flag needed.
}


inline float MachineLine::safeSpeed(fast8_t drivingAxis)
{
	float xMin = Machine::maxJerk[X_AXIS] * 0.5;
	float yMin = Machine::maxJerk[Y_AXIS] * 0.5;
	float zMin = Machine::maxJerk[Z_AXIS] * 0.5;
	float aMin = Machine::maxJerk[A_AXIS] * 0.5;
	float safe;
	bool safeFound = 0;

	if(isXMove())
	{
		safe = xMin;
		safeFound = true;
	}

	if(isYMove())
	{
		if(safeFound)
		{
			safe = RMath::min(safe, yMin * fullSpeed / fabs(speed[Y_AXIS]));
		} else
		{
			safe = yMin;
			safeFound = true;
		}
	}

	if(isZMove())
	{
		if(safeFound)
		{
			safe = RMath::min(safe, zMin * fullSpeed / fabs(speed[Z_AXIS]));
		} else
		{
			safe = zMin;
			safeFound = true;
		}
	}

	if(isAMove())
	{
		if(safeFound)
		{
			safe = RMath::min(safe, aMin * fullSpeed / fabs(speed[A_AXIS]));
		} else
		{
			safe = aMin;
		}
	}

	// Check for minimum speeds needed for numerical robustness
	if(drivingAxis == X_AXIS) safe = RMath::max(xMin, safe);
		else if(drivingAxis == Y_AXIS) safe = RMath::max(yMin, safe);
		else if(drivingAxis == Z_AXIS) safe = RMath::max(zMin, safe);
		else if(drivingAxis == A_AXIS) safe = RMath::max(aMin, safe);

    return RMath::min(safe, fullSpeed);
}


/** Check if move is new. If it is insert some dummy moves to allow the path optimizer to work since it does
not act on the first two moves in the queue. The stepper timer will spot these moves and leave some time for
processing.
*/
uint8_t MachineLine::insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines) {
    if(linesCount == 0 && pathOptimize) { // First line after some time - warm up needed
		//return 0;
		uint8_t w = 4;
        while(w--) {
            MachineLine *p = getNextWriteLine();
            p->flags = FLAG_WARMUP;
            p->joinFlags = FLAG_JOIN_STEPPARAMS_COMPUTED | FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
            p->dir = 0;
			p->setWaitForXLinesFilled(w + waitExtraLines);
			p->setWaitTicks(100000);
            pushLine();
        }
        //Com::printFLN(PSTR("InsertWait"));
        return 1;
    }
    return 0;
}

void MachineLine::logLine() {
#ifdef DEBUG_QUEUE_MOVE
    Com::printFLN(Com::tDBGId, (int)this);
	Com::printArrayFLN(Com::tDBGDelta, delta);
    Com::printFLN(Com::tDBGDir, (uint32_t)dir);
    Com::printFLN(Com::tDBGFlags, (uint32_t)flags);
    Com::printFLN(Com::tDBGFullSpeed, fullSpeed);
    Com::printFLN(Com::tDBGVMax, (int32_t)vMax);
	Com::printFLN(Com::tDBGAcceleration, accelerationDistance2);
    Com::printFLN(Com::tDBGAccelerationPrim, (int32_t)accelerationPrim);
	Com::printFLN(Com::tDBGRemainingSteps, stepsRemaining);
#endif // DEBUG_QUEUE_MOVE
}

void MachineLine::waitForXFreeLines(uint8_t b, bool allowMoves) {
    while(getLinesCount() + b > MACHINELINE_CACHE_SIZE) { // wait for a free entry in movement cache
        //GCode::readFromSource();
        Machine::checkForPeriodicalActions(allowMoves);
    }
}

#if ARC_SUPPORT
// Arc function taken from grbl
// The arc is approximated by generating a huge number of tiny, linear segments. The length of each
// segment is configured in settings.mm_per_arc_segment.
void MachineLine::queueArc(float *position, float *target, float *offset, float radius, uint8_t isclockwise) {
    //   int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
	//   plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc
    float center_axisX = position[X_AXIS] + offset[X_AXIS];
    float center_axisY = position[Y_AXIS] + offset[Y_AXIS];
	float r_axisX = -offset[X_AXIS];  // Radius vector from center to current location
    float r_axisY = -offset[Y_AXIS];
    float rt_axisX = target[X_AXIS] - center_axisX;
    float rt_axisY = target[Y_AXIS] - center_axisY;
   
    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_axisX * rt_axisY - r_axisY * rt_axisX, r_axisX * rt_axisX + r_axisY * rt_axisY);
    if (isclockwise) {
        if (angular_travel >= static_cast<float>(-ARC_ANGULAR_TRAVEL_EPSILON)) {
            angular_travel -= static_cast<float>(2.0 * M_PI);
        }
    } else {
        if (angular_travel <= static_cast<float>(ARC_ANGULAR_TRAVEL_EPSILON)) {
            angular_travel += static_cast<float>(2.0 * M_PI);
        }
    }

    //determine how many segments we will be making
    uint16_t segments = 0;
    float millimeters_of_travel = fabs(angular_travel) * radius;
    if (millimeters_of_travel > static_cast<float>(ARC_MM_MIN_TRAVEL)) {
        segments = floor(fabs(0.5f * angular_travel * radius) / sqrt(static_cast<float>(ARC_TOLERANCE) * (2.0f * radius - static_cast<float>(ARC_TOLERANCE))));
    }
    float theta_per_segment = angular_travel / segments;

    //store destination and build deltas for remaining axes
    int32_t destination_steps[A_AXIS_ARRAY];
    Machine::lastCmdPosSteps(destination_steps);

    if (segments) {
        int32_t start_stepsZ = Machine::currentPositionSteps[Z_AXIS];
        int32_t start_stepsA = Machine::currentPositionSteps[A_AXIS];
        int32_t delta_axisZ = destination_steps[Z_AXIS] - start_stepsZ;
        int32_t delta_axisA = destination_steps[A_AXIS] - start_stepsA;
        int32_t deltaSeg_axisZ = (delta_axisZ + (segments / 2)) / segments;
        int32_t deltaSeg_axisA = (delta_axisA + (segments / 2)) / segments;
        int32_t segment_steps[A_AXIS_ARRAY];
        segment_steps[Z_AXIS] = start_stepsZ;
        segment_steps[A_AXIS] = start_stepsA;

        /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
           and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
               r_T = [cos(phi) -sin(phi);
                      sin(phi)  cos(phi] * r ;

           For arc generation, the center of the circle is the axis of rotation and the radius vector is
           defined from the circle center to the initial position. Each line segment is formed by successive
           vector rotations. This requires only two cos() and sin() computations to form the rotation
           matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
           all double numbers are single precision on the Arduino. (True double precision will not have
           round off issues for CNC applications.) Single precision error can accumulate to be greater than
           tool precision in some cases. Therefore, arc path correction is implemented.

           Small angle approximation may be used to reduce computation overhead further. This approximation
           holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
           theta_per_segment would need to be greater than 0.1 rad and ARC_N_SEGMENT_CORRECT would need to be large
           to cause an appreciable drift error. ARC_N_SEGMENT_CORRECT~=25 is more than small enough to correct for
           numerical drift error. ARC_N_SEGMENT_CORRECT may be on the order a hundred(s) before error becomes an
           issue for CNC machines with the single precision Arduino calculations.

           This approximation also allows mc_arc to immediately insert a line segment into the planner
           without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
           a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
           This is important when there are successive arc motions.
        */

        // Vector rotation matrix values
        float cos_T = 2.0f - theta_per_segment * theta_per_segment;
        float sin_T = theta_per_segment * 0.16666667f * (cos_T + 4.0f);
        cos_T *= 0.5f;

        uint8_t segmentUpdate = ARC_N_SEGMENT_CORRECT;
        for (uint16_t i = 1; i < segments; i++) {
            if (!segmentUpdate--) { //25 pieces
                // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
                // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
                float cos_Ti = cos(i * theta_per_segment);
                float sin_Ti = sin(i * theta_per_segment);
                r_axisX = -offset[X_AXIS] * cos_Ti + offset[Y_AXIS] * sin_Ti;
                r_axisY = -offset[X_AXIS] * sin_Ti - offset[Y_AXIS] * cos_Ti;

                segment_steps[Z_AXIS] = start_stepsZ + (i * delta_axisZ) / segments;
                segment_steps[A_AXIS] = start_stepsA + (i * delta_axisA) / segments;

                segmentUpdate = ARC_N_SEGMENT_CORRECT;
            }
            else {
                // Apply vector rotation matrix
                float r_axisi = r_axisX * sin_T + r_axisY * cos_T;
                r_axisX = r_axisX * cos_T - r_axisY * sin_T;
                r_axisY = r_axisi;

                segment_steps[Z_AXIS] += deltaSeg_axisZ;
                segment_steps[A_AXIS] += deltaSeg_axisA;
            }

            segment_steps[X_AXIS] = static_cast<int32_t>(floor((center_axisX + r_axisX) * Machine::axisStepsPerMM[X_AXIS] + 0.5f));
            segment_steps[Y_AXIS] = static_cast<int32_t>(floor((center_axisY + r_axisY) * Machine::axisStepsPerMM[Y_AXIS] + 0.5f));

            MachineLine::queueCartesianMove(segment_steps, ALWAYS_CHECK_ENDSTOPS, true);
        }
    }

    // Ensure last segment arrives at target location.
    MachineLine::queueCartesianMove(destination_steps, ALWAYS_CHECK_ENDSTOPS, true);
}
#endif

/**
  Moves the stepper motors one step. If the last step is reached, the next movement is started.
  The function must be called from a timer loop. It returns the time for the next call.

  Normal linear algorithm
*/
uint32_t MachineLine::bresenhamStep() {
	if(!cur) {
        setCurrentLine();
        if(cur->isBlocked()) { // This step is in computation - shouldn't happen
			cur = NULL;
            return 2000;
        }
        HAL::allowInterrupts();
#ifdef INCLUDE_DEBUG_NO_MOVE
        if(Machine::debugNoMoves()) { // simulate a move, but do nothing in reality
            removeCurrentLineForbidInterrupt();
            return 1000;
        }
#endif
		ANALYZER_OFF(ANALYZER_CH0);
        if(cur->isWarmUp()) {
            // This is a warm up move to initialize the path planner correctly. Just waste
            // a bit of time to get the planning up to date.
            if(linesCount <= cur->getWaitForXLinesFilled()) {
				cur = NULL;
                return 2000;
            }
            long wait = cur->getWaitTicks();
            removeCurrentLineForbidInterrupt();
            return(wait); // waste some time for path optimization to fill up
        } // End if WARMUP

        HAL::forbidInterrupts();

        //Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
        //Determine direction of movement,check if endstop was hit
        if (cur->isXMove()) {
            Machine::enableXStepper(false);
            Machine::setXDirection(cur->isXPositiveMove(), false);
        }
        if (cur->isYMove()) {
            Machine::enableYStepper(false);
            Machine::setYDirection(cur->isYPositiveMove(), false);
        }
        if (cur->isZMove()) {
            Machine::enableZStepper(false);
            Machine::setZDirection(cur->isZPositiveMove(), false);
        }
        if (cur->isAMove()) {
            Machine::enableAStepper(false);
            Machine::setADirection(cur->isAPositiveMove(), false);
        }

		cur->fixStartAndEndSpeed();
		HAL::allowInterrupts();
        MachineLine::cur_errupd = cur->delta[cur->primaryAxis];
		if(!cur->areParameterUpToDate()) { // should never happen, but with bad timings???
			cur->updateStepsParameter();
		}
		Machine::vMaxReached = cur->vStart;
		Machine::stepNumber = 0;
		Machine::timer = 0;
		HAL::forbidInterrupts();

#if LASER_SUPPORT
        if (Machine::mode == MACHINE_MODE_LASER) {
            Laser::setIntensity(cur->laserIntensity);
        }
#endif
#if FAN_CONTROL_SUPPORT
        FanControl::setSpeed(cur->fanSpeed);
#endif
#if VACUUM_SUPPORT
        Vacuum::setState(cur->toolFlags & FLAG_TOOL_VACUUM_ON);
#endif
#if COOLANT_SUPPORT && COOLANT_MIST_PIN > -1
        CoolantMist::setState(cur->toolFlags & FLAG_TOOL_MIST_ON);
#endif
#if COOLANT_SUPPORT && COOLANT_FLOOD_PIN > -1
        CoolantFlood::setState(cur->toolFlags & FLAG_TOOL_FLOOD_ON);
#endif

#if MULTI_XENDSTOP_HOMING
		Machine::multiXHomeFlags = MULTI_XENDSTOP_ALL;  // move all x motors until endstop says differently
#endif
#if MULTI_YENDSTOP_HOMING
		Machine::multiYHomeFlags = MULTI_YENDSTOP_ALL;  // move all y motors until endstop says differently
#endif
#if MULTI_ZENDSTOP_HOMING
        Machine::multiZHomeFlags = MULTI_ZENDSTOP_ALL;  // move all z motors until endstop says differently
#endif
        Machine::stepsTillNextCalc  = 1;
        Machine::stepsSinceLastCalc = 1;

        return Machine::interval; // Wait an other 50% from last step to make the 100% full
    } // End !cur

    /////////////// Step //////////////////////////

#if PAUSE_SUPPORT
	if(!Pause::isActive() || Pause::steps() < PAUSE_STEPS) {
#endif
#if STEPPER_HIGH_DELAY > 0
        uint8_t doHighDelay = false;
#endif
#if QUICK_STEP
        while(Machine::stepsTillNextCalc) {
#else
        if (Machine::stepsTillNextCalc) {
#endif
#if STEPPER_HIGH_DELAY > 0
            if (doHighDelay) HAL::delayMicroseconds(STEPPER_HIGH_DELAY);
            else doHighDelay = true;
#endif
            if (cur->isXMove())
                if ((cur->error[X_AXIS] -= cur->delta[X_AXIS]) < 0) {
                    Machine::startXStep(false);
                    cur->error[X_AXIS] += MachineLine::cur_errupd;
#ifdef DEBUG_STEPCOUNT
                    cur->totalStepsRemaining--;
#endif
                }
            if (cur->isYMove())
                if ((cur->error[Y_AXIS] -= cur->delta[Y_AXIS]) < 0) {
                    Machine::startYStep(false);
                    cur->error[Y_AXIS] += MachineLine::cur_errupd;
#ifdef DEBUG_STEPCOUNT
                    cur->totalStepsRemaining--;
#endif
                }
            if (cur->isZMove())
                if ((cur->error[Z_AXIS] -= cur->delta[Z_AXIS]) < 0) {
                    Machine::startZStep(false);
                    cur->error[Z_AXIS] += MachineLine::cur_errupd;
#ifdef DEBUG_STEPCOUNT
                    cur->totalStepsRemaining--;
#endif
                }
            if (cur->isAMove())
                if ((cur->error[A_AXIS] -= cur->delta[A_AXIS]) < 0) {
                    Machine::startAStep(false);
                    cur->error[A_AXIS] += MachineLine::cur_errupd;
#ifdef DEBUG_STEPCOUNT
                    cur->totalStepsRemaining--;
#endif
                }

#if STEPPER_HIGH_DELAY > 0
            HAL::delayMicroseconds(STEPPER_HIGH_DELAY);
#endif
            Machine::endXYZASteps(false);

            cur->stepsRemaining--;
            Machine::stepsTillNextCalc--;
        }
#if PAUSE_SUPPORT
    }
#if LASER_SUPPORT
    else {
        Laser::setIntensity(0);
    }
#endif
#endif

#if QUICK_STEP == 0
    if (Machine::stepsTillNextCalc) {
        return Machine::interval;
    }
#endif

    /////////// main calculation interval //////////////

    cur->checkEndstops();

    // Allow interrupts for other types, timer1 is still disabled
    HAL::allowInterrupts();

#if PAUSE_SUPPORT
    uint8_t ret = Pause::calculateSteps(Machine::stepsSinceLastCalc);
#if LASER_SUPPORT
    if(ret) Laser::setIntensity(cur->laserIntensity);
#endif
#endif

#if RAMP_ACCELERATION
	//If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
    if (cur->moveAccelerating()) { // we are accelerating
		Machine::vMaxReached = HAL::ComputeV(Machine::timer, cur->fAcceleration) + cur->vStart; // v = v0 + a * t
        if(Machine::vMaxReached > cur->vMax) Machine::vMaxReached = cur->vMax;
        speed_t v = Machine::updateStepsPerTimerCall(Machine::vMaxReached);
		Machine::interval   = HAL::CPUDivU2(v);
#if QUICK_STEP
        Machine::timer += Machine::interval;
#else
        Machine::timer += Machine::interval * Machine::stepsTillNextCalc;
#endif
        Machine::stepNumber += Machine::stepsSinceLastCalc; // only used for moveAccelerating
	} else if (cur->moveDecelerating()) { // time to slow down
        uint16_t v = HAL::ComputeV(Machine::timer, cur->fAcceleration);
        if (v > Machine::vMaxReached) {  // if deceleration goes too far it can become too large
            v = cur->vEnd;
        } else {
			v = Machine::vMaxReached - v;
            if (v < cur->vEnd) v = cur->vEnd; // extra steps at the end of deceleration due to rounding errors
        }
        v = Machine::updateStepsPerTimerCall(v);
		Machine::interval = HAL::CPUDivU2(v);
#if QUICK_STEP
        Machine::timer += Machine::interval;
#else
        Machine::timer += Machine::interval * Machine::stepsTillNextCalc;
#endif
    } else 
#endif
    { // full speed reached
        Machine::updateStepsPerTimerCall(cur->vMax, cur->fullInterval);
    }

    if (Machine::stepsTillNextCalc > cur->stepsRemaining) {
        Machine::stepsTillNextCalc = cur->stepsRemaining;
    }
    Machine::stepsSinceLastCalc = Machine::stepsTillNextCalc;

#if PAUSE_SUPPORT
	if(Pause::steps()) Machine::interval += Machine::interval * Pause::steps() / PAUSE_SLOPE;
#endif
#if FEED_DIAL_SUPPORT
    if (FeedDial::value() != FEED_DIAL_MAX_VALUE) Machine::interval = (Machine::interval << FEED_DIAL_BITS) / FeedDial::value();
#endif
	if(cur->stepsRemaining <= 0 || cur->isNoMove()) { // line finished
#ifdef DEBUG_STEPCOUNT
        if(cur->totalStepsRemaining) {
            Com::printF(Com::tDBGMissedSteps, cur->totalStepsRemaining);
            Com::printFLN(Com::tComma, cur->stepsRemaining);
        }
#endif
        uint32_t interval = Machine::interval;
        removeCurrentLineForbidInterrupt();
        Machine::disableAllowedStepper();
        //At the end of all moves set all drivers to current state except the laser we want set to minimum intensity
		if(linesCount == 0) {
#if LASER_SUPPORT
            if (Machine::mode == MACHINE_MODE_LASER) {
                Laser::setIntensity(0);
            }
#endif
#if FAN_CONTROL_SUPPORT
            FanControl::setSpeed(FanControl::next());
#endif
#if VACUUM_SUPPORT
            Vacuum::setState(Vacuum::next());
#endif
#if COOLANT_SUPPORT && COOLANT_MIST_PIN > -1
            CoolantMist::setState(CoolantMistDriver::next());
#endif
#if COOLANT_SUPPORT && COOLANT_FLOOD_PIN > -1
            CoolantFlood::setState(CoolantFloodDriver::next());
#endif
        }

        interval = interval >> 1; // 50% of time to next call to do cur=0
        Machine::interval = interval;

        DEBUG_MEMORY;

        return interval;
	} // Do even

    return Machine::interval;
}
