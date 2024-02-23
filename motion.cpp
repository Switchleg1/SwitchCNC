#include "SwitchCNC.h"

// ================ Sanity checks ================
#ifndef STEP_DOUBLER_FREQUENCY
#error Please add new parameter STEP_DOUBLER_FREQUENCY to your configuration.
#else
#if STEP_DOUBLER_FREQUENCY < 7000 || STEP_DOUBLER_FREQUENCY > 20000
#error STEP_DOUBLER_FREQUENCY should be in range 10000-16000.
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
#if PRINTLINE_CACHE_SIZE < 4
#error PRINTLINE_CACHE_SIZE must be at least 5
#endif

//Inactivity shutdown variables
millis_t previousMillisCmd = 0;
millis_t maxInactiveTime = MAX_INACTIVE_TIME * 1000L;
millis_t stepperInactiveTime = STEPPER_INACTIVE_TIME * 1000L;
long baudrate = BAUDRATE;         ///< Communication speed rate.
uint8_t pwm_pos[NUM_PWM];
PrintLine PrintLine::lines[PRINTLINE_CACHE_SIZE]; ///< Cache for print moves.
PrintLine *PrintLine::cur = NULL;               ///< Current printing line
ufast8_t PrintLine::linesWritePos = 0;            ///< Position where we write the next cached line move.
volatile ufast8_t PrintLine::linesCount = 0;      ///< Number of lines cached 0 = nothing to do.
ufast8_t PrintLine::linesPos = 0;                 ///< Position for executing line movement.

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
void PrintLine::moveRelativeDistanceInSteps(int32_t x, int32_t y, int32_t z, int32_t a, float feedrate, bool waitEnd, bool checkEndstop, bool pathOptimize) {
#if MOVE_X_WHEN_HOMED == 1 || MOVE_Y_WHEN_HOMED == 1 || MOVE_Z_WHEN_HOMED == 1
    if(!Printer::isHoming() && !Printer::isNoDestinationCheck()) {
#if MOVE_X_WHEN_HOMED
        if(!Printer::isXHomed())
            x = 0;
#endif
#if MOVE_Y_WHEN_HOMED
        if(!Printer::isYHomed())
            y = 0;
#endif
#if MOVE_Z_WHEN_HOMED
        if(!Printer::isZHomed() && !Printer::isZProbingActive())
            z = 0;
#endif
    }
#endif //  MOVE_X_WHEN_HOMED == 1 || MOVE_Y_WHEN_HOMED == 1 || MOVE_Z_WHEN_HOMED == 1
    float savedFeedrate = Printer::feedrate;
    Printer::destinationSteps[X_AXIS] = Printer::currentPositionSteps[X_AXIS] + x;
	Printer::destinationSteps[Y_AXIS] = Printer::currentPositionSteps[Y_AXIS] + y;
	Printer::destinationSteps[Z_AXIS] = Printer::currentPositionSteps[Z_AXIS] + z;
	Printer::destinationSteps[A_AXIS] = Printer::currentPositionSteps[A_AXIS] + a;
	Printer::feedrate = feedrate;
#if DISTORTION_CORRECTION
    Printer::destinationSteps[Z_AXIS] -= Printer::zCorrectionStepsIncluded; // correct as it will be added later in Cartesian move computation
#endif
	queueCartesianMove(checkEndstop, pathOptimize);
    Printer::feedrate = savedFeedrate;
    Printer::updateCurrentPosition(false);
    if(waitEnd)
        Commands::waitUntilEndOfAllMoves();
    previousMillisCmd = HAL::timeInMilliseconds();
}

/** Adds the steps converted to mm to the lastCmdPos position and moves to that position using Printer::moveToReal.
Will use Printer::isPositionAllowed to prevent illegal moves.

\param x Distance in x direction in steps
\param y Distance in y direction in steps
\param z Distance in z direction in steps
\param e Distance in e direction in steps
\param feedrate Feed rate to be used in mm/s. Gets new active feedrate.
\param waitEnd If true will block until move is finished.
\param pathOptimize If false start and end speeds get fixed to minimum values.
*/
void PrintLine::moveRelativeDistanceInStepsReal(int32_t x, int32_t y, int32_t z, int32_t a, float feedrate, bool waitEnd, bool pathOptimize) {
#if MOVE_X_WHEN_HOMED == 1 || MOVE_Y_WHEN_HOMED == 1 || MOVE_Z_WHEN_HOMED == 1
    if(!Printer::isHoming() && !Printer::isNoDestinationCheck()) { // prevent movements when not homed
#if MOVE_X_WHEN_HOMED
        if(!Printer::isXHomed())
            x = 0;
#endif
#if MOVE_Y_WHEN_HOMED
        if(!Printer::isYHomed())
            y = 0;
#endif
#if MOVE_Z_WHEN_HOMED
        if(!Printer::isZHomed() && !Printer::isZProbingActive())
            z = 0;
#endif
    }
#endif // MOVE_X_WHEN_HOMED == 1 || MOVE_Y_WHEN_HOMED == 1 || MOVE_Z_WHEN_HOMED == 1
    Printer::lastCmdPos[X_AXIS] += x * Printer::invAxisStepsPerMM[X_AXIS];
    Printer::lastCmdPos[Y_AXIS] += y * Printer::invAxisStepsPerMM[Y_AXIS];
	Printer::lastCmdPos[Z_AXIS] += z * Printer::invAxisStepsPerMM[Z_AXIS];
	Printer::lastCmdPos[A_AXIS] += a * Printer::invAxisStepsPerMM[A_AXIS];

	if(!Printer::isPositionAllowed(Printer::lastCmdPos[X_AXIS], Printer::lastCmdPos[Y_AXIS], Printer::lastCmdPos[Z_AXIS])) {
		return; // ignore move
	}
	Printer::moveToReal(Printer::lastCmdPos[X_AXIS], Printer::lastCmdPos[Y_AXIS], Printer::lastCmdPos[Z_AXIS], Printer::lastCmdPos[A_AXIS], feedrate, pathOptimize);
    Printer::updateCurrentPosition();
    if(waitEnd)
        Commands::waitUntilEndOfAllMoves();
    previousMillisCmd = HAL::timeInMilliseconds();
}

#if DISTORTION_CORRECTION || ALWAYS_SPLIT_LINES
/* Special version which adds distortion correction to z. Gets called from queueCartesianMove if needed. */
void PrintLine::queueCartesianSegmentTo(uint8_t check_endstops, uint8_t pathOptimize) {

    // Correct the bumps
    Printer::zCorrectionStepsIncluded = Printer::distortion.correct(Printer::destinationSteps[X_AXIS], Printer::destinationSteps[Y_AXIS], Printer::destinationSteps[Z_AXIS]);
    Printer::destinationSteps[Z_AXIS] += Printer::zCorrectionStepsIncluded;
#if DEBUG_DISTORTION
    Com::printF(PSTR("zCorr:"), Printer::zCorrectionStepsIncluded * Printer::invAxisStepsPerMM[Z_AXIS], 3);
    Com::printF(PSTR(" atX:"), Printer::destinationSteps[X_AXIS]*Printer::invAxisStepsPerMM[X_AXIS]);
    Com::printFLN(PSTR(" atY:"), Printer::destinationSteps[Y_AXIS]*Printer::invAxisStepsPerMM[Y_AXIS]);
#endif
    PrintLine::waitForXFreeLines(1);
    uint8_t newPath = PrintLine::insertWaitMovesIfNeeded(pathOptimize, 0);
    PrintLine *p = PrintLine::getNextWriteLine();

	float axisDistanceMM[A_AXIS_ARRAY]; // Axis movement in mm
	p->flags = (check_endstops ? FLAG_CHECK_ENDSTOPS : 0);
    p->joinFlags = 0;
    if(!pathOptimize) p->setEndSpeedFixed(true);
    p->dir = 0;
    //Find direction
    //Printer::zCorrectionStepsIncluded = 0;
	for(uint8_t axis = 0; axis < A_AXIS_ARRAY; axis++) {
		p->delta[axis] = Printer::destinationSteps[axis] - Printer::currentPositionSteps[axis];
		p->secondSpeed = Printer::fanSpeed;
        if(p->delta[axis] >= 0)
            p->setPositiveDirectionForAxis(axis);
        else
            p->delta[axis] = -p->delta[axis];
        axisDistanceMM[axis] = p->delta[axis] * Printer::invAxisStepsPerMM[axis];
        if(p->delta[axis]) p->setMoveOfAxis(axis);
        Printer::currentPositionSteps[axis] = Printer::destinationSteps[axis];
    }
    if(p->isNoMove()) {
        if(newPath)   // need to delete dummy elements, otherwise commands can get locked.
            PrintLine::resetPathPlanner();
        return; // No steps included
    }
	float xydist2 = 0;
#if ENABLE_BACKLASH_COMPENSATION
    if((p->isXYZAMove()) && ((p->dir & XYZA_DIRPOS) ^ (Printer::backlashDir & XYZA_DIRPOS)) & (Printer::backlashDir >> 3)) { // We need to compensate backlash, add a move
        PrintLine::waitForXFreeLines(2);
        uint8_t wpos2 = PrintLine::linesWritePos + 1;
        if(wpos2 >= PRINTLINE_CACHE_SIZE) wpos2 = 0;
        PrintLine *p2 = &PrintLine::lines[wpos2];
        memcpy(p2, p, sizeof(PrintLine)); // Move current data to p2
		uint8_t changed = (p->dir & XYZA_DIRPOS) ^ (Printer::backlashDir & XYZA_DIRPOS);
		float back_diff[A_AXIS_ARRAY]; // Axis movement in mm
		back_diff[X_AXIS] = (changed & 1 ? (p->isXPositiveMove() ? Printer::backlash[X_AXIS] : -Printer::backlash[X_AXIS]) : 0);
		back_diff[Y_AXIS] = (changed & 2 ? (p->isYPositiveMove() ? Printer::backlash[Y_AXIS] : -Printer::backlash[Y_AXIS]) : 0);
		back_diff[Z_AXIS] = (changed & 4 ? (p->isZPositiveMove() ? Printer::backlash[Z_AXIS] : -Printer::backlash[Z_AXIS]) : 0);
		back_diff[A_AXIS] = (changed & 8 ? (p->isAPositiveMove() ? Printer::backlash[A_AXIS] : -Printer::backlash[A_AXIS]) : 0);
		p->dir &= XYZA_DIRPOS; // x,y and z are already correct
		for(uint8_t i = 0; i < A_AXIS_ARRAY; i++) {
            float f = back_diff[i] * Printer::axisStepsPerMM[i];
            p->delta[i] = abs((long)f);
            if(p->delta[i]) p->dir |= XSTEP << i;
		}

		//Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
		if(p->delta[Y_AXIS] > p->delta[X_AXIS] && p->delta[Y_AXIS] > p->delta[Z_AXIS] && p->delta[Y_AXIS] > p->delta[A_AXIS]) p->primaryAxis = Y_AXIS;
		else if(p->delta[X_AXIS] > p->delta[Z_AXIS] && p->delta[X_AXIS] > p->delta[A_AXIS]) p->primaryAxis = X_AXIS;
		else if(p->delta[Z_AXIS] > p->delta[A_AXIS]) p->primaryAxis = Z_AXIS;
		else p->primaryAxis = A_AXIS;
		p->stepsRemaining = p->delta[p->primaryAxis];

		//Feedrate calc based on XYZA travel distance
		if(p->isXMove())
			xydist2 += back_diff[X_AXIS] * back_diff[X_AXIS];

		if(p->isYMove())
			xydist2 += back_diff[Y_AXIS] * back_diff[Y_AXIS];

		if(p->isZMove())
			xydist2 += back_diff[Z_AXIS] * back_diff[Z_AXIS];

		if(p->isAMove())
			xydist2 += back_diff[A_AXIS] * back_diff[A_AXIS];

		p->distance = sqrt(xydist2);
		Printer::backlashDir = (Printer::backlashDir & 56) | (p2->dir & XYZA_DIRPOS);
        p->calculateMove(back_diff, pathOptimize, p->primaryAxis);
        p = p2; // use saved instance for the real move
    }
#endif

    //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
	if(p->delta[Y_AXIS] > p->delta[X_AXIS] && p->delta[Y_AXIS] > p->delta[Z_AXIS] && p->delta[Y_AXIS] > p->delta[A_AXIS]) p->primaryAxis = Y_AXIS;
	else if(p->delta[X_AXIS] > p->delta[Z_AXIS] && p->delta[X_AXIS] > p->delta[A_AXIS]) p->primaryAxis = X_AXIS;
	else if(p->delta[Z_AXIS] > p->delta[A_AXIS]) p->primaryAxis = Z_AXIS;
	else p->primaryAxis = A_AXIS;
	p->stepsRemaining = p->delta[p->primaryAxis];

	if(p->isXMove())
		xydist2 += axisDistanceMM[X_AXIS] * axisDistanceMM[X_AXIS];

	if(p->isYMove())
		xydist2 += axisDistanceMM[Y_AXIS] * axisDistanceMM[Y_AXIS];

	if(p->isZMove())
		xydist2 += axisDistanceMM[Z_AXIS] * axisDistanceMM[Z_AXIS];

	if(p->isAMove())
		xydist2 += axisDistanceMM[A_AXIS] * axisDistanceMM[A_AXIS];

	p->distance = (float)sqrt(xydist2);

    p->calculateMove(axisDistanceMM, pathOptimize, p->primaryAxis);
}
#endif

/**
  Put a move to the current destination coordinates into the movement cache.
  If the cache is full, the method will wait, until a place gets free. During
  wait communication and temperature control is enabled.

  destinationSteps must be excluding any z correction! We will add that if required here.

  @param check_endstops Read end stop during move.
*/
void PrintLine::queueCartesianMove(uint8_t check_endstops, uint8_t pathOptimize) {
	ENSURE_POWER
    Printer::constrainDestinationCoords();
	Printer::unsetAllSteppersDisabled();

#if ALWAYS_SPLIT_LINES
	// we are inside correction height so we split all moves in lines of max. 10 mm and add them
	// including a z correction
	int32_t deltas[A_AXIS_ARRAY], start[A_AXIS_ARRAY];
	float len = 0;
	float len2 = 0;
	for(fast8_t i = 0; i < A_AXIS_ARRAY; i++) {
		deltas[i] = Printer::destinationSteps[i] - Printer::currentPositionSteps[i];
		start[i] = Printer::currentPositionSteps[i];
		len2 = Printer::invAxisStepsPerMM[i] * deltas[i];
		len += len2 * len2;
	}
#if DISTORTION_CORRECTION
	if(Printer::distortion.isEnabled() && (Printer::destinationSteps[Z_AXIS] < Printer::distortion.zMaxSteps()) && !Printer::isZProbingActive() && !Printer::isHoming())
	{
		deltas[Z_AXIS] += Printer::zCorrectionStepsIncluded;
		start[Z_AXIS] -= Printer::zCorrectionStepsIncluded;
	}
#endif
	if(len < SEGMENT_SIZE * SEGMENT_SIZE) { // no splitting required
		queueCartesianSegmentTo(check_endstops, pathOptimize);
		return;
	}
	// we need to split longer lines to follow bed curvature
	len = sqrt(len);
	int segments = (static_cast<int>(len) + SEGMENT_SIZE - 1) / SEGMENT_SIZE;
#if DEBUG_DISTORTION
	Com::printF(PSTR("Split line len:"), len);
	Com::printFLN(PSTR(" segments:"), segments);
#endif
	for(int i = 1; i <= segments; i++) {
		for(fast8_t j = 0; j < A_AXIS_ARRAY; j++) {
			Printer::destinationSteps[j] = start[j] + (i * deltas[j]) / segments;
		}
		queueCartesianSegmentTo(check_endstops, pathOptimize);
	}
	return;
#else
#if DISTORTION_CORRECTION
	if(Printer::distortion.isEnabled() && (Printer::destinationSteps[Z_AXIS] < Printer::distortion.zMaxSteps()) && !Printer::isZProbingActive() && !Printer::isHoming()) {
		// we are inside correction height so we split all moves in lines of max. 10 mm and add them
        // including a z correction
		int32_t deltas[A_AXIS_ARRAY], start[A_AXIS_ARRAY];
		for(fast8_t i = 0; i < A_AXIS_ARRAY; i++) {
			deltas[i] = Printer::destinationSteps[i] - Printer::currentPositionSteps[i];
			start[i] = Printer::currentPositionSteps[i];
        }
		deltas[Z_AXIS] += Printer::zCorrectionStepsIncluded;
        start[Z_AXIS] -= Printer::zCorrectionStepsIncluded;
		float dx = Printer::invAxisStepsPerMM[X_AXIS] * deltas[X_AXIS];
		float dy = Printer::invAxisStepsPerMM[Y_AXIS] * deltas[Y_AXIS];
		float len = dx * dx + dy * dy;
		if(len < SEGMENT_SIZE * SEGMENT_SIZE) { // no splitting required
			queueCartesianSegmentTo(check_endstops, pathOptimize);
			return;
		}
		// we need to split longer lines to follow bed curvature
		len = sqrt(len);
		int segments = (static_cast<int>(len) + SEGMENT_SIZE - 1) / SEGMENT_SIZE;
#if DEBUG_DISTORTION
		Com::printF(PSTR("Split line len:"), len);
		Com::printFLN(PSTR(" segments:"), segments);
#endif
		for(int i = 1; i <= segments; i++) {
			for(fast8_t j = 0; j < A_AXIS_ARRAY; j++) {
				Printer::destinationSteps[j] = start[j] + (i * deltas[j]) / segments;
			}
			queueCartesianSegmentTo(check_endstops, pathOptimize);
		}
		return;
	}
#endif
    waitForXFreeLines(1);
    uint8_t newPath = insertWaitMovesIfNeeded(pathOptimize, 0);
    PrintLine *p = getNextWriteLine();

	float axisDistanceMM[A_AXIS_ARRAY]; // Axis movement in mm
	p->flags = (check_endstops ? FLAG_CHECK_ENDSTOPS : 0);
    p->joinFlags = 0;
    if(!pathOptimize) p->setEndSpeedFixed(true);
    p->dir = 0;
    //Find direction
    Printer::zCorrectionStepsIncluded = 0;
	for(uint8_t axis = 0; axis < A_AXIS_ARRAY; axis++) {
		p->delta[axis] = Printer::destinationSteps[axis] - Printer::currentPositionSteps[axis];
		p->secondSpeed = Printer::fanSpeed;
        if(p->delta[axis] >= 0)
            p->setPositiveDirectionForAxis(axis);
        else
			p->delta[axis] = -p->delta[axis];
		axisDistanceMM[axis] = p->delta[axis] * Printer::invAxisStepsPerMM[axis];
        if(p->delta[axis]) p->setMoveOfAxis(axis);
        Printer::currentPositionSteps[axis] = Printer::destinationSteps[axis];
    }
    if(p->isNoMove()) {
        if(newPath)   // need to delete dummy elements, otherwise commands can get locked.
            resetPathPlanner();
        return; // No steps included
    }
	float xydist2 = 0;
#if ENABLE_BACKLASH_COMPENSATION
	if((p->isXYZAMove()) && ((p->dir & XYZA_DIRPOS) ^ (Printer::backlashDir & XYZA_DIRPOS)) & (Printer::backlashDir >> 3)) { // We need to compensate backlash, add a move
        waitForXFreeLines(2);
        uint8_t wpos2 = linesWritePos + 1;
        if(wpos2 >= PRINTLINE_CACHE_SIZE) wpos2 = 0;
        PrintLine *p2 = &lines[wpos2];
        memcpy(p2, p, sizeof(PrintLine)); // Move current data to p2
		uint8_t changed = (p->dir & XYZA_DIRPOS) ^ (Printer::backlashDir & XYZA_DIRPOS);
		float back_diff[A_AXIS_ARRAY]; // Axis movement in mm
		back_diff[X_AXIS] = (changed & 1 ? (p->isXPositiveMove() ? Printer::backlash[X_AXIS] : -Printer::backlash[X_AXIS]) : 0);
		back_diff[Y_AXIS] = (changed & 2 ? (p->isYPositiveMove() ? Printer::backlash[Y_AXIS] : -Printer::backlash[Y_AXIS]) : 0);
		back_diff[Z_AXIS] = (changed & 4 ? (p->isZPositiveMove() ? Printer::backlash[Z_AXIS] : -Printer::backlash[Z_AXIS]) : 0);
		back_diff[A_AXIS] = (changed & 8 ? (p->isAPositiveMove() ? Printer::backlash[A_AXIS] : -Printer::backlash[A_AXIS]) : 0);
		p->dir &= XYZA_DIRPOS; // x,y and z are already correct
		for(uint8_t i = 0; i < A_AXIS_ARRAY; i++) {
            float f = back_diff[i] * Printer::axisStepsPerMM[i];
			p->delta[i] = abs((long)f);
            if(p->delta[i]) p->dir |= XSTEP << i;
        }
        //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
		if(p->delta[Y_AXIS] > p->delta[X_AXIS] && p->delta[Y_AXIS] > p->delta[Z_AXIS] && p->delta[Y_AXIS] > p->delta[A_AXIS]) p->primaryAxis = Y_AXIS;
		else if(p->delta[X_AXIS] > p->delta[Z_AXIS] && p->delta[X_AXIS] > p->delta[A_AXIS]) p->primaryAxis = X_AXIS;
		else if(p->delta[Z_AXIS] > p->delta[A_AXIS]) p->primaryAxis = Z_AXIS;
		else  p->primaryAxis = A_AXIS;
		p->stepsRemaining = p->delta[p->primaryAxis];

		//Feedrate calc based on XYZA travel distance
		if(p->isXMove())
			xydist2 += back_diff[X_AXIS] * back_diff[X_AXIS];

		if(p->isYMove())
			xydist2 += back_diff[Y_AXIS] * back_diff[Y_AXIS];

		if(p->isZMove())
			xydist2 += back_diff[Z_AXIS] * back_diff[Z_AXIS];

		if(p->isAMove())
			xydist2 += back_diff[A_AXIS] * back_diff[A_AXIS];

		p->distance = sqrt(xydist2);
        // 56 seems to be xstep|ystep|e_posdir which just seems odd
        Printer::backlashDir = (Printer::backlashDir & 56) | (p2->dir & XYZA_DIRPOS);
        p->calculateMove(back_diff, pathOptimize, p->primaryAxis);
        p = p2; // use saved instance for the real move
    }
#endif

    //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
	if(p->delta[Y_AXIS] > p->delta[X_AXIS] && p->delta[Y_AXIS] > p->delta[Z_AXIS] && p->delta[Y_AXIS] > p->delta[A_AXIS]) p->primaryAxis = Y_AXIS;
	else if(p->delta[X_AXIS] > p->delta[Z_AXIS] && p->delta[X_AXIS] > p->delta[A_AXIS]) p->primaryAxis = X_AXIS;
	else if(p->delta[Z_AXIS] > p->delta[A_AXIS]) p->primaryAxis = Z_AXIS;
	else  p->primaryAxis = A_AXIS;
	p->stepsRemaining = p->delta[p->primaryAxis];

	if(p->isXMove())
		xydist2 += axisDistanceMM[X_AXIS] * axisDistanceMM[X_AXIS];

	if(p->isYMove())
		xydist2 += axisDistanceMM[Y_AXIS] * axisDistanceMM[Y_AXIS];

	if(p->isZMove())
		xydist2 += axisDistanceMM[Z_AXIS] * axisDistanceMM[Z_AXIS];

	if(p->isAMove())
		xydist2 += axisDistanceMM[A_AXIS] * axisDistanceMM[A_AXIS];

	p->distance = (float)sqrt(xydist2);

	p->calculateMove(axisDistanceMM, pathOptimize, p->primaryAxis);
#endif
}

void PrintLine::calculateMove(float axisDistanceMM[], uint8_t pathOptimize, fast8_t drivingAxis) {
	long axisInterval[A_AXIS_ARRAY];
    //float timeForMove = (float)(F_CPU)*distance / (isXOrYMove() ? RMath::max(Printer::minimumSpeed, Printer::feedrate) : Printer::feedrate); // time is in ticks
    float timeForMove = (float)(F_CPU) * distance / Printer::feedrate; // time is in ticks
    //bool critical = Printer::isZProbingActive();
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
		axisInterval[X_AXIS] = axisDistanceMM[X_AXIS] * toTicks / (Printer::maxFeedrate[X_AXIS]); // mm*ticks/s/(mm/s*steps) = ticks/step
		limitInterval = RMath::max(axisInterval[X_AXIS], limitInterval);
    } else axisInterval[X_AXIS] = 0;
    if(isYMove()) {
		axisInterval[Y_AXIS] = axisDistanceMM[Y_AXIS] * toTicks / Printer::maxFeedrate[Y_AXIS];
		limitInterval = RMath::max(axisInterval[Y_AXIS], limitInterval);
    } else axisInterval[Y_AXIS] = 0;
	if(isZMove()) { // normally no move in z direction
		axisInterval[Z_AXIS] = axisDistanceMM[Z_AXIS] * toTicks / Printer::maxFeedrate[Z_AXIS]; // must prevent overflow!
		limitInterval = RMath::max(axisInterval[Z_AXIS], limitInterval);
	} else axisInterval[Z_AXIS] = 0;
	if(isAMove()) { // normally no move in a direction
		axisInterval[A_AXIS] = axisDistanceMM[A_AXIS] * toTicks / Printer::maxFeedrate[A_AXIS]; // must prevent overflow!
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
	uint32_t *accel = Printer::maxAccelerationStepsPerSquareSecond;
	for(fast8_t i = 0; i < A_AXIS_ARRAY ; i++) {
        if(isMoveOfAxis(i))
            // v = a * t => t = v/a = F_CPU/(c*a) => 1/t = c*a/F_CPU
            slowestAxisPlateauTimeRepro = RMath::min(slowestAxisPlateauTimeRepro, (float)axisInterval[i] * (float)accel[i]); //  steps/s^2 * step/tick  Ticks/s^2
	}

	// Errors for delta move are initialized in timer (except extruder)
	error[X_AXIS] = error[Y_AXIS] = error[Z_AXIS] = error[A_AXIS] = delta[primaryAxis] >> 1;
    invFullSpeed = 1.0 / fullSpeed;
	accelerationPrim = slowestAxisPlateauTimeRepro / axisInterval[primaryAxis]; // a = v/t = F_CPU/(c*t): Steps/s^2
	//Now we can calculate the new primary axis acceleration, so that the slowest axis max acceleration is not violated
	fAcceleration = 262144.0 * (float)accelerationPrim / F_CPU; // will overflow without float!
	accelerationDistance2 = 2.0 * distance * slowestAxisPlateauTimeRepro * fullSpeed / ((float)F_CPU); // mm^2/s^2
    startSpeed = endSpeed = minSpeed = safeSpeed(drivingAxis);
    if(startSpeed > Printer::feedrate)
        startSpeed = endSpeed = minSpeed = Printer::feedrate;
    // Can accelerate to full speed within the line
	if (startSpeed * startSpeed + accelerationDistance2 >= fullSpeed * fullSpeed)
        setNominalMove();

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
    if(Printer::debugEcho()) {
        logLine();
        Com::printFLN(Com::tDBGLimitInterval, limitInterval);
        Com::printFLN(Com::tDBGMoveDistance, distance);
        Com::printFLN(Com::tDBGCommandedFeedrate, Printer::feedrate);
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
void PrintLine::updateTrapezoids() {
    ufast8_t first = linesWritePos;
    PrintLine *firstLine;
    PrintLine *act = &lines[linesWritePos];
    InterruptProtectedBlock noInts;

    // First we find out how far back we could go with optimization.

    ufast8_t maxfirst = linesPos; // first non fixed segment we might change
    if(maxfirst != linesWritePos)
        nextPlannerIndex(maxfirst); // don't touch the line printing
    // Now ignore enough segments to gain enough time for path planning
    millis_t timeleft = 0;
    // Skip as many stored moves as needed to gain enough time for computation
#if PRINTLINE_CACHE_SIZE < 10
#define minTime 4500L * PRINTLINE_CACHE_SIZE
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
	PrintLine *previous = &lines[previousIndex]; // segment before the one we are inserting
	computeMaxJunctionSpeed(previous, act); // Set maximum junction speed if we have a real move before
    // Increase speed if possible neglecting current speed
    backwardPlanner(linesWritePos, first);
    // Reduce speed to reachable speeds
    forwardPlanner(first);

#ifdef DEBUG_PLANNER
    if(Printer::debugEcho()) {
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
        if(Printer::debugEcho()) {
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
    if(Printer::debugEcho()) {
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
inline void PrintLine::computeMaxJunctionSpeed(PrintLine *previous, PrintLine *current) {
    // if we are here we have to identical move types
    // either pure extrusion -> pure extrusion or
	// move -> move (with or without extrusion)
    // First we compute the normalized jerk for speed 1
	float factor = 1.0;
	float maxJoinSpeed = RMath::min(current->fullSpeed, previous->fullSpeed);

	float dx = fabs(current->speed[X_AXIS] - previous->speed[X_AXIS]);
	float dy = fabs(current->speed[Y_AXIS] - previous->speed[Y_AXIS]);
	float dz = fabs(current->speed[Z_AXIS] - previous->speed[Z_AXIS]);
	float da = fabs(current->speed[A_AXIS] - previous->speed[A_AXIS]);

	if(dx > Printer::maxJerk[X_AXIS]) {
		factor = Printer::maxJerk[X_AXIS] / dx;
	}
	if(dy > Printer::maxJerk[Y_AXIS]) {
		factor = RMath::min(factor, Printer::maxJerk[Y_AXIS] / dy);
	}
	if(dz > Printer::maxJerk[Z_AXIS]) {
		factor = RMath::min(factor, Printer::maxJerk[Z_AXIS] / dz);
	}
	if(da > Printer::maxJerk[A_AXIS]) {
		factor = RMath::min(factor, Printer::maxJerk[A_AXIS] / da);
	}

	previous->maxJunctionSpeed = maxJoinSpeed * factor; // set speed limit
#ifdef DEBUG_QUEUE_MOVE
    if(Printer::debugEcho()) {
        Com::printF(PSTR("ID:"), (int)previous);
        Com::printFLN(PSTR(" MJ:"), previous->maxJunctionSpeed);
    }
#endif // DEBUG_QUEUE_MOVE
}

/** Update parameter used by updateTrapezoids

Computes the acceleration/deceleration steps and advanced parameter associated.
*/
void PrintLine::updateStepsParameter() {
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
    if(Printer::debugEcho()) {
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
inline void PrintLine::backwardPlanner(ufast8_t start, ufast8_t last) {
    PrintLine *act = &lines[start], *previous;
    float lastJunctionSpeed = act->endSpeed; // Start always with safe speed

    //PREVIOUS_PLANNER_INDEX(last); // Last element is already fixed in start speed
    while(start != last) {
        previousPlannerIndex(start);
        previous = &lines[start];
        previous->block();
		// Avoid speed calculation once cruising in split delta move

        // Avoid speed calculations if we know we can accelerate within the line
        lastJunctionSpeed = (act->isNominalMove() ? act->fullSpeed : sqrt(lastJunctionSpeed * lastJunctionSpeed + act->accelerationDistance2)); // acceleration is acceleration*distance*2! What can be reached if we try?
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

void PrintLine::forwardPlanner(ufast8_t first) {
    PrintLine *act;
    PrintLine *next = &lines[first];
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
		vmaxRight = (act->isNominalMove() ? act->fullSpeed : sqrt(leftSpeed * leftSpeed + act->accelerationDistance2));
        if(vmaxRight > act->endSpeed) { // Could be higher next run?
            if(leftSpeed < act->minSpeed) {
                leftSpeed = act->minSpeed;
                act->endSpeed = sqrt(leftSpeed * leftSpeed + act->accelerationDistance2);
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
                vmaxRight = sqrt(leftSpeed * leftSpeed + act->accelerationDistance2);
            }
            act->startSpeed = leftSpeed;
            act->endSpeed = RMath::max(act->minSpeed, vmaxRight);
            next->startSpeed = leftSpeed = RMath::max(RMath::min(act->endSpeed, act->maxJunctionSpeed), next->minSpeed);
            next->setStartSpeedFixed(true);
        }
    } // While
    next->startSpeed = RMath::max(next->minSpeed, leftSpeed); // This is the new segment, which is updated anyway, no extra flag needed.
}


inline float PrintLine::safeSpeed(fast8_t drivingAxis)
{
	float xMin = Printer::maxJerk[X_AXIS] * 0.5;
	float yMin = Printer::maxJerk[Y_AXIS] * 0.5;
	float zMin = Printer::maxJerk[Z_AXIS] * 0.5;
	float aMin = Printer::maxJerk[A_AXIS] * 0.5;
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
uint8_t PrintLine::insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines) {
    if(linesCount == 0 && pathOptimize) { // First line after some time - warm up needed
		//return 0;
		uint8_t w = 4;
        while(w--) {
            PrintLine *p = getNextWriteLine();
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

void PrintLine::logLine() {
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

void PrintLine::waitForXFreeLines(uint8_t b, bool allowMoves) {
    while(getLinesCount() + b > PRINTLINE_CACHE_SIZE) { // wait for a free entry in movement cache
        //GCode::readFromSerial();
        Commands::checkForPeriodicalActions(allowMoves);
    }
}

#if ARC_SUPPORT
// Arc function taken from grbl
// The arc is approximated by generating a huge number of tiny, linear segments. The length of each
// segment is configured in settings.mm_per_arc_segment.
void PrintLine::arc(float *position, float *target, float *offset, float radius, uint8_t isclockwise) {
    //   int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
	//   plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc
    float center_axis0 = position[X_AXIS] + offset[X_AXIS];
    float center_axis1 = position[Y_AXIS] + offset[Y_AXIS];
    //float linear_travel = 0; //target[axis_linear] - position[axis_linear];
	float r_axis0 = -offset[0];  // Radius vector from center to current location
    float r_axis1 = -offset[1];
    float rt_axis0 = target[0] - center_axis0;
    float rt_axis1 = target[1] - center_axis1;
    /*long xtarget = Printer::destinationSteps[X_AXIS];
    long ytarget = Printer::destinationSteps[Y_AXIS];
    long ztarget = Printer::destinationSteps[Z_AXIS];
    */
    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
    if ((!isclockwise && angular_travel <= 0.00001) || (isclockwise && angular_travel < -0.000001)) {
        angular_travel += 2.0f * M_PI;
    }
    if (isclockwise) {
        angular_travel -= 2.0f * M_PI;
    }

    float millimeters_of_travel = fabs(angular_travel) * radius; //hypot(angular_travel*radius, fabs(linear_travel));
    if (millimeters_of_travel < 0.001f) {
        return;// treat as succes because there is nothing to do;
    }
    //uint16_t segments = (radius>=BIG_ARC_RADIUS ? floor(millimeters_of_travel/MM_PER_ARC_SEGMENT_BIG) : floor(millimeters_of_travel/MM_PER_ARC_SEGMENT));
    // Increase segment size if printing faster then computation speed allows
    uint16_t segments = (Printer::feedrate > 60.0f ? floor(millimeters_of_travel / RMath::min(static_cast<float>(MM_PER_ARC_SEGMENT_BIG), Printer::feedrate * 0.01666f * static_cast<float>(MM_PER_ARC_SEGMENT))) : floor(millimeters_of_travel / static_cast<float>(MM_PER_ARC_SEGMENT)));
    if(segments == 0) segments = 1;
    /*
      // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
      // by a number of discrete segments. The inverse feed_rate should be correct for the sum of
      // all segments.
      if (invert_feed_rate) { feed_rate *= segments; }
    */
    float theta_per_segment = angular_travel / segments;
    //float linear_per_segment = linear_travel/segments;

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
       theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
       to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
       numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
       issue for CNC machines with the single precision Arduino calculations.

       This approximation also allows mc_arc to immediately insert a line segment into the planner
       without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
       a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
       This is important when there are successive arc motions.
    */
    // Vector rotation matrix values
    float cos_T = 1 - 0.5 * theta_per_segment * theta_per_segment; // Small angle approximation
    float sin_T = theta_per_segment;

    float arc_target[4];
    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    int8_t count = 0;

    // Initialize the linear axis
	//arc_target[axis_linear] = position[axis_linear];
    for (i = 1; i < segments; i++) {
        // Increment (segments-1)

        if((count & 3) == 0) {
            //GCode::readFromSerial();
            Commands::checkForPeriodicalActions(false);
		}

        if (count < N_ARC_CORRECTION) { //25 pieces
            // Apply vector rotation matrix
            r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
            r_axis0 = r_axis0 * cos_T - r_axis1 * sin_T;
            r_axis1 = r_axisi;
            count++;
        } else {
            // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
            // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
            cos_Ti  = cos(i * theta_per_segment);
            sin_Ti  = sin(i * theta_per_segment);
            r_axis0 = -offset[0] * cos_Ti + offset[1] * sin_Ti;
            r_axis1 = -offset[0] * sin_Ti - offset[1] * cos_Ti;
            count = 0;
        }

        // Update arc_target location
        arc_target[X_AXIS] = center_axis0 + r_axis0;
        arc_target[Y_AXIS] = center_axis1 + r_axis1;
        //arc_target[axis_linear] += linear_per_segment;
		Printer::moveToReal(arc_target[X_AXIS], arc_target[Y_AXIS], IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE);
    }
    // Ensure last segment arrives at target location.
	Printer::moveToReal(target[X_AXIS], target[Y_AXIS], IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE);
}
#endif



/**
  Moves the stepper motors one step. If the last step is reached, the next movement is started.
  The function must be called from a timer loop. It returns the time for the next call.

  Normal linear algorithm
*/
int lastblk = -1;
int32_t cur_errupd;
int32_t PrintLine::bresenhamStep() { // version for Cartesian printer
	if(cur == NULL)
    {
        setCurrentLine();
        if(cur->isBlocked()) { // This step is in computation - shouldn't happen
            /*if(lastblk!=(int)cur) // can cause output errors!
            {
                HAL::allowInterrupts();
                lastblk = (int)cur;
                Com::printFLN(Com::tBLK,lines_count);
            }*/
			cur = NULL;
            return 2000;
        }
        HAL::allowInterrupts();
        lastblk = -1;
#if INCLUDE_DEBUG_NO_MOVE
        if(Printer::debugNoMoves()) { // simulate a move, but do nothing in reality
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
        //Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.

		if(cur->isXMove()) Printer::enableXStepper();
		if(cur->isYMove()) Printer::enableYStepper();
		if(cur->isZMove()) Printer::enableZStepper();
		if(cur->isAMove()) Printer::enableAStepper();

		cur->fixStartAndEndSpeed();
		HAL::allowInterrupts();
		cur_errupd = cur->delta[cur->primaryAxis];
		if(!cur->areParameterUpToDate()) { // should never happen, but with bad timings???
			cur->updateStepsParameter();
		}
		Printer::vMaxReached = cur->vStart;
		Printer::stepNumber = 0;
		Printer::timer = 0;
		HAL::forbidInterrupts();

		//Determine direction of movement,check if endstop was hit
		Printer::setXDirection(cur->isXPositiveMove());
		Printer::setYDirection(cur->isYPositiveMove());
		Printer::setZDirection(cur->isZPositiveMove());
		Printer::setADirection(cur->isAPositiveMove());

		Printer::setFanSpeedDirectly(static_cast<uint8_t>(cur->secondSpeed));
#if MULTI_XENDSTOP_HOMING
		Printer::multiXHomeFlags = MULTI_XENDSTOP_ALL;  // move all x motors until endstop says differently
#endif
#if MULTI_YENDSTOP_HOMING
		Printer::multiYHomeFlags = MULTI_YENDSTOP_ALL;  // move all y motors until endstop says differently
#endif
#if MULTI_ZENDSTOP_HOMING
        Printer::multiZHomeFlags = MULTI_ZENDSTOP_ALL;  // move all z motors until endstop says differently
#endif
        return Printer::interval; // Wait an other 50% from last step to make the 100% full
	} // End cur=0

	cur->checkEndstops();

	fast8_t max_loops = 0;
#if defined(PAUSE_PIN) && PAUSE_PIN>-1
	if(!Printer::isPaused || Printer::pauseSteps < PAUSE_STEPS)
	{
#endif // PAUSE
		max_loops = Printer::stepsPerTimerCall;
		if(cur->stepsRemaining < max_loops)
			max_loops = cur->stepsRemaining;

#if defined(PAUSE_PIN) && PAUSE_PIN>-1
		if(Printer::isPaused && max_loops + Printer::pauseSteps > PAUSE_STEPS)
			max_loops = PAUSE_STEPS - Printer::pauseSteps;
#endif

		for(fast8_t loop = 0; loop < max_loops; loop++) {
#if STEPPER_HIGH_DELAY + DOUBLE_STEP_DELAY > 0
			if(loop)
				HAL::delayMicroseconds(STEPPER_HIGH_DELAY + DOUBLE_STEP_DELAY);
#endif
			if(cur->isXMove())
				if((cur->error[X_AXIS] -= cur->delta[X_AXIS]) < 0) {
					cur->startXStep();
					cur->error[X_AXIS] += cur_errupd;
#ifdef DEBUG_STEPCOUNT
					cur->totalStepsRemaining--;
#endif
				}
			if(cur->isYMove())
				if((cur->error[Y_AXIS] -= cur->delta[Y_AXIS]) < 0) {
					cur->startYStep();
					cur->error[Y_AXIS] += cur_errupd;
#ifdef DEBUG_STEPCOUNT
					cur->totalStepsRemaining--;
#endif
				}
			if(cur->isZMove())
				if((cur->error[Z_AXIS] -= cur->delta[Z_AXIS]) < 0) {
					cur->startZStep();
					cur->error[Z_AXIS] += cur_errupd;
#ifdef DEBUG_STEPCOUNT
					cur->totalStepsRemaining--;
#endif
				}
			if(cur->isAMove())
				if((cur->error[A_AXIS] -= cur->delta[A_AXIS]) < 0) {
					cur->startAStep();
					cur->error[A_AXIS] += cur_errupd;
#ifdef DEBUG_STEPCOUNT
					cur->totalStepsRemaining--;
#endif
				}

#if STEPPER_HIGH_DELAY > 0
			HAL::delayMicroseconds(STEPPER_HIGH_DELAY);
#endif
			cur->stepsRemaining--;
			Printer::endXYZASteps();
		} // for loop
#if defined(PAUSE_PIN) && PAUSE_PIN > -1
		if(Printer::isPaused)
		{
			Printer::pauseSteps += max_loops;
		} else
		{
			Printer::pauseSteps -= max_loops;

			if(Printer::pauseSteps < 0)
				Printer::pauseSteps = 0;
		}
	}
#endif // PAUSE
    HAL::allowInterrupts(); // Allow interrupts for other types, timer1 is still disabled
#if RAMP_ACCELERATION
	//If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
    if (cur->moveAccelerating()) { // we are accelerating
		Printer::vMaxReached = HAL::ComputeV(Printer::timer, cur->fAcceleration) + cur->vStart; // v = v0 + a * t
        if(Printer::vMaxReached > cur->vMax) Printer::vMaxReached = cur->vMax;
        unsigned int v = Printer::updateStepsPerTimerCall(Printer::vMaxReached);
		Printer::interval = HAL::CPUDivU2(v);
		Printer::timer += Printer::interval;
		Printer::stepNumber += max_loops; // only used for moveAccelerating
	} else if (cur->moveDecelerating()) { // time to slow down
        unsigned int v = HAL::ComputeV(Printer::timer, cur->fAcceleration);
        if (v > Printer::vMaxReached)   // if deceleration goes too far it can become too large
            v = cur->vEnd;
        else {
			v = Printer::vMaxReached - v;
            if (v < cur->vEnd) v = cur->vEnd; // extra steps at the end of deceleration due to rounding errors
        }
		v = Printer::updateStepsPerTimerCall(v);
		Printer::interval = HAL::CPUDivU2(v);
        Printer::timer += Printer::interval;
    } else { // full speed reached
        // constant speed reached
		if(cur->vMax > STEP_DOUBLER_FREQUENCY) {
#if ALLOW_QUADSTEPPING
			if(cur->vMax > STEP_DOUBLER_FREQUENCY * 2) {
                Printer::stepsPerTimerCall = 4;
                Printer::interval = cur->fullInterval << 2;
            } else {
                Printer::stepsPerTimerCall = 2;
                Printer::interval = cur->fullInterval << 1;
            }
#else
            Printer::stepsPerTimerCall = 2;
            Printer::interval = cur->fullInterval << 1;
#endif
        } else {
            Printer::stepsPerTimerCall = 1;
            Printer::interval = cur->fullInterval;
        }
    }
#else
    Printer::stepsPerTimerCall = 1;
    Printer::interval = cur->fullInterval; // without RAMPS always use full speed
#endif // RAMP_ACCELERATION    
	uint32 interval = Printer::interval;
#if defined(PAUSE_PIN) && PAUSE_PIN > -1
	if(Printer::pauseSteps)
		interval += interval * Printer::pauseSteps / PAUSE_SLOPE;
#endif // PAUSE
#if SPEED_DIAL && SPEED_DIAL_PIN > -1
        interval = (interval << SPEED_DIAL_BITS) / Printer::speed_dial;
#endif // SPEED_DIAL
	if(cur->stepsRemaining <= 0 || cur->isNoMove()) { // line finished
#ifdef DEBUG_STEPCOUNT
        if(cur->totalStepsRemaining) {
            Com::printF(Com::tDBGMissedSteps, cur->totalStepsRemaining);
            Com::printFLN(Com::tComma, cur->stepsRemaining);
        }
#endif

        removeCurrentLineForbidInterrupt();
        Printer::disableAllowedStepper();
		if(linesCount == 0) {
			Printer::setFanSpeedDirectly(Printer::fanSpeed);
        }
#if defined(PAUSE_PIN) && PAUSE_PIN>-1
		interval = interval >> 1;
#else
		interval = Printer::interval = interval >> 1; // 50% of time to next call to do cur=0
#endif // PAUSE
        DEBUG_MEMORY;
	} // Do even
    return interval;
}
