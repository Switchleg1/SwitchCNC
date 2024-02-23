#include "SwitchCNC.h"

const int8_t sensitive_pins[] PROGMEM = SENSITIVE_PINS; // Sensitive pin list for M42
int Commands::lowestRAMValue = MAX_RAM;
int Commands::lowestRAMValueSend = MAX_RAM;

void Commands::commandLoop() {
#ifdef DEBUG_MACHINE
    debugWaitLoop = 1;
#endif
	GCode::readFromSerial();
	GCode *code = GCode::peekCurrentCommand();
	if(code) {
#if SDSUPPORT
		if(sd.savetosd) {
			if(!(code->hasM() && code->M == 29))   // still writing to file
				sd.writeCommand(code);
			else
				sd.finishWrite();
#if ECHO_ON_EXECUTE
			code->echoCommand();
#endif
		} else
#endif
		Commands::executeGCode(code);
		code->popCurrentCommand();
    }
    Machine::defaultLoopActions();
}

void Commands::checkForPeriodicalActions(bool allowNewMoves) {
    Machine::handleInterruptEvent();
	EVENT_PERIODICAL;
    if(!executePeriodical) return; // gets true every 100ms
	executePeriodical = 0;
#if defined(PAUSE_PIN) && PAUSE_PIN>-1
	bool getPaused = (READ(PAUSE_PIN) != !PAUSE_INVERTING);
	if(Machine::isPaused != getPaused)
	{
		if(!MachineLine::hasLines())
		{
			if(getPaused) Machine::pauseSteps = PAUSE_STEPS;
				else Machine::pauseSteps = 0;
		}

		if(getPaused) Com::printFLN(Com::tPaused);
			else Com::printFLN(Com::tUnpaused);
		Machine::isPaused = getPaused;
	}
#endif

#if SPEED_DIAL && SPEED_DIAL_PIN > -1
    uint8 maxSpeedValue = 1 << SPEED_DIAL_BITS;
    uint8 minSpeedValue = (uint)maxSpeedValue * SPEED_DIAL_MIN_PERCENT / 100;
    uint8 speedDivisor  = (ANALOG_MAX_VALUE >> SPEED_DIAL_BITS) * 100 / (100 - SPEED_DIAL_MIN_PERCENT);
#if SPEED_DIAL_INVERT
    uint8 value = (ANALOG_MAX_VALUE - AnalogIn::values[SPEED_DIAL_ANALOG_INDEX]) / speedDivisor + minSpeedValue;
#else
    uint8 value = AnalogIn::values[SPEED_DIAL_ANALOG_INDEX] / speedDivisor + minSpeedValue;
#endif
    if (value > maxSpeedValue) {
        value = maxSpeedValue;
    }
   
    Machine::speed_dial = value;
#endif
	EVENT_TIMER_100MS;
	if(--counter500ms == 0) {
        counter500ms = 5;
		EVENT_TIMER_500MS;
#if TMC_DRIVERS
		Machine::CheckTMCDrivers();
#endif
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif
	}
}

/** \brief Waits until movement cache is empty.

Some commands expect no movement, before they can execute. This function
waits, until the steppers are stopped. In the meanwhile it buffers incoming
commands and manages temperatures.
*/
void Commands::waitUntilEndOfAllMoves() {
#ifdef DEBUG_MACHINE
    debugWaitLoop = 8;
#endif
    while(MachineLine::hasLines()) {
        //GCode::readFromSerial();
        checkForPeriodicalActions(false);
		GCode::keepAlive(Processing);
    }
}

void Commands::waitUntilEndOfAllBuffers() {
    GCode *code = NULL;
#ifdef DEBUG_MACHINE
    debugWaitLoop = 9;
#endif
    while(MachineLine::hasLines() || (code != NULL)) {
        //GCode::readFromSerial();
		code = GCode::peekCurrentCommand();
        if(code) {
#if SDSUPPORT
            if(sd.savetosd) {
                if(!(code->hasM() && code->M == 29))   // still writing to file
                    sd.writeCommand(code);
                else
                    sd.finishWrite();
#if ECHO_ON_EXECUTE
                code->echoCommand();
#endif
            } else
#endif
                Commands::executeGCode(code);
            code->popCurrentCommand();
        }
		Commands::checkForPeriodicalActions(false); // only called from memory
    }
}

void Commands::printCurrentPosition() {
	float x, y, z, a;
	Machine::realPosition(x, y, z, a);
#ifdef DEBUG_POS
	Com::printF(PSTR("RX:"), x, 3); // to debug offset handling
	Com::printF(PSTR(" RY:"), y, 3);
	Com::printFLN(PSTR(" RZ:"), z, 3);
	Com::printFLN(PSTR(" RA:"), a, 3);
	Com::printF(PSTR("CX:"), Machine::coordinateOffset[X_AXIS], 3); // to debug offset handling
	Com::printF(PSTR(" CY:"), Machine::coordinateOffset[Y_AXIS], 3);
	Com::printFLN(PSTR(" CZ:"), Machine::coordinateOffset[Z_AXIS], 3);
#endif
	x += Machine::coordinateOffset[X_AXIS];
    y += Machine::coordinateOffset[Y_AXIS];
	z += Machine::coordinateOffset[Z_AXIS];
	Com::printF(Com::tXColon, x * (Machine::unitIsInches ? 0.03937 : 1), 2);
    Com::printF(Com::tSpaceYColon, y * (Machine::unitIsInches ? 0.03937 : 1), 2);
	Com::printF(Com::tSpaceZColon, z * (Machine::unitIsInches ? 0.03937 : 1), 3);
	Com::printFLN(Com::tSpaceAColon, a * (Machine::unitIsInches ? 0.03937 : 1), 3);
#ifdef DEBUG_POS
	Com::printF(PSTR(" XS:"), Machine::currentPositionSteps[X_AXIS], 3);
	Com::printF(PSTR(" YS:"), Machine::currentPositionSteps[Y_AXIS], 3);
	Com::printF(PSTR(" ZS:"), Machine::currentPositionSteps[Z_AXIS], 3);
	Com::printFLN(PSTR(" AS:"), Machine::currentPositionSteps[A_AXIS], 3);
#endif
}

void Commands::changeFeedrateMultiply(int factor) {
    if(factor < 25) factor = 25;
    if(factor > 500) factor = 500;
    Machine::feedrate *= (float)factor / (float)Machine::feedrateMultiply;
    Machine::feedrateMultiply = factor;
    Com::printFLN(Com::tSpeedMultiply, factor);
}

#if FEATURE_FAN_CONTROL
uint8_t fanKickstart;
#endif
#if FEATURE_FAN2_CONTROL
uint8_t fan2Kickstart;
#endif

void Commands::setFanSpeed(int speed, bool immediately) {
#if FAN_PIN >- 1 && FEATURE_FAN_CONTROL
    if(Machine::fanSpeed == speed)
        return;
    speed = constrain(speed, 0, 255);
	Machine::fanSpeed = speed;
	if(MachineLine::linesCount == 0 || immediately) {
		for(fast8_t i = 0; i < MACHINELINE_CACHE_SIZE; i++)
			MachineLine::lines[i].secondSpeed = speed;         // fill all printline buffers with new fan speed value
        Machine::setFanSpeedDirectly(speed);
    }
    Com::printFLN(Com::tFanspeed, speed); // send only new values to break update loops!
#endif
}
void Commands::setFan2Speed(int speed) {
#if FAN2_PIN >- 1 && FEATURE_FAN2_CONTROL
    speed = constrain(speed, 0, 255);
    Machine::setFan2SpeedDirectly(speed);
    Com::printFLN(Com::tFan2speed, speed); // send only new values to break update loops!
#endif
}

/**
\brief Execute the Arc command stored in com.
*/
#if ARC_SUPPORT
void Commands::processArc(GCode *com) {
	float position[A_AXIS_ARRAY];
	Machine::realPosition(position[X_AXIS], position[Y_AXIS], position[Z_AXIS], position[A_AXIS]);
	if(!Machine::setDestinationStepsFromGCode(com)) return; // For X Y Z A F
    float offset[2] = {Machine::convertToMM(com->hasI() ? com->I : 0), Machine::convertToMM(com->hasJ() ? com->J : 0)};
	float target[A_AXIS_ARRAY] = {Machine::realXPosition(), Machine::realYPosition(), Machine::realZPosition(), Machine::realAPosition()};
    float r;
    if (com->hasR()) {
        /*
        We need to calculate the center of the circle that has the designated radius and passes
        through both the current position and the target position. This method calculates the following
        set of equations where [x,y] is the vector from current to target position, d == magnitude of
        that vector, h == hypotenuse of the triangle formed by the radius of the circle, the distance to
        the center of the travel vector. A vector perpendicular to the travel vector [-y,x] is scaled to the
        length of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2] to form the new point
        [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the center of our arc.

        d^2 == x^2 + y^2
        h^2 == r^2 - (d/2)^2
        i == x/2 - y/d*h
        j == y/2 + x/d*h

        O <- [i,j]
        -  |
        r      -     |
        -        |
        -           | h
        -              |
        [0,0] ->  C -----------------+--------------- T  <- [x,y]
        | <------ d/2 ---->|

        C - Current position
        T - Target position
        O - center of circle that pass through both C and T
        d - distance from C to T
        r - designated radius
        h - distance from center of CT to O

        Expanding the equations:

        d -> sqrt(x^2 + y^2)
        h -> sqrt(4 * r^2 - x^2 - y^2)/2
        i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
        j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2

        Which can be written:

        i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
        j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2

        Which we for size and speed reasons optimize to:

        h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
        i = (x - (y * h_x2_div_d))/2
        j = (y + (x * h_x2_div_d))/2

        */
        r = Machine::convertToMM(com->R);
        // Calculate the change in position along each selected axis
        double x = target[X_AXIS] - position[X_AXIS];
        double y = target[Y_AXIS] - position[Y_AXIS];

        double h_x2_div_d = -sqrt(4 * r * r - x * x - y * y) / hypot(x, y); // == -(h * 2 / d)
        // If r is smaller than d, the arc is now traversing the complex plane beyond the reach of any
        // real CNC, and thus - for practical reasons - we will terminate promptly:
        if(isnan(h_x2_div_d)) {
            Com::printErrorFLN(Com::tInvalidArc);
            return;
        }
        // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
        if (com->G == 3) {
            h_x2_div_d = -h_x2_div_d;
        }

        /* The counter clockwise circle lies to the left of the target direction. When offset is positive,
        the left hand circle will be generated - when it is negative the right hand circle is generated.


        T  <-- Target position

        ^
        Clockwise circles with this center         |          Clockwise circles with this center will have
        will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
        \         |          /
        center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
        |
        |

        C  <-- Current position                                 */


        // Negative R is g-code-alias for "I want a circle with more than 180 degrees of travel" (go figure!),
        // even though it is advised against ever generating such circles in a single line of g-code. By
        // inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
        // travel and thus we get the inadvisable long arcs as prescribed.
        if (r < 0) {
            h_x2_div_d = -h_x2_div_d;
            r = -r; // Finished with r. Set to positive for mc_arc
        }
        // Complete the operation by calculating the actual center of the arc
        offset[0] = 0.5 * (x - (y * h_x2_div_d));
        offset[1] = 0.5 * (y + (x * h_x2_div_d));

    } else { // Offset mode specific computations
        r = hypot(offset[0], offset[1]); // Compute arc radius for arc
    }
    // Set clockwise/counter-clockwise sign for arc computations
    uint8_t isclockwise = com->G == 2;
    // Trace the arc
	MachineLine::arc(position, target, offset, r, isclockwise);
}
#endif

/**
\brief Execute the G command stored in com.
*/
void Commands::processGCode(GCode *com) {
    if(EVENT_UNHANDLED_G_CODE(com)) {
        previousMillisCmd = HAL::timeInMilliseconds();
        return;
    }
    uint32_t codenum; //throw away variable
    switch(com->G) {
    case 0: // G0 -> G1
	case 1: // G1
        if(com->hasS()) Machine::setNoDestinationCheck(com->S != 0);
		if(Machine::setDestinationStepsFromGCode(com)) // For X Y Z A F
			MachineLine::queueCartesianMove(ALWAYS_CHECK_ENDSTOPS, true);
#ifdef DEBUG_QUEUE_MOVE
        {

            InterruptProtectedBlock noInts;
            int lc = (int)MachineLine::linesCount;
            int lp = (int)MachineLine::linesPos;
            int wp = (int)MachineLine::linesWritePos;
            int n = (wp - lp);
            if(n < 0) n += MACHINELINE_CACHE_SIZE;
            noInts.unprotect();
            if(n != lc)
                Com::printFLN(PSTR("Buffer corrupted"));
        }
#endif
    break;
#if ARC_SUPPORT
    case 2: // CW Arc
	case 3: // CCW Arc MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
		processArc(com);
    break;
#endif
    case 4: // G4 dwell
        Commands::waitUntilEndOfAllMoves();
        codenum = 0;
        if(com->hasP()) codenum = com->P; // milliseconds to wait
        if(com->hasS()) codenum = com->S * 1000; // seconds to wait
        codenum += HAL::timeInMilliseconds();  // keep track of when we started waiting
        while((uint32_t)(codenum - HAL::timeInMilliseconds())  < 2000000000 ) {
            GCode::keepAlive(Processing);
            Commands::checkForPeriodicalActions(true);
        }
        break;
    case 20: // G20 Units to inches
        Machine::unitIsInches = 1;
        break;
    case 21: // G21 Units to mm
        Machine::unitIsInches = 0;
        break;
    case 28: { //G28 Home all Axis one at a time
		uint8_t homeAllAxis = (com->hasNoXYZ());
        if(homeAllAxis || !com->hasNoXYZ())
            Machine::homeAxis(homeAllAxis || com->hasX(), homeAllAxis || com->hasY(), homeAllAxis || com->hasZ());
    }
	break;
#if FEATURE_Z_PROBE
	case 31:  // G31 display hall sensor output
		Endstops::update();
		Endstops::update();
		Com::printF(Com::tZProbeState);
		Com::printF(Endstops::zProbe() ? Com::tHSpace : Com::tLSpace);
		Com::println();
		break;
#if DISTORTION_CORRECTION
	case 33:
		if(com->hasE())
		{	// G33 E(x) enable
			if(com->E > 0) Machine::distortion.enable(com->hasP() && com->P == 1);
				else Machine::distortion.disable(com->hasP() && com->P == 1);
		} else if(com->hasL())
		{ // G33 L0 - List distortion matrix
			Machine::distortion.showMatrix();
		} else if(com->hasR())
		{ // G33 R0 - Reset distortion matrix
			Machine::distortion.resetCorrection();
		} else if(com->hasX() || com->hasY() || com->hasZ())
		{ // G33 X<xpos> Y<ypos> Z<zCorrection> - Set correction for nearest point
			if(com->hasX() && com->hasY() && com->hasZ())
			{
				Machine::distortion.set(com->X, com->Y, com->Z);
			} else
			{
				Com::printErrorFLN(PSTR("You need to define X, Y and Z to set a point!"));
			}
		} else if(com->hasF() && com->F > 0)
		{ //G33 F<x> - Filter amount
			Machine::distortion.filter(com->F);
		} else if(com->hasO() && com->O > 0 && com->O < 1)
		{ //G33 O<x> - Smooth amount
       		Machine::distortion.smooth(com->O);
		} else if(com->hasP())
		{ //G33 P<x> - Do distortion measurements
			Endstops::update();
			Endstops::update(); // need to call twice for full update!
			if(Endstops::zProbe())
			{
				Com::printErrorFLN(PSTR("probe triggered before starting G33."));
			} else
			{
				if(com->hasT() && com->T > 0)
				{
					Machine::updateCurrentPosition(true);
					Machine::distortion.resetCorrection();
					Machine::distortion.disable(true);
					if(com->T > 1)
					{
						HAL::eprSetInt16(EPR_DISTORTION_POINTS, com->T);
						Machine::distortionPoints = com->T;
					}
					HAL::eprSetInt16(EPR_DISTORTION_XMIN, -Machine::coordinateOffset[X_AXIS]);
					HAL::eprSetInt16(EPR_DISTORTION_XMAX, Machine::currentPosition[X_AXIS]);
					HAL::eprSetInt16(EPR_DISTORTION_YMIN, -Machine::coordinateOffset[Y_AXIS]);
					HAL::eprSetInt16(EPR_DISTORTION_YMAX, Machine::currentPosition[Y_AXIS]);
					Machine::distortionXMIN = -Machine::coordinateOffset[X_AXIS];
					Machine::distortionXMAX = Machine::currentPosition[X_AXIS]; //SL
					Machine::distortionYMIN = -Machine::coordinateOffset[Y_AXIS];
					Machine::distortionYMAX = Machine::currentPosition[Y_AXIS]; //SL
				}

				float md = -10;
				if(com->hasA() && com->A < 0)
					md = com->A;

				int reps = Z_PROBE_REPETITIONS;
				if(com->hasH() && com->H > 0)
					reps = com->H;

				Machine::measureDistortion(md, reps);
			}
		} else Machine::distortion.reportStatus();
		break;
#endif
	case 34: { // Tool Height
		if(com->hasX())
		{
			float xp = Machine::runProbe(X_AXIS, com->X, Z_PROBE_REPETITIONS);
			if(xp != ILLEGAL_Z_PROBE)
			{
				if(com->hasA())
					if(com->X > 0) xp -= Machine::convertToMM(com->A/2);
						else xp += Machine::convertToMM(com->A/2);

				Machine::coordinateOffset[X_AXIS] = xp - Machine::currentPosition[X_AXIS];
				Com::printF(PSTR(" TOOL OFFSET:"), xp, 3);
				Com::printFLN(PSTR(" X_OFFSET:"), Machine::coordinateOffset[X_AXIS], 3);
			}
		} else if(com->hasY())
		{
			float yp = Machine::runProbe(Y_AXIS, com->Y, Z_PROBE_REPETITIONS);
			if(yp != ILLEGAL_Z_PROBE)
			{
				if(com->hasA())
					if(com->Y > 0) yp -= Machine::convertToMM(com->A/2);
						else yp += Machine::convertToMM(com->A/2);

				Machine::coordinateOffset[Y_AXIS] = yp - Machine::currentPosition[Y_AXIS];
				Com::printF(PSTR(" TOOL OFFSET:"), yp, 3);
				Com::printFLN(PSTR(" Y_OFFSET:"), Machine::coordinateOffset[Y_AXIS], 3);
			}
		} else
		{
			if(com->Z >= 0)
				com->Z = -10;

			float zp = Machine::runProbe(Z_AXIS, com->Z, Z_PROBE_REPETITIONS);
			if(zp != ILLEGAL_Z_PROBE)
			{
				if(com->hasA()) zp += Machine::convertToMM(com->A);
					else zp += EEPROM::zProbeHeight();
				Machine::coordinateOffset[Z_AXIS] = zp - Machine::currentPosition[Z_AXIS];
				Com::printF(PSTR(" TOOL OFFSET:"), zp, 3);
				Com::printFLN(PSTR(" Z_OFFSET:"), Machine::coordinateOffset[Z_AXIS], 3);
				Machine::distortion.SetStartEnd(Machine::distortionStart, Machine::distortionEnd);
			}
		}
		printCurrentPosition();
	}
	break;
#endif
#if TMC_DRIVERS
	case 41:
		Com::printF(PSTR("X Grad:"), Machine::tmcStepperX.pwm_grad_auto(), 3);
		Com::printFLN(PSTR(" X OFS:"), Machine::tmcStepperX.pwm_ofs_auto(), 3);
		Com::printF(PSTR("Y Grad:"), Machine::tmcStepperY.pwm_grad_auto(), 3);
		Com::printFLN(PSTR(" Y OFS:"), Machine::tmcStepperY.pwm_ofs_auto(), 3);
		Com::printF(PSTR("Z Grad:"), Machine::tmcStepperZ.pwm_grad_auto(), 3);
		Com::printFLN(PSTR(" Z OFS:"), Machine::tmcStepperZ.pwm_ofs_auto(), 3);
		Com::printF(PSTR("2 Grad:"), Machine::tmcStepper2.pwm_grad_auto(), 3);
		Com::printFLN(PSTR(" 2 OFS:"), Machine::tmcStepper2.pwm_ofs_auto(), 3);
	break;
#endif
    case 90: // G90
        Machine::relativeCoordinateMode = false;
        if(com->internalCommand)
            Com::printInfoFLN(PSTR("Absolute positioning"));
        break;
	case 91: // G91
		Machine::relativeCoordinateMode = true;
        if(com->internalCommand)
            Com::printInfoFLN(PSTR("Relative positioning"));
        break;
	case 92: { // G92
		float xOff = Machine::coordinateOffset[X_AXIS];
		float yOff = Machine::coordinateOffset[Y_AXIS];
		float zOff = Machine::coordinateOffset[Z_AXIS];
		if(com->hasX()) xOff = Machine::convertToMM(com->X) - Machine::currentPosition[X_AXIS];
		if(com->hasY()) yOff = Machine::convertToMM(com->Y) - Machine::currentPosition[Y_AXIS];
		if(com->hasZ()) zOff = Machine::convertToMM(com->Z) - Machine::currentPosition[Z_AXIS];
		Machine::setOrigin(xOff, yOff, zOff);
		if(com->hasA()) {
			Machine::lastCmdPos[A_AXIS] = Machine::currentPosition[A_AXIS] = Machine::convertToMM(com->A);
			Machine::destinationSteps[A_AXIS] = Machine::currentPositionSteps[A_AXIS] = static_cast<int32_t>(floor(Machine::lastCmdPos[A_AXIS] * Machine::axisStepsPerMM[A_AXIS] + 0.5f));
		}
		if(com->hasX() || com->hasY() || com->hasZ()) {
			Com::printF(PSTR("X_OFFSET:"), Machine::coordinateOffset[X_AXIS], 3);
			Com::printF(PSTR(" Y_OFFSET:"), Machine::coordinateOffset[Y_AXIS], 3);
			Com::printFLN(PSTR(" Z_OFFSET:"), Machine::coordinateOffset[Z_AXIS], 3);
		}
		Machine::distortion.SetStartEnd(Machine::distortionStart, Machine::distortionEnd);
	}
	break;
	case 93: { // G93
		float xOff = Machine::coordinateOffset[X_AXIS];
		float yOff = Machine::coordinateOffset[Y_AXIS];
		float zOff = Machine::coordinateOffset[Z_AXIS];
		if(com->hasX()) xOff = Machine::convertToMM(com->X);
		if(com->hasY()) yOff = Machine::convertToMM(com->Y);
		if(com->hasZ()) zOff = Machine::convertToMM(com->Z);
		Machine::setOrigin(xOff, yOff, zOff);
		if(com->hasX() || com->hasY() || com->hasZ()) {
			Com::printF(PSTR("X_OFFSET:"), Machine::coordinateOffset[X_AXIS], 3);
			Com::printF(PSTR(" Y_OFFSET:"), Machine::coordinateOffset[Y_AXIS], 3);
			Com::printFLN(PSTR(" Z_OFFSET:"), Machine::coordinateOffset[Z_AXIS], 3);
		}
    }
	break;
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
    case 201:
        commandG201(*com);
        break;
    case 202:
        commandG202(*com);
        break;
    case 203:
        commandG203(*com);
        break;
	case 204:
        commandG204(*com);
        break;
    case 205:
        commandG205(*com);
        break;
#endif // defined
    default:
        if(Machine::debugErrors()) {
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
    previousMillisCmd = HAL::timeInMilliseconds();
}
/**
\brief Execute the G command stored in com.
*/
void Commands::processMCode(GCode *com) {
    if(EVENT_UNHANDLED_M_CODE(com))
        return;
    switch( com->M ) {
	case 3: // Spindle
		waitUntilEndOfAllMoves();
		Endstops::update();
		Endstops::update();
		while(!Endstops::zProbe())
		{
			Com::printFLN(PSTR("Unclip Zprobe from tool prior to starting spindle!"));
			millis_t wait = 1000 + HAL::timeInMilliseconds();

			while(wait - HAL::timeInMilliseconds() < 100000) {
				Machine::defaultLoopActions();
			}
			Endstops::update();
			Endstops::update();
		}

		CNCDriver::spindleOnCW(com->hasS() ? com->S : CNC_RPM_MAX);
        break;
	case 4: // Spindle CCW
		waitUntilEndOfAllMoves();
		CNCDriver::spindleOnCCW(com->hasS() ? com->S : CNC_RPM_MAX);
        break;
	case 5: // Spindle
		waitUntilEndOfAllMoves();
		CNCDriver::spindleOff();
		break;
	case 10: // Vacuum
		CNCDriver::vacuumOn();
		break;
	case 11: //Vacuum off
		CNCDriver::vacuumOff();
		break;
	case 17: //M17 is to enable named axis
        {
			Commands::waitUntilEndOfAllMoves();
			bool named = false;
			if(com->hasX()) {
				named = true;
				Machine::enableXStepper();
			}
			if(com->hasY()) {
				named = true;
				Machine::enableYStepper();
			}
			if(com->hasZ()) {
				named = true;
				Machine::enableZStepper();
			}
			if(com->hasA()) {
				named = true;
				Machine::enableAStepper();
			}
			if(!named) {
				Machine::enableXStepper();
				Machine::enableYStepper();
				Machine::enableZStepper();
				Machine::enableAStepper();
			}
			Machine::unsetAllSteppersDisabled();
		}
		break;
	case 18: // M18 is to disable named axis
		{
			Commands::waitUntilEndOfAllMoves();
			bool named = false;
			if(com->hasX()) {
				named = true;
				Machine::disableXStepper();
			}
			if(com->hasY()) {
				named = true;
				Machine::disableYStepper();
			}
			if(com->hasZ()) {
				named = true;
				Machine::disableZStepper();
			}
			if(com->hasA()) {
				named = true;
				Machine::disableAStepper();
			}
			if(!named) {
				Machine::disableXStepper();
				Machine::disableYStepper();
				Machine::disableZStepper();
				Machine::disableAStepper();
				Machine::setAllSteppersDiabled();
			}
		}
		break;
#if SDSUPPORT
	case 20: // M20 - list SD card
        sd.ls();
        break;
    case 21: // M21 - init SD card
        sd.mount();
        break;
    case 22: //M22 - release SD card
        sd.unmount();
        break;
    case 23: //M23 - Select file
        if(com->hasString()) {
            sd.fat.chdir();
            sd.selectFile(com->text);
        }
        break;
    case 24: //M24 - Start SD print
        sd.startPrint();
        break;
    case 25: //M25 - Pause SD print
        sd.pausePrint();
        break;
    case 26: //M26 - Set SD index
        if(com->hasS())
            sd.setIndex(com->S);
        break;
    case 27: //M27 - Get SD status
        sd.printStatus();
        break;
    case 28: //M28 - Start SD write
        if(com->hasString())
            sd.startWrite(com->text);
        break;
    case 29: //M29 - Stop SD write
        //processed in write to file routine above
        //savetosd = false;
        break;
    case 30: // M30 filename - Delete file
        if(com->hasString()) {
            sd.fat.chdir();
            sd.deleteFile(com->text);
        }
        break;
    case 32: // M32 directoryname
        if(com->hasString()) {
            sd.fat.chdir();
            sd.makeDirectory(com->text);
        }
        break;
#endif
    case 42: //M42 -Change pin status via gcode
        if (com->hasP()) {
            int pin_number = com->P;
            for(uint8_t i = 0; i < (uint8_t)sizeof(sensitive_pins); i++) {
                if (pgm_read_byte(&sensitive_pins[i]) == pin_number) {
                    pin_number = -1;
                    break;
                }
            }
            if (pin_number > -1) {
                if(com->hasS()) {
                    if(com->S >= 0 && com->S <= 255) {
                        pinMode(pin_number, OUTPUT);
                        digitalWrite(pin_number, com->S);
                        analogWrite(pin_number, com->S);
                        Com::printF(Com::tSetOutputSpace, pin_number);
                        Com::printFLN(Com::tSpaceToSpace, (int)com->S);
                    } else
                        Com::printErrorFLN(PSTR("Illegal S value for M42"));
                } else {
                    pinMode(pin_number, INPUT_PULLUP);
                    Com::printF(Com::tSpaceToSpace, pin_number);
                    Com::printFLN(Com::tSpaceIsSpace, digitalRead(pin_number));
                }
            } else {
                Com::printErrorFLN(PSTR("Pin can not be set by M42, is in sensitive pins! "));
            }
        }
        break;
#if PS_ON_PIN > -1
	case 80: // M80 - ATX Power On
		Commands::waitUntilEndOfAllMoves();
        previousMillisCmd = HAL::timeInMilliseconds();
        SET_OUTPUT(PS_ON_PIN); //GND
        Machine::setPowerOn(true);
        WRITE(PS_ON_PIN, (POWER_INVERTING ? HIGH : LOW));
		break;
	case 81: // M81 - ATX Power Off
        Commands::waitUntilEndOfAllMoves();
        SET_OUTPUT(PS_ON_PIN); //GND
        Machine::setPowerOn(false);
        WRITE(PS_ON_PIN, (POWER_INVERTING ? LOW : HIGH));
		break;
#endif
    case 84: // M84
        if(com->hasS()) {
            stepperInactiveTime = com->S * 1000;
        } else {
            Commands::waitUntilEndOfAllMoves();
            Machine::kill(true);
        }
        break;
    case 85: // M85
        if(com->hasS())
            maxInactiveTime = (int32_t)com->S * 1000;
        else
            maxInactiveTime = 0;
        break;
    case 92: // M92
        if(com->hasX()) Machine::axisStepsPerMM[X_AXIS] = com->X;
        if(com->hasY()) Machine::axisStepsPerMM[Y_AXIS] = com->Y;
        if(com->hasZ()) Machine::axisStepsPerMM[Z_AXIS] = com->Z;
        Machine::updateDerivedParameter();
		break;
    case 99: { // M99 S<time>
		millis_t wait = 10000;
        if(com->hasS())
            wait = 1000 * com->S;
        if(com->hasX())
            Machine::disableXStepper();
        if(com->hasY())
            Machine::disableYStepper();
        if(com->hasZ())
            Machine::disableZStepper();
        wait += HAL::timeInMilliseconds();
#ifdef DEBUG_MACHINE
        debugWaitLoop = 2;
#endif
        while(wait - HAL::timeInMilliseconds() < 100000) {
			Machine::defaultLoopActions();
        }
        if(com->hasX())
            Machine::enableXStepper();
		if(com->hasY())
            Machine::enableYStepper();
        if(com->hasZ())
			Machine::enableZStepper();
		if(com->hasA())
			Machine::enableAStepper();
    }
    break;
#if SPEED_DIAL && SPEED_DIAL_PIN > -1
    case 105: // M105  get speed. Always returns the speed
        Com::writeToAll = false;
        Com::printF(Com::tTColon, (Machine::speed_dial * 100) >> SPEED_DIAL_BITS);
        Com::printF(Com::tSpaceSlash, 0, 0);
        Com::printF(Com::tSpaceAtColon, 0);
        Com::println();
        break;
#endif

#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    case 106: // M106 Fan On
        if(com->hasI()) {
            if(com->I != 0)
				Machine::flag1 |= MACHINE_FLAG1_IGNORE_M106_COMMAND;
            else
				Machine::flag1 &= ~MACHINE_FLAG1_IGNORE_M106_COMMAND;
        }
		if(!(Machine::flag1 & MACHINE_FLAG1_IGNORE_M106_COMMAND)) {
            if(com->hasP() && com->P == 1)
                setFan2Speed(com->hasS() ? com->S : 255);
            else
                setFanSpeed(com->hasS() ? com->S : 255);
        }
        break;
    case 107: // M107 Fan Off
		if(!(Machine::flag1 & MACHINE_FLAG1_IGNORE_M106_COMMAND)) {
            if(com->hasP() && com->P == 1)
                setFan2Speed(0);
            else
                setFanSpeed(0);
        }
        break;
#endif
    case 111: // M111 enable/disable run time debug flags
        if(com->hasS()) Machine::setDebugLevel(static_cast<uint8_t>(com->S));
        if(com->hasP()) {
            if (com->P > 0) Machine::debugSet(static_cast<uint8_t>(com->P));
            else Machine::debugReset(static_cast<uint8_t>(-com->P));
		}
        break;
    case 115: // M115
        Com::writeToAll = false;
		Com::printFLN(Com::tFirmware);
#if EEPROM_MODE != 0
        Com::cap(PSTR("EEPROM:1"));
#else
        Com::cap(PSTR("EEPROM:0"));
#endif
#if FEATURE_Z_PROBE
        Com::cap(PSTR("Z_PROBE:1"));
#else
        Com::cap(PSTR("Z_PROBE:0"));
#endif
#if PS_ON_PIN>-1
        Com::cap(PSTR("SOFTWARE_POWER:1"));
#else
        Com::cap(PSTR("SOFTWARE_POWER:0"));
#endif
#if CASE_LIGHTS_PIN > -1
        Com::cap(PSTR("TOGGLE_LIGHTS:1"));
#else
        Com::cap(PSTR("TOGGLE_LIGHTS:0"));
#endif
        break;
	case 114: // M114
        Com::writeToAll = false;
		printCurrentPosition();
        if(com->hasS() && com->S) {
            Com::printF(PSTR("XS:"), Machine::currentPositionSteps[X_AXIS]);
            Com::printF(PSTR(" YS:"), Machine::currentPositionSteps[Y_AXIS]);
            Com::printFLN(PSTR(" ZS:"), Machine::currentPositionSteps[Z_AXIS]);
        }
		break;
    case 119: // M119
        Com::writeToAll = false;
        Commands::waitUntilEndOfAllMoves();
        Endstops::update();
        Endstops::update(); // double test to get right signal. Needed for crosstalk protection.
        Endstops::report();
        break;
#if BEEPER_TYPE>0
    case 120: // M120 Test beeper function
        if(com->hasS() && com->hasP())
            beep(com->S, com->P); // Beep test
        break;
#endif
#if RAMP_ACCELERATION
    case 201: // M201
        if(com->hasX()) Machine::maxAccelerationMMPerSquareSecond[X_AXIS] = com->X;
        if(com->hasY()) Machine::maxAccelerationMMPerSquareSecond[Y_AXIS] = com->Y;
        if(com->hasZ()) Machine::maxAccelerationMMPerSquareSecond[Z_AXIS] = com->Z;
		Machine::updateDerivedParameter();
        break;
#endif
    case 203: // M203 Temperature monitor
		if(com->hasX()) {
			Machine::maxFeedrate[X_AXIS] = com->X / 60.0f;
		}
		if(com->hasY()) {
			Machine::maxFeedrate[Y_AXIS] = com->Y / 60.0f;
		}
		if(com->hasZ()) {
			Machine::maxFeedrate[Z_AXIS] = com->Z / 60.0f;
		}
		break;
    case 205: // M205 Show EEPROM settings
        Com::writeToAll = false;
        EEPROM::writeSettings();
        break;
    case 206: // M206 T[type] P[pos] [Sint(long] [Xfloat]  Set eeprom value
        Com::writeToAll = false;
        EEPROM::update(com);
        break;
    case 207: // M207 X<XY jerk> Z<Z Jerk>
		if(com->hasX())
			Machine::maxJerk[X_AXIS] = com->X;
		if(com->hasY())
			Machine::maxJerk[Y_AXIS] = com->Y;
		if(com->hasZ())
			Machine::maxJerk[Z_AXIS] = com->Z;
		if(com->hasA())
			Machine::maxJerk[A_AXIS] = com->A;
		Com::printF(Com::tXJerkColon, Machine::maxJerk[X_AXIS]);
		Com::printF(Com::tYJerkColon, Machine::maxJerk[Y_AXIS]);
		Com::printF(Com::tZJerkColon, Machine::maxJerk[Z_AXIS]);
		Com::printFLN(Com::tAJerkColon, Machine::maxJerk[A_AXIS]);
		break;
    case 220: // M220 S<Feedrate multiplier in percent>
        changeFeedrateMultiply(com->getS(100));
        break;
    case 226: // M226 P<pin> S<state 0/1> - Wait for pin getting state S
        if(!com->hasS() || !com->hasP())
            break;
        {
            bool comp = com->S;
            if(com->hasX()) {
                if(com->X == 0)
                    HAL::pinMode(com->P, INPUT);
                else
                    HAL::pinMode(com->P, INPUT_PULLUP);
            }
            do {
                Commands::checkForPeriodicalActions(true);
                GCode::keepAlive(Waiting);
            } while(HAL::digitalRead(com->P) != comp);
        }
		break;
#if Z_HOME_DIR > 0 && MAX_HARDWARE_ENDSTOP_Z
    case 251: // M251
		Machine::axisLength[Z_AXIS] -= Machine::currentPosition[Z_AXIS];
        Machine::currentPositionSteps[Z_AXIS] = 0;
		Machine::updateDerivedParameter();
		Machine::updateCurrentPosition();
		Com::printFLN(Com::tZProbePrinterHeight, Machine::axisLength[Z_AXIS]);
#if EEPROM_MODE != 0
        EEPROM::storeDataIntoEEPROM(false);
		Com::printFLN(Com::tEEPROMUpdated);
#endif
		Commands::printCurrentPosition();
        break;
#endif
    case 281: // Trigger watchdog
#if FEATURE_WATCHDOG
    {
        if(com->hasX()) {
            HAL::stopWatchdog();
            Com::printFLN(PSTR("Watchdog disabled"));
            break;
        }
        Com::printInfoFLN(PSTR("Triggering watchdog. If activated, the printer will reset."));
        Machine::kill(false);
        HAL::delayMilliseconds(200); // write output
#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
        InterruptProtectedBlock noInts;         // don't disable interrupts on mega2560 and mega1280 because of bootloader bug
#endif
        while(1) {} // Endless loop
    }
#else
    Com::printInfoFLN(PSTR("Watchdog feature was not compiled into this version!"));
#endif
    break;
#if defined(BEEPER_PIN) && BEEPER_PIN>=0
    case 300: { // M300
        int beepS = 1;
        int beepP = 1000;
        if(com->hasS()) beepS = com->S;
        if(com->hasP()) beepP = com->P;
        HAL::tone(BEEPER_PIN, beepS);
        HAL::delayMilliseconds(beepP);
        HAL::noTone(BEEPER_PIN);
    }
    break;
#endif
#if FEATURE_SERVO
    case 340: // M340
        if(com->hasP() && com->P < 4 && com->P >= 0) {
			ENSURE_POWER
            int s = 0;
            if(com->hasS())
                s = com->S;
            uint16_t r = 0;
            if(com->hasR())    // auto off time in ms
                r = com->R;
            HAL::servoMicroseconds(com->P, s, r);
        }
        break;
#endif // FEATURE_SERVO
    case 355: // M355 S<0/1> - Turn case light on/off, no S = report status
        if(com->hasS()) {
            Machine::setCaseLight(com->S);
        } else
            Machine::reportCaseLightStatus();
        break;
    case 360: // M360 - show configuration
        Com::writeToAll = false;
        Machine::showConfiguration();
        break;
    case 400: // M400 Finish all moves
        Commands::waitUntilEndOfAllMoves();
        break;
    case 401: // M401 Memory position
		Machine::SetMemoryPosition();
        break;
    case 402: // M402 Go to stored position
		Machine::GoToMemoryPosition(com->hasX(), com->hasY(), com->hasZ(), com->hasA(), (com->hasF() ? com->F : Machine::feedrate));
		break;
    case 500: { // M500
#if EEPROM_MODE != 0
        EEPROM::storeDataIntoEEPROM(false);
        Com::printInfoFLN(Com::tConfigStoredEEPROM);
#else
        Com::printErrorFLN(Com::tNoEEPROMSupport);
#endif
    }
    break;
    case 501: { // M501
#if EEPROM_MODE != 0
        EEPROM::readDataFromEEPROM();
		Com::printInfoFLN(Com::tConfigLoadedEEPROM);
#else
        Com::printErrorFLN(Com::tNoEEPROMSupport);
#endif
    }
    break;
    case 502: // M502
        EEPROM::restoreEEPROMSettingsFromConfiguration();
		break;
#ifdef DEBUG_QUEUE_MOVE
    case 533: { // M533 Write move data
        InterruptProtectedBlock noInts;
        int lc = (int)MachineLine::linesCount;
        int lp = (int)MachineLine::linesPos;
        int wp = (int)MachineLine::linesWritePos;
        int n = (wp - lp);
        if(n < 0) n += MACHINELINE_CACHE_SIZE;
        noInts.unprotect();
        if(n != lc)
            Com::printFLN(PSTR("Buffer corrupted"));
        Com::printF(PSTR("Buf:"), lc);
        Com::printF(PSTR(",LP:"), lp);
        Com::printFLN(PSTR(",WP:"), wp);
        if(MachineLine::cur == NULL) {
            Com::printFLN(PSTR("No move"));
            if(MachineLine::linesCount > 0) {
                MachineLine &cur = MachineLine::lines[MachineLine::linesPos];
                Com::printF(PSTR("JFlags:"), (int)cur.joinFlags);
                Com::printFLN(PSTR(" Flags:"), (int)cur.flags);
                if(cur.isWarmUp()) {
                    Com::printFLN(PSTR(" warmup:"), (int)cur.getWaitForXLinesFilled());
                }
            }
        } else {
            Com::printF(PSTR("Rem:"), MachineLine::cur->stepsRemaining);
            Com::printFLN(PSTR(" Int:"), Machine::interval);
        }
    }
    break;
#endif // DEBUG_QUEUE_MOVE
#ifdef DEBUG_SEGMENT_LENGTH
    case 534: // M534
        Com::printFLN(PSTR("Max. segment size:"), Machine::maxRealSegmentLength);
        if(com->hasS())
            Machine::maxRealSegmentLength = 0;
        break;
#endif
#ifdef DEBUG_REAL_JERK
        Com::printFLN(PSTR("Max. jerk measured:"), Machine::maxRealJerk);
        if(com->hasS())
            Machine::maxRealJerk = 0;
        break;
#endif
    case 670:
#if EEPROM_MODE != 0
        if(com->hasS()) {
            HAL::eprSetByte(EPR_VERSION, static_cast<uint8_t>(com->S));
            HAL::eprSetByte(EPR_INTEGRITY_BYTE, EEPROM::computeChecksum());
        }
#endif
		break;
    case 999: // Stop fatal error take down
        if(com->hasS())
            GCode::fatalError(PSTR("Testing fatal error"));
        else
            GCode::resetFatalError();
        break;
    default:
        if(Machine::debugErrors()) {
            Com::writeToAll = false;
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
}

/**
\brief Execute the command stored in com.
*/
void Commands::executeGCode(GCode *com) {
#if NEW_COMMUNICATION
    // Set return channel for private commands. By default all commands send to all receivers.
    GCodeSource *actSource = GCodeSource::activeSource;
    GCodeSource::activeSource = com->source;
    Com::writeToAll = true;
#endif
    if (INCLUDE_DEBUG_COMMUNICATION) {
        if(Machine::debugCommunication()) {
            if(com->hasG() || (com->hasM() && com->M != 111)) {
                previousMillisCmd = HAL::timeInMilliseconds();
#if NEW_COMMUNICATION
                GCodeSource::activeSource = actSource;
#endif
                return;
            }
        }
    }
    if(com->hasG()) processGCode(com);
    else if(com->hasM()) processMCode(com);
    else if(com->hasT()) {    // Process T code
		// SL set tool
    } else {
        if(Machine::debugErrors()) {
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
#if NEW_COMMUNICATION
    GCodeSource::activeSource = actSource;
#endif
}

void Commands::emergencyStop() {
#if defined(KILL_METHOD) && KILL_METHOD == 1
    HAL::resetHardware();
#else
    //HAL::forbidInterrupts(); // Don't allow interrupts to do their work
    Machine::kill(false);
    for(uint8_t i = 0; i < PWM_FAN2; i++)
        pwm_pos[i] = 0;
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    WRITE(FAN_PIN, 0);
#endif
	HAL::delayMilliseconds(200);
    InterruptProtectedBlock noInts;
    while(1) {}
#endif
}

void Commands::checkFreeMemory() {
    int newfree = HAL::getFreeRam();
    if(newfree < lowestRAMValue)
        lowestRAMValue = newfree;
}

void Commands::writeLowestFreeRAM() {
    if(lowestRAMValueSend > lowestRAMValue) {
        lowestRAMValueSend = lowestRAMValue;
        Com::printFLN(Com::tFreeRAM, lowestRAMValue);
    }
}
