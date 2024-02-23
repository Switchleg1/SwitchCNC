#include "SwitchCNC.h"

#if FEATURE_Z_PROBE
void Printer::prepareForProbing() {
#ifndef SKIP_PROBE_PREPARE
    // 1. Ensure we are homed so positions make sense
    if(!Printer::isHomedAll()) {
        Printer::homeAxis(true, true, true);
    }
    // 3. Ensure we can activate z probe at current xy position
    Printer::updateCurrentPosition(true);
	Commands::waitUntilEndOfAllMoves();
#endif
}

/** \brief Activate z-probe

Tests if switching from active tool to z-probe is possible at current position. If not the operation is aborted.
If ok, it runs start script, checks z position and applies the z-probe offset.

\param runScript Run start z-probe script from configuration.
\param enforceStartHeight If true moves z to EEPROM::zProbeBedDistance() + (EEPROM::zProbeHeight() > 0 ? EEPROM::zProbeHeight() : 0) + 0.1 if current position is higher.
\return True if activation was successful. */
bool Printer::startProbing(bool runScript, bool enforceStartHeight) {
    return true;
}

/** \brief Deactivate z-probe. */
void Printer::finishProbing() {
}

/** \brief Measure distance to bottom at current position.

This is the most important function for bed leveling. It does
1. Run probe start script if first = true and runStartScript = true
2. Position zProbe at current position if first = true. If we are more then maxStartHeight away from bed we also go down to that distance.
3. Measure the the steps until probe hits the bed.
4. Undo positioning to z probe and run finish script if last = true.

Now we compute the nozzle height as follows:
a) Compute average height from repeated measurements
b) Add zProbeHeight to correct difference between triggering point and nozzle height above bed
c) If Z_PROBE_Z_OFFSET_MODE == 1 we add zProbeZOffset() that is coating thickness if we measure below coating with indictive sensor.
d) Add distortion correction.
e) Add bending correction

Then we return the measured and corrected z distance.

\param first If true, Printer::startProbing is called.
\param last If true, Printer::finishProbing is called at the end.
\param repeat Number of repetitions to average measurement errors.
\param runStartScript If true tells startProbing to run start script.
\param enforceStartHeight Tells start script to enforce a maximum distance to bed.
\return ILLEGAL_Z_PROBE on errors or measured distance.
*/
float Printer::runZProbe(bool first, bool last, uint8_t repeat, bool runStartScript, bool enforceStartHeight) {
    if(first) {
        if(!startProbing(runStartScript, enforceStartHeight))
            return ILLEGAL_Z_PROBE;
    }
    Commands::waitUntilEndOfAllMoves();
#if defined(Z_PROBE_USE_MEDIAN) && Z_PROBE_USE_MEDIAN
	int32_t measurements[Z_PROBE_REPETITIONS];
	repeat = RMath::min(repeat, Z_PROBE_REPETITIONS);
#else
	int32_t sum = 0;
#endif
    int32_t probeDepth;
    int32_t shortMove = static_cast<int32_t>((float)Z_PROBE_SWITCHING_DISTANCE * axisStepsPerMM[Z_AXIS]); // distance to go up for repeated moves
	int32_t lastCorrection = currentPositionSteps[Z_AXIS]; // starting position
    //int32_t updateZ = 0;
	waitForZProbeStart();
#if defined(Z_PROBE_DELAY) && Z_PROBE_DELAY > 0
	HAL::delayMilliseconds(Z_PROBE_DELAY);
#endif
    Endstops::update();
    Endstops::update(); // need to call twice for full update!
    if(Endstops::zProbe()) {
        Com::printErrorFLN(PSTR("z-probe triggered before starting probing."));
        return ILLEGAL_Z_PROBE;
	}
    for(int8_t r = 0; r < repeat; r++) {
		probeDepth = 2 * (Printer::axisMaxSteps[Z_AXIS] - Printer::axisMinSteps[Z_AXIS]); // probe should always hit within this distance
        stepsRemainingAtZHit = -1; // Marker that we did not hit z probe
        setZProbingActive(true);
#if defined(Z_PROBE_DELAY) && Z_PROBE_DELAY > 0
        HAL::delayMilliseconds(Z_PROBE_DELAY);
#endif
        PrintLine::moveRelativeDistanceInSteps(0, 0, -probeDepth, 0, EEPROM::zProbeSpeed(), true, true);
        setZProbingActive(false);
        if(stepsRemainingAtZHit < 0) {
            Com::printErrorFLN(Com::tZProbeFailed);
            return ILLEGAL_Z_PROBE;
		}
        currentPositionSteps[Z_AXIS] += stepsRemainingAtZHit; // now current position is correct
#if defined(Z_PROBE_USE_MEDIAN) && Z_PROBE_USE_MEDIAN
        measurements[r] = lastCorrection - currentPositionSteps[Z_AXIS];
#else
        sum += lastCorrection - currentPositionSteps[Z_AXIS];
#endif
        //Com::printFLN(PSTR("ZHSteps:"),lastCorrection - currentPositionSteps[Z_AXIS]);
        if(r + 1 < repeat) {
            // go only shortest possible move up for repetitions
			PrintLine::moveRelativeDistanceInSteps(0, 0, shortMove, 0, EEPROM::zProbeSpeed(), true, true);
            if(Endstops::zProbe()) {
				Com::printErrorFLN(PSTR("z-probe did not untrigger on repetitive measurement - maybe you need to increase distance!"));
                return ILLEGAL_Z_PROBE;
            }
        }
#ifdef Z_PROBE_RUN_AFTER_EVERY_PROBE
        GCode::executeFString(PSTR(Z_PROBE_RUN_AFTER_EVERY_PROBE));
#endif
	}

    // Go back to start position
	PrintLine::moveRelativeDistanceInSteps(0, 0, lastCorrection - currentPositionSteps[Z_AXIS], 0, EEPROM::zProbeSpeed(), true, true);
    if(Endstops::zProbe()) { // did we untrigger? If not don't trust result!
		Com::printErrorFLN(PSTR("z-probe did not untrigger on repetitive measurement - maybe you need to increase distance!"));
        return ILLEGAL_Z_PROBE;
    }
    updateCurrentPosition(false);
    //Com::printFLN(PSTR("after probe"));
    //Commands::printCurrentPosition();
#if defined(Z_PROBE_USE_MEDIAN) && Z_PROBE_USE_MEDIAN
	// bubble sort the measurements
	int32_t tmp;
	for(fast8_t i = 0 ; i < repeat - 1; i++) {  // n numbers require at most n-1 rounds of swapping
		for(fast8_t j = 0; j < repeat - i - 1; j++)  {  //
			if( measurements[j] > measurements[j + 1] ) {   // out of order?			
				tmp = measurements[j]; // swap them:
				measurements[j] = measurements[j + 1];
				measurements[j + 1] = tmp;
			}
		}
	}
// process result
	float distance = static_cast<float>(measurements[repeat >> 1]) * invAxisStepsPerMM[Z_AXIS] + EEPROM::zProbeHeight();
#else
	float distance = static_cast<float>(sum) * invAxisStepsPerMM[Z_AXIS] / static_cast<float>(repeat) + EEPROM::zProbeHeight();
#endif
    //Com::printFLN(PSTR("OrigDistance:"),distance);
#if Z_PROBE_Z_OFFSET_MODE == 1
    distance += EEPROM::zProbeZOffset(); // We measured including coating, so we need to add coating thickness!
#endif

#if DISTORTION_CORRECTION
    float zCorr = 0;
    if(Printer::distortion.isEnabled()) {
        zCorr = distortion.correct(currentPositionSteps[X_AXIS]/* + EEPROM::zProbeXOffset() * axisStepsPerMM[X_AXIS]*/, currentPositionSteps[Y_AXIS]
								  /* + EEPROM::zProbeYOffset() * axisStepsPerMM[Y_AXIS]*/, axisMinSteps[Z_AXIS]) * invAxisStepsPerMM[Z_AXIS];
        distance += zCorr;
    }
#endif

    Com::printF(Com::tZProbe, distance, 3);
    Com::printF(Com::tSpaceXColon, realXPosition());
#if DISTORTION_CORRECTION
    if(Printer::distortion.isEnabled()) {
        Com::printF(Com::tSpaceYColon, realYPosition());
		Com::printFLN(PSTR(" zCorr:"), zCorr, 3);
    } else {
        Com::printFLN(Com::tSpaceYColon, realYPosition());
    }
#else
    Com::printFLN(Com::tSpaceYColon, realYPosition());
#endif
    if(Endstops::zProbe()) {
		Com::printErrorFLN(PSTR("z-probe did not untrigger after going back to start position."));
        return ILLEGAL_Z_PROBE;
    }
    if(last)
        finishProbing();
    return distance;
}

float Printer::runProbe(uint8_t axisDirection, float maxDistance, uint8_t repeat) {
	Commands::waitUntilEndOfAllMoves();

	int32_t sum = 0;
	int32_t probeDepth = Printer::axisStepsPerMM[axisDirection] * maxDistance;
	int32_t shortMove = static_cast<int32_t>((float)Z_PROBE_SWITCHING_DISTANCE * axisStepsPerMM[axisDirection]); // distance to go up for repeated moves
	int32_t lastCorrection = currentPositionSteps[axisDirection]; // starting position

	if(abs(probeDepth) < 20) {
		Com::printErrorFLN(PSTR("probe depth set too low."));
		return ILLEGAL_Z_PROBE;
	}

	if(shortMove > abs(probeDepth)-10)
		shortMove = abs(probeDepth)-10;

	if(probeDepth > 0)
		shortMove = shortMove * -1;

	waitForZProbeStart();

    Endstops::update();
    Endstops::update(); // need to call twice for full update!
	if(Endstops::zProbe()) {
		Com::printErrorFLN(PSTR("probe triggered before starting probing."));
		return ILLEGAL_Z_PROBE;
	}

	for(int8_t r = 0; r < repeat; r++) {
		stepsRemainingAtZHit = -1; // Marker that we did not hit z probe
        setZProbingActive(true);

		switch (axisDirection) {
			case X_AXIS:
				PrintLine::moveRelativeDistanceInSteps(probeDepth, 0, 0, 0, EEPROM::zProbeSpeed(), true, true);
				break;

			case Y_AXIS:
				PrintLine::moveRelativeDistanceInSteps(0, probeDepth, 0, 0, EEPROM::zProbeSpeed(), true, true);
				break;

			default:
				PrintLine::moveRelativeDistanceInSteps(0, 0, probeDepth, 0, EEPROM::zProbeSpeed(), true, true);
		}

        setZProbingActive(false);
        if(stepsRemainingAtZHit < 0) {
            Com::printErrorFLN(Com::tZProbeFailed);
            return ILLEGAL_Z_PROBE;
        }

		if(probeDepth < 0) currentPositionSteps[axisDirection] += stepsRemainingAtZHit; // now current position is correct
			else currentPositionSteps[axisDirection] -= stepsRemainingAtZHit;
		sum += lastCorrection - currentPositionSteps[axisDirection];
        //Com::printFLN(PSTR("ZHSteps:"),lastCorrection - currentPositionSteps[Z_AXIS]);
        if(r + 1 < repeat) {
			// go only shortest possible move up for repetitions
			switch (axisDirection) {
				case X_AXIS:
					PrintLine::moveRelativeDistanceInSteps(shortMove, 0, 0, 0, EEPROM::zProbeSpeed(), true, true);
					break;

				case Y_AXIS:
					PrintLine::moveRelativeDistanceInSteps(0, shortMove, 0, 0, EEPROM::zProbeSpeed(), true, true);
					break;

				default:
					PrintLine::moveRelativeDistanceInSteps(0, 0, shortMove, 0, EEPROM::zProbeSpeed(), true, true);
			}
			if(Endstops::zProbe()) {
				Com::printErrorFLN(PSTR("probe did not untrigger on repetitive measurement - maybe you need to increase distance!"));
                return ILLEGAL_Z_PROBE;
            }
		}
    }


	// Go back to start position
	switch (axisDirection) {
		case X_AXIS:
			PrintLine::moveRelativeDistanceInSteps(lastCorrection - currentPositionSteps[axisDirection], 0, 0, 0, EEPROM::zProbeSpeed(), true, true);
			break;

		case Y_AXIS:
			PrintLine::moveRelativeDistanceInSteps(0, lastCorrection - currentPositionSteps[axisDirection], 0, 0, EEPROM::zProbeSpeed(), true, true);
			break;

		default:
			PrintLine::moveRelativeDistanceInSteps(0, 0, lastCorrection - currentPositionSteps[axisDirection], 0, EEPROM::zProbeSpeed(), true, true);
	}
	if(Endstops::zProbe()) { // did we untrigger? If not don't trust result!
		Com::printErrorFLN(PSTR("probe did not untrigger on repetitive measurement - maybe you need to increase distance!"));
        return ILLEGAL_Z_PROBE;
    }
    updateCurrentPosition(false);

	float distance = static_cast<float>(sum) * invAxisStepsPerMM[axisDirection] / static_cast<float>(repeat);

#if DISTORTION_CORRECTION
    float zCorr = 0;
	if(Printer::distortion.isEnabled() && axisDirection == Z_AXIS) {
        zCorr = distortion.correct(currentPositionSteps[X_AXIS], currentPositionSteps[Y_AXIS]
								  , axisMinSteps[Z_AXIS]) * invAxisStepsPerMM[Z_AXIS];
        distance += zCorr;
    }
#endif

    Com::printF(Com::tZProbe, distance, 3);
    Com::printF(Com::tSpaceXColon, realXPosition());
#if DISTORTION_CORRECTION
	if(Printer::distortion.isEnabled() && axisDirection == Z_AXIS) {
        Com::printF(Com::tSpaceYColon, realYPosition());
        Com::printFLN(PSTR(" zCorr:"), zCorr, 3);
    } else {
        Com::printFLN(Com::tSpaceYColon, realYPosition());
    }
#else
    Com::printFLN(Com::tSpaceYColon, realYPosition());
#endif
    if(Endstops::zProbe()) {
		Com::printErrorFLN(PSTR("probe did not untrigger after going back to start position."));
        return ILLEGAL_Z_PROBE;
	}
	return distance;
}

/**
 * Having printer's height set properly (i.e. after calibration of Z=0), one can use this procedure to measure Z-probe height.
 * It deploys the sensor, takes several probes at center, then updates Z-probe height with average.
 */
void Printer::measureZProbeHeight(float curHeight) {
#if FEATURE_Z_PROBE
    currentPositionSteps[Z_AXIS] = curHeight * axisStepsPerMM[Z_AXIS];
    updateCurrentPosition(true);
	float startHeight = (EEPROM::zProbeHeight() > 0 ? EEPROM::zProbeHeight() : 0);
	moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, startHeight, IGNORE_COORDINATE, homingFeedrate[Z_AXIS]);
	float zheight = Printer::runZProbe(true, true, Z_PROBE_REPETITIONS, true);
	if(zheight == ILLEGAL_Z_PROBE) {
		return;
	}
    float zProbeHeight = EEPROM::zProbeHeight() + startHeight -zheight;

#if EEPROM_MODE != 0 // Com::tZProbeHeight is not declared when EEPROM_MODE is 0
    EEPROM::setZProbeHeight(zProbeHeight); // will also report on output
#else
    Com::printFLN(PSTR("Z-probe height [mm]:"), zProbeHeight);
#endif
#endif
}

void Printer::waitForZProbeStart() {
#if Z_PROBE_WAIT_BEFORE_TEST
    Endstops::update();
    Endstops::update(); // double test to get right signal. Needed for crosstalk protection.
	if(Endstops::zProbe()) return;
#ifdef DEBUG_PRINT
    debugWaitLoop = 3;
#endif
    while(!Endstops::zProbe()) {
        defaultLoopActions();
        Endstops::update();
        Endstops::update(); // double test to get right signal. Needed for crosstalk protection.
    }
#ifdef DEBUG_PRINT
    debugWaitLoop = 4;
#endif
    HAL::delayMilliseconds(30);
    while(Endstops::zProbe()) {
        defaultLoopActions();
        Endstops::update();
        Endstops::update(); // double test to get right signal. Needed for crosstalk protection.
    }
    HAL::delayMilliseconds(30);
#endif
}
#endif

