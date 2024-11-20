#include "../../SwitchCNC.h"

#if Z_PROBE_SUPPORT

uint8_t ZProbe::active;

void ZProbe::initialize() {
	active = 0;

#if Z_PROBE_PIN > -1
	HAL::pinMode(Z_PROBE_PIN, Z_PROBE_PULLUP ? INPUT_PULLUP : INPUT);
#endif
}

bool ZProbe::start() {
	// 1. Ensure we are homed so positions make sense
	if (!Machine::isHomedAll() || Pause::doCancel()) {
		return false;
	}

	Commands::waitUntilEndOfAllMoves();

	Machine::setZProbeActive(true);

    return true;
}

void ZProbe::finish() {
	Machine::setZProbeActive(false);
}

float ZProbe::run(uint8_t axisDirection, float maxDistance, uint8_t repeat) {
	Commands::waitUntilEndOfAllMoves();

	int32_t sum = 0;
	int32_t probeDepth = Machine::axisStepsPerMM[axisDirection] * maxDistance;
	int32_t shortMove = static_cast<int32_t>((float)Z_PROBE_SWITCHING_DISTANCE * Machine::axisStepsPerMM[axisDirection]); // distance to go up for repeated moves
	int32_t lastCorrection = Machine::currentPositionSteps[axisDirection]; // starting position

	if(abs(probeDepth) < 20) {
		Com::printErrorFLN(PSTR("probe depth set too low."));
		return ILLEGAL_Z_PROBE;
	}

	if(shortMove > abs(probeDepth)-10)
		shortMove = abs(probeDepth)-10;

	if(probeDepth > 0)
		shortMove = shortMove * -1;

	waitForStart();

    Endstops::update();
    Endstops::update(); // need to call twice for full update!
	if(Endstops::zProbe()) {
		Com::printErrorFLN(PSTR("probe triggered before starting probing."));
		return ILLEGAL_Z_PROBE;
	}

	for(int8_t r = 0; r < repeat; r++) {
		Machine::stepsRemainingAtZHit = OVERFLOW32; // Marker that we did not hit z probe

		active = true;
		switch (axisDirection) {
			case X_AXIS:
				MachineLine::moveRelativeDistanceInSteps(probeDepth, 0, 0, 0, EEPROM::zProbeSpeed(), true, true);
				break;

			case Y_AXIS:
				MachineLine::moveRelativeDistanceInSteps(0, probeDepth, 0, 0, EEPROM::zProbeSpeed(), true, true);
				break;

			default:
				MachineLine::moveRelativeDistanceInSteps(0, 0, probeDepth, 0, EEPROM::zProbeSpeed(), true, true);
		}
		active = false;

        if(Machine::stepsRemainingAtZHit == OVERFLOW32) {
            Com::printErrorFLN(Com::tZProbeFailed);
            return ILLEGAL_Z_PROBE;
        }

		if(probeDepth < 0) Machine::currentPositionSteps[axisDirection] += Machine::stepsRemainingAtZHit; // now current position is correct
		else Machine::currentPositionSteps[axisDirection] -= Machine::stepsRemainingAtZHit;
		sum += lastCorrection - Machine::currentPositionSteps[axisDirection];
        //Com::printFLN(PSTR("ZHSteps:"),lastCorrection - currentPositionSteps[Z_AXIS]);
        if(r + 1 < repeat) {
			// go only shortest possible move up for repetitions
			switch (axisDirection) {
				case X_AXIS:
					MachineLine::moveRelativeDistanceInSteps(shortMove, 0, 0, 0, EEPROM::zProbeSpeed(), true, true);
					break;

				case Y_AXIS:
					MachineLine::moveRelativeDistanceInSteps(0, shortMove, 0, 0, EEPROM::zProbeSpeed(), true, true);
					break;

				default:
					MachineLine::moveRelativeDistanceInSteps(0, 0, shortMove, 0, EEPROM::zProbeSpeed(), true, true);
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
			MachineLine::moveRelativeDistanceInSteps(lastCorrection - Machine::currentPositionSteps[axisDirection], 0, 0, 0, EEPROM::zProbeSpeed(), true, true);
			break;

		case Y_AXIS:
			MachineLine::moveRelativeDistanceInSteps(0, lastCorrection - Machine::currentPositionSteps[axisDirection], 0, 0, EEPROM::zProbeSpeed(), true, true);
			break;

		default:
			MachineLine::moveRelativeDistanceInSteps(0, 0, lastCorrection - Machine::currentPositionSteps[axisDirection], 0, EEPROM::zProbeSpeed(), true, true);
	}
	if(Endstops::zProbe()) { // did we untrigger? If not don't trust result!
		Com::printErrorFLN(PSTR("probe did not untrigger on repetitive measurement - maybe you need to increase distance!"));
        return ILLEGAL_Z_PROBE;
    }
	Machine::updateCurrentPosition(false);

	float distance = static_cast<float>(sum) * Machine::invAxisStepsPerMM[axisDirection] / static_cast<float>(repeat);

#if DISTORTION_CORRECTION_SUPPORT
    float zCorr = 0;
	if(Distortion::isEnabled() && axisDirection == Z_AXIS) {
        zCorr = Distortion::correct(Machine::currentPositionSteps[X_AXIS], Machine::currentPositionSteps[Y_AXIS], Machine::axisMinSteps[Z_AXIS]) * Machine::invAxisStepsPerMM[Z_AXIS];
        distance += zCorr;
    }
#endif

    Com::printF(Com::tZProbe, distance, 3);
    Com::printF(Com::tSpaceXColon, Machine::realXPosition());
#if DISTORTION_CORRECTION_SUPPORT
	if(Distortion::isEnabled() && axisDirection == Z_AXIS) {
        Com::printF(Com::tSpaceYColon, Machine::realYPosition());
        Com::printFLN(PSTR(" zCorr:"), zCorr, 3);
    } else {
        Com::printFLN(Com::tSpaceYColon, Machine::realYPosition());
    }
#else
    Com::printFLN(Com::tSpaceYColon, Machine::realYPosition());
#endif
    if(Endstops::zProbe()) {
		Com::printErrorFLN(PSTR("probe did not untrigger after going back to start position."));
        return ILLEGAL_Z_PROBE;
	}
	return distance;
}

void ZProbe::waitForStart() {
#if Z_PROBE_WAIT_BEFORE_TEST
    Endstops::update();
    Endstops::update(); // double test to get right signal. Needed for crosstalk protection.
	if(Endstops::zProbe()) return;
#ifdef DEBUG_MACHINE
    Machine::debugWaitLoop = 3;
#endif
    while(!Endstops::zProbe()) {
        defaultLoopActions();
        Endstops::update();
        Endstops::update(); // double test to get right signal. Needed for crosstalk protection.
    }
#ifdef DEBUG_MACHINE
    Machine::debugWaitLoop = 4;
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