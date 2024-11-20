#include "../../SwitchCNC.h"

#if DISTORTION_CORRECTION_SUPPORT

int16_t Distortion::XMIN;
int16_t Distortion::XMAX;
int16_t Distortion::YMIN;
int16_t Distortion::YMAX;
float   Distortion::start;
float   Distortion::end;
uint8_t Distortion::useOffset;
int32_t Distortion::xCorrectionSteps;
int32_t Distortion::xOffsetSteps;
int32_t Distortion::yCorrectionSteps;
int32_t Distortion::yOffsetSteps;
int32_t Distortion::zStart;
int32_t Distortion::zEnd;
int32_t Distortion::matrix[DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS];
uint8_t Distortion::points;
bool    Distortion::enabled;

void Distortion::init() {
    updateDerived();
#if DISTORTION_PERMANENT && EEPROM_MODE != 0
	EEPROM::getZCorrection(matrix, DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS);
#else
	resetCorrection();
#endif
#if DISTORTION_PERMANENT && EEPROM_MODE != 0
    enabled = EEPROM::isZCorrectionEnabled();
    Com::printFLN(PSTR("zDistortionCorrection:"), (int)enabled);
#else
    enabled = false;
#endif
}

void Distortion::updateDerived() {
	xCorrectionSteps	= ((float)XMAX - (float)XMIN) * Machine::axisStepsPerMM[X_AXIS] / (points - 1);			//SL
	xOffsetSteps		= (float)XMIN * Machine::axisStepsPerMM[X_AXIS];                                        //SL
	yCorrectionSteps	= ((float)YMAX - YMIN) * Machine::axisStepsPerMM[Y_AXIS] / (points - 1);				//SL
	yOffsetSteps		= (float)YMIN * Machine::axisStepsPerMM[Y_AXIS];                                        //SL
	SetStartEnd(start, end);
}

void Distortion::enable(bool permanent) {
    enabled = true;
#if DISTORTION_PERMANENT && EEPROM_MODE != 0
    if(permanent)
        EEPROM::setZCorrectionEnabled(enabled);
#endif
    Com::printFLN(Com::tZCorrectionEnabled);
}

void Distortion::disable(bool permanent) {
    enabled = false;
#if DISTORTION_PERMANENT && EEPROM_MODE != 0
    if(permanent)
        EEPROM::setZCorrectionEnabled(enabled);
#endif
	Machine::zCorrectionStepsIncluded = 0;
    Machine::updateCurrentPosition(false);
    Com::printFLN(Com::tZCorrectionDisabled);
}

bool Distortion::measure(float maxDistance, uint8_t repetitions) {
	uint8_t ix, iy;

	resetCorrection();
	disable(true);
	float z = Machine::currentPosition[Z_AXIS];
	Com::printFLN(PSTR("Reference Z for measurement:"), z, 3);
	updateDerived();
	Machine::updateCurrentPosition(true);

	int32_t zCorrection = 0;
	if(!ZProbe::start()) return false;
	Machine::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, z, IGNORE_COORDINATE, Machine::homingFeedrate[Z_AXIS]);
	for (iy = points - 1; iy >= 0; iy--) {
		for (ix = 0; ix < points; ix++) {

			float mtx = Machine::invAxisStepsPerMM[X_AXIS] * (ix * xCorrectionSteps + xOffsetSteps);
			float mty = Machine::invAxisStepsPerMM[Y_AXIS] * (iy * yCorrectionSteps + yOffsetSteps);
			//Com::printF(PSTR("mx "),mtx);
			//Com::printF(PSTR("my "),mty);
			//Com::printF(PSTR("ix "),(int)ix);
			//Com::printFLN(PSTR("iy "),(int)iy);
			Machine::moveToReal(mtx, mty, z, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
			float zp = ZProbe::run(Z_AXIS, maxDistance, repetitions);
#if defined(DISTORTION_LIMIT_TO) && DISTORTION_LIMIT_TO != 0
			if (zp == ILLEGAL_Z_PROBE || fabs(z - zp + zCorrection * Machine::invAxisStepsPerMM[Z_AXIS]) > DISTORTION_LIMIT_TO) {
#else
			if (zp == ILLEGAL_Z_PROBE) {
#endif
				Com::printErrorFLN(PSTR("Stopping distortion measurement due to errors."));
				ZProbe::finish();
				return false;
			}
			matrix[matrixIndex(ix, iy)] = floor(0.5f + Machine::axisStepsPerMM[Z_AXIS] * (z - zp)) + zCorrection;
		}
	}
	ZProbe::finish();

	// make average center
	// Disabled since we can use grid measurement to get average plane if that is what we want.
	// Shifting z with each measuring is a pain and can result in unexpected behavior.

	float sum = 0;
	for (iy = 0; iy < points; iy++) {
		for (ix = 0; ix < points; ix++) {
			sum += matrix[matrixIndex(ix, iy)];
		}
	}

	sum /= static_cast<float>(points == 0 ? 1 : (points * points));
	for (iy = 0; iy < points; iy++) {
		for (ix = 0; ix < points; ix++) {
			matrix[matrixIndex(ix, iy)] -= sum;
		}
	}
	//	Machine::zLength -= sum * Machine::invAxisStepsPerMM[Z_AXIS];

#if DISTORTION_PERMANENT && EEPROM_MODE != 0
	EEPROM::setZCorrection(matrix, DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS);
#endif

	// print matrix
	showMatrix();
	enable(false);
	return true;
}

void Distortion::reportStatus() {
    Com::printFLN(enabled ? Com::tZCorrectionEnabled : Com::tZCorrectionDisabled);
}

bool Distortion::isEnabled() {
	return enabled;
}
int32_t Distortion::zMaxSteps() {
	return zEnd;
}

void Distortion::SetStartEnd(float Start, float End) {
	zStart	= Start * Machine::axisStepsPerMM[Z_AXIS] + Machine::axisMinSteps[Z_AXIS];
	zEnd	= (Start+End) * Machine::axisStepsPerMM[Z_AXIS] + Machine::axisMinSteps[Z_AXIS];
	if(useOffset) {
		zStart	-= Machine::coordinateOffset[Z_AXIS] * Machine::axisStepsPerMM[Z_AXIS];
		zEnd	-= Machine::coordinateOffset[Z_AXIS] * Machine::axisStepsPerMM[Z_AXIS];
	}
}

void Distortion::resetCorrection() {
    Com::printInfoFLN(PSTR("Resetting Z correction"));
	for (int i = 0; i < DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS; i++) {
		matrix[i] = 0;
	}

#if DISTORTION_PERMANENT && EEPROM_MODE != 0
	EEPROM::setZCorrection(matrix, DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS);
#endif
}

void Distortion::filter(float amount) {
	for(int i=0; i < DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS; i++) {
		matrix[i] *= amount;
	}

#if DISTORTION_PERMANENT && EEPROM_MODE != 0
	EEPROM::setZCorrection(matrix, DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS);
#endif
}

void Distortion::smooth(float amount) {
	if(amount > 1.0f)
		amount = 1.0f;

	if(amount < 0.0f)
		amount = 0.0f;

	int32_t nMatrix[points * points];

	for(uint8_t x = 0; x < points; x++) {
		for(uint8_t y = 0; y < points; y++) {
			uint8_t idx11 = y * DISTORTION_CORRECTION_POINTS + x;
			uint8_t idn = y * points + x;

			uint8_t cnt = 1;
			nMatrix[idn] = matrix[idx11];

			if(x > 0) {
				nMatrix[idn] += matrix[idx11 - 1];
				cnt++;

				if(y > 0) {
					nMatrix[idn] += matrix[idx11 - DISTORTION_CORRECTION_POINTS - 1];
					cnt++;
				}
				if(y < points - 1) {
					nMatrix[idn] += matrix[idx11 + DISTORTION_CORRECTION_POINTS - 1];
					cnt++;
				}
			}

			if(x < points - 1) {
				nMatrix[idn] += matrix[idx11 + 1];
				cnt++;
				if(y > 0) {
					nMatrix[idn] += matrix[idx11 - DISTORTION_CORRECTION_POINTS + 1];
					cnt++;
				}
				if(y < points - 1) {
					nMatrix[idn] += matrix[idx11 + DISTORTION_CORRECTION_POINTS + 1];
					cnt++;
				}
			}

			if(y > 0) {
				nMatrix[idn] += matrix[idx11 - DISTORTION_CORRECTION_POINTS];
				cnt++;
			}

			if(y < points - 1) {
				nMatrix[idn] += matrix[idx11 + DISTORTION_CORRECTION_POINTS];
				cnt++;
			}

			nMatrix[idn] /= cnt;
		}
	}

	for(uint8_t x = 0; x < points; x++) {
		for(uint8_t y = 0; y < points; y++) {
			uint8_t idx11 = y * DISTORTION_CORRECTION_POINTS + x;
			uint8_t idn = y * points + x;

			matrix[idx11] = matrix[idx11] * (1.0f - amount) + nMatrix[idn] * amount;
		}
	}

#if DISTORTION_PERMANENT && EEPROM_MODE != 0
	EEPROM::setZCorrection(matrix, DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS);
#endif
}

uint8_t Distortion::setPoints(uint8_t count) {
	if (count > DISTORTION_CORRECTION_POINTS) {
		points = DISTORTION_CORRECTION_POINTS;
	} else if (count < 2) {
		points = 2;
	} else {
		points = count;
	}

	return points;
}

uint8_t Distortion::getPoints() {
	return points;
}

int Distortion::matrixIndex(uint8_t x, uint8_t y) {
	return static_cast<int>(y) * DISTORTION_CORRECTION_POINTS + x;
}

bool Distortion::isCorner(uint8_t i, uint8_t j) {
	return (i == 0 || i == points - 1)
		   && (j == 0 || j == points - 1);
}

/**
 Extrapolates the changes from p1 to p2 to p3 which has the same distance as p1-p2.
*/
inline int32_t Distortion::extrapolatePoint(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
    return 2 * matrix[matrixIndex(x2, y2)] - matrix[matrixIndex(x1, y1)];
}

void Distortion::extrapolateCorner(uint8_t x, uint8_t y, uint8_t dx, uint8_t dy) {
    matrix[matrixIndex(x, y)] = (extrapolatePoint(x + 2 * dx, y, x + dx, y) + extrapolatePoint(x, y + 2 * dy, x, y + dy)) / 2.0f;
}

void Distortion::extrapolateCorners() {
	uint8_t m = points - 1;
    extrapolateCorner(0, 0, 1, 1);
    extrapolateCorner(0, m, 1, -1);
    extrapolateCorner(m, 0, -1, 1);
    extrapolateCorner(m, m, -1, -1);

#if DISTORTION_PERMANENT && EEPROM_MODE != 0
	EEPROM::setZCorrection(matrix, DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS);
#endif
}

int32_t Distortion::correct(int32_t x, int32_t y, int32_t z) {
	if (!enabled || ZProbe::isActive()) {
		return 0;
	}
	z += Machine::axisMinSteps[Z_AXIS];

	if (z > zEnd) {
        return 0;
    }

	x -= xOffsetSteps;
    y -= yOffsetSteps;
	int32_t fxFloor = (x - (x < 0 ? xCorrectionSteps - 1 : 0)) / xCorrectionSteps; // special case floor for negative integers!
	int32_t fyFloor = (y - (y < 0 ? yCorrectionSteps - 1 : 0)) / yCorrectionSteps;
	// indexes to the matrix

	// position between cells of matrix, range=0 to 1 - outside of the matrix the value will be outside this range and the value will be extrapolated
    int32_t fx = x - fxFloor * xCorrectionSteps; // Grid normalized coordinates
    int32_t fy = y - fyFloor * yCorrectionSteps;
    if (fxFloor < 0) {
        fxFloor = 0;
        fx = 0;
	} else if (fxFloor >= points - 1) {
		fxFloor	= points - 2;
        fx		= xCorrectionSteps;
    }
    if (fyFloor < 0) {
        fyFloor = 0;
        fy = 0;
	} else if (fyFloor >= points - 1) {
		fyFloor	= points - 2;
        fy		= yCorrectionSteps;
    }

    int32_t idx11 = matrixIndex(fxFloor, fyFloor);
	int32_t m11 = matrix[idx11];
	int32_t m12 = matrix[idx11 + 1];
	int32_t m21 = matrix[idx11 + DISTORTION_CORRECTION_POINTS];
	int32_t m22 = matrix[idx11 + DISTORTION_CORRECTION_POINTS + 1];
    int32_t zx1 = m11 + ((m12 - m11) * fx) / xCorrectionSteps;
    int32_t zx2 = m21 + ((m22 - m21) * fx) / xCorrectionSteps;
	int32_t correction_z = zx1 + ((zx2 - zx1) * fy) / yCorrectionSteps;

    if (z > zStart && z > Machine::axisMinSteps[Z_AXIS]) //All variables are type int. For calculation we need float values
        correction_z = (correction_z * static_cast<float>(zEnd - z) / (zEnd - zStart));

    return correction_z;
}

void Distortion::set(float x, float y, float z) {
#if defined(DISTORTION_LIMIT_TO) && DISTORTION_LIMIT_TO != 0
    if(fabs(z) > DISTORTION_LIMIT_TO) {
        Com::printWarningFLN(PSTR("Max. distortion value exceeded - not setting this value."));
        return;
    }
#endif
    int ix = (x * Machine::axisStepsPerMM[X_AXIS] - xOffsetSteps + xCorrectionSteps / 2) / xCorrectionSteps;
	int iy = (y * Machine::axisStepsPerMM[Y_AXIS] - yOffsetSteps + yCorrectionSteps / 2) / yCorrectionSteps;
    if(ix < 0) ix = 0;
    if(iy < 0) iy = 0;
	if(ix >= points - 1) ix = points - 1;
	if(iy >= points - 1) iy = points - 1;
    int32_t idx = matrixIndex(ix, iy);
    matrix[idx] = z * Machine::axisStepsPerMM[Z_AXIS];

#if DISTORTION_PERMANENT && EEPROM_MODE != 0
	EEPROM::setZCorrection(matrix, DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS);
#endif
}

void Distortion::showMatrix() {
	for(int ix = 0; ix < points; ix++) {
		for(int iy = 0; iy < points; iy++) {
            float x = (xOffsetSteps + ix * xCorrectionSteps) * Machine::invAxisStepsPerMM[X_AXIS];
            float y = (yOffsetSteps + iy * yCorrectionSteps) * Machine::invAxisStepsPerMM[Y_AXIS];
            int32_t idx = matrixIndex(ix, iy);
            float z = matrix[idx] * Machine::invAxisStepsPerMM[Z_AXIS];
            Com::printF(PSTR("G33 X"), x, 2);
            Com::printF(PSTR(" Y"), y, 2);
            Com::printFLN(PSTR(" Z"), z, 3);
        }
    }
}

#endif
