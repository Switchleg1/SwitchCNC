/*
    This file is part of Repetier-Firmware.

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

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#include "Repetier.h"

#if DISTORTION_CORRECTION

Distortion Printer::distortion;

void Printer::measureDistortion(float maxDistance, int repetitions) {
    float oldFeedrate = Printer::feedrate;
	float oldOffsetX = Printer::coordinateOffset[X_AXIS];
	float oldOffsetY = Printer::coordinateOffset[Y_AXIS];
	float oldOffsetZ = Printer::coordinateOffset[Z_AXIS];

	Printer::coordinateOffset[X_AXIS] = Printer::coordinateOffset[Y_AXIS] = Printer::coordinateOffset[Z_AXIS] = 0;

	if(!distortion.measure(maxDistance, repetitions)) {
		Com::printErrorFLN(PSTR("G33 failed!"));
		//GCode::fatalError(PSTR("G33 failed!"));
		//return;
	}
	Printer::feedrate = oldFeedrate;
	Printer::coordinateOffset[X_AXIS] = oldOffsetX;
	Printer::coordinateOffset[Y_AXIS] = oldOffsetY;
	Printer::coordinateOffset[Z_AXIS] = oldOffsetZ;
}

Distortion::Distortion() {
}

void Distortion::init() {
    updateDerived();
#if !DISTORTION_PERMANENT
	resetCorrection();
#endif
#if EEPROM_MODE != 0
    enabled = EEPROM::isZCorrectionEnabled();
    Com::printFLN(PSTR("zDistortionCorrection:"), (int)enabled);
#else
    enabled = false;
#endif
}

void Distortion::updateDerived() {
	xCorrectionSteps = ((float)Printer::distortionXMAX - (float)Printer::distortionXMIN) * Printer::axisStepsPerMM[X_AXIS] / (Printer::distortionPoints - 1);      //SL
	xOffsetSteps = (float)Printer::distortionXMIN * Printer::axisStepsPerMM[X_AXIS];                                                                        //SL
	yCorrectionSteps = ((float)Printer::distortionYMAX - Printer::distortionYMIN) * Printer::axisStepsPerMM[Y_AXIS] / (Printer::distortionPoints - 1);      //SL
	yOffsetSteps = (float)Printer::distortionYMIN * Printer::axisStepsPerMM[Y_AXIS];                                                                        //SL
	SetStartEnd(Printer::distortionStart, Printer::distortionEnd);
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
	Printer::zCorrectionStepsIncluded = 0;
    Printer::updateCurrentPosition(false);
    Com::printFLN(Com::tZCorrectionDisabled);
}

void Distortion::reportStatus() {
    Com::printFLN(enabled ? Com::tZCorrectionEnabled : Com::tZCorrectionDisabled);
}

void Distortion::SetStartEnd(float Start, float End)
{
	zStart = Start * Printer::axisStepsPerMM[Z_AXIS] + Printer::axisMinSteps[Z_AXIS];
	zEnd = (Start+End) * Printer::axisStepsPerMM[Z_AXIS] + Printer::axisMinSteps[Z_AXIS];
	if(Printer::distortionUseOffset)
	{
		zStart -= Printer::coordinateOffset[Z_AXIS] * Printer::axisStepsPerMM[Z_AXIS];
		zEnd -= Printer::coordinateOffset[Z_AXIS] * Printer::axisStepsPerMM[Z_AXIS];
	}
}

void Distortion::resetCorrection(void) {
    Com::printInfoFLN(PSTR("Resetting Z correction"));
	for(int i = 0; i < DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS; i++)
        setMatrix(0, i);
}

void Distortion::filter(float amount)
{
	for(int i=0; i < DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS; i++)
	{
		setMatrix(getMatrix(i)*amount, i);
	}
}

void Distortion::smooth(float amount)
{
	if(amount > 1.0f)
		amount = 1.0f;

	if(amount < 0.0f)
		amount = 0.0f;

	int32_t nMatrix[Printer::distortionPoints * Printer::distortionPoints];

	for(uint8_t x=0; x < Printer::distortionPoints; x++)
	{
		for(uint8_t y=0; y < Printer::distortionPoints; y++)
		{
			uint8_t idx11 = y*DISTORTION_CORRECTION_POINTS+x;
			uint8_t idn = y*Printer::distortionPoints+x;

			uint8_t cnt = 1;
			nMatrix[idn] = getMatrix(idx11);

			if(x>0)
			{
				nMatrix[idn] += getMatrix(idx11-1);
				cnt++;

				if(y>0)
				{
					nMatrix[idn] += getMatrix(idx11-DISTORTION_CORRECTION_POINTS-1);
					cnt++;
				}
				if(y<Printer::distortionPoints-1)
				{
					nMatrix[idn] += getMatrix(idx11+DISTORTION_CORRECTION_POINTS-1);
					cnt++;
				}
			}

			if(x<Printer::distortionPoints-1)
			{
				nMatrix[idn] += getMatrix(idx11+1);
				cnt++;
				if(y>0)
				{
					nMatrix[idn] += getMatrix(idx11-DISTORTION_CORRECTION_POINTS+1);
					cnt++;
				}
				if(y<Printer::distortionPoints-1)
				{
					nMatrix[idn] += getMatrix(idx11+DISTORTION_CORRECTION_POINTS+1);
					cnt++;
				}
			}

			if(y>0)
			{
				nMatrix[idn] += getMatrix(idx11-DISTORTION_CORRECTION_POINTS);
				cnt++;
			}

			if(y<Printer::distortionPoints-1)
			{
				nMatrix[idn] += getMatrix(idx11+DISTORTION_CORRECTION_POINTS);
				cnt++;
			}

			nMatrix[idn] /= cnt;
		}
	}

	for(uint8_t x=0; x < Printer::distortionPoints; x++)
	{
		for(uint8_t y=0; y < Printer::distortionPoints; y++)
		{
			uint8_t idx11 = y*DISTORTION_CORRECTION_POINTS+x;
			uint8_t idn = y*Printer::distortionPoints+x;

			setMatrix(getMatrix(idx11)*(1.0f-amount) + nMatrix[idn]*amount, idx11);
		}
	}
}

int Distortion::matrixIndex(fast8_t x, fast8_t y) const {
	return static_cast<int>(y) * DISTORTION_CORRECTION_POINTS + x;
}

int32_t Distortion::getMatrix(int index) const {
#if DISTORTION_PERMANENT
    return EEPROM::getZCorrection(index);
#else
    return matrix[index];
#endif
}
void Distortion::setMatrix(int32_t val, int index) {
#if DISTORTION_PERMANENT
#if EEPROM_MODE != 0
    EEPROM::setZCorrection(val, index);
#endif
#else
    matrix[index] = val;
#endif
}

bool Distortion::isCorner(fast8_t i, fast8_t j) const {
	return (i == 0 || i == Printer::distortionPoints - 1)
		   && (j == 0 || j == Printer::distortionPoints - 1);
}

/**
 Extrapolates the changes from p1 to p2 to p3 which has the same distance as p1-p2.
*/
inline int32_t Distortion::extrapolatePoint(fast8_t x1, fast8_t y1, fast8_t x2, fast8_t y2) const {
    return 2 * getMatrix(matrixIndex(x2, y2)) - getMatrix(matrixIndex(x1, y1));
}

void Distortion::extrapolateCorner(fast8_t x, fast8_t y, fast8_t dx, fast8_t dy) {
    setMatrix((extrapolatePoint(x + 2 * dx, y, x + dx, y) + extrapolatePoint(x, y + 2 * dy, x, y + dy)) / 2.0,
              matrixIndex(x, y));
}

void Distortion::extrapolateCorners() {
	const fast8_t m = Printer::distortionPoints - 1;
    extrapolateCorner(0, 0, 1, 1);
    extrapolateCorner(0, m, 1, -1);
    extrapolateCorner(m, 0, -1, 1);
    extrapolateCorner(m, m, -1, -1);
}

bool Distortion::measure(float maxDistance, int repetitions) {
	fast8_t ix, iy;

	resetCorrection();
	disable(true);
	//Printer::prepareForProbing();
	float z = Printer::currentPosition[Z_AXIS];
	Com::printFLN(PSTR("Reference Z for measurement:"), z, 3);
	updateDerived();
	Printer::updateCurrentPosition(true);

	int32_t zCorrection = 0;
	Printer::startProbing(true);
	Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, z, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
	for (iy = Printer::distortionPoints - 1; iy >= 0; iy--)
		for (ix = 0; ix < Printer::distortionPoints; ix++) {

			float mtx = Printer::invAxisStepsPerMM[X_AXIS] * (ix * xCorrectionSteps + xOffsetSteps);
			float mty = Printer::invAxisStepsPerMM[Y_AXIS] * (iy * yCorrectionSteps + yOffsetSteps);
			//Com::printF(PSTR("mx "),mtx);
			//Com::printF(PSTR("my "),mty);
			//Com::printF(PSTR("ix "),(int)ix);
			//Com::printFLN(PSTR("iy "),(int)iy);
			Printer::moveToReal(mtx, mty, z, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
			float zp = Printer::runProbe(Z_AXIS, maxDistance, repetitions);
#if defined(DISTORTION_LIMIT_TO) && DISTORTION_LIMIT_TO != 0
			if(zp == ILLEGAL_Z_PROBE || fabs(z - zp + zCorrection * Printer::invAxisStepsPerMM[Z_AXIS]) > DISTORTION_LIMIT_TO) {
#else
            if(zp == ILLEGAL_Z_PROBE) {
#endif
				Com::printErrorFLN(PSTR("Stopping distortion measurement due to errors."));
				Printer::finishProbing();
                return false;
			}
            setMatrix(floor(0.5f + Printer::axisStepsPerMM[Z_AXIS] * (z - zp)) + zCorrection,
                      matrixIndex(ix, iy));
        }
	Printer::finishProbing();

	// make average center
    // Disabled since we can use grid measurement to get average plane if that is what we want.
    // Shifting z with each measuring is a pain and can result in unexpected behavior.

	float sum = 0;
	for (iy = 0; iy < Printer::distortionPoints; iy++)
		for (ix = 0; ix < Printer::distortionPoints; ix++)
			sum += getMatrix(matrixIndex(ix, iy));

	sum /= static_cast<float>(Printer::distortionPoints == 0 ? 1 : (Printer::distortionPoints * Printer::distortionPoints));
	for (iy = 0; iy < Printer::distortionPoints; iy++)
		for (ix = 0; ix < Printer::distortionPoints; ix++)
			setMatrix(getMatrix(matrixIndex(ix, iy)) - sum, matrixIndex(ix, iy));
//	Printer::zLength -= sum * Printer::invAxisStepsPerMM[Z_AXIS];

#if EEPROM_MODE
	EEPROM::storeDataIntoEEPROM();
#endif
// print matrix
/*	Com::printInfoFLN(PSTR("Distortion correction matrix:"));
	for (iy = Printer::distortionPoints - 1; iy >= 0 ; iy--) {
		for(ix = 0; ix < Printer::distortionPoints; ix++)
			Com::printF(ix ? PSTR(", ") : PSTR(""), getMatrix(matrixIndex(ix, iy)));
		Com::println();
	} */
	showMatrix();
	enable(false);
	return true;
}


int32_t Distortion::correct(int32_t x, int32_t y, int32_t z) const {
	if (!enabled || Printer::isZProbingActive()) {
		return 0;
	}
	z += Printer::axisMinSteps[Z_AXIS];

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
	} else if (fxFloor >= Printer::distortionPoints - 1) {
		fxFloor = Printer::distortionPoints - 2;
        fx = xCorrectionSteps;
    }
    if (fyFloor < 0) {
        fyFloor = 0;
        fy = 0;
	} else if (fyFloor >= Printer::distortionPoints - 1) {
		fyFloor = Printer::distortionPoints - 2;
        fy = yCorrectionSteps;
    }

    int32_t idx11 = matrixIndex(fxFloor, fyFloor);
	int32_t m11 = getMatrix(idx11);
	int32_t m12 = getMatrix(idx11 + 1);
	int32_t m21 = getMatrix(idx11 + DISTORTION_CORRECTION_POINTS);
	int32_t m22 = getMatrix(idx11 + DISTORTION_CORRECTION_POINTS + 1);
    int32_t zx1 = m11 + ((m12 - m11) * fx) / xCorrectionSteps;
    int32_t zx2 = m21 + ((m22 - m21) * fx) / xCorrectionSteps;
	int32_t correction_z = zx1 + ((zx2 - zx1) * fy) / yCorrectionSteps;

    if (z > zStart && z > Printer::axisMinSteps[Z_AXIS]) //All variables are type int. For calculation we need float values
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
    int ix = (x * Printer::axisStepsPerMM[X_AXIS] - xOffsetSteps + xCorrectionSteps / 2) / xCorrectionSteps;
	int iy = (y * Printer::axisStepsPerMM[Y_AXIS] - yOffsetSteps + yCorrectionSteps / 2) / yCorrectionSteps;
    if(ix < 0) ix = 0;
    if(iy < 0) iy = 0;
	if(ix >= Printer::distortionPoints - 1) ix = Printer::distortionPoints - 1;
	if(iy >= Printer::distortionPoints - 1) iy = Printer::distortionPoints - 1;
    int32_t idx = matrixIndex(ix, iy);
    setMatrix(z * Printer::axisStepsPerMM[Z_AXIS], idx);
}

void Distortion::showMatrix() {
	for(int ix = 0; ix < Printer::distortionPoints; ix++) {
		for(int iy = 0; iy < Printer::distortionPoints; iy++) {
            float x = (xOffsetSteps + ix * xCorrectionSteps) * Printer::invAxisStepsPerMM[X_AXIS];
            float y = (yOffsetSteps + iy * yCorrectionSteps) * Printer::invAxisStepsPerMM[Y_AXIS];
            int32_t idx = matrixIndex(ix, iy);
            float z = getMatrix(idx) * Printer::invAxisStepsPerMM[Z_AXIS];
            Com::printF(PSTR("G33 X"), x, 2);
            Com::printF(PSTR(" Y"), y, 2);
            Com::printFLN(PSTR(" Z"), z, 3);
        }
    }
}

#endif // DISTORTION_CORRECTION
