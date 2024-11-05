#include "../../SwitchCNC.h"

#if BACKLASH_COMPENSATION_SUPPORT

float Backlash::backlash[A_AXIS_ARRAY];
uint8_t Backlash::dir;

void Backlash::initialize() {
	backlash[X_AXIS] = X_BACKLASH;
	backlash[Y_AXIS] = Y_BACKLASH;
	backlash[Z_AXIS] = Z_BACKLASH;
	backlash[A_AXIS] = A_BACKLASH;

	dir = 0;
}

void Backlash::buildDirection() {
	dir &= XYZA_DIRPOS;

	if (backlash[X_AXIS] != 0) dir |= 16;
	if (backlash[Y_AXIS] != 0) dir |= 32;
	if (backlash[Z_AXIS] != 0) dir |= 64;
	if (backlash[A_AXIS] != 0) dir |= 128;
}

void Backlash::buildDiff(float* offset, uint8_t changed, MachineLine* p) {
	offset[X_AXIS] = (changed & 1 ? (p->isXPositiveMove() ? backlash[X_AXIS] : -backlash[X_AXIS]) : 0);
	offset[Y_AXIS] = (changed & 2 ? (p->isYPositiveMove() ? backlash[Y_AXIS] : -backlash[Y_AXIS]) : 0);
	offset[Z_AXIS] = (changed & 4 ? (p->isZPositiveMove() ? backlash[Z_AXIS] : -backlash[Z_AXIS]) : 0);
	offset[A_AXIS] = (changed & 8 ? (p->isAPositiveMove() ? backlash[A_AXIS] : -backlash[A_AXIS]) : 0);
}

#endif