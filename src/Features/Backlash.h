#ifndef BACKLASH_H
#define BACKLASH_H

#if BACKLASH_COMPENSATION_SUPPORT

class Backlash {
public:
	static void initialize();

	static void buildDirection();
	static void buildDiff(float* offset, uint8_t changed, MachineLine* p);

	static float backlash[A_AXIS_ARRAY];
	static uint8_t dir;

private:

};

#endif

#endif