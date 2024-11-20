#ifndef _DISTORTION_H
#define _DISTORTION_H

#if DISTORTION_CORRECTION_SUPPORT
/** \brief Handle distortion related stuff.

Distortion correction can be used to solve problems resulting from an uneven build plate.
It allows measuring a nxn grid with a z-probe and add these correction to all moves.
Normally you start at the bottom with 100% correction and at 0.5mm you start reducing correction
until it vanishes completely at 1-3 mm.

The stored values are steps required to reach the bumped level assuming you are at zMin. So if you have a 1mm indentation
it contains -steps per mm.
*/
class Distortion {
public:
    static void init();
    static void enable(bool permanent = true);
    static void disable(bool permanent = true);
    static bool measure(float maxDistance, uint8_t repetitions);
	/** \brief Compute distortion correction at given position.

    The current tool offset is added to the CNC position to reference the right distortion point.

    \param x coordinate in CMC steps.
    \param y coordinate in CMC steps.
    \param z coordinate in CMC steps.
    \return Correction required in z steps.
    */
    static int32_t correct(int32_t x, int32_t y, int32_t z);
    static void    updateDerived();
    static void    reportStatus();
    static bool    isEnabled();
    static int32_t zMaxSteps();
    static void    SetStartEnd(float Start, float End);
    static void    set(float x, float y, float z);
    static void    showMatrix();
    static void    resetCorrection();
    static void    filter(float amount);
    static void    smooth(float amount);
    static uint8_t setPoints(uint8_t count);
    static uint8_t getPoints();

    static int16_t XMIN;
    static int16_t XMAX;
    static int16_t YMIN;
    static int16_t YMAX;
    static float   start;
    static float   end;
    static uint8_t useOffset;

private:
    static int16_t matrixIndex(uint8_t x, uint8_t y);
    static bool isCorner(uint8_t i, uint8_t j);
    static INLINE int32_t extrapolatePoint(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
    static void extrapolateCorner(uint8_t x, uint8_t y, uint8_t dx, uint8_t dy);
    static void extrapolateCorners();

	// attributes
    static int32_t xCorrectionSteps, xOffsetSteps;
    static int32_t yCorrectionSteps, yOffsetSteps;
    static int32_t zStart, zEnd;
    static int32_t matrix[DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS];
    static uint8_t points;
    static bool    enabled;
};
#endif

#endif
