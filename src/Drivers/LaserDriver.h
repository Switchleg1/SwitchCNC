#ifndef LASERDRIVER_H
#define LASERDRIVER_H

#if defined(SUPPORT_LASER) && SUPPORT_LASER
/**
With laser support you can exchange a extruder by a laser. A laser gets controlled by a digital pin.
By default all intensities > 200 are always on, and lower values are always off. You can overwrite
this with a programmed event EVENT_SET_LASER(intensity) that return false to signal the default
implementation that it has set it's value already.
EVENT_INITIALIZE_LASER should return false to prevent default initialization.
*/
class LaserDriver {
public:
	static secondspeed_t intensity; // Intensity to use for next move queued. This is NOT the current value!
	static secondspeed_t minIntensity;
	static bool laserOn; // Enabled by M3?
	static bool firstMove;
	static void initialize();
	static void changeIntensity(secondspeed_t newIntensity);
	static void turnOn();
	static void turnOff(bool instantOff = false);
	static float temperature;
	static uint8_t tempCount;
	static uint16_t tempRaw;
};
#endif

#endif