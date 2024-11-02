#ifndef LASERDRIVER_H
#define LASERDRIVER_H

/**
With laser support you can exchange a extruder by a laser. A laser gets controlled by a digital pin.
By default all intensities > 200 are always on, and lower values are always off. You can overwrite
this with a programmed event EVENT_SET_LASER(intensity) that return false to signal the default
implementation that it has set it's value already.
EVENT_INITIALIZE_LASER should return false to prevent default initialization.
*/
class LaserDriver {
public:
	static void initialize();
	static void turnOn(uint8_t intensity = minimumIntensity, uint8_t quiet = false);
	static void turnOff(uint8_t quiet = false);
	static void setIntensity(uint16_t intensity);
	static void setNextIntensity(uint8_t intensity, uint8_t immediately = false);
	static void setIntensityMultiplier(uint8_t multiplier);
	static void setMinimumIntensity(uint8_t intensity, uint8_t quiet = false);
	static void updateTemperature();

	static inline float temperature() {
		return currentTemperature;
	}

	static inline uint8_t intensity() {
		return currentIntensity;
	}

	static inline uint8_t minIntensity() {
		return minimumIntensity;
	}

	static inline uint8_t isOn() {
		return laserOn;
	}

	static inline uint8_t next() {
		return nextIntensity;
	}

private:
	static void printState();

	static uint8_t laserOn; // Enabled by M3?
	static uint8_t intensityMultiplier;
	
	static uint8_t currentIntensity; // Intensity to use for next move queued. This is NOT the current value!
	static uint8_t minimumIntensity;
	static uint8_t nextIntensity;

	static float currentTemperature;
};

#endif