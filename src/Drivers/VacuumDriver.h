#ifndef VACUUMDRIVER_H
#define VACUUMDRIVER_H

#if defined(SUPPORT_VACUUM) && SUPPORT_VACUUM

class VacuumDriver {
public:
	static void initialize();
	static void turnOn();
	static void turnOff();
};

#endif

#endif