#include "../../SwitchCNC.h"

#if defined(SUPPORT_VACUUM) && SUPPORT_VACUUM

void VacuumDriver::initialize()
{
#if VACUUM_PIN>-1
	SET_OUTPUT(VACUUM_PIN);
	WRITE(VACUUM_PIN, LOW);
#endif
}

void VacuumDriver::turnOn()
{
#if VACUUM_PIN>-1
	WRITE(VACUUM_PIN, HIGH);
#endif
}

void VacuumDriver::turnOff()
{
#if VACUUM_PIN>-1
	WRITE(VACUUM_PIN, LOW);
#endif
}

#endif