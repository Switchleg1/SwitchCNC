#include "../../SwitchCNC.h"

#if TMC_DRIVER_SUPPORT

#if TMC_X_TYPE==TMC_5160
TMC5160Stepper TMC::stepperX(TMC_X_CS, TMC_X_RSENSE);
#endif
#if TMC_Y_TYPE==TMC_5160
TMC5160Stepper TMC::stepperY(TMC_Y_CS, TMC_Y_RSENSE);
#endif
#if TMC_Z_TYPE==TMC_5160
TMC5160Stepper TMC::stepperZ(TMC_Z_CS, TMC_Z_RSENSE);
#endif
#if TMC_A_TYPE==TMC_5160
TMC5160Stepper TMC::stepperA(TMC_A_CS, TMC_A_RSENSE);
#endif
#if TMC_2_TYPE==TMC_5160
TMC5160Stepper TMC::stepper2(TMC_2_CS, TMC_2_RSENSE);
#endif

void TMC::initialize() {
#if TMC_X_TYPE==TMC_5160
	configTMC5160(&stepperX, TMC_X_INTPOL, TMC_X_RMS, TMC_X_HOLD, TMC_X_HOLDDELAY, TMC_X_TPWRDOWN, TMC_X_HSTART, TMC_X_HEND, TMC_X_TOFF, TMC_X_TBL, TMC_X_TPFD, TMC_X_PWM_FREQ, TMC_X_TPWMTHRS, TMC_X_TCOOLTHRS, TMC_X_THIGHTHRS, TMC_X_SEMIN, TMC_X_SEMAX, TMC_X_SGT, TMC_X_S2VS, TMC_X_S2G, TMC_X_SFILTER, TMC_X_MICROSTEP, TMC_X_PWM_GRAD, TMC_X_PWM_OFS, TMC_X_PWM_LIM, TMC_X_MODE);
#endif
#if TMC_Y_TYPE==TMC_5160
	configTMC5160(&stepperY, TMC_Y_INTPOL, TMC_Y_RMS, TMC_Y_HOLD, TMC_Y_HOLDDELAY, TMC_Y_TPWRDOWN, TMC_Y_HSTART, TMC_Y_HEND, TMC_Y_TOFF, TMC_Y_TBL, TMC_Y_TPFD, TMC_Y_PWM_FREQ, TMC_Y_TPWMTHRS, TMC_Y_TCOOLTHRS, TMC_Y_THIGHTHRS, TMC_Y_SEMIN, TMC_Y_SEMAX, TMC_Y_SGT, TMC_Y_S2VS, TMC_Y_S2G, TMC_Y_SFILTER, TMC_Y_MICROSTEP, TMC_Y_PWM_GRAD, TMC_Y_PWM_OFS, TMC_Y_PWM_LIM, TMC_Y_MODE);
#endif
#if TMC_Z_TYPE==TMC_5160
	configTMC5160(&stepperZ, TMC_Z_INTPOL, TMC_Z_RMS, TMC_Z_HOLD, TMC_Z_HOLDDELAY, TMC_Z_TPWRDOWN, TMC_Z_HSTART, TMC_Z_HEND, TMC_Z_TOFF, TMC_Z_TBL, TMC_Z_TPFD, TMC_Z_PWM_FREQ, TMC_Z_TPWMTHRS, TMC_Z_TCOOLTHRS, TMC_Z_THIGHTHRS, TMC_Z_SEMIN, TMC_Z_SEMAX, TMC_Z_SGT, TMC_Z_S2VS, TMC_Z_S2G, TMC_Z_SFILTER, TMC_Z_MICROSTEP, TMC_Z_PWM_GRAD, TMC_Z_PWM_OFS, TMC_Z_PWM_LIM, TMC_Z_MODE);
#endif
#if TMC_A_TYPE==TMC_5160
	configTMC5160(&stepperA, TMC_A_INTPOL, TMC_A_RMS, TMC_A_HOLD, TMC_A_HOLDDELAY, TMC_A_TPWRDOWN, TMC_A_HSTART, TMC_A_HEND, TMC_A_TOFF, TMC_A_TBL, TMC_A_TPFD, TMC_A_PWM_FREQ, TMC_A_TPWMTHRS, TMC_A_TCOOLTHRS, TMC_A_THIGHTHRS, TMC_A_SEMIN, TMC_A_SEMAX, TMC_A_SGT, TMC_A_S2VS, TMC_A_S2G, TMC_A_SFILTER, TMC_A_MICROSTEP, TMC_A_PWM_GRAD, TMC_A_PWM_OFS, TMC_A_PWM_LIM, TMC_A_MODE);
#endif
#if TMC_2_TYPE==TMC_5160
	configTMC5160(&stepper2, TMC_2_INTPOL, TMC_2_RMS, TMC_2_HOLD, TMC_2_HOLDDELAY, TMC_2_TPWRDOWN, TMC_2_HSTART, TMC_2_HEND, TMC_2_TOFF, TMC_2_TBL, TMC_2_TPFD, TMC_2_PWM_FREQ, TMC_2_TPWMTHRS, TMC_2_TCOOLTHRS, TMC_2_THIGHTHRS, TMC_2_SEMIN, TMC_2_SEMAX, TMC_2_SGT, TMC_2_S2VS, TMC_2_S2G, TMC_2_SFILTER, TMC_2_MICROSTEP, TMC_2_PWM_GRAD, TMC_2_PWM_OFS, TMC_2_PWM_LIM, TMC_2_MODE);
#endif
}

void TMC::checkStatus() {
#if TMC_X_TYPE==TMC_5160
	if (stepperX.DRV_STATUS() & TMC_DRV_STATUS) {
		Com::printErrorFLN(PSTR("X TMC Driver faulted."));
		Commands::emergencyStop();
	}
#endif
#if TMC_Y_TYPE==TMC_5160
	if (stepperY.DRV_STATUS() & TMC_DRV_STATUS) {
		Com::printErrorFLN(PSTR("Y TMC Driver faulted."));
		Commands::emergencyStop();
	}
#endif
#if TMC_Z_TYPE==TMC_5160
	if (stepperZ.DRV_STATUS() & TMC_DRV_STATUS) {
		Com::printErrorFLN(PSTR("Z TMC Driver faulted."));
		Commands::emergencyStop();
	}
#endif
#if TMC_A_TYPE==TMC_5160
	if (stepperA.DRV_STATUS() & TMC_DRV_STATUS) {
		Com::printErrorFLN(PSTR("A TMC Driver faulted."));
		Commands::emergencyStop();
	}
#endif
#if TMC_2_TYPE==TMC_5160
	if (stepper2.DRV_STATUS() & TMC_DRV_STATUS) {
		Com::printErrorFLN(PSTR("2 TMC Driver faulted."));
		Commands::emergencyStop();
	}
#endif
}

void TMC::configTMC5160(TMC5160Stepper* driver, uint8_t intpol, uint16_t rms, float hold_mult, uint8_t hold_delay, uint8_t tpower_down,
	uint8_t hstart, uint8_t hend, uint8_t toff, uint8_t tbl, uint8_t tpfd, uint8_t pwm_freq, uint16_t tpwmthrs,
	uint16_t tcoolthrs, uint16_t thighthrs, uint8_t semin, uint8_t semax, int8_t sgt, uint8_t s2vs, uint8_t s2g,
	uint8_t sfilter, uint16_t microsteps, uint8_t pwm_grad, uint8_t pwm_ofs, uint8_t pwm_lim, uint8_t mode) {
	wdt_reset();
	driver->begin();
	driver->intpol(intpol);
	driver->rms_current(rms, hold_mult);	// Set motor RMS current
	driver->iholddelay(hold_delay);
	driver->TPOWERDOWN(tpower_down);
	driver->hysteresis_start(hstart);
	driver->hysteresis_end(hend);
	driver->toff(toff);                 	// Enables driver in software
	driver->tbl(tbl);
	driver->tpfd(tpfd);
	driver->pwm_freq(pwm_freq);
	driver->TPWMTHRS(tpwmthrs);
	driver->TCOOLTHRS(tcoolthrs);
	driver->THIGH(thighthrs);
	driver->semin(semin);
	driver->semax(semax);
	driver->sgt(sgt);
	driver->s2vs_level(s2vs);
	driver->s2g_level(s2g);
	driver->shortfilter(sfilter);
	driver->microsteps(microsteps);
	if (mode == TMC_STEALTH) {
		driver->en_pwm_mode(true);       	// Toggle stealthChop on TMC2130/2160/5130/5160
		driver->pwm_grad(pwm_grad);
		driver->pwm_ofs(pwm_ofs);
		driver->pwm_lim(pwm_lim);
		driver->pwm_autoscale(true);     	// Needed for stealthChop
		driver->pwm_autograd(true);
	}
	else {
		driver->en_pwm_mode(false);       	// Toggle stealthChop on TMC2130/2160/5130/5160
	}
	wdt_reset();
}

#endif