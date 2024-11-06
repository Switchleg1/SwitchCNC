#ifndef TMC_DRIVER_H
#define TMC_DRIVER_H

#if TMC_DRIVER_SUPPORT

#include <TMCStepper.h>

//TMC Driver modes
#define TMC_STEALTH	    1
#define TMC_SPREAD	    2

//TMC Driver types
#define TMC_NONE		0
#define TMC_5160	    1
#define TMC_2130		2

#define TMC_DRV_STATUS  0b00011010000000000011000000000000

class TMC {
public:
    static void initialize();
	static void checkStatus();

#if TMC_X_TYPE==TMC_2130
	static TMC2130Stepper stepperX;
#elif TMC_X_TYPE==TMC_5160
	static TMC5160Stepper stepperX;
#endif
#if TMC_Y_TYPE==TMC_2130
	static TMC2130Stepper stepperY;
#elif TMC_Y_TYPE==TMC_5160
	static TMC5160Stepper stepperY;
#endif
#if TMC_Z_TYPE==TMC_2130
	static TMC2130Stepper stepperZ;
#elif TMC_Z_TYPE==TMC_5160
	static TMC5160Stepper stepperZ;
#endif
#if TMC_A_TYPE==TMC_2130
	static TMC2130Stepper stepperA;
#elif TMC_A_TYPE==TMC_5160
	static TMC5160Stepper stepperA;
#endif
#if TMC_2_TYPE==TMC_2130
	static TMC2130Stepper stepper2;
#elif TMC_2_TYPE==TMC_5160
	static TMC5160Stepper stepper2;
#endif

private:
	static void configTMC2130(TMC2130Stepper* driver, uint8_t intpol, uint16_t rms, float hold_mult, uint8_t hold_delay, uint8_t tpower_down, uint8_t hstart, uint8_t hend, uint8_t toff, uint8_t tbl, uint8_t pwm_freq, uint16_t tpwmthrs, uint16_t tcoolthrs, uint16_t thighthrs, uint8_t semin, uint8_t semax, int8_t sgt, uint16_t microsteps, uint8_t pwm_grad, uint8_t mode);
	static void configTMC5160(TMC5160Stepper* driver, uint8_t intpol, uint16_t rms, float hold_mult, uint8_t hold_delay, uint8_t tpower_down, uint8_t hstart, uint8_t hend, uint8_t toff, uint8_t tbl, uint8_t tpfd, uint8_t pwm_freq, uint16_t tpwmthrs, uint16_t tcoolthrs, uint16_t thighthrs, uint8_t semin, uint8_t semax, int8_t sgt, uint8_t s2vs, uint8_t s2g, uint8_t sfilter, uint16_t microsteps, uint8_t pwm_grad, uint8_t pwm_ofs, uint8_t pwm_lim, uint8_t mode);
};

#endif

#endif