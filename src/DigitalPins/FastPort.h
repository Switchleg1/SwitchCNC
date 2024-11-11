#ifndef FAST_PORT_H
#define FAST_PORT_H

#if defined(__AVR_ATmega32U4__)
#ifdef CORE_TEENSY
#define FAST_PORT_MAX_PIN	25
#else
#define FAST_PORT_MAX_PIN	30
#endif
#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
#define FAST_PORT_MAX_PIN	46
#elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega64__)\
		|| defined(__AVR_ATmega32__) || defined(__AVR_ATmega324__) || defined(__AVR_ATmega16__)
#define FAST_PORT_MAX_PIN	32
#elif defined (__AVR_ATmega168__) || defined (__AVR_ATmega168P__) || defined (__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
#define FAST_PORT_MAX_PIN	20
#elif defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
#define FAST_PORT_MAX_PIN	70
#endif

#define FAST_PORT_SAFE_CHECK(pin, reg)		(!__builtin_constant_p(pin) || reg > reinterpret_cast<uint8_t*>(0X3F))

struct FastPortPin {
	volatile uint8_t* port;
	volatile uint8_t* pin;
	volatile uint8_t* ddr;
	const uint8_t mask;
	const uint8_t maskInv;
};

class FastPort {
public:
	static volatile inline __attribute__((always_inline))
	void mode(uint8_t pin, uint8_t mode, uint8_t safe = true) {
		if (pin >= FAST_PORT_MAX_PIN) {
			return;
		}

		uint8_t sreg;
		if (safe && FAST_PORT_SAFE_CHECK(pin, pins[pin].ddr)) {
			sreg = SREG;
			cli();
		}

		changeRegister(pins[pin].ddr, pins[pin].mask, pins[pin].maskInv, mode == OUTPUT);

		if (mode != OUTPUT) {
			write(pin, mode == INPUT_PULLUP, false);
		}

		if (safe && FAST_PORT_SAFE_CHECK(pin, pins[pin].ddr)) {
			SREG = sreg;
		}
	}

	static volatile inline __attribute__((always_inline))
	void write(uint8_t pin, uint8_t value, uint8_t safe = true) {
		if (pin >= FAST_PORT_MAX_PIN) {
			return;
		}

		uint8_t sreg;
		if (safe && FAST_PORT_SAFE_CHECK(pin, pins[pin].port)) {
			sreg = SREG;
			cli();
		}

		changeRegister(pins[pin].port, pins[pin].mask, pins[pin].maskInv, value);

		if (safe && FAST_PORT_SAFE_CHECK(pin, pins[pin].port)) {
			SREG = sreg;
		}
	}

	static volatile inline __attribute__((always_inline))
	uint8_t read(uint8_t pin) {
		if (pin >= FAST_PORT_MAX_PIN) return 0;
		return (*pins[pin].pin & pins[pin].mask) != 0;
	}

private:
	static volatile inline __attribute__((always_inline))
	void changeRegister(volatile uint8_t* reg, uint8_t mask, uint8_t maskInv, uint8_t value) {
		if (!value) *reg &= maskInv;
		else *reg |= mask;
	}

	static const FastPortPin pins[];
};

#endif