#include "Repetier.h"
#include <compat/twi.h>

#if ANALOG_INPUTS > 0
uint8_t AnalogIn::inputPosition = 0; // Current sampling position
uint8_t AnalogIn::inputCounter[ANALOG_INPUTS];
#if (ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE)  <= 8
uint8_t AnalogIn::inputValues[ANALOG_INPUTS];
#elif (ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE)  <= 16
uint16_t AnalogIn::inputValues[ANALOG_INPUTS];
#else
uint32_t AnalogIn::inputValues[ANALOG_INPUTS];
#endif
#if ANALOG_OUTPUT_BITS <= 8
volatile uint8_t AnalogIn::values[ANALOG_INPUTS];
#elif ANALOG_OUTPUT_BITS <= 16
volatile uint16_t AnalogIn::values[ANALOG_INPUTS];
#else
volatile uint32_t AnalogIn::values[ANALOG_INPUTS];
#endif
const uint8_t AnalogIn::inputChannels[ANALOG_INPUTS] PROGMEM = ANALOG_INPUT_CHANNELS;


void AnalogIn::start() {
    ADMUX = ANALOG_REF; // reference voltage
    for (uint8_t i = 0; i < ANALOG_INPUTS; i++) {
        AnalogIn::inputCounter[i] = 0;
        AnalogIn::inputValues[i] = 0;
        AnalogIn::values[i] = 0;
    }
    ADCSRA = _BV(ADEN) | _BV(ADSC) | ANALOG_PRESCALER;
    //ADCSRA |= _BV(ADSC);                  // start ADC-conversion
    while (ADCSRA & _BV(ADSC)) {} // wait for conversion
    /* ADCW must be read once, otherwise the next result is wrong. */
    //uint dummyADCResult;
    //dummyADCResult = ADCW;
    // Enable interrupt driven conversion loop
    uint8_t channel = pgm_read_byte(&AnalogIn::inputChannels[AnalogIn::inputPosition]);
#if defined(ADCSRB) && defined(MUX5)
    if (channel & 8)  // Reading channel 0-7 or 8-15?
        ADCSRB |= _BV(MUX5);
    else
        ADCSRB &= ~_BV(MUX5);
#endif
    ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);
    ADCSRA |= _BV(ADSC); // start conversion without interrupt!
}

void AnalogIn::read() {
    if ((ADCSRA & _BV(ADSC)) == 0) { // Conversion finished?
        AnalogIn::inputValues[AnalogIn::inputPosition] += ADCW;
        if (++AnalogIn::inputCounter[AnalogIn::inputPosition] >= _BV(ANALOG_INPUT_SAMPLE)) {
#if ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE < ANALOG_OUTPUT_BITS
            AnalogIn::values[AnalogIn::inputPosition] =
                AnalogIn::inputValues[AnalogIn::inputPosition] << (ANALOG_OUTPUT_BITS - ANALOG_INPUT_BITS - ANALOG_INPUT_SAMPLE);
#endif
#if ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE > ANALOG_OUTPUT_BITS
            AnalogIn::values[AnalogIn::inputPosition] =
                AnalogIn::inputValues[AnalogIn::inputPosition] >> (ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE - ANALOG_OUTPUT_BITS);
#endif
#if ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE == ANALOG_OUTPUT_BITS
            AnalogIn::outputValues[AnalogIn::inputPosition] = inputValues[inputPosition];
#endif
            AnalogIn::inputValues[AnalogIn::inputPosition] = 0;
            AnalogIn::inputCounter[AnalogIn::inputPosition] = 0;

            // Start next conversion
            if (++AnalogIn::inputPosition >= ANALOG_INPUTS)  AnalogIn::inputPosition = 0;
            uint8_t channel = pgm_read_byte(&AnalogIn::inputChannels[AnalogIn::inputPosition]);
#if defined(ADCSRB) && defined(MUX5)
            if (channel & 8)  // Reading channel 0-7 or 8-15?
                ADCSRB |= _BV(MUX5);
            else
                ADCSRB &= ~_BV(MUX5);
#endif
            ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);
        }
        ADCSRA |= _BV(ADSC);  // start next conversion
    }
}

#endif
