#include "SwitchCNC.h"
#include <compat/twi.h>

#if ANALOG_INPUTS > 0

const uint8_t Analog::inputChannels[ANALOG_INPUTS] PROGMEM = ANALOG_INPUT_CHANNELS;

Analog::Analog() {
    inputPosition = 0; // Current sampling position
}

void Analog::start() {
    ADMUX = ANALOG_REF; // reference voltage
    for (uint8_t i = 0; i < ANALOG_INPUTS; i++) {
        inputCounter[i] = 0;
        inputValues[i]  = 0;
        values[i]       = 0;
    }
    ADCSRA = _BV(ADEN) | _BV(ADSC) | ANALOG_PRESCALER;
    //ADCSRA |= _BV(ADSC);                  // start ADC-conversion
    while (ADCSRA & _BV(ADSC)) {} // wait for conversion
    /* ADCW must be read once, otherwise the next result is wrong. */
    //uint dummyADCResult;
    //dummyADCResult = ADCW;
    // Enable interrupt driven conversion loop
    uint8_t channel = pgm_read_byte(&inputChannels[inputPosition]);
#if defined(ADCSRB) && defined(MUX5)
    if (channel & 8)  // Reading channel 0-7 or 8-15?
        ADCSRB |= _BV(MUX5);
    else
        ADCSRB &= ~_BV(MUX5);
#endif
    ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);
    ADCSRA |= _BV(ADSC); // start conversion without interrupt!
}

void Analog::read() {
    if ((ADCSRA & _BV(ADSC)) == 0) { // Conversion finished?
        inputValues[inputPosition] += ADCW;
        if (++inputCounter[inputPosition] >= _BV(ANALOG_INPUT_SAMPLE)) {
#if ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE < ANALOG_OUTPUT_BITS
            values[inputPosition] =
                inputValues[inputPosition] << (ANALOG_OUTPUT_BITS - ANALOG_INPUT_BITS - ANALOG_INPUT_SAMPLE);
#endif
#if ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE > ANALOG_OUTPUT_BITS
            values[inputPosition] =
                inputValues[inputPosition] >> (ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE - ANALOG_OUTPUT_BITS);
#endif
#if ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE == ANALOG_OUTPUT_BITS
            outputValues[inputPosition] = inputValues[inputPosition];
#endif
            inputValues[inputPosition] = 0;
            inputCounter[inputPosition] = 0;

            // Start next conversion
            if (++inputPosition >= ANALOG_INPUTS)  inputPosition = 0;
            uint8_t channel = pgm_read_byte(&inputChannels[inputPosition]);
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
