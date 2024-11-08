#include "../../SwitchCNC.h"
#include "SysCall.h"
#if defined(UDR0)
#include "MinimumSerial.h"
const uint16_t MIN_2X_BAUD = F_CPU / (4 * (2 * 0XFFF + 1)) + 1;
//------------------------------------------------------------------------------
int MinimumSerial::available() {
    return UCSR0A & (1 << RXC0) ? 1 : 0;
}
//------------------------------------------------------------------------------
void MinimumSerial::begin(uint32_t baud) {
    uint16_t baud_setting;
    // don't worry, the compiler will squeeze out F_CPU != 16000000UL
    if ((F_CPU != 16000000UL || baud != 57600) && baud > MIN_2X_BAUD) {
        // Double the USART Transmission Speed
        UCSR0A = 1 << U2X0;
        baud_setting = (F_CPU / 4 / baud - 1) / 2;
    } else {
        // hardcoded exception for compatibility with the bootloader shipped
        // with the Duemilanove and previous boards and the firmware on the 8U2
        // on the Uno and Mega 2560.
        UCSR0A = 0;
        baud_setting = (F_CPU / 8 / baud - 1) / 2;
    }
    // assign the baud_setting
    UBRR0H = baud_setting >> 8;
    UBRR0L = baud_setting;
    // enable transmit and receive
    UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
}
//------------------------------------------------------------------------------
void MinimumSerial::flush() {
    while (((1 << UDRIE0) & UCSR0B) || !(UCSR0A & (1 << UDRE0))) {
    }
}
//------------------------------------------------------------------------------
int MinimumSerial::read() {
    if (UCSR0A & (1 << RXC0)) {
        return UDR0;
    }
    return -1;
}
//------------------------------------------------------------------------------
size_t MinimumSerial::write(uint8_t b) {
    while (((1 << UDRIE0) & UCSR0B) || !(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = b;
    return 1;
}
#endif //  defined(UDR0)
