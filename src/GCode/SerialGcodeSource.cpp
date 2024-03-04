#include "../../SwitchCNC.h"

#if NEW_COMMUNICATION
SerialGCodeSource serial0Source(&RFSERIAL);
#if BLUETOOTH_SERIAL > 0
SerialGCodeSource serial1Source(&RFSERIAL2);
#endif
#endif

// ----- serial connection source -----
SerialGCodeSource::SerialGCodeSource(Stream* p) {
    stream = p;
}
bool SerialGCodeSource::isOpen() {
    return true;
}
bool SerialGCodeSource::supportsWrite() { ///< true if write is a non dummy function
    return true;
}
bool SerialGCodeSource::closeOnError() { // return true if the channel can not interactively correct errors.
    return false;
}
bool SerialGCodeSource::dataAvailable() { // would read return a new byte?
    return stream->available();
}
int SerialGCodeSource::readByte() {
    return stream->read();
}
void SerialGCodeSource::writeByte(uint8_t byte) {
    stream->write(byte);
}
void SerialGCodeSource::close() {
}