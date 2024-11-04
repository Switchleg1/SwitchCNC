#include "../../SwitchCNC.h"

// ----- SD card source -----
#if SDCARD_SUPPORT
bool SDCardGCodeSource::isOpen() {
    return (sd.sdmode > 0 && sd.sdmode < 100);
}
bool SDCardGCodeSource::supportsWrite() { ///< true if write is a non dummy function
    return false;
}
bool SDCardGCodeSource::closeOnError() { // return true if the channel can not interactively correct errors.
    return true;
}
bool SDCardGCodeSource::dataAvailable() { // would read return a new byte?
    if (sd.sdmode == 1) {
        if (sd.sdpos == sd.filesize) {
            close();
            return false;
        }
        return true;
    }
    return false;
}
int SDCardGCodeSource::readByte() {
    int n = sd.file.read();
    if (n == -1) {
        Com::printFLN(Com::tSDReadError);

        // Second try in case of recoverable errors
        sd.file.seekSet(sd.sdpos);
        n = sd.file.read();
        if (n == -1) {
            Com::printErrorFLN(PSTR("SD error did not recover!"));
            close();
            return 0;
        }
    }
    sd.sdpos++; // = file.curPosition();
    return n;
}
void SDCardGCodeSource::writeByte(uint8_t byte) {
    // dummy
}
void SDCardGCodeSource::close() {
    sd.sdmode = 0;
    GCodeSource::removeSource(this);
    Com::printFLN(Com::tDoneMilling);
}
#endif