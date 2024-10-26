#include "../../SwitchCNC.h"

FlashGCodeSource flashSource;

FlashGCodeSource::FlashGCodeSource() :GCodeSource() {
    finished = true;
}
bool FlashGCodeSource::isOpen() {
    return !finished;
}
bool FlashGCodeSource::supportsWrite() { ///< true if write is a non dummy function
    return false;
}
bool FlashGCodeSource::closeOnError() { // return true if the channel can not interactively correct errors.
    return true;
}
bool FlashGCodeSource::dataAvailable() { // would read return a new byte?
    return !finished;
}
int FlashGCodeSource::readByte() {
    if (finished) {
        return 0;
    }
    uint8_t data = HAL::readFlashByte(pointer++);
    //printAllFLN(PSTR("FR:"),(int32_t)data);
    if (data == 0) {
        close();
    }
    return data;
}
void FlashGCodeSource::close() {
    if (!finished) {
        finished = true;
        //printAllFLN(PSTR("FlashFinished"));
        GCodeSource::removeSource(this);
    }
}

void FlashGCodeSource::writeByte(uint8_t byte) {
    // dummy
}

/** Execute the commands at the given memory. If already an other string is
running, the command will wait until that command finishes. If wait is true it
will also wait for given command to be enqueued completely. */
void FlashGCodeSource::executeCommands(FSTRINGPARAM(data), bool waitFinish, int action) {
    while (!finished) {
        Commands::commandLoop(); // might get trouble as we are called from command loop, but it's the only way to keep communication going
    }
    pointer = data;
    finished = false;
    actionOnFinish = action;
    GCodeSource::registerSource(this);
    if (waitFinish) {
        while (!finished) {
            Commands::commandLoop(); // might get trouble as we are called from command loop, but it's the only way to keep communication going
        }
    }
}
