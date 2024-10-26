#ifndef FLASHGCODESOURCH_H
#define FLASHGCODESOURCE_H

class FlashGCodeSource : public GCodeSource {
public:
    FSTRINGPARAM(pointer);
    volatile bool finished;
    int actionOnFinish;

    FlashGCodeSource();
    virtual bool isOpen();
    virtual bool supportsWrite(); ///< true if write is a non dummy function
    virtual bool closeOnError(); // return true if the channel can not interactively correct errors.
    virtual bool dataAvailable(); // would read return a new byte?
    virtual int readByte();
    virtual void writeByte(uint8_t byte);
    virtual void close();

    /** Execute the commands at the given memory. If already an other string is
    running, the command will wait until that command finishes. If wait is true it
    will also wait for given command to be enqueued completely. */
    void executeCommands(FSTRINGPARAM(data), bool waitFinish, int action);
};

extern FlashGCodeSource flashSource;

#endif