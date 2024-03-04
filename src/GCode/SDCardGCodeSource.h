#ifndef SDCARDGCODESOURCE_H
#define SDCARDGCODESOURCE_H

#if SDSUPPORT
class SDCardGCodeSource : public GCodeSource {
public:
    virtual bool isOpen();
    virtual bool supportsWrite(); ///< true if write is a non dummy function
    virtual bool closeOnError(); // return true if the channel can not interactively correct errors.
    virtual bool dataAvailable(); // would read return a new byte?
    virtual int readByte();
    virtual void writeByte(uint8_t byte);
    virtual void close();
};
#endif

#endif