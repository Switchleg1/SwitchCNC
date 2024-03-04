#ifndef SERIALGCODESOURCE_H
#define SERIALGCODESOURCE_H

class SerialGCodeSource : public GCodeSource {
    Stream* stream;
public:
    SerialGCodeSource(Stream* p);
    virtual bool isOpen();
    virtual bool supportsWrite(); ///< true if write is a non dummy function
    virtual bool closeOnError(); // return true if the channel can not interactively correct errors.
    virtual bool dataAvailable(); // would read return a new byte?
    virtual int readByte();
    virtual void writeByte(uint8_t byte);
    virtual void close();
};

#if NEW_COMMUNICATION
extern SerialGCodeSource serial0Source;
#if BLUETOOTH_SERIAL > 0
extern SerialGCodeSource serial1Source;
#endif
#endif

#endif