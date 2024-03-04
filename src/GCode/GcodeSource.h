#ifndef GCODESOURCE_H
#define GCODESOURCE_H

#ifndef MAX_DATA_SOURCES
#define MAX_DATA_SOURCES 4
#endif

/** This class defines the general interface to handle gcode communication with the firmware. This
allows it to connect to different data sources and handle them all inside the same data structure.
If several readers are active, the first one sending a byte pauses all other inputs until the command
is complete. Only then the next reader will be queried. New queries are started in round robin fashion
so every channel gets the same chance to send commands.

Available source types are:
- serial communication port
- sd card
- flash memory
*/
class GCodeSource {
    static fast8_t numSources; ///< Number of data sources available
    static fast8_t numWriteSources;
    static GCodeSource* sources[MAX_DATA_SOURCES];
    static GCodeSource* writeableSources[MAX_DATA_SOURCES];
public:
    static GCodeSource* activeSource;
    static void registerSource(GCodeSource* newSource);
    static void removeSource(GCodeSource* delSource);
    static void rotateSource(); ///< Move active to next source
    static void writeToAll(uint8_t byte); ///< Write to all listening sources
    static void printAllFLN(FSTRINGPARAM(text));
    static void printAllFLN(FSTRINGPARAM(text), int32_t v);
    uint32_t lastLineNumber;
    uint8_t wasLastCommandReceivedAsBinary; ///< Was the last successful command in binary mode?
    millis_t timeOfLastDataPacket;
    int8_t waitingForResend; ///< Waiting for line to be resend. -1 = no wait.

    GCodeSource();
    virtual ~GCodeSource() {}
    virtual bool isOpen() = 0;
    virtual bool supportsWrite() = 0; ///< true if write is a non dummy function
    virtual bool closeOnError() = 0; // return true if the channel can not interactively correct errors.
    virtual bool dataAvailable() = 0; // would read return a new byte?
    virtual int readByte() = 0;
    virtual void close() = 0;
    virtual void writeByte(uint8_t byte) = 0;
};

#endif