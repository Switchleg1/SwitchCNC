#ifndef _GCODE_H
#define _GCODE_H

#define MAX_CMD_SIZE            96
#define ARRAY_SIZE(_x)          (sizeof(_x)/sizeof(_x[0]))

enum FirmwareState {
    NotBusy=0,
    Processing,
    Paused,
    Waiting
};

class SDCard;
class Commands;

class GCode   // 52 uint8_ts per command needed
{
    uint16_t params;
    uint16_t params2;
public:
    uint16_t N; ///< Line number reduced to 16 bit
    uint16_t M; ///< G-code M value if set
    uint16_t G; ///< G-code G value if set
    float X; ///< G-code X value if set
    float Y; ///< G-code Y value if set
    float Z; ///< G-code Z value if set
    float E; ///< G-code E value if set
    float F; ///< G-code F value if set
    int32_t S; ///< G-code S value if set
    int32_t P; ///< G-code P value if set
    float I; ///< G-code I value if set
    float J; ///< G-code J value if set
    float R; ///< G-code R value if set
    float D; ///< G-code D value if set
    float C; ///< G-code C value if set
    float H; ///< G-code H value if set
    float A; ///< G-code A value if set
    float B; ///< G-code B value if set
    float K; ///< G-code K value if set
    float L; ///< G-code L value if set
    float O; ///< G-code O value if set

    char *text; ///< Text message of g-code if present.
    //moved the byte to the end and aligned ints on short boundary
    // Old habit from PC, which require alignments for data types such as int and long to be on 2 or 4 byte boundary
    // Otherwise, the compiler adds padding, wasted space.
    uint8_t T; // This may not matter on any of these controllers, but it can't hurt
    // True if origin did not come from serial console. That way we can send status messages to
    // a host only if he would normally not know about the mode switch.
    bool internalCommand;

    GCodeSource* source;
    inline bool hasM()
    {
        return ((params & 2)!=0);
    }
    inline bool hasN()
    {
        return ((params & 1)!=0);
    }
    inline bool hasG()
    {
        return ((params & 4)!=0);
    }
    inline void setG()
    {
        params |= 4;
    }
    inline bool hasX()
    {
        return ((params & 8)!=0);
    }
	inline void unsetX() {
		params &= ~8;
	}
    inline bool hasY()
    {
        return ((params & 16)!=0);
    }
	inline void unsetY() {
		params &= ~16;
	}
    inline bool hasZ()
    {
        return ((params & 32)!=0);
    }
	inline void unsetZ() {
		params &= ~32;
	}
    inline bool hasNoXYZ()
    {
		return ((params & 56)==0);
	}
	inline bool hasNoXYZA()
    {
		return ((params & 56)==0 && (params2 & 64)==0);
	}
    inline bool hasE()
    {
        return ((params & 64)!=0);
    }
    inline bool hasF()
    {
        return ((params & 256)!=0);
    }
    inline bool hasT()
    {
        return ((params & 512)!=0);
    }
    inline bool hasS()
    {
        return ((params & 1024)!=0);
    }
    inline void setS()
    {
        params |= 1024;
    }
    inline bool hasP()
    {
        return ((params & 2048)!=0);
    }
    inline bool isV2()
    {
        return ((params & 4096)!=0);
    }
    inline bool hasString()
    {
        return ((params & 32768)!=0);
    }
    inline bool hasI()
    {
        return ((params2 & 1)!=0);
    }
    inline bool hasJ()
    {
        return ((params2 & 2)!=0);
    }
    inline bool hasR()
    {
        return ((params2 & 4)!=0);
    }
    inline bool hasD()
    {
        return ((params2 & 8)!=0);
    }
    inline bool hasC()
    {
        return ((params2 & 16)!=0);
    }
    inline bool hasH()
    {
        return ((params2 & 32)!=0);
    }
    inline bool hasA()
	{
		return ((params2 & 64)!=0);
    }
    inline bool hasB()
    {
        return ((params2 & 128)!=0);
    }
    inline bool hasK()
    {
        return ((params2 & 256)!=0);
    }
    inline bool hasL()
    {
        return ((params2 & 512)!=0);
    }
    inline bool hasO()
    {
        return ((params2 & 1024)!=0);
    }
    inline long getS(long def)
    {
        return (hasS() ? S : def);
    }
    inline long getP(long def)
    {
        return (hasP() ? P : def);
    }
    inline void setFormatError() {
        params2 |= 32768;
    }
    inline bool hasFormatError() {
        return ((params2 & 32768)!=0);
    }
    inline uint8_t isSendingBinary() {
        return sendAsBinary;
    }
    void printCommand();
    bool parseBinary(uint8_t *buffer,bool fromSerial);
    bool parseAscii(char *line,bool fromSerial);
    void popCurrentCommand();
    void echoCommand();
    /** Get next command in command buffer. After the command is processed, call gcode_command_finished() */
    static GCode *peekCurrentCommand();
    /** Frees the cache used by the last command fetched. */
    static void readFromSource();
    static void pushCommand();
    static void executeFString(FSTRINGPARAM(cmd));
    static uint8_t computeBinarySize(char *ptr);
	static void fatalError(FSTRINGPARAM(message));
	static void reportFatalError();
	static void resetFatalError();
	inline static bool hasFatalError() {
		return fatalErrorMsg != NULL;
	}
	static void keepAlive(enum FirmwareState state);
	static uint32_t keepAliveInterval;
    friend class SDCard;
	static FSTRINGPARAM(fatalErrorMsg);
    friend class GCodeSource;    
protected:
    void debugCommandBuffer();
    void checkAndPushCommand();
    static void requestResend();
    inline float parseFloatValue(char *s)
    {
        char *endPtr;
        while(*s == 32) s++; // skip spaces
        float f = (strtod(s, &endPtr));
        if(s == endPtr) f=0.0; // treat empty string "x " as "x0"
        return f;
    }
    inline long parseLongValue(char *s)
    {
        char *endPtr;
        while(*s == 32) s++; // skip spaces
        long l = (strtol(s, &endPtr, 10));
        if(s == endPtr) l=0; // treat empty string argument "p " as "p0"
        return l;
    }

    static GCode commandsBuffered[GCODE_BUFFER_SIZE]; ///< Buffer for received commands.
    static uint8_t bufferReadIndex; ///< Read position in gcode_buffer.
    static uint8_t bufferWriteIndex; ///< Write position in gcode_buffer.
    static uint8_t commandReceiving[MAX_CMD_SIZE]; ///< Current received command.
    static uint8_t commandsReceivingWritePosition; ///< Writing position in gcode_transbuffer.
    static uint8_t sendAsBinary; ///< Flags the command as binary input.
    static uint8_t commentDetected; ///< Flags true if we are reading the comment part of a command.
    static uint8_t binaryCommandSize; ///< Expected size of the incoming binary command.
    static bool waitUntilAllCommandsAreParsed; ///< Don't read until all commands are parsed. Needed if gcode_buffer is misused as storage for strings.
    static uint32_t actLineNumber; ///< Line number of current command.
    static volatile uint8_t bufferLength; ///< Number of commands stored in gcode_buffer
    static uint8_t formatErrors; ///< Number of sequential format errors
	static millis_t lastBusySignal; ///< When was the last busy signal
};

#endif

