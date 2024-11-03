#ifndef COMMANDS_H_INCLUDED
#define COMMANDS_H_INCLUDED

class Commands
{
public:
    static void commandLoop();
    static void waitUntilEndOfAllMoves();
    static void waitUntilEndOfAllBuffers();
    static void printCurrentPosition();
    static void emergencyStop();
    static void checkFreeMemory();
    static void writeLowestFreeRAM();
    static void executeGCode(GCode* com);
    static void processMove(GCode* com, uint8_t linear);
    static void processArc(GCode* com);
    static void processGCode(GCode* com);
    static void processMCode(GCode* com);
    static void processTCode(GCode* com);

    static uint8_t allowPartialGCode;
    
private:
    static uint16_t lowestRAMValue;
    static uint16_t lowestRAMValueSend;
#if ALLOW_PARTIAL_GCODE_AS_MOVE
    static int8_t lastMoveType;
#endif
};

#endif // COMMANDS_H_INCLUDED
