#ifndef COMMANDS_H_INCLUDED
#define COMMANDS_H_INCLUDED

class Commands
{
public:
    static void commandLoop();
    static void processArc(GCode *com);
    static void processGCode(GCode *com);
    static void processMCode(GCode *com);
    static void processTCode(GCode* com);
    static void executeGCode(GCode *com);
    static void waitUntilEndOfAllMoves();
    static void waitUntilEndOfAllBuffers();
    static void printCurrentPosition();
    static void emergencyStop();
    static void checkFreeMemory();
    static void writeLowestFreeRAM();
private:
    static uint16_t lowestRAMValue;
    static uint16_t lowestRAMValueSend;
};

#endif // COMMANDS_H_INCLUDED
