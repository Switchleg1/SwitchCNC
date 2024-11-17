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
    static void processMove(GCode* com);
    static void processLinear(GCode* com);
#if ARC_SUPPORT
    static void processArc(GCode* com);
#endif
    static void ProcessDwell(GCode* com);
    static void ProcessHomeAxis(GCode* com);
#if Z_PROBE_SUPPORT
    static void ProcessDisplayHallSensor(GCode* com);
#if DISTORTION_CORRECTION_SUPPORT
    static void ProcessDistortionCorrection(GCode* com);
#endif
    static void ProcessToolHeight(GCode* com);
#endif
#if TMC_DRIVER_SUPPORT
    static void processShowTMCInfo(GCode* com);
#endif
    static void processSetPosition(GCode* com);
    static void ProcessEnableSteppers(GCode* com);
    static void ProcessDisableSteppers(GCode* com);
    static void ProcessChangePinState(GCode* com);
    static void ProcessPowerCommand(uint8_t on);
    static void ProcessToolInfoRequest(GCode* com);
#if FAN_CONTROL_SUPPORT
    static void ProcessFanCommand(GCode* com);
#endif
    static void ProcessWaitForPinState(GCode* com);
#if PIEZO_PIN > -1
    static void ProcessBeepCommand(GCode* com);
#endif
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
