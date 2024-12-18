#include "../../SwitchCNC.h"

GCode    GCode::commandsBuffered[GCODE_BUFFER_SIZE]; ///< Buffer for received commands.
uint8_t  GCode::bufferReadIndex = 0; ///< Read position in gcode_buffer.
uint8_t  GCode::bufferWriteIndex = 0; ///< Write position in gcode_buffer.
uint8_t  GCode::commandReceiving[MAX_CMD_SIZE]; ///< Current received command.
uint8_t  GCode::commandsReceivingWritePosition = 0; ///< Writing position in gcode_transbuffer.
uint8_t  GCode::sendAsBinary; ///< Flags the command as binary input.
uint8_t  GCode::commentDetected = false; ///< Flags true if we are reading the comment part of a command.
uint8_t  GCode::binaryCommandSize; ///< Expected size of the incoming binary command.
bool     GCode::waitUntilAllCommandsAreParsed = false; ///< Don't read until all commands are parsed. Needed if gcode_buffer is misused as storage for strings.
uint32_t GCode::actLineNumber; ///< Line number of current command.
volatile uint8_t GCode::bufferLength = 0; ///< Number of commands stored in gcode_buffer
uint8_t  GCode::formatErrors = 0;
PGM_P GCode::fatalErrorMsg = NULL; ///< message unset = no fatal error 
millis_t GCode::lastBusySignal = 0; ///< When was the last busy signal
uint32_t GCode::keepAliveInterval = KEEP_ALIVE_INTERVAL;

/** \page Repetier-protocol

\section Introduction

The repetier-protocol was developed, to overcome some shortcomings in the standard
RepRap communication method, while remaining backward compatible. To use the improved
features of this protocol, you need a host which speaks it. On Windows the recommended
host software is Repetier-Host. It is developed in parallel to this firmware and supports
all implemented features.

\subsection Improvements

- With higher speeds, the serial connection is more likely to produce communication failures.
  The standard method is to transfer a checksum at the end of the line. This checksum is the
  XORd value of all characters send. The value is limited to a range between 0 and 127. It can
  not detect two identical missing characters or a wrong order. Therefore the new protocol
  uses Fletchers checksum, which overcomes these shortcommings.
- The new protocol send data in binary format. This reduces the data size to less then 50% and
  it speeds up decoding the command. No slow conversion from string to floats are needed.

*/

/** \brief Computes size of binary data from bitfield.

In the repetier-protocol in binary mode, the first 2 uint8_ts define the
data. From this bitfield, this function computes the size of the command
including the 2 uint8_ts of the bitfield and the 2 uint8_ts for the checksum.

Gcode Letter to Bit and Datatype:

- N : Bit 0 : 16-Bit Integer
- M : Bit 1 :  8-Bit unsigned uint8_t
- G : Bit 2 :  8-Bit unsigned uint8_t
- X : Bit 3 :  32-Bit Float
- Y : Bit 4 :  32-Bit Float
- Z : Bit 5 :  32-Bit Float
- E : Bit 6 :  32-Bit Float
-  : Bit 7 :  always set to distinguish binary from ASCII line.
- F : Bit 8 :  32-Bit Float
- T : Bit 9 :  8 Bit Integer
- S : Bit 10 : 32 Bit Value
- P : Bit 11 : 32 Bit Integer
- V2 : Bit 12 : Version 2 command for additional commands/sizes
- Ext : Bit 13 : There are 2 more uint8_ts following with Bits, only for future versions
- Int :Bit 14 : Marks it as internal command,
- Text : Bit 15 : 16 Byte ASCII String terminated with 0
Second word if V2:
- I : Bit 0 : 32-Bit float
- J : Bit 1 : 32-Bit float
- R : Bit 2 : 32-Bit float
- D : Bit 3 : 32-Bit float
- C : Bit 4 : 32-Bit float
- H : Bit 5 : 32-Bit float
- A : Bit 6 : 32-Bit float
- B : Bit 7 : 32-Bit float
- K : Bit 8 : 32-Bit float
- L : Bit 9 : 32-Bit float
- O : Bit 0 : 32-Bit float
*/
uint8_t GCode::computeBinarySize(char *ptr)  // unsigned int bitfield) {
{
    uint8_t s = 4; // include checksum and bitfield
    uint16_t bitfield = *(uint16_t*)ptr;
    if(bitfield & 1) s += 2;
    if(bitfield & 8) s += 4;
    if(bitfield & 16) s += 4;
    if(bitfield & 32) s += 4;
    if(bitfield & 64) s += 4;
    if(bitfield & 256) s += 4;
    if(bitfield & 512) s += 1;
    if(bitfield & 1024) s += 4;
    if(bitfield & 2048) s += 4;
    if(bitfield & 4096)   // Version 2 or later
    {
        s += 2; // for bitfield 2
        uint16_t bitfield2 = *(uint16_t*)(ptr + 2);
        if(bitfield & 2) s += 2;
        if(bitfield & 4) s += 2;
        if(bitfield2 & 1) s += 4;
        if(bitfield2 & 2) s += 4;
        if(bitfield2 & 4) s += 4;
        if(bitfield2 & 8) s += 4;
        if(bitfield2 & 16) s += 4;
        if(bitfield2 & 32) s += 4;
        if(bitfield2 & 64) s += 4;
        if(bitfield2 & 128) s += 4;
        if(bitfield2 & 256) s += 4;
        if(bitfield2 & 512) s += 4;
        if(bitfield2 & 1024) s += 4;
        if(bitfield2 & 2048) s += 4;
        if(bitfield2 & 4096) s += 4;
        if(bitfield2 & 8192) s += 4;
        if(bitfield2 & 16384) s += 4;
        if(bitfield2 & 32768) s += 4;
        if(bitfield & 32768) s += RMath::min(80,(uint8_t)ptr[4] + 1);
    }
    else
    {
        if(bitfield & 2) s += 1;
        if(bitfield & 4) s += 1;
        if(bitfield & 32768) s += 16;
    }
    return s;
}

void GCode::keepAlive(enum FirmwareState state) {
	millis_t now = HAL::timeInMilliseconds();
	
	if(state != NotBusy && keepAliveInterval != 0) {
		if(now - lastBusySignal < keepAliveInterval)
			return;
		if(state == Paused) {
			GCodeSource::printAllFLN(PSTR("busy:paused for user interaction"));	
		} else if(state == Waiting) {
			GCodeSource::printAllFLN(PSTR("busy:heating"));	
		} else { // processing and uncaught cases
			GCodeSource::printAllFLN(PSTR("busy:processing"));
		}
	}
	lastBusySignal = now;
}

void GCode::requestResend()
{
    HAL::serialFlush();
    commandsReceivingWritePosition = 0;
    if(sendAsBinary)
    GCodeSource::activeSource->waitingForResend = 30;
    else
    GCodeSource::activeSource->waitingForResend = 14;
    Com::println();
    Com::printFLN(Com::tResend,GCodeSource::activeSource->lastLineNumber + 1); 
    Com::printFLN(Com::tOk);
}

/**
  Check if result is plausible. If it is, an ok is send and the command is stored in queue.
  If not, a resend and ok is send.
*/
void GCode::checkAndPushCommand()
{
    if(hasM())
    {
        if(M == 110) {  // Reset line number
            GCodeSource::activeSource->lastLineNumber = actLineNumber;
            Com::printFLN(Com::tOk);
            GCodeSource::activeSource->waitingForResend = -1;        
            return;
        }
        if(M == 112) {  // Emergency kill - freeze printer
            Commands::emergencyStop();
        }
#ifdef DEBUG_COM_ERRORS
        if(M == 666) // force an communication error
        {
            GCodeSource::activeSource->lastLineNumber++;       
            return;
        } else if(M == 668) {
            GCodeSource::activeSource->lastLineNumber = 0;  // simulate a reset so lines are out of resend buffer        
        }
#endif // DEBUG_COM_ERRORS
    }
    if(hasN())
    {
        if((((GCodeSource::activeSource->lastLineNumber + 1) & 0xffff) != (actLineNumber & 0xffff)))    
        {
            if(static_cast<uint16_t>(GCodeSource::activeSource->lastLineNumber - actLineNumber) < 40)   
            {
                // we have seen that line already. So we assume it is a repeated resend and we ignore it
                commandsReceivingWritePosition = 0;
                Com::printFLN(Com::tSkip,actLineNumber);
                Com::printFLN(Com::tOk);
            }        
            else if(GCodeSource::activeSource->waitingForResend < 0)  // after a resend, we have to skip the garbage in buffers, no message for this
            {
                if(Machine::debugErrors()) {
                    Com::printF(Com::tExpectedLine, GCodeSource::activeSource->lastLineNumber + 1);                 
                    Com::printFLN(Com::tGot, actLineNumber);
                }
                requestResend(); // Line missing, force resend
            }
            else
            {
                --GCodeSource::activeSource->waitingForResend;
                commandsReceivingWritePosition = 0;
                Com::printFLN(Com::tSkip, actLineNumber);
                Com::printFLN(Com::tOk);
            }
            return;
        }
        GCodeSource::activeSource->lastLineNumber = actLineNumber;     
    } /*
	This test is not compatible with all hosts. Replaced by forbidding backward switch of protocols.
	else if(lastLineNumber && !(hasM() && M == 117)) { // once line number always line number!
		if(Machine::debugErrors())
        {
			Com::printErrorFLN(PSTR("Missing linenumber"));
		}
		requestResend();
		return;
	}*/
	if(GCode::hasFatalError() && !(hasM() && M==999)) {
		GCode::reportFatalError();
	} else {
		pushCommand();
	}
#ifdef DEBUG_COM_ERRORS
    if(hasM() && M == 667)
        return; // omit ok
#endif
#if ACK_WITH_LINENUMBER
    Com::printFLN(Com::tOkSpace, actLineNumber);
#else
    Com::printFLN(Com::tOk);
#endif
    GCodeSource::activeSource->wasLastCommandReceivedAsBinary = sendAsBinary;
	keepAlive(NotBusy);
	GCodeSource::activeSource->waitingForResend = -1; // everything is ok.
}

void GCode::pushCommand()
{
#if !ECHO_ON_EXECUTE
    commandsBuffered[bufferWriteIndex].echoCommand();
#endif
    if(++bufferWriteIndex >= GCODE_BUFFER_SIZE) bufferWriteIndex = 0;
    bufferLength++;
}

/**
  Get the next buffered command. Returns 0 if no more commands are buffered. For each
  returned command, the gcode_command_finished() function must be called.
*/
GCode *GCode::peekCurrentCommand()
{
    if(bufferLength == 0) return NULL; // No more data
    return &commandsBuffered[bufferReadIndex];
}

/** \brief Removes the last returned command from cache. */
void GCode::popCurrentCommand()
{
    if(!bufferLength) return; // Should not happen, but safety first
#if ECHO_ON_EXECUTE
    echoCommand();
#endif
    if(++bufferReadIndex == GCODE_BUFFER_SIZE) bufferReadIndex = 0;
    bufferLength--;
}

void GCode::echoCommand()
{
    if(Machine::debugEcho())
    {
        Com::printF(Com::tEcho);
        printCommand();
    }
}

void GCode::debugCommandBuffer()
{
    Com::printF(PSTR("CommandBuffer"));
    for(int i = 0; i < commandsReceivingWritePosition; i++)
        Com::printF(Com::tColon,(int)commandReceiving[i]);
    Com::println();
    Com::printFLN(PSTR("Binary:"), (int)sendAsBinary);
    if(!sendAsBinary)
    {
        Com::print((char*)commandReceiving);
        Com::println();
    }
}

/** \brief Execute commands in progmem stored string. Multiple commands are separated by \n 
Used to execute memory stored parts called from gcodes. For new commands use the
flash sender instead.
*/
void GCode::executeFString(FSTRINGPARAM(cmd))
{
    char buf[80];
    uint8_t buflen;
    char c = 0;
    GCode code;
    do
    {
        // Wait for a free place in command buffer
        // Scan next command from string
        uint8_t comment = 0;
        buflen = 0;
        do
        {
            c = HAL::readFlashByte(cmd++);
            if(c == 0 || c == '\n') break;
            if(c == ';') comment = 1;
            if(comment) continue;
            buf[buflen++] = c;
        }
        while(buflen < 79);
        if(buflen == 0) {  // empty line ignore
			if(!c) return; // Special case \n0
            continue;
		}
        buf[buflen] = 0;
        // Send command into command buffer
        if(code.parseAscii((char *)buf,false) && (code.params & 518))   // Success
        {
#ifdef DEBUG_MACHINE
            Machine::debugWaitLoop = 7;
#endif

            Commands::executeGCode(&code);
            Machine::defaultLoopActions();
        }
    }
    while(c);
}

/** \brief Read from serial console or sd card.

This function is the main function to read the commands from serial console or from sd card.
It must be called frequently to empty the incoming buffer.
*/
void GCode::readFromSource()
{
    if(bufferLength >= GCODE_BUFFER_SIZE || (waitUntilAllCommandsAreParsed && bufferLength)) {
		keepAlive(Processing);
		return; // all buffers full
	}
    waitUntilAllCommandsAreParsed = false;
    millis_t time = HAL::timeInMilliseconds();
    
    bool lastWTA = Com::writeToAll;
    Com::writeToAll = false;
    if(!GCodeSource::activeSource->dataAvailable())
    {
        if(GCodeSource::activeSource->closeOnError()) { // this device does not support resends so all errors are final and we always expect there is a new char!
            if(commandsReceivingWritePosition > 0) { // it's only an error if we have started reading a command
                GCodeSource::activeSource->close();
                GCodeSource::rotateSource();
                Com::writeToAll = lastWTA;
                return;
            }            
        } else {
          if((GCodeSource::activeSource->waitingForResend >= 0 || commandsReceivingWritePosition > 0) && time - GCodeSource::activeSource->timeOfLastDataPacket > 200) // only if we get no further data after 200ms it is a problem
          {
            // Com::printF(PSTR("WFR:"),waitingForResend);Com::printF(PSTR(" CRWP:"),commandsReceivingWritePosition);commandReceiving[commandsReceivingWritePosition] = 0;Com::printFLN(PSTR(" GOT:"),(char*)commandReceiving);
            requestResend(); // Something is wrong, a started line was not continued in the last second
            GCodeSource::activeSource->timeOfLastDataPacket = time;
          }
        #ifdef WAITING_IDENTIFIER
          else if(bufferLength == 0 && time - GCodeSource::activeSource->timeOfLastDataPacket > 1000)   // Don't do it if buffer is not empty. It may be a slow executing command.
          {
            Com::printFLN(Com::tWait); // Unblock communication in case the last ok was not received correct.
            GCodeSource::activeSource->timeOfLastDataPacket = time;
          }
        #endif
        }        
        if(commandsReceivingWritePosition == 0) // nothing read, we can rotate to next input source
            GCodeSource::rotateSource();
    }
    while(GCodeSource::activeSource->dataAvailable() && commandsReceivingWritePosition < MAX_CMD_SIZE)    // consume data until no data or buffer full
    {
        GCodeSource::activeSource->timeOfLastDataPacket = time; //HAL::timeInMilliseconds();
        commandReceiving[commandsReceivingWritePosition++] = GCodeSource::activeSource->readByte();
        // first lets detect, if we got an old type ascii command
        if(commandsReceivingWritePosition == 1 && commentDetected == false)
        {
            if(GCodeSource::activeSource->waitingForResend >= 0 && GCodeSource::activeSource->wasLastCommandReceivedAsBinary)
            {
                if(!commandReceiving[0])
                GCodeSource::activeSource->waitingForResend--;   // Skip 30 zeros to get in sync
                else
                GCodeSource::activeSource->waitingForResend = 30;
                commandsReceivingWritePosition = 0;
                continue;
            }
            if(!commandReceiving[0]) // Ignore zeros
            {
                commandsReceivingWritePosition = 0;
                GCodeSource::rotateSource(); // could also be end of file, so let's rotate source if it closed it self
                Com::writeToAll = lastWTA;
                return;
            }
            sendAsBinary = (commandReceiving[0] & 128) != 0;
        } // first byte detection
        if(sendAsBinary) {
            if(commandsReceivingWritePosition < 2 )
                continue;

            if(commandsReceivingWritePosition == 5 || commandsReceivingWritePosition == 4) 
                binaryCommandSize = computeBinarySize((char*)commandReceiving);

            if(commandsReceivingWritePosition == binaryCommandSize) {
                GCode *act = &commandsBuffered[bufferWriteIndex];
                act->source = GCodeSource::activeSource; // we need to know where to write answers to
                if(act->parseBinary(commandReceiving, true)) {  // Success
                    act->checkAndPushCommand();
                } else {
                    if(GCodeSource::activeSource->closeOnError()) { // this device does not support resends so all errors are final!
                        GCodeSource::activeSource->close();
                    } else {
                        requestResend();
                    }                    
                }
                GCodeSource::rotateSource();
                Com::writeToAll = lastWTA;
                return;
            }
        }
        else     // ASCII command
        {
            char ch = commandReceiving[commandsReceivingWritePosition - 1];
            if(ch == 0 || ch == '\n' || ch == '\r' || !GCodeSource::activeSource->isOpen() /*|| (!commentDetected && ch == ':')*/)  // complete line read
            {
                commandReceiving[commandsReceivingWritePosition - 1] = 0;
#ifdef DEBUG_ECHO_ASCII
                Com::printF(PSTR("Got:"));Com::print((char*)commandReceiving);Com::println();
#endif
                commentDetected = false;
                if(commandsReceivingWritePosition == 1)   // empty line ignore
                {
                    commandsReceivingWritePosition = 0;
                    continue;
                }
                GCode *act = &commandsBuffered[bufferWriteIndex];
                act->source = GCodeSource::activeSource; // we need to know where to write answers to
                if(act->parseAscii((char *)commandReceiving, true)) {  // Success
                  act->checkAndPushCommand();
                } else {
                  if(GCodeSource::activeSource->closeOnError()) { // this device doe snot support resends so all errors are final!
                      GCodeSource::activeSource->close();
                      } else {
                      requestResend();
                  }                    
                }
                GCodeSource::rotateSource();
                commandsReceivingWritePosition = 0;
                Com::writeToAll = lastWTA;
                return;
            }
            else
            {
                if(ch == ';') commentDetected = true; // ignore new data until line end
                if(commentDetected) commandsReceivingWritePosition--;
            }
        }
        if(commandsReceivingWritePosition == MAX_CMD_SIZE)
        {
          if(GCodeSource::activeSource->closeOnError()) { // this device does not support resends so all errors are final!
            GCodeSource::activeSource->close();
              GCodeSource::rotateSource();
            } else {            
              requestResend();
            }            
         }
       } // while
       Com::writeToAll = lastWTA;
}

/**
  Converts a binary uint8_tfield containing one GCode line into a GCode structure.
  Returns true if checksum was correct.
*/
bool GCode::parseBinary(uint8_t *buffer, bool fromSerial)
{
    internalCommand = !fromSerial;
    uint16_t sum1 = 0, sum2 = 0; // for fletcher-16 checksum
    // first do fletcher-16 checksum tests see
    // http://en.wikipedia.org/wiki/Fletcher's_checksum
    uint8_t *p = buffer;
    uint8_t len = binaryCommandSize - 2;
    while (len) {
        uint8_t tlen = len > 21 ? 21 : len;
        len -= tlen;
        do {
            sum1 += *p++;
            if (sum1 >= 255) sum1 -= 255;
            sum2 += sum1;
            if (sum2 >= 255) sum2 -= 255;
        }
        while (--tlen);
    }
    sum1 -= *p++;
    sum2 -= *p;
    if(sum1 | sum2) {
        if(Machine::debugErrors()) {
            Com::printErrorFLN(Com::tWrongChecksum);
        }
        return false;
    }
    p = buffer;
    params = *(uint16_t *)p;
    p += 2;
    uint8_t textlen = 16;
    if(isV2()) {
        params2 = *(uint16_t *)p;
        p += 2;
        if (hasString()) {
            textlen = *p++;
        }
    }
    else params2 = 0;
    if(params & 1) {
        actLineNumber = N = *(uint16_t *)p;
        p += 2;
    }
    if(isV2()) {   // Read G,M as 16 bit value
        if(hasM()) {
            M = *(uint16_t *)p;
            p += 2;
        }
        if(hasG()) {
            G = *(uint16_t *)p;
            p += 2;
        }
    } else {
        if(hasM()) {
            M = *p++;
        }
        if(hasG()) {
            G = *p++;
        }
    }
    //if(code->params & 8) {memcpy(&code->X,p,4);p+=4;}
    if(hasX()) {
        X = *(float *)p;
        p += 4;
    }
    if(hasY()) {
        Y = *(float *)p;
        p += 4;
    }
    if(hasZ()) {
        Z = *(float *)p;
        p += 4;
    }
    if(hasE()) {
        E = *(float *)p;
        p += 4;
    }
    if(hasF()) {
        F = *(float *)p;
        p += 4;
    }
    if(hasT()) {
        T = *p++;
    }
    if(hasS()) {
        S = *(int32_t*)p;
        p += 4;
    }
    if(hasP()) {
        P = *(int32_t*)p;
        p += 4;
    }
    if(hasI()) {
        I = *(float *)p;
        p += 4;
    }
    if(hasJ()) {
        J = *(float *)p;
        p += 4;
    }
    if(hasR()) {
        R = *(float *)p;
        p += 4;
    }
    if(hasD()) {
        D = *(float *)p;
        p += 4;
    }
    if(hasC()) {
        C = *(float *)p;
        p += 4;
    }
    if(hasH()) {
        H = *(float *)p;
        p += 4;
    }
    if(hasA()) {
        A = *(float *)p;
        p += 4;
    }
    if(hasB()) {
        B = *(float *)p;
        p += 4;
    }
    if(hasK()) {
        K = *(float *)p;
        p += 4;
    }
    if(hasL()) {
        L = *(float *)p;
        p += 4;
    }
    if(hasO()) {
        O = *(float *)p;
        p += 4;
    }
    if(hasString()) {  // set text pointer to string
        text = (char*)p;
        text[textlen] = 0; // Terminate string overwriting checksum
        waitUntilAllCommandsAreParsed = true; // Don't destroy string until executed
    }
    formatErrors = 0;
    return true;
}

/**
  Converts a ASCII GCode line into a GCode structure.
*/
bool GCode::parseAscii(char *line,bool fromSerial)
{
    char *pos = line;
    params = 0;
    params2 = 0;
    internalCommand = !fromSerial;
	bool hasChecksum = false;
    char c;
    while ( (c = *(pos++)) )
    {
        if(c == '(' || c == '%') break; // alternative comment or program block
        switch(c)
        {
        case 'N':
        case 'n':
        {
            actLineNumber = parseLongValue(pos);
            params |=1;
            N = actLineNumber;
            break;
        }
        case 'G':
        case 'g':
        {
            G = parseLongValue(pos) & 0xffff;
            params |= 4;
            if(G > 255) params |= 4096;
            break;
        }
        case 'M':
        case 'm':
        {
            M = parseLongValue(pos) & 0xffff;
            params |= 2;
            if(M > 255) params |= 4096;
            // handle non standard text arguments that some M codes have
            if (M == 20 || M == 23 || M == 28 || M == 29 || M == 30 || M == 32 || M == 36 || M == 117 || M == 531)
            {
                // after M command we got a filename or text
                char digit;
                while( (digit = *pos) )
                {
                    if (digit < '0' || digit > '9') break;
                    pos++;
                }
                while( (digit = *pos) )
                {
                    if (digit != ' ') break;
                    pos++;
                    // skip leading white spaces (may be no white space)
                }
                text = pos;
                while (*pos)
                {
                    if((M != 117 && M != 20 && M != 531 && *pos==' ') || *pos=='*') break;
                    pos++; // find a space as file name end
                }
                *pos = 0; // truncate filename by erasing space with null, also skips checksum
                waitUntilAllCommandsAreParsed = true; // don't risk string be deleted
                params |= 32768;
            }
            break;
        }
        case 'X':
        case 'x':
        {
            X = parseFloatValue(pos);
            params |= 8;
            break;
        }
        case 'Y':
        case 'y':
        {
            Y = parseFloatValue(pos);
            params |= 16;
            break;
        }
        case 'Z':
        case 'z':
        {
            Z = parseFloatValue(pos);
            params |= 32;
            break;
        }
        case 'E':
        case 'e':
        {
            E = parseFloatValue(pos);
            params |= 64;
            break;
        }
        case 'F':
        case 'f':
        {
            F = parseFloatValue(pos);
            params |= 256;
            break;
        }
        case 'T':
        case 't':
        {
            T = parseLongValue(pos) & 0xff;
            params |= 512;
            break;
        }
        case 'S':
        case 's':
        {
            S = parseLongValue(pos);
            params |= 1024;
            break;
        }
        case 'P':
        case 'p':
        {
            P = parseLongValue(pos);
            params |= 2048;
            break;
        }
        case 'I':
        case 'i':
        {
            I = parseFloatValue(pos);
            params2 |= 1;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'J':
        case 'j':
        {
            J = parseFloatValue(pos);
            params2 |= 2;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'R':
        case 'r':
        {
            R = parseFloatValue(pos);
            params2 |= 4;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'D':
        case 'd':
        {
            D = parseFloatValue(pos);
            params2 |= 8;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'C':
        case 'c':
        {
	        C = parseFloatValue(pos);
	        params2 |= 16;
	        params |= 4096; // Needs V2 for saving
	        break;
        }
        case 'H':
        case 'h':
        {
	        H = parseFloatValue(pos);
	        params2 |= 32;
	        params |= 4096; // Needs V2 for saving
	        break;
        }
        case 'A':
        case 'a':
        {
	        A = parseFloatValue(pos);
	        params2 |= 64;
	        params |= 4096; // Needs V2 for saving
	        break;
        }
        case 'B':
        case 'b':
        {
	        B = parseFloatValue(pos);
	        params2 |= 128;
	        params |= 4096; // Needs V2 for saving
	        break;
        }
        case 'K':
        case 'k':
        {
	        K = parseFloatValue(pos);
	        params2 |= 256;
	        params |= 4096; // Needs V2 for saving
	        break;
        }
        case 'L':
        case 'l':
        {
	        L = parseFloatValue(pos);
	        params2 |= 512;
	        params |= 4096; // Needs V2 for saving
	        break;
        }
        case 'O':
        case 'o':
        {
	        O = parseFloatValue(pos);
	        params2 |= 1024;
	        params |= 4096; // Needs V2 for saving
	        break;
        }
        case '*' : //checksum
        {
            uint8_t checksum_given = parseLongValue(pos);
            uint8_t checksum = 0;
            while(line != (pos - 1)) checksum ^= *line++;
            if(checksum != checksum_given)  {
                if(Machine::debugErrors()) {
                    Com::printErrorFLN(Com::tWrongChecksum);
                }
                return false; // mismatch
            }
			hasChecksum = true;
            break;
        }
        default:
            break;
        }// end switch
    }// end while
	if(GCodeSource::activeSource->wasLastCommandReceivedAsBinary && !hasChecksum && fromSerial && !waitUntilAllCommandsAreParsed) {     
		Com::printErrorFLN(PSTR("Checksum required when switching back to ASCII protocol."));
		return false;
	}
    if(hasFormatError() /*|| (params & 518) == 0*/)   // Must contain G, M or T command and parameter need to have variables!
    {
        formatErrors++;
        if(Machine::debugErrors())
            Com::printErrorFLN(Com::tFormatError);
        if(formatErrors < 3) return false;
    }
    else formatErrors = 0;
    return true;
}

/** \brief Print command on serial console */
void GCode::printCommand()
{
    if(hasN()) {
        Com::print('N');
        Com::print((int32_t)N);
        Com::print(' ');
    }
    if(hasM())
    {
        Com::print('M');
        Com::print((int)M);
        Com::print(' ');
    }
    if(hasG())
    {
        Com::print('G');
        Com::print((int)G);
        Com::print(' ');
    }
    if(hasT())
    {
        Com::print('T');
        Com::print((int)T);
        Com::print(' ');
    }
    if(hasX())
    {
        Com::printF(Com::tX,X);
    }
    if(hasY())
    {
        Com::printF(Com::tY,Y);
    }
    if(hasZ())
    {
        Com::printF(Com::tZ,Z);
    }
    if(hasE())
    {
        Com::printF(Com::tE,E,4);
    }
    if(hasF())
    {
        Com::printF(Com::tF,F);
    }
    if(hasS())
    {
        Com::printF(Com::tS,S);
    }
    if(hasP())
    {
        Com::printF(Com::tP,P);
    }
    if(hasI())
    {
        Com::printF(Com::tI,I);
    }
    if(hasJ())
    {
        Com::printF(Com::tJ,J);
    }
    if(hasR())
    {
        Com::printF(Com::tR,R);
    }
    if(hasD())
    {
        Com::printF(Com::tD,D);
    }
    if(hasC())
    {
        Com::printF(Com::tC,C);
    }
    if(hasH())
    {
        Com::printF(Com::tH,H);
    }
    if(hasA())
    {
        Com::printF(Com::tA,A);
    }
    if(hasB())
    {
        Com::printF(Com::tB,B);
    }
    if(hasK())
    {
        Com::printF(Com::tK,K);
    }
    if(hasL())
    {
        Com::printF(Com::tL,L);
    }
    if(hasO())
    {
        Com::printF(Com::tO,O);
    }
    if(hasString())
    {
        Com::print(text);
    }
    Com::println();
}

void GCode::fatalError(FSTRINGPARAM(message)) {
	fatalErrorMsg = message;
	flashSource.close(); // stop flash printing if busy
#if SDCARD_SUPPORT
	sd.stopPrint();
#endif
	GCodeSource::printAllFLN(PSTR("RequestStop:"));
	if(Machine::currentPosition[Z_AXIS] < Machine::axisMin[Z_AXIS] + Machine::axisLength[Z_AXIS] - 15)
		MachineLine::moveRelativeDistanceInSteps(0, 0, 10 * Machine::axisStepsPerMM[Z_AXIS], 0, Machine::homingFeedrate[Z_AXIS], true, true);
	EVENT_FATAL_ERROR_OCCURED
	Commands::waitUntilEndOfAllMoves();
	Machine::kill(false);
	reportFatalError();
}

void GCode::reportFatalError() {
    Com::writeToAll = true;
	Com::printF(Com::tFatal);
	Com::printF(fatalErrorMsg);
	Com::printFLN(PSTR(" - Printer stopped. Fix error and restart with M999."));
}

void GCode::resetFatalError() {
    Com::writeToAll = true;
	Machine::debugReset(8); // disable dry run
	fatalErrorMsg = NULL;
	EVENT_CONTINUE_FROM_FATAL_ERROR
	Com::printFLN(PSTR("info:Continue from fatal state"));
}