#include "../../SwitchCNC.h"

#if BLUETOOTH_SERIAL > 0
uint8_t GCodeSource::numSources = 2; ///< Number of data sources available
uint8_t GCodeSource::numWriteSources = 2;
GCodeSource* GCodeSource::sources[MAX_DATA_SOURCES] = { &serial0Source, &serial1Source };
GCodeSource* GCodeSource::writeableSources[MAX_DATA_SOURCES] = { &serial0Source, &serial1Source };
#else
uint8_t GCodeSource::numSources = 1; ///< Number of data sources available
uint8_t GCodeSource::numWriteSources = 1;
GCodeSource* GCodeSource::sources[MAX_DATA_SOURCES] = { &serial0Source };
GCodeSource* GCodeSource::writeableSources[MAX_DATA_SOURCES] = { &serial0Source };
#endif    
GCodeSource* GCodeSource::activeSource = &serial0Source;

void GCodeSource::registerSource(GCodeSource* newSource) {
    for (uint8_t i = 0; i < numSources; i++) { // skip register if already contained
        if (sources[i] == newSource) {
            return;
        }
    }
    //printAllFLN(PSTR("AddSource:"),numSources);
    sources[numSources++] = newSource;
    if (newSource->supportsWrite())
        writeableSources[numWriteSources++] = newSource;
}

void GCodeSource::removeSource(GCodeSource* delSource) {
    for (uint8_t i = 0; i < numSources; i++) {
        if (sources[i] == delSource) {
            //printAllFLN(PSTR("DelSource:"),i);
            sources[i] = sources[--numSources];
            break;
        }
    }
    for (uint8_t i = 0; i < numWriteSources; i++) {
        if (writeableSources[i] == delSource) {
            writeableSources[i] = writeableSources[--numWriteSources];
            break;
        }
    }
    if (activeSource == delSource)
        rotateSource();
}

void GCodeSource::rotateSource() { ///< Move active to next source
    uint8_t bestIdx = 0; //,oldIdx = 0;
    for (uint8_t i = 0; i < numSources; i++) {
        if (sources[i] == activeSource) {
            //oldIdx = 
            bestIdx = i;
            break;
        }
    }
    for (uint8_t i = 0; i < numSources; i++) {
        if (++bestIdx >= numSources) bestIdx = 0;
        if (sources[bestIdx]->dataAvailable()) break;
    }
    //if(oldIdx != bestIdx)
    //    printAllFLN(PSTR("Rotate:"),(int32_t)bestIdx);
    activeSource = sources[bestIdx];
    GCode::commandsReceivingWritePosition = 0;
}

void GCodeSource::writeToAll(uint8_t byte) { ///< Write to all listening sources 
    if (Com::writeToAll) {
        for (uint8_t i = 0; i < numWriteSources; i++) {
            writeableSources[i]->writeByte(byte);
        }
    }
    else {
        activeSource->writeByte(byte);
    }     
}

void GCodeSource::printAllFLN(FSTRINGPARAM(text)) {
    bool old = Com::writeToAll;
    Com::writeToAll = true;
    Com::printFLN(text);
    Com::writeToAll = old;
}
void GCodeSource::printAllFLN(FSTRINGPARAM(text), int32_t v) {
    bool old = Com::writeToAll;
    Com::writeToAll = true;
    Com::printFLN(text, v);
    Com::writeToAll = old;
}


GCodeSource::GCodeSource() {
    lastLineNumber = 0;
    wasLastCommandReceivedAsBinary = false;
    waitingForResend = -1;
}
