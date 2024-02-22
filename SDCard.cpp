/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/

#include "Repetier.h"

#if SDSUPPORT

char tempLongFilename[LONG_FILENAME_LENGTH + 1];
char fullName[LONG_FILENAME_LENGTH * SD_MAX_FOLDER_DEPTH + SD_MAX_FOLDER_DEPTH + 1];
#if NEW_COMMUNICATION
SDCardGCodeSource sdSource;
#endif
SDCard sd;

SDCard::SDCard() {
    sdmode = 0;
    sdactive = false;
    savetosd = false;
    Printer::setAutomount(false);
}

void SDCard::automount() {
#if SDCARDDETECT > -1
    if(READ(SDCARDDETECT) != SDCARDDETECTINVERTED) {
        if(sdactive || sdmode == 100) { // Card removed
			Com::printFLN(PSTR("SD card removed"));
			unmount();
        }
    } else {
        if(!sdactive && sdmode != 100) {
			mount();
            if(sdmode != 100) // send message only if we have success
				Com::printFLN(PSTR("SD card inserted")); // Not translatable or host will not understand signal
        }
    }
#endif
}

void SDCard::initsd() {
    sdactive = false;
#if SDSS > -1
#if SDCARDDETECT > -1
    if(READ(SDCARDDETECT) != SDCARDDETECTINVERTED)
        return;
#endif
    HAL::pingWatchdog();
    HAL::delayMilliseconds(50); // wait for stabilization of contacts, bootup ...
#if defined(ENABLE_SOFTWARE_SPI_CLASS) && ENABLE_SOFTWARE_SPI_CLASS
	fat.begin(SDSS);
#else
	fat.begin(SDSS, SD_SCK_MHZ(4)); // dummy init of SD_CARD
#endif
    HAL::delayMilliseconds(50);       // wait for init end
    HAL::pingWatchdog();
    /*if(dir[0].isOpen())
        dir[0].close();*/
    if (!fat.begin(SDSS, SD_SCK_MHZ(4))) {
        Com::printFLN(Com::tSDInitFail);
        sdmode = 100; // prevent automount loop!
        if (fat.card()->errorCode()) {
            Com::printFLN(PSTR(
                              "\nSD initialization failed.\n"
                              "Do not reformat the card!\n"
                              "Is the card correctly inserted?\n"
                              "Is chipSelect set to the correct value?\n"
                              "Does another SPI device need to be disabled?\n"
                              "Is there a wiring/soldering problem?"));
            Com::printFLN(PSTR("errorCode: "), int(fat.card()->errorCode()));
            return;
        }
        if (fat.vol()->fatType() == 0) {
            Com::printFLN(PSTR("Can't find a valid FAT16/FAT32 partition.\n"));
            return;
        }
        if (!fat.vwd()->isOpen()) {
            Com::printFLN(PSTR("Can't open root directory.\n"));
            return;
        }
        return;
    }
    Com::printFLN(PSTR("Card successfully initialized."));
    sdactive = true;
	HAL::pingWatchdog();

    fat.chdir();

#if defined(EEPROM_AVAILABLE) && EEPROM_AVAILABLE == EEPROM_SDCARD
    HAL::importEEPROM();
#endif
    if(selectFile("init.g", true)) {
        startPrint();
    }
#endif
}

void SDCard::mount() {
    sdmode = 0;
    initsd();
}

void SDCard::unmount() {
    sdmode = 0;
    sdactive = false;
    savetosd = false;
    Printer::setAutomount(false);
	Com::printFLN(PSTR("SD Card unmounted"));
}

void SDCard::startPrint() {
    if(!sdactive) return;
	sdmode = 1;
#if NEW_COMMUNICATION
    GCodeSource::registerSource(&sdSource);
#endif
}

void SDCard::pausePrint(bool intern) {
    if(!sdactive) return;
	sdmode = 2; // finish running line
#if NEW_COMMUNICATION
    GCodeSource::removeSource(&sdSource);
#endif
    if(EVENT_SD_PAUSE_START(intern)) {
        if(intern) {
            Commands::waitUntilEndOfAllBuffers();
            //sdmode = 0; // why ?
            Printer::MemoryPosition();
            Printer::lastCmdPos[X_AXIS] = Printer::currentPosition[X_AXIS];
            Printer::lastCmdPos[Y_AXIS] = Printer::currentPosition[Y_AXIS];
            Printer::lastCmdPos[Z_AXIS] = Printer::currentPosition[Z_AXIS];
            GCode::executeFString(PSTR(PAUSE_START_COMMANDS));
        }
    }
    EVENT_SD_PAUSE_END(intern);
}

void SDCard::continuePrint(bool intern) {
    if(!sd.sdactive) return;
    if(EVENT_SD_CONTINUE_START(intern)) {
        if(intern) {
            GCode::executeFString(PSTR(PAUSE_END_COMMANDS));
			Printer::GoToMemoryPosition(true, true, false, false, Printer::maxFeedrate[X_AXIS]);
			Printer::GoToMemoryPosition(false, false, true, false, Printer::maxFeedrate[Z_AXIS] / 2.0f);
			Printer::GoToMemoryPosition(false, false, false, true, Printer::maxFeedrate[A_AXIS] / 2.0f);
        }
    }
    EVENT_SD_CONTINUE_END(intern);
#if NEW_COMMUNICATION
    GCodeSource::registerSource(&sdSource);
#endif
	sdmode = 1;
}

void SDCard::stopPrint() {
    if(!sd.sdactive) return;
    if(sdmode)
        Com::printFLN(PSTR("SD print stopped by user."));
    sdmode = 0;
#if NEW_COMMUNICATION
    GCodeSource::removeSource(&sdSource);
#endif
    if(EVENT_SD_STOP_START) {
        GCode::executeFString(PSTR(SD_RUN_ON_STOP));
        if(SD_STOP_MOTORS_ON_STOP) {
            Commands::waitUntilEndOfAllMoves();
            Printer::kill(false);
        }
    }
    EVENT_SD_STOP_END;
}

void SDCard::writeCommand(GCode *code) {
    unsigned int sum1 = 0, sum2 = 0; // for fletcher-16 checksum
    uint8_t buf[100];
    uint8_t p = 2;
    file.clearWriteError();
    uint16_t params = 128 | (code->params & ~1);
    memcopy2(buf, &params);
    //*(int*)buf = params;
    if(code->isV2()) { // Read G,M as 16 bit value
        memcopy2(&buf[p], &code->params2);
        //*(int*)&buf[p] = code->params2;
        p += 2;
        if(code->hasString())
            buf[p++] = strlen(code->text);
        if(code->hasM()) {
            memcopy2(&buf[p], &code->M);
            //*(int*)&buf[p] = code->M;
            p += 2;
        }
        if(code->hasG()) {
            memcopy2(&buf[p], &code->G);
            //*(int*)&buf[p]= code->G;
            p += 2;
        }
    } else {
        if(code->hasM()) {
            buf[p++] = (uint8_t)code->M;
        }
        if(code->hasG()) {
            buf[p++] = (uint8_t)code->G;
        }
    }
    if(code->hasX()) {
        memcopy4(&buf[p], &code->X);
        //*(float*)&buf[p] = code->X;
        p += 4;
    }
    if(code->hasY()) {
        memcopy4(&buf[p], &code->Y);
        //*(float*)&buf[p] = code->Y;
        p += 4;
    }
    if(code->hasZ()) {
        memcopy4(&buf[p], &code->Z);
        //*(float*)&buf[p] = code->Z;
        p += 4;
    }
    if(code->hasE()) {
        memcopy4(&buf[p], &code->E);
        //*(float*)&buf[p] = code->E;
        p += 4;
    }
    if(code->hasF()) {
        memcopy4(&buf[p], &code->F);
        //*(float*)&buf[p] = code->F;
        p += 4;
    }
    if(code->hasT()) {
        buf[p++] = code->T;
    }
    if(code->hasS()) {
        memcopy4(&buf[p], &code->S);
        //*(int32_t*)&buf[p] = code->S;
        p += 4;
    }
    if(code->hasP()) {
        memcopy4(&buf[p], &code->P);
        //*(int32_t*)&buf[p] = code->P;
        p += 4;
    }
    if(code->hasI()) {
        memcopy4(&buf[p], &code->I);
        //*(float*)&buf[p] = code->I;
        p += 4;
    }
    if(code->hasJ()) {
        memcopy4(&buf[p], &code->J);
        //*(float*)&buf[p] = code->J;
        p += 4;
    }
    if(code->hasR()) {
        memcopy4(&buf[p], &code->R);
        //*(float*)&buf[p] = code->R;
        p += 4;
    }
    if(code->hasD()) {
        memcopy4(&buf[p], &code->D);
        //*(float*)&buf[p] = code->D;
        p += 4;
    }
    if(code->hasC()) {
        memcopy4(&buf[p], &code->C);
        //*(float*)&buf[p] = code->C;
        p += 4;
    }
    if(code->hasH()) {
        memcopy4(&buf[p], &code->H);
        //*(float*)&buf[p] = code->H;
        p += 4;
    }
    if(code->hasA()) {
        memcopy4(&buf[p], &code->A);
        //*(float*)&buf[p] = code->A;
        p += 4;
    }
    if(code->hasB()) {
        memcopy4(&buf[p], &code->B);
        //*(float*)&buf[p] = code->B;
        p += 4;
    }
    if(code->hasK()) {
        memcopy4(&buf[p], &code->K);
        //*(float*)&buf[p] = code->K;
        p += 4;
    }
    if(code->hasL()) {
        memcopy4(&buf[p], &code->L);
        //*(float*)&buf[p] = code->L;
        p += 4;
    }
    if(code->hasO()) {
        memcopy4(&buf[p], &code->O);
        //*(float*)&buf[p] = code->O;
        p += 4;
    }
    if(code->hasString()) { // read 16 uint8_t into string
        char *sp = code->text;
        if(code->isV2()) {
            uint8_t i = strlen(code->text);
            for(; i; i--) buf[p++] = *sp++;
        } else {
            for(uint8_t i = 0; i < 16; ++i) buf[p++] = *sp++;
        }
    }
    uint8_t *ptr = buf;
    uint8_t len = p;
    while (len) {
        uint8_t tlen = len > 21 ? 21 : len;
        len -= tlen;
        do {
            sum1 += *ptr++;
            if(sum1 >= 255) sum1 -= 255;
            sum2 += sum1;
            if(sum2 >= 255) sum2 -= 255;
        } while (--tlen);
    }
    buf[p++] = sum1;
    buf[p++] = sum2;
    // Debug
    /*Com::printF(PSTR("Buf: "));
    for(int i=0;i<p;i++)
    Com::printF(PSTR(" "),(int)buf[i]);
    Com::println();*/
    if(params == 128) {
        Com::printErrorFLN(Com::tAPIDFinished);
    } else
        file.write(buf, p);
    if (file.getWriteError()) {
        Com::printFLN(Com::tErrorWritingToFile);
    }
}

char *SDCard::createFilename(char *buffer, const dir_t &p) {
    char *pos = buffer, *src = (char*)p.name;
    for (uint8_t i = 0; i < 11; i++, src++) {
        if (*src == ' ') continue;
        if (i == 8)
            *pos++ = '.';
        *pos++ = *src;
    }
    *pos = 0;
    return pos;
}

bool SDCard::showFilename(const uint8_t *name) {
    if (*name == DIR_NAME_DELETED || *name == '.') return false;
    return true;
}

int8_t RFstricmp(const char* s1, const char* s2) {
    while(*s1 && (tolower(*s1) == tolower(*s2)))
        s1++, s2++;
    return (const uint8_t)tolower(*s1) - (const uint8_t)tolower(*s2);
}

int8_t RFstrnicmp(const char* s1, const char* s2, size_t n) {
    while(n--) {
        if(tolower(*s1) != tolower(*s2))
            return (uint8_t)tolower(*s1) - (uint8_t)tolower(*s2);
        s1++;
        s2++;
    }
    return 0;
}

void SDCard::ls() {
    SdBaseFile file;

    Com::printFLN(Com::tBeginFileList);
    fat.chdir();

    file.openRoot(fat.vol());
    file.ls(0, 0);
    Com::printFLN(Com::tEndFileList);
}

bool SDCard::selectFile(const char* filename, bool silent) {
    const char* oldP = filename;

    if (!sdactive)
        return false;
    sdmode = 0;

    file.close();
    // Filename for progress view
    if (file.open(fat.vwd(), filename, O_READ)) {
        if ((oldP = strrchr(filename, '/')) != NULL)
            oldP++;
        else
            oldP = filename;

        if (!silent) {
            Com::printF(Com::tFileOpened, oldP);
            Com::printFLN(Com::tSpaceSizeColon, file.fileSize());
		}

        sdpos = 0;
        filesize = file.fileSize();
        Com::printFLN(Com::tFileSelected);
        return true;
    } else {
        if (!silent)
            Com::printFLN(Com::tFileOpenFailed);
        return false;
    }
}

void SDCard::printStatus() {
    if(sdactive) {
        Com::printF(Com::tSDPrintingByte, sdpos);
        Com::printFLN(Com::tSlash, filesize);
    } else {
        Com::printFLN(Com::tNotSDPrinting);
    }
}

void SDCard::startWrite(char *filename) {
    if(!sdactive) return;
    file.close();
    sdmode = 0;
    fat.chdir();
    if(!file.open(filename, O_CREAT | O_APPEND | O_WRITE | O_TRUNC)) {
        Com::printFLN(Com::tOpenFailedFile, filename);
    } else {
		savetosd = true;
        Com::printFLN(Com::tWritingToFile, filename);
    }
}

void SDCard::finishWrite() {
    if(!savetosd) return; // already closed or never opened
    file.sync();
    file.close();
    savetosd = false;
    Com::printFLN(Com::tDoneSavingFile);
}

void SDCard::deleteFile(char *filename) {
    if(!sdactive) return;
    sdmode = 0;
    file.close();
    if(fat.remove(filename)) {
        Com::printFLN(Com::tFileDeleted);
    } else {
        if(fat.rmdir(filename))
            Com::printFLN(Com::tFileDeleted);
        else
            Com::printFLN(Com::tDeletionFailed);
    }
}

void SDCard::makeDirectory(char *filename) {
    if(!sdactive) return;
    sdmode = 0;
    file.close();
    if(fat.mkdir(filename)) {
        Com::printFLN(Com::tDirectoryCreated);
    } else {
        Com::printFLN(Com::tCreationFailed);
    }
}

#ifdef GLENN_DEBUG
void SDCard::writeToFile() {
    size_t nbyte;
    char szName[10];

    strcpy(szName, "Testing\r\n");
    nbyte = file.write(szName, strlen(szName));
    Com::print("L=");
    Com::print((long)nbyte);
    Com::println();
}

#endif

#endif
