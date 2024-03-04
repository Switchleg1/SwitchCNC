#ifndef SDCARD_H
#define SDCARD_H

extern char tempLongFilename[LONG_FILENAME_LENGTH + 1];
extern char fullName[LONG_FILENAME_LENGTH * SD_MAX_FOLDER_DEPTH + SD_MAX_FOLDER_DEPTH + 1];

#if SDSUPPORT
#define SHORT_FILENAME_LENGTH 14
#include "src/SdFat/SdFat.h"

enum LsAction { LS_SerialPrint, LS_Count, LS_GetFilename };
class SDCard
{
public:
    SdFat fat;
    //Sd2Card card; // ~14 Byte
    //SdVolume volume;
    //SdFile root;
    //SdFile dir[SD_MAX_FOLDER_DEPTH+1];
    SdFile file;
    uint32_t filesize;
    uint32_t sdpos;
    //char fullName[13*SD_MAX_FOLDER_DEPTH+13]; // Fill name
    char* shortname; // Pointer to start of filename itself
    char* pathend; // File to char where pathname in fullname ends
    uint8_t sdmode;  // 1 if we are printing from sd card, 2 = stop accepting new commands
    bool sdactive;
    //int16_t n;
    bool savetosd;
    SdBaseFile parentFound;

    SDCard();
    void initsd();
    void writeCommand(GCode* code);
    bool selectFile(const char* filename, bool silent = false);
    void mount();
    void unmount();
    void startPrint();
    void pausePrint(bool intern = false);
    void continuePrint(bool intern = false);
    void stopPrint();
    inline void setIndex(uint32_t  newpos)
    {
        if (!sdactive) return;
        sdpos = newpos;
        file.seekSet(sdpos);
    }
    void printStatus();
    void ls();
    void startWrite(char* filename);
    void deleteFile(char* filename);
    void finishWrite();
    char* createFilename(char* buffer, const dir_t& p);
    void makeDirectory(char* filename);
    bool showFilename(const uint8_t* name);
    void automount();
#ifdef GLENN_DEBUG
    void writeToFile();
#endif
private:
    uint8_t lsRecursive(SdBaseFile* parent, uint8_t level, char* findFilename);
    // SdFile *getDirectory(char* name);
};

extern SDCard sd;
#if NEW_COMMUNICATION
extern SDCardGCodeSource sdSource;
#endif
#endif

#endif