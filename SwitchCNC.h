#ifndef SWITCHCNC_H
#define SWITCHCNC_H

#include <math.h>
#include <stdint.h>

#define SWITCHCNC_VERSION       "1.0.3"
#define MACHINE_TYPE            "MILL"
#define FIRMWARE_URL            "https://github.com/switchleg1/SwitchCNC"

#define MACHINE_MODE_SPINDLE    0
#define MACHINE_MODE_LASER      1


// Use new communication model for multiple channels - only until stable, then old version gets deleted
#define NEW_COMMUNICATION 1
// ##########################################################################################
// ##                                  Debug configuration                                 ##
// ##########################################################################################
// These are run time switchable debug flags
enum debugFlags {DEB_ECHO = 0x1, DEB_INFO = 0x2, DEB_ERROR = 0x4,
                 DEB_COMMUNICATION = 0x10, DEB_NOMOVES = 0x20, DEB_DEBUG = 0x40
                };

/** Uncomment, to see detailed data for every move. Only for debugging purposes! */
//#define DEBUG_QUEUE_MOVE
/** write infos about path planner changes */
//#define DEBUG_PLANNER
/** Allows M111 to set bit 5 (16) which disables all commands except M111. This can be used
to test your data throughput or search for communication problems. */
#define INCLUDE_DEBUG_COMMUNICATION 1
// Echo all ascii commands after receiving
//#define DEBUG_ECHO_ASCII
/** Allows M111 so set bit 6 (32) which disables moves, at the first tried step. In combination
with a dry run, you can test the speed of path computations, which are still performed. */
#define INCLUDE_DEBUG_NO_MOVE 1
/** Writes the free RAM to output, if it is less then at the last test. Should always return
values >500 for safety, since it doesn't catch every function call. Nice to tweak cache
usage or for searching for memory induced errors. Switch it off for production, it costs execution time. */
//#define DEBUG_FREE_MEMORY
/** If enabled, steps to move and moved steps are compared. */
//#define DEBUG_STEPCOUNT
/** This enables code to make M666 drop an ok, so you get problems with communication. It is to test host robustness. */
//#define DEBUG_COM_ERRORS
/** Adds a menu point in quick settings to write debug informations to the host in case of hangs where the ui still works. */
//#define DEBUG_MACHINE
// Find the longest segment length during a print
//#define DEBUG_SEGMENT_LENGTH
// Find the maximum real jerk during a print
//#define DEBUG_REAL_JERK
// Debug reason for not mounting a sd card
//#define DEBUG_SD_ERROR
// Uncomment the following line to enable debugging. You can better control debugging below the following line
//#define DEBUG

#define DEBUG_MSG(x) {if(Machine::debugEcho()) { Com::printFLN(PSTR(x));HAL::delayMilliseconds(20);}}
#define DEBUG_MSG2(x,y) {if(Machine::debugEcho()) {Com::printFLN(PSTR(x),y);HAL::delayMilliseconds(20);}}
#define DEBUG_MSG_FAST(x) {if(Machine::debugEcho()) {Com::printFLN(PSTR(x));}}
#define DEBUG_MSG2_FAST(x,y) {if(Machine::debugEcho()) {Com::printFLN(PSTR(x),y);}}

#define IGNORE_COORDINATE 999999

#define IS_MAC_TRUE(x) (x!=0)
#define IS_MAC_FALSE(x) (x==0)
#define HAS_PIN(x) (defined( x ## _PIN) && x > -1)

// Uncomment if no analyzer is connected
//#define ANALYZER
// Channel->pin assignments
#define ANALYZER_CH0 63 // New move
#define ANALYZER_CH1 40 // Step loop
#define ANALYZER_CH2 53 // X Step
#define ANALYZER_CH3 65 // Y Step
#define ANALYZER_CH4 59 // X Direction
#define ANALYZER_CH5 64 // Y Direction
#define ANALYZER_CH6 58 // xsig
#define ANALYZER_CH7 57 // ysig

#ifdef ANALYZER
#define ANALYZER_ON(a) {WRITE(a,HIGH);}
#define ANALYZER_OFF(a) {WRITE(a,LOW);}
#else
#define ANALYZER_ON(a)
#define ANALYZER_OFF(a)
#endif

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define A_AXIS 3
// How big an array to hold X_AXIS..<MAX_AXIS>
#define Z_AXIS_ARRAY 3
#define A_AXIS_ARRAY 4


#define HOME_ORDER_XYZ 1
#define HOME_ORDER_XZY 2
#define HOME_ORDER_YXZ 3
#define HOME_ORDER_YZX 4
#define HOME_ORDER_ZXY 5
#define HOME_ORDER_ZYX 6

//direction flags
#define X_DIRPOS 1
#define Y_DIRPOS 2
#define Z_DIRPOS 4
#define A_DIRPOS 8
#define XYZ_DIRPOS 7
#define XYZA_DIRPOS 15

//step flags
#define XSTEP 16
#define YSTEP 32
#define ZSTEP 64
#define ASTEP 128

//combo's
#define XY_STEP 48
#define XYZ_STEP 112
#define XYZA_STEP 240
#define X_STEP_DIRPOS 17
#define Y_STEP_DIRPOS 34
#define Z_STEP_DIRPOS 68
#define A_STEP_DIRPOS 136

#define ILLEGAL_Z_PROBE -888

// we can not prevent this as some configurations need a parameter and others not
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

#include "Configuration.h"

typedef uint8_t secondspeed_t;

#ifndef MOVE_X_WHEN_HOMED
#define MOVE_X_WHEN_HOMED 0
#endif
#ifndef MOVE_Y_WHEN_HOMED
#define MOVE_Y_WHEN_HOMED 0
#endif
#ifndef MOVE_Z_WHEN_HOMED
#define MOVE_Z_WHEN_HOMED 0
#endif

#ifndef BOARD_FAN_SPEED
#define BOARD_FAN_SPEED
#endif


inline void memcopy2(void *dest,void *source) {
	*((int16_t*)dest) = *((int16_t*)source);
}
inline void memcopy4(void *dest,void *source) {
	*((int32_t*)dest) = *((int32_t*)source);
}

#if FEATURE_Z_PROBE && Z_PROBE_PIN < 0
#error You need to define Z_PROBE_PIN to use z probe!
#endif

#if DISTORTION_CORRECTION
#if !FEATURE_Z_PROBE
#error Distortion correction requires the z probe feature to be enabled and configured!
#endif
#endif

#ifndef ZHOME_X_POS
#define ZHOME_X_POS IGNORE_COORDINATE
#endif
#ifndef ZHOME_Y_POS
#define ZHOME_Y_POS IGNORE_COORDINATE
#endif

#define GCODE_BUFFER_SIZE 1

#if !defined(Z_PROBE_REPETITIONS) || Z_PROBE_REPETITIONS < 1
#define Z_PROBE_SWITCHING_DISTANCE 0.5 // Distance to safely untrigger probe
#define Z_PROBE_REPETITIONS 1
#endif

#ifndef MINMAX_HARDWARE_ENDSTOP_Z2
#define MINMAX_HARDWARE_ENDSTOP_Z2 0
#define Z2_MINMAX_PIN -1
#endif

#if MINMAX_HARDWARE_ENDSTOP_Z2 && Z2_MINMAX_PIN > -1
#undef MULTI_ZENDSTOP_HOMING 
#define MULTI_ZENDSTOP_HOMING 1
#define MULTI_ZENDSTOP_ALL 3
#else
#define MULTI_ZENDSTOP_HOMING 0
#endif

#if (X_HOME_DIR < 0 && HAS_PIN(X2_MIN) && MIN_HARDWARE_ENDSTOP_X2) || (X_HOME_DIR > 0 && HAS_PIN(X2_MAX) && MAX_HARDWARE_ENDSTOP_X2)
#define MULTI_XENDSTOP_HOMING 1
#define MULTI_XENDSTOP_ALL 3
#else
#define MULTI_XENDSTOP_HOMING 0
#endif

#if (Y_HOME_DIR < 0 && HAS_PIN(Y2_MIN) && MIN_HARDWARE_ENDSTOP_Y2) || (Y_HOME_DIR > 0 && HAS_PIN(Y2_MAX) && MAX_HARDWARE_ENDSTOP_Y2)
#define MULTI_YENDSTOP_HOMING 1
#define MULTI_YENDSTOP_ALL 3
#else
#define MULTI_YENDSTOP_HOMING 0
#endif

#define SPEED_MIN_MILLIS 400
#define SPEED_MAX_MILLIS 60
#define SPEED_MAGNIFICATION 100.0f

/**  \brief Horizontal distance bridged by the diagonal push rod when the end effector is in the center. It is pretty close to 50% of the push rod length (250 mm).
*/

#ifdef FEATURE_Z_PROBE
#define MANUAL_CONTROL 1
#endif

//Step to split a circle in small Lines
#ifndef MM_PER_ARC_SEGMENT
#define MM_PER_ARC_SEGMENT 0.5
#define MM_PER_ARC_SEGMENT_BIG 2
#else
#define MM_PER_ARC_SEGMENT_BIG MM_PER_ARC_SEGMENT
#endif
//After this count of steps a new SIN / COS calculation is started to correct the circle interpolation
#define N_ARC_CORRECTION 25

#ifndef START_STEP_WITH_HIGH
#define START_STEP_WITH_HIGH 1
#endif


#ifndef DEBUG_FREE_MEMORY
#define DEBUG_MEMORY
#else
#define DEBUG_MEMORY Commands::checkFreeMemory();
#endif

#ifndef KEEP_ALIVE_INTERVAL
#define KEEP_ALIVE_INTERVAL 2000
#endif

#include "HAL.h"
#ifndef MAX_VFAT_ENTRIES
#ifdef AVR_BOARD
#define MAX_VFAT_ENTRIES (2)
#else
#define MAX_VFAT_ENTRIES (3)
#endif
#endif
/** Total size of the buffer used to store the long filenames */
#define LONG_FILENAME_LENGTH (13*MAX_VFAT_ENTRIES+1)
#define SD_MAX_FOLDER_DEPTH 2

#include "Communication.h"

#ifndef SDCARDDETECT
#define SDCARDDETECT       -1
#endif

#ifndef SDSUPPORT
#define SDSUPPORT 0
#endif

#if SDSUPPORT
#include "src/SdFat/SdFat.h"
#endif

#include "gcode.h"

#define uint uint16_t
#define uint8 uint8_t
#define int8 int8_t
#define uint32 uint32_t
#define int32 int32_t


#undef min
#undef max

class RMath
{
public:
    static inline float min(float a,float b)
    {
        if(a < b) return a;
        return b;
    }
    static inline float max(float a,float b)
    {
        if(a < b) return b;
        return a;
    }
    static inline int32_t min(int32_t a,int32_t b)
    {
        if(a < b) return a;
        return b;
    }
    static inline int32_t min(int32_t a,int32_t b, int32_t c)
    {
        if(a < b) return a < c ? a : c;
        return b<c ? b : c;
    }
    static inline float min(float a,float b, float c)
    {
        if(a < b) return a < c ? a : c;
        return b < c ? b : c;
    }
    static inline int32_t max(int32_t a,int32_t b)
    {
        if(a < b) return b;
        return a;
    }
    static inline int min(int a,int b)
    {
        if(a < b) return a;
        return b;
    }
    static inline uint16_t min(uint16_t a,uint16_t b)
    {
        if(a < b) return a;
        return b;
    }
    static inline int16_t max(int16_t a,int16_t b)
    {
        if(a < b) return b;
        return a;
    }
    static inline uint16_t max(uint16_t a,uint16_t b)
    {
        if(a < b) return b;
        return a;
    }
    static inline unsigned long absLong(long a)
    {
        return a >= 0 ? a : -a;
    }
    static inline int32_t sqr(int32_t a)
    {
        return a*a;
    }
    static inline uint32_t sqr(uint32_t a)
    {
        return a*a;
    }
#ifdef SUPPORT_64_BIT_MATH
    static inline int64_t sqr(int64_t a)
    {
        return a*a;
    }
    static inline uint64_t sqr(uint64_t a)
    {
        return a*a;
    }
#endif

    static inline float sqr(float a)
    {
        return a*a;
    }
};

class RVector3
{
public:
    float x, y, z;
    RVector3(float _x = 0,float _y = 0,float _z = 0):x(_x),y(_y),z(_z) {};
    RVector3(const RVector3 &a):x(a.x),y(a.y),z(a.z) {};


/*    const float &operator[](std::size_t idx) const
    {
        if(idx == 0) return x;
        if(idx == 1) return y;
        return z;
    };

    float &operator[](std::size_t idx)
    {
        switch(idx) {
        case 0: return x;
        case 1: return y;
        case 2: return z;
        }
        return 0;
    };*/

    inline bool operator==(const RVector3 &rhs)
    {
        return x==rhs.x && y==rhs.y && z==rhs.z;
    }
    inline bool operator!=(const RVector3 &rhs)
    {
        return !(*this==rhs);
    }
    inline RVector3& operator=(const RVector3 &rhs)
    {
        if(this!=&rhs)
        {
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
        }
        return *this;
    }

    inline RVector3& operator+=(const RVector3 &rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    inline RVector3& operator-=(const RVector3 &rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }
    inline RVector3 operator-() const
    {
        return RVector3(-x,-y,-z);
    }

    inline float length() const
    {
        return sqrt(x * x + y * y + z * z);
    }

    inline float lengthSquared() const
    {
        return (x*x+y*y+z*z);
    }

    inline RVector3 cross(const RVector3 &b) const
    {
        return RVector3(y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x);
    }
    inline float scalar(const RVector3 &b) const
    {
        return (x*b.x+y*b.y+z*b.z);
    }
    inline RVector3 scale(float factor) const
    {
        return RVector3(x*factor,y*factor,z*factor);
    }
    inline void scaleIntern(float factor)
    {
        x*=factor;
        y*=factor;
        z*=factor;
    }
    inline void setMinimum(const RVector3 &b)
    {
        x = RMath::min(x,b.x);
        y = RMath::min(y,b.y);
        z = RMath::min(z,b.z);
    }
    inline void setMaximum(const RVector3 &b)
    {
        x = RMath::max(x,b.x);
        y = RMath::max(y,b.y);
        z = RMath::max(z,b.z);
    }
    inline float distance(const RVector3 &b) const
    {
        float dx = b.x-x,dy = b.y-y, dz = b.z-z;
        return (sqrt(dx*dx+dy*dy+dz*dz));
    }
    inline float angle(RVector3 &direction)
    {
        return static_cast<float>(acos(scalar(direction)/(length()*direction.length())));
    }

    inline RVector3 normalize() const
    {
        float len = length();
        if(len != 0) len = static_cast<float>(1.0/len);
        return RVector3(x*len,y*len,z*len);
    }

    inline RVector3 interpolatePosition(const RVector3 &b, float pos) const
    {
        float pos2 = 1.0f - pos;
        return RVector3(x * pos2 + b.x * pos, y * pos2 + b.y * pos, z * pos2 + b.z * pos);
    }

    inline RVector3 interpolateDirection(const RVector3 &b,float pos) const
    {
        //float pos2 = 1.0f - pos;

        float dot = scalar(b);
        if (dot > 0.9995 || dot < -0.9995)
            return interpolatePosition(b,pos); // cases cause trouble, use linear interpolation here

        float theta = acos(dot) * pos; // interpolated position
        float st = sin(theta);
        RVector3 t(b);
        t -= scale(dot);
        float lengthSq = t.lengthSquared();
        float dl = st * ((lengthSq < 0.0001f) ? 1.0f : 1.0f / sqrt(lengthSq));
        t.scaleIntern(dl);
        t += scale(cos(theta));
        return t.normalize();
    }
};
	inline RVector3 operator+(RVector3 lhs, const RVector3& rhs) // first arg by value, second by const ref
	{
		lhs.x += rhs.x;
		lhs.y += rhs.y;
		lhs.z += rhs.z;
		return lhs;
	}

	inline RVector3 operator-(RVector3 lhs, const RVector3& rhs) // first arg by value, second by const ref
	{
		lhs.x -= rhs.x;
		lhs.y -= rhs.y;
		lhs.z -= rhs.z;
		return lhs;
	}

    inline RVector3 operator*(const RVector3 &lhs,float rhs) {
        return lhs.scale(rhs);
    }

    inline RVector3 operator*(float lhs,const RVector3 &rhs) {
        return rhs.scale(lhs);
    }

#if !defined(MAX_FAN_PWM) || MAX_FAN_PWM == 255
#define TRIM_FAN_PWM(x) x
#undef MAX_FAN_PWM
#define MAX_FAN_PWM 255
#else
#define TRIM_FAN_PWM(x) static_cast<uint8_t>(static_cast<unsigned int>(x) * MAX_FAN_PWM / 255)
#endif

#define PWM_SPINDLE		  0
#define PWM_BOARD_FAN     PWM_SPINDLE + 1
#define PWM_FAN1          PWM_BOARD_FAN + 1
#define PWM_FAN2          PWM_FAN1 + 1
#define NUM_PWM           PWM_FAN2 + 1
extern uint8_t pwm_pos[NUM_PWM];

void manage_inactivity(uint8_t debug);

extern void finishNextSegment();
extern void linear_move(long steps_remaining[]);


extern millis_t previousMillisCmd;
extern millis_t maxInactiveTime;
extern millis_t stepperInactiveTime;

#include "AnalogIn.h"
#include "Machine.h"
#include "motion.h"
extern long baudrate;

// #include "HAL.h"


extern unsigned int counterPeriodical;
extern volatile uint8_t executePeriodical;
extern uint8_t counter500ms;
extern void writeMonitor();
#if FEATURE_FAN_CONTROL
extern uint8_t fanKickstart;
#endif
#if FEATURE_FAN2_CONTROL
extern uint8_t fan2Kickstart;
#endif

extern char tempLongFilename[LONG_FILENAME_LENGTH+1];
extern char fullName[LONG_FILENAME_LENGTH*SD_MAX_FOLDER_DEPTH+SD_MAX_FOLDER_DEPTH+1];
#if SDSUPPORT
#define SHORT_FILENAME_LENGTH 14
#include "src/SdFat/SdFat.h"

enum LsAction {LS_SerialPrint,LS_Count,LS_GetFilename};
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
    char *shortname; // Pointer to start of filename itself
    char *pathend; // File to char where pathname in fullname ends
    uint8_t sdmode;  // 1 if we are printing from sd card, 2 = stop accepting new commands
    bool sdactive;
    //int16_t n;
    bool savetosd;
    SdBaseFile parentFound;

    SDCard();
    void initsd();
    void writeCommand(GCode *code);
    bool selectFile(const char *filename,bool silent=false);
    void mount();
    void unmount();
    void startPrint();
    void pausePrint(bool intern = false);
    void continuePrint(bool intern = false);
    void stopPrint();
    inline void setIndex(uint32_t  newpos)
    {
        if(!sdactive) return;
        sdpos = newpos;
        file.seekSet(sdpos);
    }
    void printStatus();
    void ls();
    void startWrite(char *filename);
    void deleteFile(char *filename);
    void finishWrite();
    char *createFilename(char *buffer,const dir_t &p);
    void makeDirectory(char *filename);
    bool showFilename(const uint8_t *name);
    void automount();
#ifdef GLENN_DEBUG
    void writeToFile();
#endif
private:
    uint8_t lsRecursive(SdBaseFile *parent,uint8_t level,char *findFilename);
// SdFile *getDirectory(char* name);
};

extern SDCard sd;
#endif

#ifdef DEBUG_MACHINE
extern int debugWaitLoop;
#endif

#define STR(s) #s
#define XSTR(s) STR(s)
#include "Commands.h"
#include "Eeprom.h"

#define DELAY1MICROSECOND        __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t")
#define DELAY2MICROSECOND        __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\tnop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t")

#ifdef FAST_INTEGER_SQRT
#define SQRT(x) ( HAL::integerSqrt(x) )
#else
#define SQRT(x) sqrt(x)
#endif

#include "Drivers.h"

#include "Events.h"
#if defined(CUSTOM_EVENTS)
#include "CustomEvents.h"
#endif

#endif
