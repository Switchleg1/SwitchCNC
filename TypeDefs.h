#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#define uint		uint16_t
#define uint8		uint8_t
#define int8		int8_t
#define uint32		uint32_t
#define int32		int32_t

typedef uint16_t    speed_t;
typedef uint32_t    ticks_t;
typedef uint32_t    millis_t;
typedef uint8_t     flag8_t;
typedef int8_t      fast8_t;
typedef uint8_t     ufast8_t;

// ##########################################################################################
// ##                                  Debug configuration                                 ##
// ##########################################################################################
// These are run time switchable debug flags
enum debugFlags {
    DEB_ECHO = 0x1,
    DEB_INFO = 0x2,
    DEB_ERROR = 0x4,
    DEB_COMMUNICATION = 0x10,
    DEB_NOMOVES = 0x20,
    DEB_DEBUG = 0x40
};

#endif