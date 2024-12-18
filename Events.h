#ifndef EVENTS_H_INCLUDED
#define EVENTS_H_INCLUDED

/*
Event system in a nutshell:

All printers are different and my need additions in the one or other place.
It is not very convenient to add these code parts across the firmware. For this
reason repetier-firmware uses a simple event system that comes at no cost if
a event is not used.

- simple: Only one subscriber is possible
- cost effective: Macros work as event caller. By default all macros are empty

How to use the system:

1. In Configuration.h add
#define CUSTOM_EVENTS
2. Add a file "CustomEvents.h" which overrides all event macros you need.
   It should also include the function declarations used.
3. Add a file "CustomEventsImpl.h" which includes all function definitions.
   Also it is named .h it will be included inside a cpp file only once.
   This is to compile only when selected and still keep ArduinoIDE happy.

Each of the following events describe the parameter and when it is called.
*/

// This gets called every 0.1 second
#define EVENT_TIMER_100MS {}
// This gets called every 0.5 second
#define EVENT_TIMER_500MS {}
// Gets called on a regular basis as time allows
#define EVENT_PERIODICAL {}
// Gets called when kill gets called. only_steppes = true -> we only want to disable steppers, not everything.
#define EVENT_KILL(only_steppers) {}
// Gets called if a nextPrevius actions gets executed.
#define EVENT_START_NEXTPREVIOUS(action,increment) {}
// Gets called before a move is queued. Gives the ability to limit moves.
#define EVENT_CONTRAIN_DESTINATION_COORDINATES
// Gets called when a fatal error occurs and all actions should be stopped
#define EVENT_FATAL_ERROR_OCCURED
// Gets called after a M999 to continue from fatal errors
#define EVENT_CONTINUE_FROM_FATAL_ERROR

// Initialize CNC
#define EVENT_INITIALIZE_SPINDLE true
// Turn off spindle
#define EVENT_SPINDLE_OFF true
// Turn spindle clockwise
#define EVENT_SPINDLE_CW(rpm) true
// Turn spindle counter clockwise
#define EVENT_SPINDLE_CCW(rpm) true

// Called to initialize laser pins. Return false to prevent default initialization.
#define EVENT_INITIALIZE_LASER true
// Set laser to intensity level 0 = off, 255 = full. Return false if you have overridden the setting routine.
// with true the default solution will set it as digital value.
#define EVENT_SET_LASER(intensity) true

// Allow adding new G and M codes. To implement it create a function
// bool eventUnhandledGCode(GCode *com)
// that returns true if it handled the code, otherwise false.
// Event define would then be
// #define EVENT_UNHANDLED_G_CODE(c) eventUnhandledGCode(c)
#define EVENT_UNHANDLED_G_CODE(c) false
#define EVENT_UNHANDLED_M_CODE(c) false

// This gets called every time the user has saved a value to eeprom
// or any other reason why dependent values may need recomputation.
#define EVENT_UPDATE_DERIVED {}

// Gets called after HAL is initialized, but before the regular pin settings is defined.
#define EVENT_INITIALIZE_EARLY {}
// This gets called after the basic firmware functions have initialized.
// Use this to initialize your hardware etc.
#define EVENT_INITIALIZE {}

// Allows adding custom symbols in strings that get parsed. Return false if not replaced so defaults can trigger.
// override function signature: bool parser(uint8_t c1,uint8_t c2)
#define EVENT_CUSTOM_TEXT_PARSER(c1,c2) false

// the following 2 events are equivalent to slow and fast key function and allow adding extra keys in event system.
// make sure action is called by reference so it can be changed and returned.
// Set action only if key is hit
#define EVENT_CHECK_FAST_KEYS(action) {}
#define EVENT_CHECK_SLOW_KEYS(action) {}

// Events on sd pause
#define EVENT_SD_PAUSE_START(intern) true
#define EVENT_SD_PAUSE_END(intern) {}
#define EVENT_SD_CONTINUE_START(intern) true
#define EVENT_SD_CONTINUE_END(intern) {}
#define EVENT_SD_STOP_START true
#define EVENT_SD_STOP_END {}

#define EVENT_BEFORE_Z_HOME {}

#endif // EVENTS_H_INCLUDED
