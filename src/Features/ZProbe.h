#ifndef _Z_PROBE_H_
#define _Z_PROBE_H_

#if Z_PROBE_SUPPORT

class ZProbe {
public:
    static void initialize();
    static bool start();
    static void finish();

    static float run(uint8_t axisDirection, float maxDistance, uint8_t repeat);

    static inline uint8_t isActive() {
        return active;
    }
private:
    static void waitForStart();

    static uint8_t active;
};

#endif

#endif