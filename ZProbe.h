#ifndef _Z_PROBE_H_
#define _Z_PROBE_H_

#if FEATURE_Z_PROBE

class ZProbe {
public:
    static bool start();
    static void finish();

    static float run(uint8_t axisDirection, float maxDistance, uint8_t repeat);

private:
    static void waitForStart();
};

#endif

#endif