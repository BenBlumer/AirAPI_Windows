#ifndef AIRSAMPLE_H
#define AIRSAMPLE_H

#include <iostream>
#include <cstdint>

class AirSample {
public:
    uint64_t tick;
    int32_t ang_vel[3];
    int32_t accel[3];
    friend std::ostream &operator<<(std::ostream &os, const AirSample &obj);
};

class AirSampleProcessed {
public:
    AirSampleProcessed(const AirSample &sample);
    float ang_vel[3];
    float accel[3];
    uint64_t tick;
    friend std::ostream &operator<<(std::ostream &os, const AirSampleProcessed &obj);
};

#endif // AIRSAMPLE_H
