#include "AirSample.h"
// based on 24bit signed int w/ FSR = +/-2000 dps, datasheet option
#define GYRO_SCALAR (1.0f / 8388608.0f * 2000.0f)

// based on 24bit signed int w/ FSR = +/-16 g, datasheet option
#define ACCEL_SCALAR (1.0f / 8388608.0f * 16.0f)
std::ostream &operator<<(std::ostream &os, const AirSample &obj) {
    os << "Time: " << obj.tick << ", ";

    os << "Angular Velocity: (";
    for (int i = 0; i < 3; ++i) {
        os << obj.ang_vel[i];
        if (i != 2) os << ", ";
    }
    os << "), ";

    os << "Acceleration: (";
    for (int i = 0; i < 3; ++i) {
        os << obj.accel[i];
        if (i != 2) os << ", ";
    }
    os << ")";

    return os;
}

AirSampleProcessed::AirSampleProcessed(const AirSample &sample) {
    ang_vel[0] = (float)(sample.ang_vel[0]) * -1.0f * GYRO_SCALAR;
    ang_vel[1] = (float)(sample.ang_vel[2]) * GYRO_SCALAR;
    ang_vel[2] = (float)(sample.ang_vel[1]) * GYRO_SCALAR;
    accel[0] = (float)(sample.accel[0]) * ACCEL_SCALAR;
    accel[1] = (float)(sample.accel[2]) * ACCEL_SCALAR;
    accel[2] = (float)(sample.accel[1]) * ACCEL_SCALAR;
    tick = sample.tick;
}

std::ostream &operator<<(std::ostream &os, const AirSampleProcessed &obj) {
    os << "Time: " << obj.tick << ", ";

    os << "Angular Velocity: (";
    for (int i = 0; i < 3; ++i) {
        os << obj.ang_vel[i];
        if (i != 2) os << ", ";
    }
    os << "), ";

    os << "Acceleration: (";
    for (int i = 0; i < 3; ++i) {
        os << obj.accel[i];
        if (i != 2) os << ", ";
    }
    os << ")";

    return os;
}
