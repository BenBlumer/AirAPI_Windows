#ifndef FUSIONAHRSREADER_H
#define FUSIONAHRSREADER_H

#include "AirSample.h"
#include "../deps/Fusion/Fusion/Fusion.h"
#define SAMPLE_RATE (1000) // replace this with actual sample rate

class FusionAhrsReader {
public:
    FusionAhrsReader(const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
                     const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f},
                     const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f},
                     const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
                     const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f},
                     const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f},
                     const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
                     const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f},
                     const FusionAhrsSettings settings = {
                         .gain = 0.5f,
                         .accelerationRejection = 10.0f,
                         .magneticRejection = 20.0f,
                         .rejectionTimeout = 5 * SAMPLE_RATE /* 5 seconds */
                     });
    FusionQuaternion get_estimate(AirSampleProcessed sample);
    FusionEuler get_last_estimate_as_euler();
    FusionVector get_last_estimate_of_earth_vec();

private:
    const FusionMatrix gyroscopeMisalignment;
    const FusionVector gyroscopeSensitivity;
    const FusionVector gyroscopeOffset;
    const FusionMatrix accelerometerMisalignment;
    const FusionVector accelerometerSensitivity;
    const FusionVector accelerometerOffset;
    const FusionMatrix softIronMatrix;
    const FusionVector hardIronOffset;
    uint64_t previousTimestamp;
    FusionOffset offset;
    FusionAhrs ahrs;
    FusionAhrsSettings settings;
    FusionQuaternion last_estimate;
};

#endif // FUSIONAHRSREADER_H
