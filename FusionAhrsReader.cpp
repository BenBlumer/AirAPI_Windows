#include "FusionAhrsReader.h"
#include "AirSample.h"
#include "deps/Fusion/Fusion/Fusion.h"

FusionAhrsReader::FusionAhrsReader(const FusionMatrix gyroscopeMisalignment,
                                   const FusionVector gyroscopeSensitivity,
                                   const FusionVector gyroscopeOffset,
                                   const FusionMatrix accelerometerMisalignment,
                                   const FusionVector accelerometerSensitivity,
                                   const FusionVector accelerometerOffset,
                                   const FusionMatrix softIronMatrix,
                                   const FusionVector hardIronOffset,
                                   const FusionAhrsSettings settings)
    : gyroscopeMisalignment(gyroscopeMisalignment),
      gyroscopeSensitivity(gyroscopeSensitivity),
      gyroscopeOffset(gyroscopeOffset),
      accelerometerMisalignment(accelerometerMisalignment),
      accelerometerSensitivity(accelerometerSensitivity),
      accelerometerOffset(accelerometerOffset),
      softIronMatrix(softIronMatrix),
      hardIronOffset(hardIronOffset),
      settings(settings) {
    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);
    FusionAhrsSetSettings(&ahrs, &settings);
    previousTimestamp = 0;
}

FusionQuaternion FusionAhrsReader::get_estimate(AirSampleProcessed sample) {
    const uint64_t timestamp = sample.tick;
    FusionVector gyroscope = {sample.ang_vel[0], sample.ang_vel[1], sample.ang_vel[2]};
    FusionVector accelerometer = {sample.accel[0], sample.accel[1], sample.accel[2]};

    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

    gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    float deltaTime = (float)(timestamp - previousTimestamp) / (float)1e9;
    previousTimestamp = timestamp;

    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);
    last_estimate = FusionAhrsGetQuaternion(&ahrs);
    return last_estimate;
}

FusionEuler FusionAhrsReader::get_last_estimate_as_euler() {
    return FusionQuaternionToEuler(last_estimate);
}

FusionVector FusionAhrsReader::get_last_estimate_of_earth_vec() {
    return FusionAhrsGetEarthAcceleration(&ahrs);
}
