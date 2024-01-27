#ifndef XREALAIR_H
#define XREALAIR_H

#include "AirSample.h"
#include "../deps/hidapi/hidapi/hidapi.h"
#include <iostream>
#include <stdexcept>

class XrealAir {
public:
    XrealAir();
    ~XrealAir();
    hid_device *imu_device;
    hid_device *brightness_device;
    unsigned char read_buffer[64];
    static hid_device *open_device_by_interface(int interface_number);
    AirSample parse_report(const unsigned char *buffer_in);
    AirSample get_raw_reading();
    AirSampleProcessed get_reading();

private:
    // Private members if any
};

#endif // XREALAIR_H
