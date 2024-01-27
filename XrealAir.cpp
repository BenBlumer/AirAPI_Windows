#include "XrealAir.h"
#include "AirSample.h"
#include "deps/hidapi/hidapi/hidapi.h"
#include <iostream>
#include <stdexcept>

#define AIR_VID 0x3318
#define AIR_PID 0x0424
#define AIR_2_PID 0x0428
#define AIR_2_PRO_PID 0x0432

// class XrealAir
// {
// public:
	XrealAir::XrealAir()
	{
		imu_device = open_device_by_interface(3);
		brightness_device = open_device_by_interface(4);
	}
	XrealAir::~XrealAir()
	{
		hid_close(imu_device);
		hid_close(brightness_device);
	}
	hid_device *imu_device;
	hid_device *brightness_device;
	unsigned char read_buffer[64];

	hid_device *XrealAir::open_device_by_interface(int interface_number)
	{
		struct hid_device_info *devs = NULL;
		struct hid_device_info *devs_1 = hid_enumerate(AIR_VID, AIR_PID);
		struct hid_device_info *devs_2 = hid_enumerate(AIR_VID, AIR_2_PID);
		struct hid_device_info *devs_2_pro = hid_enumerate(AIR_VID, AIR_2_PRO_PID);

		struct hid_device_info *cur_dev = NULL;

		if (devs_1 != NULL)
		{
			cur_dev = devs_1;
			devs = devs_1;
		}
		else if (devs_2 != NULL)
		{
			cur_dev = devs_2;
			devs = devs_2;
		}
		else if (devs_2_pro != NULL)
		{
			cur_dev = devs_2_pro;
			devs = devs_2_pro;
		}

		hid_device *device = NULL;

		while (devs)
		{
			if (cur_dev->interface_number == interface_number)
			{
				device = hid_open_path(cur_dev->path);
				std::cout << "Interface " << interface_number << " bound" << std::endl;
				break;
			}

			cur_dev = cur_dev->next;
		}

		hid_free_enumeration(devs);

		if (interface_number == 3)
		{
			uint8_t magic_payload[] = {0x00, 0xaa, 0xc5, 0xd1, 0x21, 0x42, 0x04, 0x00, 0x19, 0x01};
			int res = hid_write(device, magic_payload, sizeof(magic_payload));
			if (res < 0)
			{
				std::cout << "Unable to write to device" << std::endl;
				throw std::runtime_error("Unable to write to device");
			}
		}
		return device;
	};

AirSample XrealAir::parse_report(const unsigned char *buffer_in)
{
	AirSample out_sample;

	// clock in nanoseconds
	buffer_in += 4;
	out_sample.tick = ((uint64_t) * (buffer_in++));
	out_sample.tick = out_sample.tick | (((uint64_t) * (buffer_in++)) << 8);
	out_sample.tick = out_sample.tick | (((uint64_t) * (buffer_in++)) << 16);
	out_sample.tick = out_sample.tick | (((uint64_t) * (buffer_in++)) << 24);
	out_sample.tick = out_sample.tick | (((uint64_t) * (buffer_in++)) << 32);
	out_sample.tick = out_sample.tick | (((uint64_t) * (buffer_in++)) << 40);
	out_sample.tick = out_sample.tick | (((uint64_t) * (buffer_in++)) << 48);
	out_sample.tick = out_sample.tick | (((uint64_t) * (buffer_in++)) << 56);

	uint32_t t0v, t1v, t2v, t3v, t0a, t1a, t2a, t3a;
	// gyroscope measurements
	buffer_in += 6;
	if (*(buffer_in + 2) & 0x80)
	{
		t0v = (0xff << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample.ang_vel[0] = t0v | t1v | t2v | t3v;
		// out_sample.ang_vel[0] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else
	{
		t0v = (0x00 << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample.ang_vel[0] = t0v | t1v | t2v | t3v;
		// out_sample.ang_vel[0] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	if (*(buffer_in + 2) & 0x80)
	{
		t0v = (0xff << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample.ang_vel[1] = t0v | t1v | t2v | t3v;
		// out_sample.ang_vel[1] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else
	{
		t0v = (0x00 << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample.ang_vel[1] = t0v | t1v | t2v | t3v;
		// out_sample.ang_vel[1] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	if (*(buffer_in + 2) & 0x80)
	{
		t0v = (0xff << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample.ang_vel[2] = t0v | t1v | t2v | t3v;
		// out_sample.ang_vel[2] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else
	{
		t0v = (0x00 << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample.ang_vel[2] = t0v | t1v | t2v | t3v;
		// out_sample.ang_vel[2] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	// accelerometer data
	buffer_in += 6;
	if (*(buffer_in + 2) & 0x80)
	{
		t0a = (0xff << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample.accel[0] = t0a | t1a | t2a | t3a;
		// out_sample.accel[0] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else
	{
		t0a = (0x00 << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample.accel[0] = t0a | t1a | t2a | t3a;
		// out_sample.accel[0] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	if (*(buffer_in + 2) & 0x80)
	{
		t0a = (0xff << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample.accel[1] = t0a | t1a | t2a | t3a;
		// out_sample.accel[1] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else
	{
		t0a = (0x00 << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample.accel[1] = t0a | t1a | t2a | t3a;
		// out_sample.accel[1] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	if (*(buffer_in + 2) & 0x80)
	{
		t0a = (0xff << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample.accel[2] = t0a | t1a | t2a | t3a;
		// out_sample.accel[2] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else
	{
		t0a = (0x00 << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample.accel[2] = t0a | t1a | t2a | t3a;
		// out_sample.accel[2] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	return out_sample;
};


	AirSample XrealAir::get_raw_reading()
	{
		AirSample sample = {};

		int res = hid_read(this->imu_device, this->read_buffer, sizeof(this->read_buffer));
		if (res < 0)
		{

			throw std::runtime_error("Unable to get a reading from device");
		}
		sample = parse_report(this->read_buffer);
		return sample;
	};

	AirSampleProcessed XrealAir::get_reading()
	{
		AirSample sample = get_raw_reading();
		return AirSampleProcessed(sample);
	};

