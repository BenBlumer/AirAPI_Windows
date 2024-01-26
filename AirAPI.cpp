// #include "pch.h"
#include "AirAPI.h"
#include "deps/hidapi/hidapi/hidapi.h"
#include "deps/Fusion/Fusion/Fusion.h"
#include <iostream>
#include <mutex>
#include <array>
#include <cstdint>
#include <vector>
// Air USB VID and PID
#define AIR_VID 0x3318
#define AIR_PID 0x0424
#define AIR_2_PID 0x0428
#define AIR_2_PRO_PID 0x0432

// ticks are in nanoseconds, 1000 Hz packets
#define TICK_LEN (1.0f / 1E9f)

// based on 24bit signed int w/ FSR = +/-2000 dps, datasheet option
#define GYRO_SCALAR (1.0f / 8388608.0f * 2000.0f)

// based on 24bit signed int w/ FSR = +/-16 g, datasheet option
#define ACCEL_SCALAR (1.0f / 8388608.0f * 16.0f)

#define SAMPLE_RATE (1000) // replace this with actual sample rate

class AirSample
{
public:
	uint64_t tick;
	int32_t ang_vel[3];
	int32_t accel[3];
	friend std::ostream &operator<<(std::ostream &os, const AirSample &obj);
};

std::ostream &operator<<(std::ostream &os, const AirSample &obj)
{
	os << "Time: " << obj.tick << ", ";

	os << "Angular Velocity: (";
	for (int i = 0; i < 3; ++i)
	{
		os << obj.ang_vel[i];
		if (i != 2)
			os << ", ";
	}
	os << "), ";

	os << "Acceleration: (";
	for (int i = 0; i < 3; ++i)
	{
		os << obj.accel[i];
		if (i != 2)
			os << ", ";
	}
	os << ")";

	return os;
}

class AirSampleProcessed
{
public:
	AirSampleProcessed(const AirSample &sample)
	{
		// these scale and bias corrections are all rough guesses
		ang_vel[0] = (float)(sample.ang_vel[0]) * -1.0f * GYRO_SCALAR;
		ang_vel[1] = (float)(sample.ang_vel[2]) * GYRO_SCALAR;
		ang_vel[2] = (float)(sample.ang_vel[1]) * GYRO_SCALAR;
		accel[0] = (float)(sample.accel[0]) * ACCEL_SCALAR;
		accel[1] = (float)(sample.accel[2]) * ACCEL_SCALAR;
		accel[2] = (float)(sample.accel[1]) * ACCEL_SCALAR;
		tick = sample.tick;
	}

	float ang_vel[3];
	float accel[3];
	uint64_t tick;
	friend std::ostream &operator<<(std::ostream &os, const AirSampleProcessed &obj);
};
std::ostream &operator<<(std::ostream &os, const AirSampleProcessed &obj)
{
	// Print "Time" and the tick value
	os << "Time: " << obj.tick << ", ";

	// Print angular velocity
	os << "Angular Velocity: (";
	for (int i = 0; i < 3; ++i)
	{
		os << obj.ang_vel[i];
		if (i != 2)
			os << ", ";
	}
	os << "), ";

	// Print acceleration
	os << "Acceleration: (";
	for (int i = 0; i < 3; ++i)
	{
		os << obj.accel[i];
		if (i != 2)
			os << ", ";
	}
	os << ")";

	return os;
}

class XrealAir
{
public:
	XrealAir()
	{
		imu_device = open_device_by_interface(3);
		brightness_device = open_device_by_interface(4);
	}
	~XrealAir()
	{
		hid_close(imu_device);
		hid_close(brightness_device);
	}
	hid_device *imu_device;
	hid_device *brightness_device;
	unsigned char read_buffer[64];

	static hid_device *open_device_by_interface(int interface_number)
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

AirSample parse_report(const unsigned char *buffer_in)
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


	AirSample get_raw_reading()
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

	AirSampleProcessed get_reading()
	{
		AirSample sample = get_raw_reading();
		return AirSampleProcessed(sample);
	};
};

// int brightness = 0;
// DWORD WINAPI interface4Handler(LPVOID lpParam)
// {
// 	// get initial brightness from device
// 	std::array<uint8_t, 17> initBrightness = {0x00, 0xfd, 0x1e, 0xb9, 0xf0, 0x68, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
// 	hid_write(device4, initBrightness.data(), initBrightness.size());
// 	while (g_isListening)
// 	{
// 		std::array<uint8_t, 65> recv = {};
// 		int res = hid_read(device4, recv.data(), recv.size());
// 		if (res > 0)
// 		{
// 			switch (recv[22])
// 			{
// 			case 0x03: // Brightness down press
// 				it4.lock();
// 				brightness = recv[30];
// 				it4.unlock();
// 				break;
// 			case 0x02: // Brightness up press
// 				it4.lock();
// 				brightness = recv[30];
// 				it4.unlock();
// 				break;
// 			default:
// 				// std::cout << "Unknown Packet! " << (int)recv[22] << std::endl;
// 				break;
// 			}
// 			switch (recv[15])
// 			{
// 			case 0x03: // Brightness from cmd
// 				it4.lock();
// 				brightness = recv[23];
// 				it4.unlock();
// 				break;
// 			default:
// 				// todo
// 				break;
// 			}
// 		}
// 	}
// 	return 0;
// }
// int StopConnection()
// {
// 	if (g_isTracking)
// 	{
// 		g_isTracking = false;
// 		g_isListening = false;
// 		// Wait for the track thread to finish
// 		WaitForSingleObject(trackThread, INFINITE);
// 		TerminateThread(trackThread, 0);
// 		CloseHandle(trackThread);
// 		// Wait for the listen thread to finish
// 		WaitForSingleObject(listenThread, INFINITE);
// 		TerminateThread(listenThread, 0);
// 		CloseHandle(listenThread);
// 		return 1;
// 	}
// 	else
// 	{
// 		return -1;
// 	}
// }
// float *q = new float[4];
// float *GetQuaternion()
// {
// 	mtx.lock();
// 	q[0] = qt.array[0];
// 	q[1] = qt.array[1];
// 	q[2] = qt.array[2];
// 	q[3] = qt.array[3];
// 	mtx.unlock();
// 	return q;
// }
// float *e = new float[3];
// float *GetEuler()
// {
// 	mtx.lock();
// 	e[0] = euler.angle.pitch;
// 	e[1] = euler.angle.roll;
// 	e[2] = euler.angle.yaw;
// 	mtx.unlock();
// 	return e;
// }
// int GetBrightness()
// {
// 	int curBrightness;
// 	it4.lock();
// 	curBrightness = brightness;
// 	it4.unlock();
// 	return curBrightness;
// }

void print_three_vector(float *vec)
{
	std::cout << "Angular Velocity: (";
	for (int i = 0; i < 3; ++i)
	{
		std::cout << vec[i];
		if (i != 2)
			std::cout << ", ";
	}
	std::cout << ")" << std::endl;
};

class FusionAhrsReader
{
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
						 .rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
					 }) : gyroscopeMisalignment(gyroscopeMisalignment), gyroscopeSensitivity(gyroscopeSensitivity), gyroscopeOffset(gyroscopeOffset), accelerometerMisalignment(accelerometerMisalignment), accelerometerSensitivity(accelerometerSensitivity), accelerometerOffset(accelerometerOffset), softIronMatrix(softIronMatrix), hardIronOffset(hardIronOffset), settings(settings)
	{

		FusionOffsetInitialise(&offset, SAMPLE_RATE);
		FusionAhrsInitialise(&ahrs);
		FusionAhrsSetSettings(&ahrs, &settings);
	}
	FusionQuaternion get_estimate(AirSampleProcessed sample)
	{

		const uint64_t timestamp = sample.tick;												// replace this with actual gyroscope timestamp
		FusionVector gyroscope = {sample.ang_vel[0], sample.ang_vel[1], sample.ang_vel[2]}; // replace this with actual gyroscope data in degrees/s
		FusionVector accelerometer = {sample.accel[0], sample.accel[1], sample.accel[2]};	// replace this with actual accelerometer data in g

		gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
		accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

		// Update gyroscope offset correction algorithm
		gyroscope = FusionOffsetUpdate(&offset, gyroscope);

		// Calculate delta time (in seconds) to account for gyroscope sample clock error
		static uint64_t previousTimestamp;
		const float deltaTime = (float)(timestamp - previousTimestamp) / (float)1e9;
		previousTimestamp = timestamp;

		// // Update gyroscope AHRS algorithm
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);
		last_estimate = FusionAhrsGetQuaternion(&ahrs);
		return last_estimate;
	}
	FusionEuler get_last_estimate_as_euler()
	{
		return FusionQuaternionToEuler(last_estimate);
	}

	FusionVector get_last_estimate_of_earth_vec()
	{
		return FusionAhrsGetEarthAcceleration(&ahrs);
	}

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
int main()
{
	XrealAir xreals = XrealAir();

	// unsigned char buffer[64] = {};
	FusionAhrsReader reader;
	while (1)
	{
		auto processed_sample = xreals.get_reading();

		// parse_report(buffer, sizeof(buffer), &sample);
		// auto processed_sample = AirSampleProcessed(sample);
		auto quat = reader.get_estimate(processed_sample);
		static FusionEuler euler;
		static FusionVector earth;
		euler = reader.get_last_estimate_as_euler();
		std::cout << "Roll: " << euler.array[0] << std::endl;
		std::cout << "Pitch: " << euler.array[1] << std::endl;
		std::cout << "Yaw: " << euler.array[2] << std::endl;
	}

	return 0;
}