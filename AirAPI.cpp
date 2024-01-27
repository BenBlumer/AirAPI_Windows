// #include "pch.h"
// #include "AirAPI.h"
#include "deps/hidapi/hidapi/hidapi.h"
#include "deps/Fusion/Fusion/Fusion.h"
#include <iostream>
#include <mutex>
#include <array>
#include <cstdint>
#include <vector>

#include "XrealAir.h"
#include "FusionAhrsReader.h"
#include <iostream>

// Air USB VID and PID


// ticks are in nanoseconds, 1000 Hz packets
#define TICK_LEN (1.0f / 1E9f)




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

int main() {
    try {
        XrealAir xreals = XrealAir();
        FusionAhrsReader reader;

        while (true) {
            auto processed_sample = xreals.get_reading();
            auto quat = reader.get_estimate(processed_sample);

            FusionEuler euler = reader.get_last_estimate_as_euler();
            FusionVector earth = reader.get_last_estimate_of_earth_vec();

            std::cout << "Roll: " << euler.array[0] << ", ";
            std::cout << "Pitch: " << euler.array[1] << ", ";
            std::cout << "Yaw: " << euler.array[2] << std::endl;

            std::cout << "Earth Vector: (";
            for (int i = 0; i < 3; ++i) {
                std::cout << earth.array[i];
                if (i != 2) std::cout << ", ";
            }
            std::cout << ")" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

// int main()
// {
// 	XrealAir xreals = XrealAir();

// 	// unsigned char buffer[64] = {};
// 	FusionAhrsReader reader;
// 	while (1)
// 	{
// 		auto processed_sample = xreals.get_reading();

// 		// parse_report(buffer, sizeof(buffer), &sample);
// 		// auto processed_sample = AirSampleProcessed(sample);
// 		auto quat = reader.get_estimate(processed_sample);
// 		static FusionEuler euler;
// 		static FusionVector earth;
// 		euler = reader.get_last_estimate_as_euler();
// 		std::cout << "Roll: " << euler.array[0] << std::endl;
// 		std::cout << "Pitch: " << euler.array[1] << std::endl;
// 		std::cout << "Yaw: " << euler.array[2] << std::endl;
// 	}

// 	return 0;
// }