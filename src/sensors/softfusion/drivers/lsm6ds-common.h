/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Gorbit99 & SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

#include "perfstats.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

template <typename I2CImpl>
struct LSM6DSOutputHandler {
	LSM6DSOutputHandler(I2CImpl i2c, SlimeVR::Logging::Logger& logger)
		: i2c(i2c)
		, logger(logger) {}

	I2CImpl i2c;
	SlimeVR::Logging::Logger& logger;

#pragma pack(push, 1)
	struct FifoEntryAligned {
		union {
			int16_t xyz[3];
			uint8_t raw[6];
		};
	};
#pragma pack(pop)

	static constexpr size_t FullFifoEntrySize = sizeof(FifoEntryAligned) + 1;

	template <
		typename AccelCall,
		typename GyroCall,
		typename TemperatureCall,
		typename Regs>
	void bulkRead(
		AccelCall& processAccelSample,
		GyroCall& processGyroSample,
		TemperatureCall& processTemperatureSample,
		float GyrTs,
		float AccTs,
		float TempTs
	) {
		constexpr auto FIFO_SAMPLES_MASK = 0x3ff;
		constexpr auto FIFO_OVERRUN_LATCHED_MASK = 0x800;

		// A bus error leaves the buffers holding whatever was in them before.
		// Before this the reads were not checked, so a failed read was parsed
		// as FIFO entries anyway and fed leftover memory to the fusion as gyro
		// and accelerometer samples. Now a failed read is dropped and the poll
		// ends; the readings are still in the FIFO for the next one.
		uint16_t fifo_status = 0;
		if (!i2c.readBytes(
				Regs::FifoStatus,
				sizeof(fifo_status),
				reinterpret_cast<uint8_t*>(&fifo_status)
			)) {
			PerfStats::i2cReadFailures++;
			return;
		}
		if (fifo_status & FIFO_OVERRUN_LATCHED_MASK) {
			PerfStats::fifoOverruns++;
			// FIFO overrun is expected to happen during startup and calibration.
			// Said once and then every hundredth time: printing it on every poll
			// slowed the loop down and made the next overrun more likely.
			if (overrunCount++ % 100 == 0) {
				logger.error(
					"FIFO OVERRUN! This occuring during normal usage is an issue. "
					"(%u so far)",
					static_cast<unsigned>(overrunCount)
				);
			}
		}

		// One read takes at most 8 readings, and a poll normally finds about
		// five. When the loop has been held up - a bus error can hold it for
		// 150 ms - one read per poll barely keeps pace and the FIFO overruns,
		// so read until it is empty, up to a few reads per poll.
		size_t available_entries = fifo_status & FIFO_SAMPLES_MASK;
		std::array<uint8_t, FullFifoEntrySize * 8> read_buffer;
		for (size_t chunk = 0; chunk < MaxReadsPerPoll && available_entries > 0;
			 chunk++) {
			const size_t entries = std::min(
				available_entries,
				read_buffer.size() / FullFifoEntrySize
			);
			const size_t bytes_to_read = entries * FullFifoEntrySize;
			if (!i2c.readBytes(Regs::FifoData, bytes_to_read, read_buffer.data())) {
				PerfStats::i2cReadFailures++;
				return;
			}
			available_entries -= entries;
			parseFifo(
				read_buffer.data(),
				bytes_to_read,
				processAccelSample,
				processGyroSample,
				processTemperatureSample,
				GyrTs,
				AccTs,
				TempTs
			);
		}
	}

	static constexpr size_t MaxReadsPerPoll = 4;
	uint32_t overrunCount = 0;

	template <typename AccelCall, typename GyroCall, typename TemperatureCall>
	static void parseFifo(
		const uint8_t* read_buffer,
		size_t bytes_to_read,
		AccelCall& processAccelSample,
		GyroCall& processGyroSample,
		TemperatureCall& processTemperatureSample,
		float GyrTs,
		float AccTs,
		float TempTs
	) {
		for (auto i = 0u; i < bytes_to_read; i += FullFifoEntrySize) {
			FifoEntryAligned entry;
			uint8_t tag = read_buffer[i] >> 3;
			memcpy(
				entry.raw,
				&read_buffer[i + 0x1],
				sizeof(FifoEntryAligned)
			);  // skip fifo header

			switch (tag) {
				case 0x01:  // Gyro NC
					processGyroSample(entry.xyz, GyrTs);
					break;
				case 0x02:  // Accel NC
					processAccelSample(entry.xyz, AccTs);
					break;
				case 0x03:  // Temperature
					processTemperatureSample(entry.xyz[0], TempTs);
					break;
			}
		}
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
