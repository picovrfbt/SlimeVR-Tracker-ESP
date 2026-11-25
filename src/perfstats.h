#pragma once

#include <cstddef>
#include <cstdint>

/**
 * Where the tracker spends its time, for "GET PERF" in the serial console.
 *
 * The main loop has to come round every few milliseconds: the IMU's FIFO
 * fills at about 780 readings a second and overruns if it is not read, and
 * every stall shows up as a gap in rotation. Anything that holds the loop up
 * is a slowdown, and these numbers say which part of the loop it was, on a
 * real tracker, instead of guessing from the code.
 *
 * Header-only and Arduino-free, so the sensor drivers can count into it and
 * still be built on a PC for the native tests.
 */
namespace PerfStats {

enum class Section : uint8_t { Console, Network, Sensors, Battery, Leds, Count };

struct SectionStats {
	uint64_t totalMicros = 0;
	uint32_t maxMicros = 0;
};

inline SectionStats sections[static_cast<size_t>(Section::Count)];

inline uint32_t loops = 0;
inline uint32_t maxLoopMicros = 0;
/// Longest time from one loop starting to the next: includes anything that
/// runs between loops, such as the Wi-Fi stack.
inline uint32_t maxGapMicros = 0;

/// Time spent in the fusion (VQF) itself, out of the Sensors section.
inline uint64_t fusionMicros = 0;
inline uint32_t gyroSamples = 0;
inline uint32_t accelSamples = 0;
inline uint32_t rotationPackets = 0;

inline uint32_t i2cReadFailures = 0;
inline uint32_t fifoOverruns = 0;

inline void add(Section section, uint32_t micros) {
	auto& stats = sections[static_cast<size_t>(section)];
	stats.totalMicros += micros;
	if (micros > stats.maxMicros) {
		stats.maxMicros = micros;
	}
}

inline void reset() {
	for (auto& stats : sections) {
		stats = SectionStats{};
	}
	loops = 0;
	maxLoopMicros = 0;
	maxGapMicros = 0;
	fusionMicros = 0;
	gyroSamples = 0;
	accelSamples = 0;
	rotationPackets = 0;
	i2cReadFailures = 0;
	fifoOverruns = 0;
}

inline const char* name(Section section) {
	switch (section) {
		case Section::Console:
			return "console";
		case Section::Network:
			return "network";
		case Section::Sensors:
			return "sensors";
		case Section::Battery:
			return "battery";
		case Section::Leds:
			return "leds";
		default:
			return "?";
	}
}

}  // namespace PerfStats
