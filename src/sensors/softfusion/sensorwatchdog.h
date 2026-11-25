/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 PicoVR Contributors

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

#include <cstdint>

namespace SlimeVR::Sensors {

/**
 * Decides when an IMU has stopped producing readings, and when to try it
 * again.
 *
 * Kept apart from the sensor itself because then it can be run without an
 * IMU, a bus, or an Arduino: all it does is compare a clock against the last
 * time a reading arrived. test/native/sensorwatchdog_test.cpp drives it
 * through a whole outage and back.
 *
 * All the arithmetic is written as `now - then`, never `then + span > now`,
 * so it stays correct when the millisecond clock wraps.
 */
class SensorWatchdog {
public:
	enum class Action {
		/// Carry on.
		Nothing,
		/// It has just stopped: say so, and configure the sensor again.
		Stopped,
		/// Still stopped: configure it again.
		Retry,
		/// Readings are back.
		Recovered,
	};

	/// Longer than this without a reading and the sensor counts as stopped.
	/// Readings arrive in the hundreds per second, so this is far beyond any
	/// ordinary gap.
	static constexpr uint32_t stoppedAfterMillis = 2000;

	/// How often to configure a stopped sensor again.
	static constexpr uint32_t retryEveryMillis = 5000;

	/// Starts the clock, once the sensor has been set up.
	void start(uint32_t now) {
		m_lastReading = now;
		m_lastRetry = now;
		m_stopped = false;
		m_running = true;
	}

	/// Call when the sensor has handed over a reading.
	Action reading(uint32_t now) {
		if (!m_running) {
			return Action::Nothing;
		}
		m_lastReading = now;
		if (!m_stopped) {
			return Action::Nothing;
		}
		m_stopped = false;
		return Action::Recovered;
	}

	/// Call when a poll found no reading.
	Action polled(uint32_t now) {
		if (!m_running || now - m_lastReading < stoppedAfterMillis) {
			return Action::Nothing;
		}

		if (!m_stopped) {
			m_stopped = true;
			m_lastRetry = now;
			return Action::Stopped;
		}

		if (now - m_lastRetry < retryEveryMillis) {
			return Action::Nothing;
		}
		m_lastRetry = now;
		return Action::Retry;
	}

	bool stopped() const { return m_stopped; }

	/// How long since the last reading, for the message that says so.
	uint32_t sinceReading(uint32_t now) const { return now - m_lastReading; }

private:
	uint32_t m_lastReading = 0;
	uint32_t m_lastRetry = 0;
	bool m_stopped = false;
	bool m_running = false;
};

/**
 * Notices a gyroscope whose readings have frozen.
 *
 * A working gyroscope never hands over the same reading on all three axes for
 * long: its own noise is worth about a count, so some axis always flickers,
 * even on a desk. Readings frozen at one value - zero or anything else - are
 * dead readings rather than none. They still update the fusion, which quietly
 * learns the frozen value as bias, so the rotation stops while everything
 * looks healthy and SensorWatchdog is perfectly satisfied.
 */
template <typename T>
class FrozenReadingDetector {
public:
	/// At 480 samples a second, over a thousand identical readings in a row.
	static constexpr uint32_t frozenAfterMillis = 3000;

	void start(uint32_t now) { m_lastChange = now; }

	void sample(const T xyz[3], uint32_t now) {
		if (xyz[0] == m_last[0] && xyz[1] == m_last[1] && xyz[2] == m_last[2]) {
			return;
		}
		m_last[0] = xyz[0];
		m_last[1] = xyz[1];
		m_last[2] = xyz[2];
		m_lastChange = now;
	}

	bool frozen(uint32_t now) const { return now - m_lastChange >= frozenAfterMillis; }

private:
	T m_last[3]{0, 0, 0};
	uint32_t m_lastChange = 0;
};

}  // namespace SlimeVR::Sensors
