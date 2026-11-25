// Runs the gyroscope-stopped watchdog and the LSM6DSV driver on a PC, with a
// fake chip on a fake bus, so the recovery can be exercised without a tracker.
//
// It answers the question a customer's report raised: when a gyroscope stops
// while the accelerometer carries on, does the firmware notice, and does
// configuring the sensor again actually turn the gyroscope back on?
//
//   g++ -std=c++20 -I src -o /tmp/watchdog test/native/sensorwatchdog_test.cpp && /tmp/watchdog
//
// No Arduino, no PlatformIO: the two pieces under test are the real ones, and
// everything they lean on is stubbed below.

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <random>
#include <string>
#include <vector>

// ---------------------------------------------------------------- stubs ----

static uint32_t fakeDelayTotal = 0;
void delay(uint32_t ms) { fakeDelayTotal += ms; }

namespace SlimeVR::Logging {
class Logger {
public:
	explicit Logger(const char* = "") {}
	void error(const char* fmt, ...) { record("ERROR", fmt); }
	void warn(const char* fmt, ...) { record("WARN", fmt); }
	void info(const char* fmt, ...) { record("INFO", fmt); }
	void debug(const char* fmt, ...) { record("DEBUG", fmt); }
	std::vector<std::string> lines;

private:
	void record(const char* level, const char* fmt) {
		lines.push_back(std::string(level) + " " + fmt);
	}
};
}  // namespace SlimeVR::Logging

enum class ImuID : uint8_t { Unknown = 0, LSM6DSV = 13 };

// Only the fields our driver sets, in the order the real VQFParams declares
// them, because the driver uses designated initialisers.
struct VQFParams {
	bool motionBiasEstEnabled = true;
	float biasSigmaInit = 0.5f;
	float biasClip = 2.0f;
	float restThGyr = 2.0f;
	float restThAcc = 0.5f;
};

#include "sensors/softfusion/drivers/lsm6dsv.h"
#include "sensors/softfusion/sensorwatchdog.h"

using SlimeVR::Sensors::FrozenReadingDetector;
using SlimeVR::Sensors::SensorWatchdog;

// ------------------------------------------------------------ fake chip ----

/**
 * An LSM6DSV as far as the driver can tell: a register file and a FIFO.
 *
 * `gyroBlockDead` is the fault a customer saw - the gyroscope stops producing
 * readings while the accelerometer carries on, with the bus still working and
 * no register changed. Configuring the chip again begins with a software
 * reset, which is what a real chip needs to bring that block back, so the
 * fake does the same: a reset clears the fault.
 */
struct FakeLSM6DSV {
	std::map<uint8_t, uint8_t> registers;
	std::deque<std::array<uint8_t, 7>> fifo;  // tag byte + 6 data bytes
	bool gyroBlockDead = false;
	// The other shape of the fault: readings keep coming, frozen at one value.
	bool gyroStuck = false;
	int resets = 0;
	// A real gyroscope's noise: about a count at this range. Its bias moves as
	// it warms, which is slow next to the noise.
	std::mt19937 rng{12345};
	double noiseCounts = 1.1;
	double biasCounts = 25.0;

	static constexpr uint8_t WHO_AM_I = 0x0f;
	static constexpr uint8_t CTRL2_GYRO_ODR = 0x11;
	static constexpr uint8_t CTRL3 = 0x12;
	static constexpr uint8_t FIFO_CTRL3_BDR = 0x09;

	bool gyroConfigured() const {
		auto ctrl2 = registers.find(CTRL2_GYRO_ODR);
		auto bdr = registers.find(FIFO_CTRL3_BDR);
		const bool odrOn = ctrl2 != registers.end() && (ctrl2->second & 0x0f) != 0;
		const bool batched = bdr != registers.end() && (bdr->second & 0xf0) != 0;
		return odrOn && batched;
	}

	void write(uint8_t reg, uint8_t value) {
		if (reg == CTRL3 && (value & 0x01)) {  // SW_RESET
			registers.clear();
			fifo.clear();
			gyroBlockDead = false;  // a reset restarts the gyroscope block
			gyroStuck = false;
			resets++;
			return;
		}
		registers[reg] = value;
	}

	/// One poll interval's worth of readings appears in the FIFO.
	void produce(int gyroSamples, int accelSamples) {
		for (int i = 0; i < accelSamples; i++) {
			fifo.push_back({0x02 << 3, 1, 0, 2, 0, 3, 0});
		}
		if (gyroBlockDead || !gyroConfigured()) {
			return;
		}
		std::normal_distribution<double> noise(0.0, noiseCounts);
		for (int i = 0; i < gyroSamples; i++) {
			std::array<uint8_t, 7> entry{0x01 << 3, 0, 0, 0, 0, 0, 0};
			for (int axis = 0; axis < 3; axis++) {
				const int16_t value = gyroStuck
					? static_cast<int16_t>(-312 + axis)
					: static_cast<int16_t>(std::lround(biasCounts + noise(rng)));
				entry[1 + axis * 2] = static_cast<uint8_t>(value & 0xff);
				entry[2 + axis * 2] = static_cast<uint8_t>((value >> 8) & 0xff);
			}
			fifo.push_back(entry);
		}
	}
};

/// The I2C interface the driver template expects, wired to the fake chip.
struct FakeI2C {
	FakeLSM6DSV* chip;
	static constexpr size_t MaxTransactionLength = 128;

	uint8_t readReg(uint8_t reg) const {
		if (reg == FakeLSM6DSV::WHO_AM_I) return 0x70;
		auto found = chip->registers.find(reg);
		return found == chip->registers.end() ? 0 : found->second;
	}
	uint16_t readReg16(uint8_t) const {
		return static_cast<uint16_t>(chip->fifo.size());  // FIFO_STATUS: count
	}
	void writeReg(uint8_t reg, uint8_t value) { chip->write(reg, value); }
	void writeReg16(uint8_t, uint16_t) {}
	void writeBytes(uint8_t, uint16_t, uint8_t*) {}
	void readBytes(uint8_t, size_t count, uint8_t* out) {
		for (size_t i = 0; i + 7 <= count; i += 7) {
			if (chip->fifo.empty()) {
				std::memset(out + i, 0, 7);
				continue;
			}
			std::memcpy(out + i, chip->fifo.front().data(), 7);
			chip->fifo.pop_front();
		}
	}
};

// ----------------------------------------------------------------- test ----

static int failures = 0;
static void check(bool ok, const std::string& what) {
	std::printf("%s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (!ok) failures++;
}

using Driver = SlimeVR::Sensors::SoftFusion::Drivers::LSM6DSV<FakeI2C>;

/// Reads the FIFO once, the way the sensor's poll does.
static void poll(Driver& driver, int& gyroSeen, int& accelSeen) {
	driver.bulkRead(
		[&](const int16_t*, float) { accelSeen++; },
		[&](const int16_t*, float) { gyroSeen++; },
		[&](int16_t, float) {}
	);
}

int main() {
	SlimeVR::Logging::Logger logger("test");
	FakeLSM6DSV chip;
	Driver driver(FakeI2C{&chip}, logger);

	// --- the driver turns the gyroscope on when it configures the chip
	check(driver.initialize(), "the driver configures the chip");
	check(chip.gyroConfigured(), "configuring it sets a gyroscope rate and batches it");
	check(chip.resets == 1, "configuring begins with a software reset");

	int gyro = 0, accel = 0;
	chip.produce(2, 1);
	poll(driver, gyro, accel);
	check(gyro == 2 && accel == 1, "a healthy chip delivers gyroscope and accelerometer");

	// --- the fault: the gyroscope block stops, the accelerometer carries on
	chip.gyroBlockDead = true;
	gyro = accel = 0;
	for (int i = 0; i < 10; i++) {
		chip.produce(2, 1);
		poll(driver, gyro, accel);
	}
	check(gyro == 0, "with the gyroscope stopped, no gyroscope readings arrive");
	check(accel == 10, "the accelerometer keeps delivering, so the bus is fine");

	// --- the watchdog notices, once, and then asks for a retry every 5s
	SensorWatchdog watchdog;
	uint32_t now = 100000;  // well away from zero
	watchdog.start(now);

	int stopped = 0, retries = 0, recovered = 0;
	auto act = [&](SensorWatchdog::Action a) {
		if (a == SensorWatchdog::Action::Stopped) stopped++;
		if (a == SensorWatchdog::Action::Retry) retries++;
		if (a == SensorWatchdog::Action::Recovered) recovered++;
	};

	// Poll every 6 ms for 1.9 s with no readings: not yet stopped.
	for (; now < 100000 + 1900; now += 6) act(watchdog.polled(now));
	check(stopped == 0, "a gap shorter than two seconds is not a fault");

	// Carry on to 20 s.
	for (; now < 100000 + 20000; now += 6) act(watchdog.polled(now));
	check(stopped == 1, "it is reported once, not on every poll");
	check(retries == 3, "after that it retries every five seconds");

	// --- a reading comes back
	act(watchdog.reading(now));
	check(recovered == 1, "a reading clears the fault");
	check(!watchdog.stopped(), "and it is no longer stopped");
	now += 6;
	act(watchdog.reading(now));
	check(recovered == 1, "recovery is announced once, not on every reading");

	// --- and the real recovery: configure the chip again
	const int resetsBefore = chip.resets;
	check(driver.initialize(), "the recovery configures the chip again");
	check(chip.resets == resetsBefore + 1, "which resets it");
	check(!chip.gyroBlockDead, "the reset restarts the gyroscope block");
	check(chip.gyroConfigured(), "and the gyroscope is configured and batched again");

	gyro = accel = 0;
	chip.produce(2, 1);
	poll(driver, gyro, accel);
	check(gyro == 2 && accel == 1, "the gyroscope delivers again after recovery");

	// --- a sensor that never comes back keeps being reported as stopped
	SensorWatchdog dead;
	dead.start(0);
	int deadStopped = 0, deadRetries = 0;
	for (uint32_t t = 0; t < 60000; t += 6) {
		auto a = dead.polled(t);
		if (a == SensorWatchdog::Action::Stopped) deadStopped++;
		if (a == SensorWatchdog::Action::Retry) deadRetries++;
	}
	check(deadStopped == 1 && deadRetries == 11, "a sensor that stays dead is retried, not spammed");
	check(dead.stopped(), "and stays marked stopped");

	// --- the millisecond clock wraps after 49 days; that is not a fault
	SensorWatchdog wrapping;
	const uint32_t nearWrap = 0xFFFFFF00;
	wrapping.start(nearWrap);
	int wrapStopped = 0;
	for (uint32_t step = 0; step < 200; step++) {
		const uint32_t t = nearWrap + step * 6;  // wraps through zero
		if (wrapping.reading(t) == SensorWatchdog::Action::Recovered) wrapStopped++;
		if (wrapping.polled(t) == SensorWatchdog::Action::Stopped) wrapStopped++;
	}
	check(wrapStopped == 0, "a clock that wraps does not look like a stopped sensor");

	// --- the sensor's loop, end to end, with the chip, the frozen-reading
	// check and the watchdog together, the way motionLoop wires them.
	struct Run {
		int stops = 0, retries = 0, recoveries = 0, inits = 0, gyroSamples = 0;
	};
	auto runLoop = [&](FakeLSM6DSV& c, Driver& d, uint32_t from, uint32_t until,
					   SensorWatchdog& w, FrozenReadingDetector<int16_t>& f) {
		Run r;
		// 6 ms poll: about 3 gyroscope and 1.4 accelerometer readings each.
		int tick = 0;
		for (uint32_t t = from; t < until; t += 6, tick++) {
			c.produce(3, (tick % 5 < 2) ? 2 : 1);
			bool updated = false;
			d.bulkRead(
				[&](const int16_t*, float) {},
				[&](const int16_t* xyz, float) {
					f.sample(xyz, t);
					updated = true;
					r.gyroSamples++;
				},
				[&](int16_t, float) {}
			);
			SensorWatchdog::Action a = SensorWatchdog::Action::Nothing;
			if (!updated || f.frozen(t)) {
				a = w.polled(t);
			} else {
				a = w.reading(t);
			}
			if (a == SensorWatchdog::Action::Stopped) r.stops++;
			if (a == SensorWatchdog::Action::Retry) r.retries++;
			if (a == SensorWatchdog::Action::Recovered) r.recoveries++;
			if (a == SensorWatchdog::Action::Stopped || a == SensorWatchdog::Action::Retry) {
				d.initialize();
				r.inits++;
			}
		}
		return r;
	};

	{
		// Ten minutes of a healthy gyroscope, warming, never trips anything.
		FakeLSM6DSV c;
		Driver d(FakeI2C{&c}, logger);
		d.initialize();
		SensorWatchdog w;
		FrozenReadingDetector<int16_t> f;
		w.start(1000);
		f.start(1000);
		Run r;
		for (int minute = 0; minute < 10; minute++) {
			c.biasCounts = 25.0 + minute * 2.0;  // the bias creeping as it warms
			Run part = runLoop(c, d, 1000 + minute * 60000, 1000 + (minute + 1) * 60000, w, f);
			r.stops += part.stops;
			r.gyroSamples += part.gyroSamples;
		}
		check(r.gyroSamples > 280000, "ten minutes of readings went through the loop");
		check(r.stops == 0, "a healthy gyroscope is never taken for a frozen one");

		// Quieter than any real part, to leave a margin: still never trips.
		c.noiseCounts = 0.5;
		Run quiet = runLoop(c, d, 700000, 760000, w, f);
		check(quiet.stops == 0, "even half the datasheet noise is never taken for frozen");
	}

	{
		// The gyroscope freezes on a non-zero value, as a stuck one would.
		FakeLSM6DSV c;
		Driver d(FakeI2C{&c}, logger);
		d.initialize();
		SensorWatchdog w;
		FrozenReadingDetector<int16_t> f;
		w.start(1000);
		f.start(1000);
		Run before = runLoop(c, d, 1000, 61000, w, f);
		check(before.stops == 0, "fine for the first minute");

		c.gyroStuck = true;
		const int resetsBefore = c.resets;
		Run stuck = runLoop(c, d, 61000, 61000 + 2900, w, f);
		check(stuck.stops == 0, "frozen for under three seconds is not yet a fault");
		Run caught = runLoop(c, d, 63900, 70000, w, f);
		check(caught.stops == 1, "frozen readings are caught after three seconds");
		check(c.resets > resetsBefore, "and the chip is reset and configured again");
		check(caught.recoveries == 1, "after which the readings move and it counts as recovered");
		check(!w.stopped() && !f.frozen(70000), "and it is back to normal");
		Run after = runLoop(c, d, 70000, 130000, w, f);
		check(after.stops == 0 && after.gyroSamples > 25000, "and keeps going afterwards");
	}

	{
		// Frozen at exactly zero is caught the same way.
		FrozenReadingDetector<int16_t> f;
		f.start(0);
		const int16_t zero[3]{0, 0, 0};
		for (uint32_t t = 0; t < 3000; t += 2) f.sample(zero, t);
		check(f.frozen(3000), "readings frozen at zero count as frozen");
		const int16_t moved[3]{0, 1, 0};
		f.sample(moved, 3002);
		check(!f.frozen(3002), "one reading that moves clears it");
	}

	std::printf("\n%s\n", failures == 0 ? "all checks passed" : "SOME CHECKS FAILED");
	return failures == 0 ? 0 : 1;
}
