#ifndef SLIMEVR_OTA_H_
#define SLIMEVR_OTA_H_

#include <Arduino.h>

// Receiving a firmware image over whichever link the tracker is on, Wi-Fi or
// the PicoVR Dongle. The server sends OTA_BEGIN, then chunks, then OTA_END;
// the image goes into the app slot that is not running, and the bootloader
// rolls back by itself if the new firmware never confirms it works.
namespace PicoOta {

enum class State : uint8_t {
	Idle = 0,
	Receiving = 1,
	Ready = 2,
	Failed = 3,
};

enum class Error : uint8_t {
	None = 0,
	TooShort = 1,
	NoRoom = 2,
	WriteFailed = 3,
	WrongSize = 4,
	VerifyFailed = 5,
};

/** Starts an update. body = size (4) + sha256 (32). */
bool begin(const uint8_t* body, size_t length);

/** One chunk. body = offset (4) + bytes. Out of order chunks are ignored. */
bool chunk(const uint8_t* body, size_t length);

/** Finishes, checks the image and arms the slot. True means "restart now". */
bool end();

void abort();

State state();

Error error();

/** Bytes written, which is also the offset the tracker wants next. */
uint32_t received();

/** Total size the server said it would send. */
uint32_t expected();

/**
 * Called once this firmware has talked to the server, so the bootloader stops
 * treating it as on trial. Doing nothing here means the next restart goes back
 * to the firmware that was working before.
 */
void confirm();

/** True while an update is being received: the sensors should stay quiet. */
inline bool busy() { return state() == State::Receiving; }

}  // namespace PicoOta

#endif  // SLIMEVR_OTA_H_
