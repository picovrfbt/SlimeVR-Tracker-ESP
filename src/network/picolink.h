/*
	PicoVR link: the ESP-NOW frames shared by trackers and the PicoVR Dongle.

	Trackers in dongle mode send their normal server protocol packets inside
	FRAME_DATA_UP frames. The dongle passes them to the PC unchanged, tagged with
	the tracker's address, so the server sees the same tracker it would see over
	Wi-Fi. Server replies come back as FRAME_DATA_DOWN. The dongle broadcasts a
	FRAME_BEACON about ten times a second so trackers can find its channel.

	Every frame carries the dongle's network id, derived from the pairing key, so
	trackers and dongles that belong to someone else are ignored. The network id
	is not a secret.
*/

#ifndef PICOVR_LINK_H_
#define PICOVR_LINK_H_

#include <stddef.h>
#include <stdint.h>

namespace PicoLink {

constexpr uint8_t LINK_VERSION = 1;

constexpr uint8_t FRAME_DATA_UP = 0x01;  // tracker -> dongle
constexpr uint8_t FRAME_DATA_DOWN = 0x02;  // dongle -> tracker
constexpr uint8_t FRAME_BEACON = 0x03;  // dongle -> everyone

constexpr size_t MAX_FRAME = 250;  // ESP-NOW v1 payload limit
constexpr size_t KEY_SIZE = 16;
constexpr uint8_t MAX_TRACKERS = 8;
constexpr uint8_t MIN_CHANNEL = 1;
constexpr uint8_t MAX_CHANNEL = 13;

// Milliseconds between beacons, and how long without one before a tracker
// starts looking for the dongle on other channels.
constexpr uint32_t BEACON_INTERVAL_MS = 100;
constexpr uint32_t BEACON_LOST_MS = 1500;

struct __attribute__((packed)) FrameHeader {
	uint8_t type;
	uint8_t version;
	uint32_t netId;
};

struct __attribute__((packed)) Beacon {
	FrameHeader header;
	uint8_t channel;
	uint8_t trackerCount;
};

constexpr size_t MAX_PAYLOAD = MAX_FRAME - sizeof(FrameHeader);

// FNV-1a over the pairing key.
inline uint32_t netIdFromKey(const uint8_t* key) {
	uint32_t hash = 2166136261u;
	for (size_t i = 0; i < KEY_SIZE; i++) {
		hash ^= key[i];
		hash *= 16777619u;
	}
	return hash;
}

inline bool isValidChannel(int channel) {
	return channel >= MIN_CHANNEL && channel <= MAX_CHANNEL;
}

}  // namespace PicoLink

#endif  // PICOVR_LINK_H_
