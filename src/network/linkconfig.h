/*
	Which link a tracker uses, and the dongle it is bound to. Kept in its own
	small file (/link.bin) next to the main configuration, so the main config
	format does not change. A factory reset (FRST) clears it with everything
	else, which puts the tracker back on Wi-Fi.
*/

#ifndef SLIMEVR_NETWORK_LINKCONFIG_H_
#define SLIMEVR_NETWORK_LINKCONFIG_H_

#include <stdint.h>

#include "picolink.h"

namespace SlimeVR {
namespace Network {

enum class LinkMode : uint8_t {
	// Wi-Fi only, the original behaviour.
	Wifi = 0,
	// The bound PicoVR Dongle only.
	Dongle = 1,
	// The dongle, falling back to saved Wi-Fi while the dongle is missing.
	Auto = 2,
};

struct LinkSettings {
	LinkMode mode = LinkMode::Wifi;
	uint8_t dongleMac[6] = {0};
	uint8_t channel = 0;
	uint8_t key[PicoLink::KEY_SIZE] = {0};

	bool hasDongle() const { return PicoLink::isValidChannel(channel); }
};

namespace LinkConfig {

LinkSettings load();
bool save(const LinkSettings& settings);

const char* modeName(LinkMode mode);
bool parseMode(const char* text, LinkMode& mode);

}  // namespace LinkConfig

}  // namespace Network
}  // namespace SlimeVR

#endif  // SLIMEVR_NETWORK_LINKCONFIG_H_
