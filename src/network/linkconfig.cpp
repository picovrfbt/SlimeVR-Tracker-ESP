#include "linkconfig.h"

#include <LittleFS.h>
#include <string.h>
#include <strings.h>

namespace SlimeVR {
namespace Network {
namespace LinkConfig {

static constexpr const char* PATH = "/link.bin";
static constexpr uint8_t MAGIC[3] = {'P', 'V', 'L'};
static constexpr uint8_t FORMAT_VERSION = 1;

struct __attribute__((packed)) StoredLink {
	uint8_t magic[3];
	uint8_t version;
	uint8_t mode;
	uint8_t dongleMac[6];
	uint8_t channel;
	uint8_t key[PicoLink::KEY_SIZE];
};

LinkSettings load() {
	LinkSettings settings;
	if (!LittleFS.exists(PATH)) {
		return settings;
	}

	File file = LittleFS.open(PATH, "r");
	StoredLink stored;
	size_t read = file.read(reinterpret_cast<uint8_t*>(&stored), sizeof(stored));
	file.close();

	if (read != sizeof(stored) || memcmp(stored.magic, MAGIC, sizeof(MAGIC)) != 0
		|| stored.version != FORMAT_VERSION || stored.mode > (uint8_t)LinkMode::Auto) {
		return settings;
	}

	settings.mode = static_cast<LinkMode>(stored.mode);
	memcpy(settings.dongleMac, stored.dongleMac, sizeof(settings.dongleMac));
	settings.channel = stored.channel;
	memcpy(settings.key, stored.key, sizeof(settings.key));

	// A dongle mode without a dongle can never connect.
	if (!settings.hasDongle()) {
		settings.mode = LinkMode::Wifi;
	}
	return settings;
}

bool save(const LinkSettings& settings) {
	StoredLink stored;
	memcpy(stored.magic, MAGIC, sizeof(MAGIC));
	stored.version = FORMAT_VERSION;
	stored.mode = static_cast<uint8_t>(settings.mode);
	memcpy(stored.dongleMac, settings.dongleMac, sizeof(stored.dongleMac));
	stored.channel = settings.channel;
	memcpy(stored.key, settings.key, sizeof(stored.key));

	File file = LittleFS.open(PATH, "w");
	if (!file) {
		return false;
	}
	size_t written = file.write(reinterpret_cast<const uint8_t*>(&stored), sizeof(stored));
	file.close();
	return written == sizeof(stored);
}

const char* modeName(LinkMode mode) {
	switch (mode) {
		case LinkMode::Wifi:
			return "WIFI";
		case LinkMode::Dongle:
			return "DONGLE";
		case LinkMode::Auto:
			return "AUTO";
	}
	return "UNKNOWN";
}

bool parseMode(const char* text, LinkMode& mode) {
	if (text == nullptr) {
		return false;
	}
	if (strcasecmp(text, "WIFI") == 0) {
		mode = LinkMode::Wifi;
	} else if (strcasecmp(text, "DONGLE") == 0) {
		mode = LinkMode::Dongle;
	} else if (strcasecmp(text, "AUTO") == 0) {
		mode = LinkMode::Auto;
	} else {
		return false;
	}
	return true;
}

}  // namespace LinkConfig
}  // namespace Network
}  // namespace SlimeVR
