#include "ota.h"

#include <Update.h>
#include <esp_ota_ops.h>
#include <mbedtls/sha256.h>

#include "logging/Logger.h"

namespace PicoOta {

namespace {
SlimeVR::Logging::Logger logger("OTA");

State currentState = State::Idle;
Error lastError = Error::None;
uint32_t bytesWritten = 0;
uint32_t totalSize = 0;
uint8_t wantedHash[32] = {0};
mbedtls_sha256_context hashContext;
bool hashStarted = false;

uint32_t readUint32(const uint8_t* at) {
	return (uint32_t)at[0] | ((uint32_t)at[1] << 8) | ((uint32_t)at[2] << 16)
		 | ((uint32_t)at[3] << 24);
}

void fail(Error why) {
	lastError = why;
	currentState = State::Failed;
	if (hashStarted) {
		mbedtls_sha256_free(&hashContext);
		hashStarted = false;
	}
	Update.abort();
}
}  // namespace

bool begin(const uint8_t* body, size_t length) {
	if (length < 4 + sizeof(wantedHash)) {
		fail(Error::TooShort);
		return false;
	}
	abort();

	totalSize = readUint32(body);
	memcpy(wantedHash, body + 4, sizeof(wantedHash));

	if (!Update.begin(totalSize, U_FLASH)) {
		logger.error("Can't start: %s", Update.errorString());
		fail(Error::NoRoom);
		return false;
	}

	mbedtls_sha256_init(&hashContext);
	mbedtls_sha256_starts(&hashContext, 0);
	hashStarted = true;

	bytesWritten = 0;
	lastError = Error::None;
	currentState = State::Receiving;
	logger.info("Receiving %u bytes of firmware", totalSize);
	return true;
}

bool chunk(const uint8_t* body, size_t length) {
	if (currentState != State::Receiving || length < 4) {
		return false;
	}
	// The server resends from whatever offset the status packet reports, so a
	// chunk that is not the next one is simply dropped.
	if (readUint32(body) != bytesWritten) {
		return false;
	}

	const uint8_t* payload = body + 4;
	size_t payloadLength = length - 4;
	if (payloadLength == 0) {
		return true;
	}
	if (bytesWritten + payloadLength > totalSize) {
		fail(Error::WrongSize);
		return false;
	}
	if (Update.write(const_cast<uint8_t*>(payload), payloadLength) != payloadLength) {
		logger.error("Write failed: %s", Update.errorString());
		fail(Error::WriteFailed);
		return false;
	}

	mbedtls_sha256_update(&hashContext, payload, payloadLength);
	bytesWritten += payloadLength;
	return true;
}

bool end() {
	if (currentState != State::Receiving) {
		return false;
	}
	if (bytesWritten != totalSize) {
		fail(Error::WrongSize);
		return false;
	}

	uint8_t got[32];
	mbedtls_sha256_finish(&hashContext, got);
	mbedtls_sha256_free(&hashContext);
	hashStarted = false;

	if (memcmp(got, wantedHash, sizeof(got)) != 0) {
		logger.error("The firmware we received isn't the one the server sent");
		fail(Error::VerifyFailed);
		return false;
	}
	if (!Update.end(true)) {
		logger.error("Can't finish: %s", Update.errorString());
		fail(Error::VerifyFailed);
		return false;
	}

	currentState = State::Ready;
	logger.info("Firmware received and checked, restarting into it");
	return true;
}

void abort() {
	if (currentState == State::Receiving) {
		if (hashStarted) {
			mbedtls_sha256_free(&hashContext);
			hashStarted = false;
		}
		Update.abort();
	}
	bytesWritten = 0;
	totalSize = 0;
	lastError = Error::None;
	currentState = State::Idle;
}

State state() { return currentState; }

Error error() { return lastError; }

uint32_t received() { return bytesWritten; }

uint32_t expected() { return totalSize; }

void confirm() {
	const esp_partition_t* running = esp_ota_get_running_partition();
	esp_ota_img_states_t imageState;
	if (esp_ota_get_state_partition(running, &imageState) != ESP_OK) {
		return;
	}
	if (imageState == ESP_OTA_IMG_PENDING_VERIFY) {
		esp_ota_mark_app_valid_cancel_rollback();
		logger.info("This firmware works; the old one won't be restored");
	}
}

}  // namespace PicoOta
