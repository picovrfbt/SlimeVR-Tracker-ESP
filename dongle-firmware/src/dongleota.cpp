#include "dongleota.h"

#include <Update.h>
#include <esp_ota_ops.h>
#include <mbedtls/sha256.h>
#include <string.h>

#include "dongleprotocol.h"

namespace DongleOta {

namespace {

bool receiving = false;
uint32_t written = 0;
uint32_t total = 0;
uint8_t wantedHash[32] = {0};
mbedtls_sha256_context hashContext;
bool hashStarted = false;

uint32_t readUint32(const uint8_t* at) {
	return (uint32_t)at[0] | ((uint32_t)at[1] << 8) | ((uint32_t)at[2] << 16)
		 | ((uint32_t)at[3] << 24);
}

void stopHash() {
	if (hashStarted) {
		mbedtls_sha256_free(&hashContext);
		hashStarted = false;
	}
}

uint8_t fail(uint8_t status) {
	stopHash();
	Update.abort();
	receiving = false;
	written = 0;
	total = 0;
	return status;
}

}  // namespace

uint8_t begin(const uint8_t* body, size_t length) {
	if (length < 4 + sizeof(wantedHash)) {
		return DongleProtocol::STATUS_BAD_ARGUMENTS;
	}
	abort();

	total = readUint32(body);
	memcpy(wantedHash, body + 4, sizeof(wantedHash));
	if (total == 0) {
		return DongleProtocol::STATUS_BAD_ARGUMENTS;
	}
	// Erases the slot the image goes into, which takes a moment.
	if (!Update.begin(total, U_FLASH)) {
		return fail(DongleProtocol::STATUS_NO_ROOM);
	}

	mbedtls_sha256_init(&hashContext);
	mbedtls_sha256_starts(&hashContext, 0);
	hashStarted = true;
	written = 0;
	receiving = true;
	return DongleProtocol::STATUS_OK;
}

uint8_t chunk(const uint8_t* body, size_t length) {
	if (!receiving) {
		return DongleProtocol::STATUS_NOT_UPDATING;
	}
	if (length < 4 || length - 4 > DongleProtocol::OTA_MAX_CHUNK) {
		return DongleProtocol::STATUS_BAD_ARGUMENTS;
	}
	// Each chunk is acknowledged before the next is sent, so the only way one
	// can be out of order is a confused server: say so rather than guess.
	if (readUint32(body) != written) {
		return DongleProtocol::STATUS_OUT_OF_ORDER;
	}

	const uint8_t* payload = body + 4;
	size_t payloadLength = length - 4;
	if (payloadLength == 0) {
		return DongleProtocol::STATUS_OK;
	}
	if (written + payloadLength > total) {
		return fail(DongleProtocol::STATUS_BAD_ARGUMENTS);
	}
	if (Update.write(const_cast<uint8_t*>(payload), payloadLength) != payloadLength) {
		return fail(DongleProtocol::STATUS_WRITE_FAILED);
	}

	mbedtls_sha256_update(&hashContext, payload, payloadLength);
	written += payloadLength;
	return DongleProtocol::STATUS_OK;
}

uint8_t end() {
	if (!receiving) {
		return DongleProtocol::STATUS_NOT_UPDATING;
	}
	if (written != total) {
		return fail(DongleProtocol::STATUS_BAD_ARGUMENTS);
	}

	uint8_t got[32];
	mbedtls_sha256_finish(&hashContext, got);
	stopHash();
	if (memcmp(got, wantedHash, sizeof(got)) != 0) {
		return fail(DongleProtocol::STATUS_BAD_IMAGE);
	}
	if (!Update.end(true)) {
		return fail(DongleProtocol::STATUS_BAD_IMAGE);
	}

	receiving = false;
	return DongleProtocol::STATUS_OK;
}

void abort() {
	if (receiving) {
		stopHash();
		Update.abort();
	}
	receiving = false;
	written = 0;
	total = 0;
}

bool busy() { return receiving; }

uint32_t received() { return written; }

void confirm() {
	const esp_partition_t* running = esp_ota_get_running_partition();
	esp_ota_img_states_t imageState;
	if (running == nullptr || esp_ota_get_state_partition(running, &imageState) != ESP_OK) {
		return;
	}
	if (imageState == ESP_OTA_IMG_PENDING_VERIFY) {
		esp_ota_mark_app_valid_cancel_rollback();
	}
}

}  // namespace DongleOta
