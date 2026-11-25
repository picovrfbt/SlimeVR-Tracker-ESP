/*
	Updating the dongle's own firmware over the USB port it already talks on.

	The server sends CMD_OTA_BEGIN with the size and SHA-256 of the image, then
	the image in chunks, then CMD_OTA_END. Each command is answered before the
	next arrives, so there is no window to keep track of and nothing to resend.

	The image goes into whichever app slot is not running. It is only armed once
	every byte has been written and the hash matches what the server promised,
	and the bootloader puts the old firmware back by itself unless the new one
	reaches confirm().
*/

#ifndef PICOVR_DONGLE_OTA_H_
#define PICOVR_DONGLE_OTA_H_

#include <stddef.h>
#include <stdint.h>

namespace DongleOta {

/** Starts an update. body = size (4, LE) + sha256 (32). A DongleProtocol status. */
uint8_t begin(const uint8_t* body, size_t length);

/** One chunk, in order. body = offset (4, LE) + bytes. A DongleProtocol status. */
uint8_t chunk(const uint8_t* body, size_t length);

/**
 * Checks the whole image and arms the slot. STATUS_OK means the dongle is
 * about to restart into the new firmware.
 */
uint8_t end();

void abort();

/** True while an image is being received. */
bool busy();

/** Bytes written so far, which is also the offset wanted next. */
uint32_t received();

/**
 * Called once the server has talked to us, so the bootloader stops treating
 * this firmware as on trial. Without it, the next restart goes back to the
 * firmware that was working before.
 */
void confirm();

}  // namespace DongleOta

#endif  // PICOVR_DONGLE_OTA_H_
