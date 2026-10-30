/*
	PicoVR Dongle <-> PC protocol over USB serial.

	Every frame is: type (1) + body + CRC-16/CCITT-FALSE of type+body (2, little
	endian), COBS-encoded and ended with a 0x00 byte. Anything that fails to
	decode or check is dropped, so boot messages on the same port do no harm:
	the reader just waits for the next 0x00.

	The PC tells a dongle apart from a tracker (both show up as 303A:1001) by
	sending HELLO when it opens the port: only a dongle answers with
	HELLO_REPLY.
*/

#ifndef PICOVR_DONGLE_PROTOCOL_H_
#define PICOVR_DONGLE_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

namespace DongleProtocol {

// PC -> dongle
constexpr uint8_t CMD_HELLO = 0x01;  // (none)
constexpr uint8_t CMD_ALLOW = 0x02;  // tracker address (6)
constexpr uint8_t CMD_REMOVE = 0x03;  // tracker address (6)
constexpr uint8_t CMD_SET_CHANNEL = 0x04;  // channel (1)
constexpr uint8_t CMD_SEND = 0x05;  // tracker address (6) + server packet
constexpr uint8_t CMD_LIST = 0x06;  // (none)
constexpr uint8_t CMD_GET_KEY = 0x07;  // (none)
constexpr uint8_t CMD_SET_KEY = 0x08;  // pairing key (16)
constexpr uint8_t CMD_SCAN = 0x09;  // (none): how busy is each channel?
constexpr uint8_t CMD_NEW_PAIRING = 0x0A;  // (none): new key, forget all trackers
// Updating the dongle's own firmware over this port. Every one of these is
// answered with MSG_RESULT before the next is sent, so the transfer paces
// itself and nothing needs to be resent: USB does not lose bytes.
constexpr uint8_t CMD_OTA_BEGIN = 0x0B;  // size (4, LE) + sha256 (32)
constexpr uint8_t CMD_OTA_CHUNK = 0x0C;  // offset (4, LE) + bytes
constexpr uint8_t CMD_OTA_END = 0x0D;  // (none): check it, then restart into it
constexpr uint8_t CMD_OTA_ABORT = 0x0E;  // (none)

// Dongle -> PC
constexpr uint8_t MSG_HELLO = 0x81;
// "PICOVR-DONGLE" (13) + link version (1) + firmware major, minor, patch (3) +
// dongle address (6) + channel (1) + trackers (1) + max trackers (1) +
// network id (4, LE)
constexpr uint8_t MSG_DATA = 0x82;  // tracker address (6) + RSSI (1, signed) + packet
constexpr uint8_t MSG_RESULT = 0x83;  // command (1) + status (1)
constexpr uint8_t MSG_LIST = 0x84;
// count (1) + per tracker: address (6) + RSSI (1) + seconds since heard (2, LE;
// 0xFFFF = not heard since boot)
constexpr uint8_t MSG_KEY = 0x85;  // pairing key (16)
constexpr uint8_t MSG_SCAN = 0x86;  // frames heard on channels 1-13 (13 x 2, LE)
constexpr uint8_t MSG_LOG = 0x87;  // text

constexpr char HELLO_MAGIC[] = "PICOVR-DONGLE";
constexpr int HELLO_MAGIC_LENGTH = 13;

// MSG_RESULT status
constexpr uint8_t STATUS_OK = 0;
constexpr uint8_t STATUS_BAD_ARGUMENTS = 1;
constexpr uint8_t STATUS_FULL = 2;
constexpr uint8_t STATUS_NOT_FOUND = 3;
constexpr uint8_t STATUS_RADIO_ERROR = 4;
constexpr uint8_t STATUS_UNKNOWN_COMMAND = 5;
constexpr uint8_t STATUS_NO_ROOM = 6;
constexpr uint8_t STATUS_OUT_OF_ORDER = 7;
constexpr uint8_t STATUS_WRITE_FAILED = 8;
constexpr uint8_t STATUS_BAD_IMAGE = 9;
constexpr uint8_t STATUS_NOT_UPDATING = 10;

// The most firmware one CMD_OTA_CHUNK may carry.
constexpr size_t OTA_MAX_CHUNK = 1024;

}  // namespace DongleProtocol

#endif  // PICOVR_DONGLE_PROTOCOL_H_
