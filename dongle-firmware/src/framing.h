/*
	COBS framing and CRC-16/CCITT-FALSE for the dongle's USB protocol
	(see dongleprotocol.h). Plain C++, no Arduino, so it can be tested on a PC.
*/

#ifndef PICOVR_DONGLE_FRAMING_H_
#define PICOVR_DONGLE_FRAMING_H_

#include <stddef.h>
#include <stdint.h>

namespace Framing {

inline uint16_t crc16(const uint8_t* data, size_t length) {
	uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < length; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (int bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
		}
	}
	return crc;
}

// Worst case output size for COBS-encoding `length` bytes (no delimiter).
constexpr size_t encodedSize(size_t length) { return length + length / 254 + 1; }

// Encodes without the trailing 0x00 delimiter. Returns the encoded length.
inline size_t cobsEncode(const uint8_t* in, size_t length, uint8_t* out) {
	size_t read = 0;
	size_t write = 1;
	size_t codeAt = 0;
	uint8_t code = 1;
	while (read < length) {
		if (in[read] == 0) {
			out[codeAt] = code;
			codeAt = write++;
			code = 1;
		} else {
			out[write++] = in[read];
			code++;
			if (code == 0xFF) {
				out[codeAt] = code;
				codeAt = write++;
				code = 1;
			}
		}
		read++;
	}
	out[codeAt] = code;
	return write;
}

// Decodes one frame (without its delimiter). Returns the decoded length, or 0
// if the input is not valid COBS.
inline size_t cobsDecode(const uint8_t* in, size_t length, uint8_t* out, size_t outSize) {
	size_t read = 0;
	size_t write = 0;
	while (read < length) {
		uint8_t code = in[read++];
		if (code == 0 || read + code - 1 > length) {
			return 0;
		}
		for (uint8_t i = 1; i < code; i++) {
			if (write >= outSize) {
				return 0;
			}
			out[write++] = in[read++];
		}
		if (code != 0xFF && read < length) {
			if (write >= outSize) {
				return 0;
			}
			out[write++] = 0;
		}
	}
	return write;
}

}  // namespace Framing

#endif  // PICOVR_DONGLE_FRAMING_H_
