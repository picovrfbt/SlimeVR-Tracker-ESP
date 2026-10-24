/*
	A packet transport for Connection: Wi-Fi/UDP or the PicoVR Dongle over
	ESP-NOW. Connection builds and parses the same server protocol either way.
*/

#ifndef SLIMEVR_NETWORK_TRANSPORT_H_
#define SLIMEVR_NETWORK_TRANSPORT_H_

#include <stddef.h>
#include <stdint.h>

namespace SlimeVR {
namespace Network {

class Transport {
public:
	virtual ~Transport() = default;

	// Called when Connection resets, before the first packet.
	virtual void begin() = 0;

	// Outgoing packet to the server: begin, write any number of times, end.
	virtual bool beginPacket() = 0;
	virtual size_t write(const uint8_t* buffer, size_t size) = 0;
	virtual bool endPacket() = 0;
	virtual int getWriteError() { return 0; }

	// Incoming packet from the server: size of the next one (0 if none), then
	// read it.
	virtual int parsePacket() = 0;
	virtual int read(uint8_t* buffer, size_t size) = 0;

	// The handshake answer came from the last parsed packet's sender.
	virtual void serverFound() {}

	// For logs: who the last parsed packet came from.
	virtual const char* remoteName() = 0;
};

}  // namespace Network
}  // namespace SlimeVR

#endif  // SLIMEVR_NETWORK_TRANSPORT_H_
