#ifndef SLIMEVR_NETWORK_UDPTRANSPORT_H_
#define SLIMEVR_NETWORK_UDPTRANSPORT_H_

#include <Arduino.h>
#include <WiFiUdp.h>

#include "transport.h"

namespace SlimeVR {
namespace Network {

// The original Wi-Fi path: broadcast until the server answers the handshake,
// then talk to that server's address.
class UdpTransport : public Transport {
public:
	UdpTransport();

	void begin() override;
	bool beginPacket() override;
	size_t write(const uint8_t* buffer, size_t size) override;
	bool endPacket() override;
	int getWriteError() override;
	int parsePacket() override;
	int read(uint8_t* buffer, size_t size) override;
	void serverFound() override;
	const char* remoteName() override;

private:
	WiFiUDP m_UDP;
	int m_ServerPort = 6969;
	IPAddress m_ServerHost = IPAddress(255, 255, 255, 255);
	char m_RemoteName[24] = {0};
};

}  // namespace Network
}  // namespace SlimeVR

#endif  // SLIMEVR_NETWORK_UDPTRANSPORT_H_
