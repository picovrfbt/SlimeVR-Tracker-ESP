#include "udptransport.h"

namespace SlimeVR {
namespace Network {

UdpTransport::UdpTransport() {
#ifdef SERVER_IP
	m_ServerHost.fromString(SERVER_IP);
#endif

#ifdef SERVER_PORT
	m_ServerPort = SERVER_PORT;
#endif
}

void UdpTransport::begin() { m_UDP.begin(m_ServerPort); }

bool UdpTransport::beginPacket() {
	// This *technically* should *never* fail, since the underlying UDP library
	// just returns 1.
	return m_UDP.beginPacket(m_ServerHost, m_ServerPort) > 0;
}

size_t UdpTransport::write(const uint8_t* buffer, size_t size) {
	return m_UDP.write(buffer, size);
}

bool UdpTransport::endPacket() {
	// 0 is usually just `ERR_ABRT`, but the UDP client doesn't expose the full
	// error code to us, so we just have to live with it.
	return m_UDP.endPacket() > 0;
}

int UdpTransport::getWriteError() { return m_UDP.getWriteError(); }

int UdpTransport::parsePacket() { return m_UDP.parsePacket(); }

int UdpTransport::read(uint8_t* buffer, size_t size) {
	return m_UDP.read(buffer, size);
}

void UdpTransport::serverFound() {
	m_ServerHost = m_UDP.remoteIP();
	m_ServerPort = m_UDP.remotePort();
}

const char* UdpTransport::remoteName() {
	snprintf(
		m_RemoteName,
		sizeof(m_RemoteName),
		"%s:%d",
		m_UDP.remoteIP().toString().c_str(),
		m_UDP.remotePort()
	);
	return m_RemoteName;
}

}  // namespace Network
}  // namespace SlimeVR
