/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2023 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#ifndef SLIMEVR_NETWORK_MANAGER_H_
#define SLIMEVR_NETWORK_MANAGER_H_

#include "espnowtransport.h"
#include "globals.h"
#include "linkconfig.h"
#include "logging/Logger.h"
#include "packets.h"
#include "udptransport.h"
#include "wifihandler.h"
#include "wifiprovisioning.h"

namespace SlimeVR {
namespace Network {

/*
	Picks the link to the server: Wi-Fi, the PicoVR Dongle, or Auto (the dongle,
	falling back to saved Wi-Fi while the dongle is missing). The choice is a
	saved setting, so switching never needs a reflash.

	Ways a tracker switches:
	- USB serial: SET DONGLE (bind, mode Auto), SET MODE, and SET WIFI / SET BWIFI
	  (back to Wi-Fi).
	- From the server over the current link: PACKET_PICOVR_SET_LINK.
	- On its own in Auto mode: dongle missing for 30 s with Wi-Fi saved -> Wi-Fi;
	  on fallback Wi-Fi with no server -> listen for the dongle again every 20 s.
*/
class Manager {
public:
	void setup();
	void update();

	// false if a dongle mode was asked for but no dongle is bound.
	bool setMode(LinkMode mode);
	// Bind to a dongle and switch to Auto.
	bool bindDongle(const uint8_t mac[6], uint8_t channel, const uint8_t key[16]);
	// New Wi-Fi credentials are about to be set: go back to Wi-Fi mode.
	void useWifi();
	// A PACKET_PICOVR_SET_LINK payload from the server. Applied on the next
	// update, outside the packet handler, since it swaps the transport.
	void requestLink(const uint8_t* payload, int length);

	const LinkSettings& getSettings() const { return m_Settings; }
	bool isDongleActive() const { return m_DongleActive; }
	void printLinkState();

private:
	void startWifi();
	bool startDongle(bool probe);
	void applyMode();
	void applyRequest();
	void save();

	LinkSettings m_Settings;
	UdpTransport m_Udp;
	EspNowTransport m_EspNow;

	bool m_DongleActive = false;
	// A short look for the dongle from fallback Wi-Fi.
	bool m_Probing = false;
	bool m_IsConnected = false;
	uint32_t m_LinkStartedAt = 0;
	uint32_t m_LastRssiSentAt = 0;

	bool m_HasRequest = false;
	uint8_t m_Request[1 + 6 + 1 + PicoLink::KEY_SIZE] = {0};
	int m_RequestLength = 0;

	Logging::Logger m_Logger = Logging::Logger("Link");
};

}  // namespace Network
}  // namespace SlimeVR

#endif  // SLIMEVR_NETWORK_MANAGER_H_
