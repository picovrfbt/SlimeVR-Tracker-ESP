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
#include "manager.h"

#include "GlobalVars.h"

#include <string.h>

namespace SlimeVR {
namespace Network {

// Auto: how long the dongle may be missing before trying saved Wi-Fi.
constexpr uint32_t AUTO_FALLBACK_MS = 30000;
// Auto on fallback Wi-Fi without a server: how often to look for the dongle,
// and for how long each time.
constexpr uint32_t WIFI_RETRY_DONGLE_MS = 20000;
constexpr uint32_t DONGLE_PROBE_MS = 3000;
constexpr uint32_t SIGNAL_REPORT_MS = 2000;

void Manager::setup() {
	m_Settings = LinkConfig::load();
	networkConnection.setTransport(&m_Udp);

	if (m_Settings.mode == LinkMode::Wifi || !startDongle(false)) {
		startWifi();
	}
	printLinkState();
}

void Manager::update() {
	if (m_HasRequest) {
		m_HasRequest = false;
		applyRequest();
	}

	bool linkUp;
	if (m_DongleActive) {
		m_EspNow.upkeep();
		linkUp = m_EspNow.dongleHeard();

		if (linkUp) {
			m_Probing = false;
			// The dongle moved; remember where, so the next boot goes straight there.
			if (m_EspNow.channel() != m_Settings.channel) {
				m_Settings.channel = m_EspNow.channel();
				save();
			}
			if (networkConnection.isConnected()
				&& millis() - m_LastRssiSentAt >= SIGNAL_REPORT_MS) {
				m_LastRssiSentAt = millis();
				networkConnection.sendSignalStrength(m_EspNow.rssi());
			}
		} else if (m_Settings.mode == LinkMode::Auto && WiFiNetwork::hasSavedCredentials()) {
			uint32_t limit = m_Probing ? DONGLE_PROBE_MS : AUTO_FALLBACK_MS;
			if (m_EspNow.msSinceBeacon() >= limit) {
				if (!m_Probing) {
					m_Logger.warn("Dongle missing, switching to Wi-Fi");
				}
				startWifi();
				linkUp = false;
			}
		}
	} else {
		WiFiNetwork::upkeep();
		linkUp = WiFiNetwork::isConnected();

		// On fallback Wi-Fi with no server to tell us when the dongle is back:
		// go and listen for it now and then.
		if (m_Settings.mode == LinkMode::Auto && m_Settings.hasDongle()
			&& !networkConnection.isConnected()
			&& millis() - m_LinkStartedAt >= WIFI_RETRY_DONGLE_MS) {
			startDongle(true);
			linkUp = false;
		}
	}

	auto wasConnected = m_IsConnected;
	m_IsConnected = linkUp;

	if (!m_IsConnected) {
		return;
	}

	if (!wasConnected) {
		// Link (re)gained: rediscover the server and reconnect
		networkConnection.reset();
	}

	networkConnection.update();
}

void Manager::startWifi() {
	m_EspNow.stop();
	m_DongleActive = false;
	m_Probing = false;
	networkConnection.setTransport(&m_Udp);
	m_IsConnected = false;
	::WiFiNetwork::setUp();
	m_LinkStartedAt = millis();
}

bool Manager::startDongle(bool probe) {
#if ESP32
	if (!m_Settings.hasDongle()) {
		return false;
	}

	WiFiNetwork::stop();
	if (!m_EspNow.start(m_Settings.dongleMac, m_Settings.channel, m_Settings.key)) {
		m_Logger.error("Could not start the dongle radio, staying on Wi-Fi");
		startWifi();
		return false;
	}

	networkConnection.setTransport(&m_EspNow);
	m_IsConnected = false;
	m_DongleActive = true;
	m_Probing = probe;
	m_LinkStartedAt = millis();
	if (!probe) {
		m_Logger.info("Using the PicoVR Dongle on channel %d", m_Settings.channel);
	}
	return true;
#else
	(void)probe;
	m_Logger.error("The PicoVR Dongle needs an ESP32 tracker");
	return false;
#endif
}

void Manager::applyMode() {
	bool wantDongle = m_Settings.mode != LinkMode::Wifi;
	if (wantDongle && (!m_DongleActive || m_Probing)) {
		if (!startDongle(false)) {
			m_Settings.mode = LinkMode::Wifi;
			save();
		}
	} else if (!wantDongle && m_DongleActive) {
		startWifi();
	}
}

bool Manager::setMode(LinkMode mode) {
	if (mode != LinkMode::Wifi && !m_Settings.hasDongle()) {
		return false;
	}
	m_Settings.mode = mode;
	save();
	applyMode();
	return true;
}

bool Manager::bindDongle(
	const uint8_t mac[6],
	uint8_t channel,
	const uint8_t key[16]
) {
	if (!PicoLink::isValidChannel(channel)) {
		return false;
	}
	memcpy(m_Settings.dongleMac, mac, sizeof(m_Settings.dongleMac));
	m_Settings.channel = channel;
	memcpy(m_Settings.key, key, sizeof(m_Settings.key));
	m_Settings.mode = LinkMode::Auto;
	save();

	// Restart on the new dongle even if another one was in use.
	m_DongleActive = false;
	applyMode();
	return m_DongleActive;
}

void Manager::useWifi() {
	m_Settings.mode = LinkMode::Wifi;
	save();
	if (m_DongleActive) {
		// The caller sets the new credentials, which starts the Wi-Fi connection.
		m_EspNow.stop();
		m_DongleActive = false;
		m_Probing = false;
		networkConnection.setTransport(&m_Udp);
		m_IsConnected = false;
		m_LinkStartedAt = millis();
	}
}

void Manager::requestLink(const uint8_t* payload, int length) {
	if (length < 1) {
		return;
	}
	m_RequestLength = length < (int)sizeof(m_Request) ? length : sizeof(m_Request);
	memcpy(m_Request, payload, m_RequestLength);
	m_HasRequest = true;
}

void Manager::applyRequest() {
	uint8_t mode = m_Request[0];
	if (mode > (uint8_t)LinkMode::Auto) {
		m_Logger.warn("Server asked for an unknown link mode %d", mode);
		return;
	}

	if (m_RequestLength >= (int)sizeof(m_Request)) {
		const uint8_t* mac = m_Request + 1;
		uint8_t channel = m_Request[7];
		const uint8_t* key = m_Request + 8;
		m_Logger.info("Server asked to bind to a dongle on channel %d", channel);
		if (!bindDongle(mac, channel, key)) {
			m_Logger.warn("Could not bind to the dongle");
			return;
		}
	}

	m_Logger.info("Server asked for link mode %s", LinkConfig::modeName((LinkMode)mode));
	if (!setMode((LinkMode)mode)) {
		m_Logger.warn("No dongle bound, staying on Wi-Fi");
	}
}

void Manager::save() {
	if (!LinkConfig::save(m_Settings)) {
		m_Logger.error("Could not save the link settings");
	}
}

void Manager::printLinkState() {
	const uint8_t* mac = m_Settings.dongleMac;
	m_Logger.info(
		"Link mode: %s, using: %s, dongle: %02X:%02X:%02X:%02X:%02X:%02X, channel: "
		"%d, dongle heard: %s, signal: %d dBm, wifi saved: %s",
		LinkConfig::modeName(m_Settings.mode),
		m_DongleActive ? "DONGLE" : "WIFI",
		mac[0],
		mac[1],
		mac[2],
		mac[3],
		mac[4],
		mac[5],
		m_Settings.channel,
		m_EspNow.dongleHeard() ? "yes" : "no",
		m_EspNow.rssi(),
		WiFiNetwork::hasSavedCredentials() ? "yes" : "no"
	);
}

}  // namespace Network
}  // namespace SlimeVR
