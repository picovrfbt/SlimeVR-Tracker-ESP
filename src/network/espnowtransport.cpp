#include "espnowtransport.h"

#include <string.h>

#if ESP32
#include <WiFi.h>
#include <esp_idf_version.h>
#include <esp_now.h>
#include <esp_wifi.h>
#endif

namespace SlimeVR {
namespace Network {

// How long to listen on each channel while hunting for a lost dongle: a little
// over two beacon intervals.
constexpr uint32_t HOP_DWELL_MS = 2 * PicoLink::BEACON_INTERVAL_MS + 50;
constexpr int RX_QUEUE_LENGTH = 8;

#if ESP32

static EspNowTransport* activeTransport = nullptr;

struct EspNowCallbacks {
#if ESP_IDF_VERSION_MAJOR >= 5
	static void onReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
		if (activeTransport == nullptr || info == nullptr) {
			return;
		}
		int8_t rssi = info->rx_ctrl != nullptr ? info->rx_ctrl->rssi : 0;
		activeTransport->handleFrame(info->src_addr, data, len, rssi);
	}
#else
	static void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
		if (activeTransport == nullptr) {
			return;
		}
		activeTransport->handleFrame(mac, data, len, 0);
	}
#endif

	static void onSent(const uint8_t* mac, esp_now_send_status_t status) {
		(void)mac;
		if (activeTransport != nullptr && status != ESP_NOW_SEND_SUCCESS) {
			activeTransport->m_SendFailures = activeTransport->m_SendFailures + 1;
		}
	}
};

bool EspNowTransport::start(
	const uint8_t dongleMac[6],
	uint8_t channel,
	const uint8_t key[16]
) {
	stop();

	memcpy(m_DongleMac, dongleMac, sizeof(m_DongleMac));
	m_NetId = PicoLink::netIdFromKey(key);
	m_Channel = PicoLink::isValidChannel(channel) ? channel : 1;
	m_HuntChannel = m_Channel;
	m_LastBeaconAt = 0;
	m_EverHeard = false;
	m_LastRssi = 0;
	m_SendFailures = 0;
	m_HasPending = false;

	if (m_RxQueue == nullptr) {
		m_RxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(RxItem));
		if (m_RxQueue == nullptr) {
			return false;
		}
	}
	xQueueReset(m_RxQueue);

	WiFi.mode(WIFI_STA);
	// Trackers must hear every beacon and server packet: no modem sleep here.
	esp_wifi_set_ps(WIFI_PS_NONE);
	setRadioChannel(m_Channel);

	if (esp_now_init() != ESP_OK) {
		return false;
	}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
	esp_now_rate_config_t rate = {};
	rate.phymode = WIFI_PHY_MODE_11G;
	rate.rate = WIFI_PHY_RATE_24M;
#else
	// 24 Mbps: at the 1 Mbps default, 8 moving trackers need more airtime than
	// the channel has.
	esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_24M);
#endif

	esp_now_peer_info_t peer = {};
	memcpy(peer.peer_addr, m_DongleMac, sizeof(m_DongleMac));
	peer.channel = 0;  // whatever channel the radio is on
	peer.ifidx = WIFI_IF_STA;
	peer.encrypt = false;
	if (esp_now_add_peer(&peer) != ESP_OK) {
		esp_now_deinit();
		return false;
	}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
	esp_now_set_peer_rate_config(m_DongleMac, &rate);
#endif

	activeTransport = this;
	esp_now_register_recv_cb(EspNowCallbacks::onReceive);
	esp_now_register_send_cb(EspNowCallbacks::onSent);

	m_StartedAt = millis();
	m_LastHopAt = m_StartedAt;
	m_Running = true;
	return true;
}

void EspNowTransport::stop() {
	if (!m_Running) {
		return;
	}
	m_Running = false;
	esp_now_unregister_recv_cb();
	esp_now_unregister_send_cb();
	activeTransport = nullptr;
	esp_now_deinit();
}

void EspNowTransport::setRadioChannel(uint8_t channel) {
	esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

void EspNowTransport::handleFrame(
	const uint8_t* mac,
	const uint8_t* data,
	int len,
	int8_t rssi
) {
	// Runs in the Wi-Fi task: keep it short, no logging.
	if (len < (int)sizeof(PicoLink::FrameHeader)
		|| memcmp(mac, m_DongleMac, sizeof(m_DongleMac)) != 0) {
		return;
	}
	PicoLink::FrameHeader header;
	memcpy(&header, data, sizeof(header));
	if (header.version != PicoLink::LINK_VERSION || header.netId != m_NetId) {
		return;
	}

	if (header.type == PicoLink::FRAME_BEACON) {
		if (len < (int)sizeof(PicoLink::Beacon)) {
			return;
		}
		PicoLink::Beacon beacon;
		memcpy(&beacon, data, sizeof(beacon));
		if (PicoLink::isValidChannel(beacon.channel)) {
			m_Channel = beacon.channel;
		}
		m_LastRssi = rssi;
		m_LastBeaconAt = millis();
		m_EverHeard = true;
		return;
	}

	if (header.type == PicoLink::FRAME_DATA_DOWN && m_RxQueue != nullptr) {
		RxItem item;
		size_t payload = len - sizeof(PicoLink::FrameHeader);
		item.len = payload;
		memcpy(item.data, data + sizeof(PicoLink::FrameHeader), payload);
		m_LastRssi = rssi;
		// Server packets also prove the dongle is there.
		m_LastBeaconAt = millis();
		m_EverHeard = true;
		xQueueSend(m_RxQueue, &item, 0);
	}
}

void EspNowTransport::upkeep() {
	// Stay on the current channel until the beacon has really gone missing
	// (or, just after starting, until it had time to arrive).
	if (!m_Running || msSinceBeacon() < PicoLink::BEACON_LOST_MS) {
		return;
	}

	// Beacon missing: hop to the next channel after each dwell.
	uint32_t now = millis();
	if (now - m_LastHopAt < HOP_DWELL_MS) {
		return;
	}
	m_LastHopAt = now;
	m_HuntChannel = m_HuntChannel >= PicoLink::MAX_CHANNEL
					  ? PicoLink::MIN_CHANNEL
					  : m_HuntChannel + 1;
	setRadioChannel(m_HuntChannel);
}

bool EspNowTransport::dongleHeard() const {
	return m_Running && m_EverHeard && msSinceBeacon() < PicoLink::BEACON_LOST_MS;
}

uint32_t EspNowTransport::msSinceBeacon() const {
	uint32_t since = m_EverHeard ? m_LastBeaconAt : m_StartedAt;
	return millis() - since;
}

void EspNowTransport::begin() {
	m_HasPending = false;
	if (m_RxQueue != nullptr) {
		xQueueReset(m_RxQueue);
	}
}

bool EspNowTransport::beginPacket() {
	if (!m_Running) {
		return false;
	}
	PicoLink::FrameHeader header;
	header.type = PicoLink::FRAME_DATA_UP;
	header.version = PicoLink::LINK_VERSION;
	header.netId = m_NetId;
	memcpy(m_TxBuffer, &header, sizeof(header));
	m_TxLength = sizeof(header);
	m_TxOverflow = false;
	return true;
}

size_t EspNowTransport::write(const uint8_t* buffer, size_t size) {
	if (m_TxLength + size > sizeof(m_TxBuffer)) {
		m_TxOverflow = true;
		return 0;
	}
	memcpy(m_TxBuffer + m_TxLength, buffer, size);
	m_TxLength += size;
	return size;
}

bool EspNowTransport::endPacket() {
	if (!m_Running || m_TxOverflow || m_TxLength <= sizeof(PicoLink::FrameHeader)) {
		return false;
	}
	return esp_now_send(m_DongleMac, m_TxBuffer, m_TxLength) == ESP_OK;
}

int EspNowTransport::getWriteError() { return m_TxOverflow ? 1 : 0; }

int EspNowTransport::parsePacket() {
	if (!m_HasPending) {
		if (m_RxQueue == nullptr || xQueueReceive(m_RxQueue, &m_Pending, 0) != pdTRUE) {
			return 0;
		}
		m_HasPending = true;
	}
	return m_Pending.len;
}

int EspNowTransport::read(uint8_t* buffer, size_t size) {
	if (!m_HasPending) {
		return 0;
	}
	size_t n = m_Pending.len < size ? m_Pending.len : size;
	memcpy(buffer, m_Pending.data, n);
	m_HasPending = false;
	return n;
}

const char* EspNowTransport::remoteName() {
	snprintf(
		m_RemoteName,
		sizeof(m_RemoteName),
		"dongle %02X%02X%02X ch%d",
		m_DongleMac[3],
		m_DongleMac[4],
		m_DongleMac[5],
		m_Channel
	);
	return m_RemoteName;
}

#else  // ESP8266: the PicoVR Dongle needs an ESP32 tracker.

bool EspNowTransport::start(const uint8_t*, uint8_t, const uint8_t*) { return false; }
void EspNowTransport::stop() {}
void EspNowTransport::upkeep() {}
void EspNowTransport::setRadioChannel(uint8_t) {}
void EspNowTransport::handleFrame(const uint8_t*, const uint8_t*, int, int8_t) {}
bool EspNowTransport::dongleHeard() const { return false; }
uint32_t EspNowTransport::msSinceBeacon() const { return UINT32_MAX; }
void EspNowTransport::begin() {}
bool EspNowTransport::beginPacket() { return false; }
size_t EspNowTransport::write(const uint8_t*, size_t) { return 0; }
bool EspNowTransport::endPacket() { return false; }
int EspNowTransport::getWriteError() { return 1; }
int EspNowTransport::parsePacket() { return 0; }
int EspNowTransport::read(uint8_t*, size_t) { return 0; }
const char* EspNowTransport::remoteName() { return "dongle"; }

#endif

}  // namespace Network
}  // namespace SlimeVR
