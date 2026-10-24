/*
	PicoVR Dongle: up to eight trackers talk to it over ESP-NOW, and it passes
	their packets to the PicoVR server over USB serial, unchanged and tagged with
	the tracker's address. Server packets go back the other way.

	- Beacons on its channel ten times a second so bound trackers can find it.
	- Only trackers on its allowed list get through (MAX_TRACKERS).
	- Channel, pairing key and the allowed list live in NVS.

	USB protocol: dongleprotocol.h. Radio frames: picolink.h.
*/

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_random.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <string.h>

#include "dongleprotocol.h"
#include "framing.h"
#include "picolink.h"

namespace {

constexpr uint8_t FIRMWARE_MAJOR = 1;
constexpr uint8_t FIRMWARE_MINOR = 0;
constexpr uint8_t FIRMWARE_PATCH = 0;

constexpr uint8_t DEFAULT_CHANNEL = 11;
constexpr uint32_t SCAN_DWELL_MS = 120;
constexpr int RX_QUEUE_LENGTH = 32;
const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct Tracker {
	uint8_t mac[6];
	int8_t rssi;
	uint32_t lastHeardAt;  // 0 = not since boot
};

struct RxItem {
	uint8_t mac[6];
	int8_t rssi;
	uint8_t length;
	uint8_t data[PicoLink::MAX_PAYLOAD];
};

Preferences prefs;
Tracker trackers[PicoLink::MAX_TRACKERS];
uint8_t trackerCount = 0;
uint8_t channel = DEFAULT_CHANNEL;
uint8_t key[PicoLink::KEY_SIZE];
uint32_t netId = 0;
uint8_t ownMac[6];

QueueHandle_t rxQueue;
portMUX_TYPE trackersLock = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t scanCount = 0;
bool radioReady = false;

uint32_t lastBeaconAt = 0;

// USB serial: frames in, frames out.
uint8_t serialIn[Framing::encodedSize(260) + 8];
size_t serialInLength = 0;
bool serialInOverflow = false;

// ---------------------------------------------------------------- settings

void saveSettings() {
	uint8_t macs[6 * PicoLink::MAX_TRACKERS] = {0};
	for (uint8_t i = 0; i < trackerCount; i++) {
		memcpy(macs + 6 * i, trackers[i].mac, 6);
	}
	prefs.putUChar("ch", channel);
	prefs.putBytes("key", key, sizeof(key));
	prefs.putUChar("n", trackerCount);
	prefs.putBytes("macs", macs, sizeof(macs));
}

void loadSettings() {
	prefs.begin("picodongle", false);
	channel = prefs.getUChar("ch", DEFAULT_CHANNEL);
	if (!PicoLink::isValidChannel(channel)) {
		channel = DEFAULT_CHANNEL;
	}
	if (prefs.getBytes("key", key, sizeof(key)) != sizeof(key)) {
		// First boot: a fresh pairing key.
		esp_fill_random(key, sizeof(key));
		prefs.putBytes("key", key, sizeof(key));
	}
	trackerCount = prefs.getUChar("n", 0);
	if (trackerCount > PicoLink::MAX_TRACKERS) {
		trackerCount = 0;
	}
	uint8_t macs[6 * PicoLink::MAX_TRACKERS] = {0};
	prefs.getBytes("macs", macs, sizeof(macs));
	for (uint8_t i = 0; i < trackerCount; i++) {
		memcpy(trackers[i].mac, macs + 6 * i, 6);
		trackers[i].rssi = 0;
		trackers[i].lastHeardAt = 0;
	}
	netId = PicoLink::netIdFromKey(key);
}

int findTracker(const uint8_t* mac) {
	for (uint8_t i = 0; i < trackerCount; i++) {
		if (memcmp(trackers[i].mac, mac, 6) == 0) {
			return i;
		}
	}
	return -1;
}

// ---------------------------------------------------------------- USB frames

void sendFrame(uint8_t type, const uint8_t* body, size_t length) {
	uint8_t raw[1 + 6 + 1 + PicoLink::MAX_PAYLOAD + 2];
	if (length + 3 > sizeof(raw)) {
		return;
	}
	raw[0] = type;
	if (length > 0) {
		memcpy(raw + 1, body, length);
	}
	uint16_t crc = Framing::crc16(raw, length + 1);
	raw[length + 1] = crc & 0xFF;
	raw[length + 2] = crc >> 8;

	uint8_t encoded[Framing::encodedSize(sizeof(raw)) + 1];
	size_t n = Framing::cobsEncode(raw, length + 3, encoded);
	encoded[n++] = 0;

	// Never block: with nobody reading the port, the frame is simply dropped.
	if (Serial.availableForWrite() >= (int)n) {
		Serial.write(encoded, n);
	}
}

void sendResult(uint8_t command, uint8_t status) {
	uint8_t body[2] = {command, status};
	sendFrame(DongleProtocol::MSG_RESULT, body, sizeof(body));
}

void sendLog(const char* text) {
	sendFrame(DongleProtocol::MSG_LOG, (const uint8_t*)text, strlen(text));
}

void sendHello() {
	uint8_t body[DongleProtocol::HELLO_MAGIC_LENGTH + 1 + 3 + 6 + 1 + 1 + 1 + 4];
	size_t at = 0;
	memcpy(body, DongleProtocol::HELLO_MAGIC, DongleProtocol::HELLO_MAGIC_LENGTH);
	at += DongleProtocol::HELLO_MAGIC_LENGTH;
	body[at++] = PicoLink::LINK_VERSION;
	body[at++] = FIRMWARE_MAJOR;
	body[at++] = FIRMWARE_MINOR;
	body[at++] = FIRMWARE_PATCH;
	memcpy(body + at, ownMac, 6);
	at += 6;
	body[at++] = channel;
	body[at++] = trackerCount;
	body[at++] = PicoLink::MAX_TRACKERS;
	for (int i = 0; i < 4; i++) {
		body[at++] = (netId >> (8 * i)) & 0xFF;
	}
	sendFrame(DongleProtocol::MSG_HELLO, body, at);
}

void sendList() {
	uint8_t body[1 + 9 * PicoLink::MAX_TRACKERS];
	size_t at = 0;
	uint32_t now = millis();
	portENTER_CRITICAL(&trackersLock);
	body[at++] = trackerCount;
	for (uint8_t i = 0; i < trackerCount; i++) {
		memcpy(body + at, trackers[i].mac, 6);
		at += 6;
		body[at++] = (uint8_t)trackers[i].rssi;
		uint32_t age = trackers[i].lastHeardAt == 0
						 ? 0xFFFF
						 : (now - trackers[i].lastHeardAt) / 1000;
		if (age > 0xFFFF) {
			age = 0xFFFF;
		}
		body[at++] = age & 0xFF;
		body[at++] = age >> 8;
	}
	portEXIT_CRITICAL(&trackersLock);
	sendFrame(DongleProtocol::MSG_LIST, body, at);
}

// ---------------------------------------------------------------- radio

void onReceive(const esp_now_recv_info_t* info, const uint8_t* data, int length) {
	// Wi-Fi task: copy and get out.
	if (info == nullptr || length < (int)sizeof(PicoLink::FrameHeader)) {
		return;
	}
	PicoLink::FrameHeader header;
	memcpy(&header, data, sizeof(header));
	if (header.type != PicoLink::FRAME_DATA_UP || header.version != PicoLink::LINK_VERSION
		|| header.netId != netId) {
		return;
	}

	int8_t rssi = info->rx_ctrl != nullptr ? info->rx_ctrl->rssi : 0;
	bool allowed = false;
	portENTER_CRITICAL(&trackersLock);
	int index = findTracker(info->src_addr);
	if (index >= 0) {
		allowed = true;
		trackers[index].rssi = rssi;
		trackers[index].lastHeardAt = millis();
	}
	portEXIT_CRITICAL(&trackersLock);
	if (!allowed) {
		return;
	}

	RxItem item;
	memcpy(item.mac, info->src_addr, 6);
	item.rssi = rssi;
	item.length = length - sizeof(PicoLink::FrameHeader);
	memcpy(item.data, data + sizeof(PicoLink::FrameHeader), item.length);
	xQueueSend(rxQueue, &item, 0);
}

void onScanPacket(void* buffer, wifi_promiscuous_pkt_type_t type) {
	(void)buffer;
	(void)type;
	scanCount = scanCount + 1;
}

void setRate(const uint8_t* mac) {
	esp_now_rate_config_t rate = {};
	rate.phymode = WIFI_PHY_MODE_11G;
	rate.rate = WIFI_PHY_RATE_24M;
	esp_now_set_peer_rate_config(mac, &rate);
}

bool addPeer(const uint8_t* mac) {
	if (esp_now_is_peer_exist(mac)) {
		return true;
	}
	esp_now_peer_info_t peer = {};
	memcpy(peer.peer_addr, mac, 6);
	peer.channel = 0;  // follow the radio
	peer.ifidx = WIFI_IF_STA;
	peer.encrypt = false;
	if (esp_now_add_peer(&peer) != ESP_OK) {
		return false;
	}
	setRate(mac);
	return true;
}

void setChannel(uint8_t newChannel) {
	channel = newChannel;
	esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

bool startRadio() {
	WiFi.mode(WIFI_STA);
	WiFi.disconnect(false, false);
	esp_wifi_set_ps(WIFI_PS_NONE);
	setChannel(channel);
	if (esp_now_init() != ESP_OK) {
		return false;
	}
	esp_now_register_recv_cb(onReceive);
	if (!addPeer(BROADCAST)) {
		return false;
	}
	for (uint8_t i = 0; i < trackerCount; i++) {
		addPeer(trackers[i].mac);
	}
	return true;
}

void sendBeacon() {
	PicoLink::Beacon beacon;
	beacon.header.type = PicoLink::FRAME_BEACON;
	beacon.header.version = PicoLink::LINK_VERSION;
	beacon.header.netId = netId;
	beacon.channel = channel;
	beacon.trackerCount = trackerCount;
	esp_now_send(BROADCAST, (const uint8_t*)&beacon, sizeof(beacon));
}

// How busy is each channel? Count every frame heard for a moment on each.
void scanChannels() {
	uint8_t body[2 * PicoLink::MAX_CHANNEL];
	uint8_t home = channel;
	esp_wifi_set_promiscuous_rx_cb(onScanPacket);
	esp_wifi_set_promiscuous(true);
	for (uint8_t ch = PicoLink::MIN_CHANNEL; ch <= PicoLink::MAX_CHANNEL; ch++) {
		esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
		scanCount = 0;
		delay(SCAN_DWELL_MS);
		uint32_t count = scanCount > 0xFFFF ? 0xFFFF : scanCount;
		body[2 * (ch - 1)] = count & 0xFF;
		body[2 * (ch - 1) + 1] = count >> 8;
	}
	esp_wifi_set_promiscuous(false);
	setChannel(home);
	sendFrame(DongleProtocol::MSG_SCAN, body, sizeof(body));
}

// ---------------------------------------------------------------- commands

void handleCommand(const uint8_t* frame, size_t length) {
	uint8_t command = frame[0];
	const uint8_t* body = frame + 1;
	size_t bodyLength = length - 1;

	switch (command) {
		case DongleProtocol::CMD_HELLO:
			sendHello();
			return;

		case DongleProtocol::CMD_ALLOW: {
			if (bodyLength != 6) {
				break;
			}
			if (findTracker(body) >= 0) {
				sendResult(command, DongleProtocol::STATUS_OK);
				return;
			}
			if (trackerCount >= PicoLink::MAX_TRACKERS) {
				sendResult(command, DongleProtocol::STATUS_FULL);
				return;
			}
			if (!addPeer(body)) {
				sendResult(command, DongleProtocol::STATUS_RADIO_ERROR);
				return;
			}
			portENTER_CRITICAL(&trackersLock);
			memcpy(trackers[trackerCount].mac, body, 6);
			trackers[trackerCount].rssi = 0;
			trackers[trackerCount].lastHeardAt = 0;
			trackerCount++;
			portEXIT_CRITICAL(&trackersLock);
			saveSettings();
			sendResult(command, DongleProtocol::STATUS_OK);
			return;
		}

		case DongleProtocol::CMD_REMOVE: {
			if (bodyLength != 6) {
				break;
			}
			portENTER_CRITICAL(&trackersLock);
			int index = findTracker(body);
			if (index >= 0) {
				for (uint8_t i = index; i + 1 < trackerCount; i++) {
					trackers[i] = trackers[i + 1];
				}
				trackerCount--;
			}
			portEXIT_CRITICAL(&trackersLock);
			if (index < 0) {
				sendResult(command, DongleProtocol::STATUS_NOT_FOUND);
				return;
			}
			esp_now_del_peer(body);
			saveSettings();
			sendResult(command, DongleProtocol::STATUS_OK);
			return;
		}

		case DongleProtocol::CMD_SET_CHANNEL:
			if (bodyLength != 1 || !PicoLink::isValidChannel(body[0])) {
				break;
			}
			// Trackers lose the beacon and hunt for it on the new channel.
			setChannel(body[0]);
			saveSettings();
			sendResult(command, DongleProtocol::STATUS_OK);
			return;

		case DongleProtocol::CMD_SEND: {
			if (bodyLength <= 6 || bodyLength - 6 > PicoLink::MAX_PAYLOAD) {
				break;
			}
			if (findTracker(body) < 0) {
				sendResult(command, DongleProtocol::STATUS_NOT_FOUND);
				return;
			}
			uint8_t out[PicoLink::MAX_FRAME];
			PicoLink::FrameHeader header;
			header.type = PicoLink::FRAME_DATA_DOWN;
			header.version = PicoLink::LINK_VERSION;
			header.netId = netId;
			memcpy(out, &header, sizeof(header));
			memcpy(out + sizeof(header), body + 6, bodyLength - 6);
			// No result on success: the server sends many of these.
			if (esp_now_send(body, out, sizeof(header) + bodyLength - 6) != ESP_OK) {
				sendResult(command, DongleProtocol::STATUS_RADIO_ERROR);
			}
			return;
		}

		case DongleProtocol::CMD_LIST:
			sendList();
			return;

		case DongleProtocol::CMD_GET_KEY:
			sendFrame(DongleProtocol::MSG_KEY, key, sizeof(key));
			return;

		case DongleProtocol::CMD_SET_KEY:
			if (bodyLength != sizeof(key)) {
				break;
			}
			memcpy(key, body, sizeof(key));
			netId = PicoLink::netIdFromKey(key);
			saveSettings();
			sendResult(command, DongleProtocol::STATUS_OK);
			return;

		case DongleProtocol::CMD_SCAN:
			scanChannels();
			return;

		case DongleProtocol::CMD_NEW_PAIRING:
			portENTER_CRITICAL(&trackersLock);
			for (uint8_t i = 0; i < trackerCount; i++) {
				esp_now_del_peer(trackers[i].mac);
			}
			trackerCount = 0;
			portEXIT_CRITICAL(&trackersLock);
			esp_fill_random(key, sizeof(key));
			netId = PicoLink::netIdFromKey(key);
			saveSettings();
			sendResult(command, DongleProtocol::STATUS_OK);
			return;

		default:
			sendResult(command, DongleProtocol::STATUS_UNKNOWN_COMMAND);
			return;
	}
	sendResult(command, DongleProtocol::STATUS_BAD_ARGUMENTS);
}

void handleSerialFrame() {
	uint8_t frame[sizeof(serialIn)];
	size_t length = Framing::cobsDecode(serialIn, serialInLength, frame, sizeof(frame));
	if (length < 3) {
		return;
	}
	uint16_t expected = frame[length - 2] | (frame[length - 1] << 8);
	if (Framing::crc16(frame, length - 2) != expected) {
		return;
	}
	handleCommand(frame, length - 2);
}

void readSerial() {
	while (Serial.available() > 0) {
		int c = Serial.read();
		if (c < 0) {
			return;
		}
		if (c == 0) {
			if (serialInLength > 0 && !serialInOverflow) {
				handleSerialFrame();
			}
			serialInLength = 0;
			serialInOverflow = false;
		} else if (serialInLength < sizeof(serialIn)) {
			serialIn[serialInLength++] = c;
		} else {
			// Too long to be a frame: drop it and wait for the next delimiter.
			serialInOverflow = true;
		}
	}
}

void forwardTrackerPackets() {
	RxItem item;
	while (xQueueReceive(rxQueue, &item, 0) == pdTRUE) {
		uint8_t body[6 + 1 + PicoLink::MAX_PAYLOAD];
		memcpy(body, item.mac, 6);
		body[6] = (uint8_t)item.rssi;
		memcpy(body + 7, item.data, item.length);
		sendFrame(DongleProtocol::MSG_DATA, body, 7 + item.length);
	}
}

}  // namespace

void setup() {
	Serial.begin(115200);
	Serial.setTxTimeoutMs(0);

	rxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(RxItem));
	loadSettings();
	radioReady = startRadio();
	WiFi.macAddress(ownMac);

	sendLog(radioReady ? "PicoVR Dongle ready" : "PicoVR Dongle: radio failed to start");
}

void loop() {
	readSerial();
	forwardTrackerPackets();

	uint32_t now = millis();
	if (radioReady && now - lastBeaconAt >= PicoLink::BEACON_INTERVAL_MS) {
		lastBeaconAt = now;
		sendBeacon();
	}
	delay(1);
}
