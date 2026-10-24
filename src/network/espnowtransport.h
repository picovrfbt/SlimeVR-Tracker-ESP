/*
	Talks to the server through a PicoVR Dongle over ESP-NOW (ESP32 only).

	Packets to the server go to the bound dongle inside FRAME_DATA_UP frames;
	server packets come back as FRAME_DATA_DOWN. The dongle's beacon says it is
	in range. When the beacon goes missing the transport hops through channels
	1-13 until it hears the dongle again, so the dongle can change channel
	without rebinding.
*/

#ifndef SLIMEVR_NETWORK_ESPNOWTRANSPORT_H_
#define SLIMEVR_NETWORK_ESPNOWTRANSPORT_H_

#include <Arduino.h>

#include "picolink.h"
#include "transport.h"

#if ESP32
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#endif

namespace SlimeVR {
namespace Network {

class EspNowTransport : public Transport {
public:
	// Wi-Fi must not be connected: ESP-NOW needs the radio on the dongle's
	// channel. Returns false if the radio could not be started.
	bool start(const uint8_t dongleMac[6], uint8_t channel, const uint8_t key[16]);
	void stop();
	bool isRunning() const { return m_Running; }

	// Call every loop.
	void upkeep();

	bool dongleHeard() const;
	uint32_t msSinceBeacon() const;
	uint32_t msSinceStart() const { return millis() - m_StartedAt; }
	int8_t rssi() const { return m_LastRssi; }
	uint8_t channel() const { return m_Channel; }

	void begin() override;
	bool beginPacket() override;
	size_t write(const uint8_t* buffer, size_t size) override;
	bool endPacket() override;
	int getWriteError() override;
	int parsePacket() override;
	int read(uint8_t* buffer, size_t size) override;
	const char* remoteName() override;

private:
	struct RxItem {
		uint8_t len;
		uint8_t data[PicoLink::MAX_PAYLOAD];
	};

	void handleFrame(const uint8_t* mac, const uint8_t* data, int len, int8_t rssi);
	void setRadioChannel(uint8_t channel);

	bool m_Running = false;
	uint8_t m_DongleMac[6] = {0};
	uint32_t m_NetId = 0;
	volatile uint8_t m_Channel = 0;
	volatile uint32_t m_LastBeaconAt = 0;
	volatile bool m_EverHeard = false;
	volatile int8_t m_LastRssi = 0;
	uint32_t m_StartedAt = 0;

	// Channel hunting while the beacon is missing.
	uint32_t m_LastHopAt = 0;
	uint8_t m_HuntChannel = 0;

	uint8_t m_TxBuffer[PicoLink::MAX_FRAME];
	size_t m_TxLength = 0;
	bool m_TxOverflow = false;
	volatile uint32_t m_SendFailures = 0;

	RxItem m_Pending;
	bool m_HasPending = false;
	char m_RemoteName[24] = {0};

#if ESP32
	QueueHandle_t m_RxQueue = nullptr;
#endif

	friend struct EspNowCallbacks;
};

}  // namespace Network
}  // namespace SlimeVR

#endif  // SLIMEVR_NETWORK_ESPNOWTRANSPORT_H_
