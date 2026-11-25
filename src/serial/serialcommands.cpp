/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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

#include "serialcommands.h"

#include <CmdCallback.hpp>

#include "GlobalVars.h"
#include "base64.hpp"
#include "batterymonitor.h"
#include "logging/Logger.h"
#include "utils.h"

#if ESP32
#include "nvs_flash.h"
#endif

namespace SerialCommands {
SlimeVR::Logging::Logger logger("SerialCommands");

CmdCallback<6> cmdCallbacks;
CmdParser cmdParser;
CmdBuffer<256> cmdBuffer;

bool lengthCheck(
	const char* const text,
	unsigned int length,
	const char* const cmd,
	const char* const name
) {
	size_t l = text != nullptr ? strlen(text) : 0;
	if ((l > length)) {
		logger.error(
			"%s ERROR: %s is longer than %d bytes / Characters",
			cmd,
			name,
			length
		);
		return false;
	}
	return true;
}

// Hex text to bytes. Accepts "34B4721A0CE8" or "34:B4:72:1A:0C:E8".
bool parseHex(const char* text, uint8_t* out, size_t length) {
	if (text == nullptr) {
		return false;
	}
	size_t count = 0;
	int high = -1;
	for (const char* c = text; *c != '\0'; c++) {
		if (*c == ':' || *c == '-') {
			continue;
		}
		int value;
		if (*c >= '0' && *c <= '9') {
			value = *c - '0';
		} else if (*c >= 'a' && *c <= 'f') {
			value = *c - 'a' + 10;
		} else if (*c >= 'A' && *c <= 'F') {
			value = *c - 'A' + 10;
		} else {
			return false;
		}
		if (high < 0) {
			high = value;
		} else {
			if (count >= length) {
				return false;
			}
			out[count++] = (high << 4) | value;
			high = -1;
		}
	}
	return count == length && high < 0;
}

unsigned int
decode_base64_length_null(const char* const b64char, unsigned int* b64ssidlength) {
	if (b64char == NULL) {
		return 0;
	}
	*b64ssidlength = (unsigned int)strlen(b64char);
	return decode_base64_length((unsigned char*)b64char, *b64ssidlength);
}

void cmdSet(CmdParser* parser) {
	if (parser->getParamCount() != 1) {
		if (parser->equalCmdParam(1, "WIFI")) {
			if (parser->getParamCount() < 3) {
				logger.error("CMD SET WIFI ERROR: Too few arguments");
				logger.info("Syntax: SET WIFI \"<SSID>\" \"<PASSWORD>\"");
			} else {
				const char* sc_ssid = parser->getCmdParam(2);
				const char* sc_pw = parser->getCmdParam(3);

				if (!lengthCheck(sc_ssid, 32, "CMD SET WIFI", "SSID")
					&& !lengthCheck(sc_pw, 64, "CMD SET WIFI", "Password")) {
					return;
				}

				networkManager.useWifi();
				WiFiNetwork::setWiFiCredentials(sc_ssid, sc_pw);
				logger.info("CMD SET WIFI OK: New wifi credentials set, reconnecting");
			}
		} else if (parser->equalCmdParam(1, "BWIFI")) {
			if (parser->getParamCount() < 3) {
				logger.error("CMD SET BWIFI ERROR: Too few arguments");
				logger.info("Syntax: SET BWIFI <B64SSID> <B64PASSWORD>");
			} else {
				const char* b64ssid = parser->getCmdParam(2);
				const char* b64pass = parser->getCmdParam(3);
				unsigned int b64ssidlength = 0;
				unsigned int b64passlength = 0;
				unsigned int ssidlength
					= decode_base64_length_null(b64ssid, &b64ssidlength);
				unsigned int passlength
					= decode_base64_length_null(b64pass, &b64passlength);

				// alloc the strings and set them to 0 (null terminating)
				char ssid[ssidlength + 1];
				memset(ssid, 0, ssidlength + 1);
				char pass[passlength + 1];
				memset(pass, 0, passlength + 1);
				// make a pointer to pass
				char* ppass = pass;
				decode_base64(
					(const unsigned char*)b64ssid,
					b64ssidlength,
					(unsigned char*)ssid
				);
				if (!lengthCheck(ssid, 32, "CMD SET BWIFI", "SSID")) {
					return;
				}

				if ((b64pass != NULL) && (b64passlength > 0)) {
					decode_base64(
						(const unsigned char*)b64pass,
						b64passlength,
						(unsigned char*)pass
					);
					if (!lengthCheck(pass, 64, "CMD SET BWIFI", "Password")) {
						return;
					}
				} else {
					// set the pointer for pass to null for no password
					ppass = NULL;
				}
				networkManager.useWifi();
				WiFiNetwork::setWiFiCredentials(ssid, ppass);
				logger.info("CMD SET BWIFI OK: New wifi credentials set, reconnecting");
			}
		} else if (parser->equalCmdParam(1, "DONGLE")) {
			// SET DONGLE <dongle address> <channel> <32 hex digit pairing key>
			uint8_t mac[6];
			uint8_t key[PicoLink::KEY_SIZE];
			int channel = parser->getParamCount() >= 5 ? atoi(parser->getCmdParam(3)) : 0;
			if (parser->getParamCount() < 5 || !parseHex(parser->getCmdParam(2), mac, 6)
				|| !PicoLink::isValidChannel(channel)
				|| !parseHex(parser->getCmdParam(4), key, sizeof(key))) {
				logger.error("CMD SET DONGLE ERROR: Invalid arguments");
				logger.info("Syntax: SET DONGLE <ADDRESS> <CHANNEL 1-13> <KEY 32 HEX>");
			} else if (networkManager.bindDongle(mac, channel, key)) {
				logger.info("CMD SET DONGLE OK: Bound to the dongle, mode AUTO");
			} else {
				logger.error("CMD SET DONGLE ERROR: Could not start the dongle radio");
			}
		} else if (parser->equalCmdParam(1, "MODE")) {
			// SET MODE WIFI | DONGLE | AUTO
			SlimeVR::Network::LinkMode mode;
			if (!SlimeVR::Network::LinkConfig::parseMode(parser->getCmdParam(2), mode)) {
				logger.error("CMD SET MODE ERROR: Unknown mode");
				logger.info("Syntax: SET MODE <WIFI|DONGLE|AUTO>");
			} else if (networkManager.setMode(mode)) {
				logger.info(
					"CMD SET MODE OK: %s",
					SlimeVR::Network::LinkConfig::modeName(mode)
				);
			} else {
				logger.error("CMD SET MODE ERROR: No dongle bound, use SET DONGLE first");
			}
		} else {
			logger.error("CMD SET ERROR: Unrecognized variable to set");
		}
	} else {
		logger.error("CMD SET ERROR: No variable to set");
	}
}

void printState() {
	logger.info(
		"SlimeVR Tracker, board: %d, hardware: %d, protocol: %d, firmware: %s, "
		"address: "
		"%s, mac: %s, status: %d, wifi state: %d, link: %s",
		BOARD,
		HARDWARE_MCU,
		PROTOCOL_VERSION,
		FIRMWARE_VERSION,
		WiFiNetwork::getAddress().toString().c_str(),
		WiFi.macAddress().c_str(),
		statusManager.getStatus(),
		WiFiNetwork::getWiFiState(),
		networkManager.isDongleActive() ? "DONGLE" : "WIFI"
	);
	for (auto& sensor : sensorManager.getSensors()) {
		logger.info(
			"Sensor[%d]: %s (%.3f %.3f %.3f %.3f) is working: %s, had data: %s",
			sensor->getSensorId(),
			getIMUNameByType(sensor->getSensorType()),
			UNPACK_QUATERNION(sensor->getFusedRotation()),
			sensor->isWorking() ? "true" : "false",
			sensor->getHadData() ? "true" : "false"
		);
	}
	logger.info(
		"Battery voltage: %.3f, level: %.1f%%",
		battery.getVoltage(),
		battery.getLevel() * 100
	);
}

void cmdGet(CmdParser* parser) {
	if (parser->getParamCount() < 2) {
		return;
	}

	if (parser->equalCmdParam(1, "INFO")) {
		printState();

		// We don't want to print this on every timed state output
		logger.info("Git commit: %s", GIT_REV);
	}

	if (parser->equalCmdParam(1, "LINK")) {
		networkManager.printLinkState();
	}

	if (parser->equalCmdParam(1, "CONFIG")) {
		String str
			= "BOARD=%d\n"
			  "IMU=%d\n"
			  "SECOND_IMU=%d\n"
			  "IMU_ROTATION=%f\n"
			  "SECOND_IMU_ROTATION=%f\n"
			  "BATTERY_MONITOR=%d\n"
			  "BATTERY_SHIELD_RESISTANCE=%d\n"
			  "BATTERY_SHIELD_R1=%d\n"
			  "BATTERY_SHIELD_R2=%d\n"
			  "PIN_IMU_SDA=%d\n"
			  "PIN_IMU_SCL=%d\n"
			  "PIN_IMU_INT=%d\n"
			  "PIN_IMU_INT_2=%d\n"
			  "PIN_BATTERY_LEVEL=%d\n"
			  "LED_PIN=%d\n"
			  "LED_INVERTED=%d\n";

		Serial.printf(
			str.c_str(),
			BOARD,
			static_cast<int>(sensorManager.getSensorType(0)),
			static_cast<int>(sensorManager.getSensorType(1)),
			IMU_ROTATION,
			SECOND_IMU_ROTATION,
			BATTERY_MONITOR,
			BATTERY_SHIELD_RESISTANCE,
			BATTERY_SHIELD_R1,
			BATTERY_SHIELD_R2,
			PIN_IMU_SDA,
			PIN_IMU_SCL,
			PIN_IMU_INT,
			PIN_IMU_INT_2,
			PIN_BATTERY_LEVEL,
			LED_PIN,
			LED_INVERTED
		);
	}

	if (parser->equalCmdParam(1, "TEST")) {
		logger.info(
			"[TEST] Board: %d, hardware: %d, protocol: %d, firmware: %s, address: %s, "
			"mac: %s, status: %d, wifi state: %d",
			BOARD,
			HARDWARE_MCU,
			PROTOCOL_VERSION,
			FIRMWARE_VERSION,
			WiFiNetwork::getAddress().toString().c_str(),
			WiFi.macAddress().c_str(),
			statusManager.getStatus(),
			WiFiNetwork::getWiFiState()
		);
		auto& sensor0 = sensorManager.getSensors()[0];
		sensor0->motionLoop();
		logger.info(
			"[TEST] Sensor[0]: %s (%.3f %.3f %.3f %.3f) is working: %s, had data: %s",
			getIMUNameByType(sensor0->getSensorType()),
			UNPACK_QUATERNION(sensor0->getFusedRotation()),
			sensor0->isWorking() ? "true" : "false",
			sensor0->getHadData() ? "true" : "false"
		);
		if (!sensor0->getHadData()) {
			logger.error("[TEST] Sensor[0] didn't send any data yet!");
		} else {
			logger.info("[TEST] Sensor[0] sent some data, looks working.");
		}
	}

	// GET WIFI - the network this tracker is on, so the app can put it back
	// after a flash.
	//
	// Flashing writes the whole chip and the credentials go with it, so the
	// only way a tracker comes back on the network by itself is if something
	// read them off it first. Nothing else could: the app only knew a network
	// if the player had typed one into it, and anybody who set their trackers
	// up through the web builder never did, so their trackers came back mute
	// and had to be redone by hand.
	//
	// Base64 for both, matching SET BWIFI, so the pair survives spaces, quotes
	// and anything else a network name is allowed to contain and round-trips
	// byte for byte.
	//
	// This does put the password on the serial line in a recoverable form.
	// That is the point - it is the only way back - but it means a serial log
	// captured while this runs contains it, so whatever reads this should not
	// write it down.
	if (parser->equalCmdParam(1, "WIFI")) {
		String ssid = WiFi.SSID();
		String psk = WiFi.psk();

		if (ssid.length() == 0) {
			logger.info("[WIFI] NONE");
			return;
		}

		unsigned char b64ssid[64];
		unsigned char b64psk[128];
		unsigned int ssidLen = encode_base64(
			(unsigned char*)ssid.c_str(),
			ssid.length(),
			b64ssid
		);
		unsigned int pskLen = encode_base64(
			(unsigned char*)psk.c_str(),
			psk.length(),
			b64psk
		);
		(void)ssidLen;
		(void)pskLen;

		logger.info("[WIFI] B64 %s %s", (char*)b64ssid, (char*)b64psk);
		return;
	}

	if (parser->equalCmdParam(1, "WIFISCAN")) {
		logger.info("[WSCAN] Scanning for WiFi networks...");

		// Scan would fail if connecting, stop connecting before scan
		if (WiFi.status() != WL_CONNECTED) {
			WiFi.disconnect();
		}
		if (WiFiNetwork::isProvisioning()) {
			WiFiNetwork::stopProvisioning();
		}

		WiFi.scanNetworks();

		int scanRes = WiFi.scanComplete();
		if (scanRes >= 0) {
			logger.info("[WSCAN] Found %d networks:", scanRes);
			for (int i = 0; i < scanRes; i++) {
				logger.info(
					"[WSCAN] %d:\t%02d\t%s\t(%d)\t%s",
					i,
					WiFi.SSID(i).length(),
					WiFi.SSID(i).c_str(),
					WiFi.RSSI(i),
					((WiFi.encryptionType(i) == 0) ? "OPEN" : "PASSWD")
				);
			}
			WiFi.scanDelete();
		} else {
			logger.info("[WSCAN] Scan failed!");
		}

		// Restore conencting state
		if (WiFi.status() != WL_CONNECTED) {
			WiFi.begin();
		}
	}
}

void cmdReboot(CmdParser* parser) {
	logger.info("REBOOT");
	ESP.restart();
}

void cmdFactoryReset(CmdParser* parser) {
	logger.info("FACTORY RESET");

	configuration.reset();

	WiFi.disconnect(true);  // Clear WiFi credentials
#if ESP8266
	ESP.eraseConfig();  // Clear ESP config
#elif ESP32
	nvs_flash_erase();
#else
#warning SERIAL COMMAND FACTORY RESET NOT SUPPORTED
	logger.info("FACTORY RESET NOT SUPPORTED");
	return;
#endif

#if defined(WIFI_CREDS_SSID) && defined(WIFI_CREDS_PASSWD)
#warning FACTORY RESET does not clear your hardcoded WiFi credentials!
	logger.warn("FACTORY RESET does not clear your hardcoded WiFi credentials!");
#endif

	delay(3000);
	ESP.restart();
}

void cmdTemperatureCalibration(CmdParser* parser) {
	if (parser->getParamCount() > 1) {
		if (parser->equalCmdParam(1, "PRINT")) {
			for (auto& sensor : sensorManager.getSensors()) {
				sensor->printTemperatureCalibrationState();
			}
			return;
		} else if (parser->equalCmdParam(1, "DEBUG")) {
			for (auto& sensor : sensorManager.getSensors()) {
				sensor->printDebugTemperatureCalibrationState();
			}
			return;
		} else if (parser->equalCmdParam(1, "RESET")) {
			for (auto& sensor : sensorManager.getSensors()) {
				sensor->resetTemperatureCalibrationState();
			}
			return;
		} else if (parser->equalCmdParam(1, "SAVE")) {
			for (auto& sensor : sensorManager.getSensors()) {
				sensor->saveTemperatureCalibration();
			}
			return;
		}
	}
	logger.info("Usage:");
	logger.info("  TCAL PRINT: print current temperature calibration config");
	logger.info(
		"  TCAL DEBUG: print debug values for the current temperature calibration "
		"profile"
	);
	logger.info(
		"  TCAL RESET: reset current temperature calibration in RAM (does not delete "
		"already saved)"
	);
	logger.info("  TCAL SAVE: save current temperature calibration to persistent flash"
	);
	logger.info("Note:");
	logger.info(
		"  Temperature calibration config saves automatically when calibration percent "
		"is at 100%"
	);
}

void cmdDeleteCalibration(CmdParser* parser) {
	logger.info("ERASE CALIBRATION");

	configuration.eraseSensors();
}

void setUp() {
	cmdCallbacks.addCmd("SET", &cmdSet);
	cmdCallbacks.addCmd("GET", &cmdGet);
	cmdCallbacks.addCmd("FRST", &cmdFactoryReset);
	cmdCallbacks.addCmd("DELCAL", &cmdDeleteCalibration);
	cmdCallbacks.addCmd("REBOOT", &cmdReboot);
	cmdCallbacks.addCmd("TCAL", &cmdTemperatureCalibration);
}

void update() { cmdCallbacks.updateCmdProcessing(&cmdParser, &cmdBuffer, &Serial); }
}  // namespace SerialCommands
