# PicoVR Dongle

A USB dongle that up to **8 trackers** talk to over ESP-NOW, instead of Wi-Fi.
It runs on a **XIAO ESP32-C3** (any ESP32-C3 board with native USB works) and
passes the trackers' packets to the PicoVR server over USB serial, unchanged.

Trackers keep both radio paths in one firmware. A saved setting picks the
link, so switching between Wi-Fi and the dongle never needs a reflash; only
the one-time update to this dual-mode firmware does.

## Link modes (tracker)

| Mode | What it does |
|---|---|
| `WIFI` | Wi-Fi only, exactly as before. The default. |
| `DONGLE` | The bound dongle only. Never joins Wi-Fi. |
| `AUTO` | The dongle; if it is missing for **30 s** and a Wi-Fi network is saved, uses Wi-Fi. While on that fallback Wi-Fi with no server, it listens for the dongle for 3 s every 20 s. With a server, the server switches it back. |

Binding (`SET DONGLE`) selects `AUTO`. `AUTO` without saved Wi-Fi behaves
like `DONGLE`.

The mode, dongle address, channel and pairing key live in `/link.bin`
(LittleFS), apart from the main configuration. A factory reset (`FRST`) erases
it, which puts the tracker back on Wi-Fi. The Wi-Fi network stays saved by the
radio in every mode.

### Switching

| How | Command |
|---|---|
| USB: bind to a dongle | `SET DONGLE <address> <channel> <key>`, e.g. `SET DONGLE 34B4721A0CE8 11 00112233445566778899AABBCCDDEEFF` |
| USB: pick a mode | `SET MODE WIFI` / `SET MODE DONGLE` / `SET MODE AUTO` (dongle modes need a bound dongle) |
| USB: back to Wi-Fi | `SET WIFI "<ssid>" "<password>"` or `SET BWIFI ...` also set mode `WIFI` |
| USB: show it | `GET LINK`; `GET INFO` ends with `link: WIFI` or `link: DONGLE` |
| Server, no cable | `PACKET_PICOVR_SET_LINK` (200): type (4) + packet number (8) + mode (1), optionally + dongle address (6) + channel (1) + key (16) to bind first |

### How the tracker code is laid out

- `src/network/transport.h`: what `Connection` needs from a link.
- `udptransport.*`: the original Wi-Fi/UDP path.
- `espnowtransport.*`: the dongle path. Wraps each server packet in a
  `FRAME_DATA_UP`, receives `FRAME_DATA_DOWN`, tracks the beacon, and hops
  through channels 1–13 (250 ms each) when the beacon is lost for 1.5 s.
- `linkconfig.*`: `/link.bin`.
- `manager.*`: the mode state machine; owns both transports and hands the
  active one to `Connection`, which then rediscovers the server over it.
- `picolink.h`: the radio frame format, shared with the dongle.

The server protocol on top is untouched: handshake, sensor info, rotation,
bundles, feature flags and config flags all work the same over the dongle.

## Radio (ESP-NOW)

Frames start with `type (1) + link version (1) + network id (4)`. The network
id is an FNV-1a hash of the dongle's pairing key; anything with another id is
ignored, so a neighbour's dongle and trackers never mix with yours. It is not
a secret.

| Frame | From | Body |
|---|---|---|
| `FRAME_DATA_UP` (1) | tracker | one server protocol packet |
| `FRAME_DATA_DOWN` (2) | dongle | one server protocol packet |
| `FRAME_BEACON` (3) | dongle, broadcast, every 100 ms | channel (1) + trackers bound (1) |

ESP-NOW runs at **24 Mbps** on both sides. At the 1 Mbps default, eight
moving trackers at 120 Hz would need more airtime than the channel has; at
24 Mbps they use roughly 20% (estimate, to be measured).

## USB protocol (dongle ⇄ PC)

COBS frames ended by `0x00`, each `type (1) + body + CRC-16/CCITT-FALSE (2, LE)`.
Bad frames are dropped, so boot messages on the port are harmless. Details in
[`src/dongleprotocol.h`](src/dongleprotocol.h).

The dongle shows up with the same USB ID as a tracker (303A:1001). The server
sends `CMD_HELLO` when it opens the port; only a dongle answers
(`MSG_HELLO`, starting with `PICOVR-DONGLE`).

| PC → dongle | | Dongle → PC | |
|---|---|---|---|
| `HELLO` | identify | `HELLO` | magic, versions, address, channel, trackers, network id |
| `ALLOW` addr | add to the allowed list (max 8) | `DATA` | tracker addr + RSSI + packet |
| `REMOVE` addr | drop a tracker | `RESULT` | command + status |
| `SET_CHANNEL` ch | move channel; trackers follow | `LIST` | allowed trackers, RSSI, seconds since heard |
| `SEND` addr + packet | to a tracker | `KEY` | pairing key |
| `LIST`, `GET_KEY`, `SET_KEY` | | `SCAN` | frames heard on channels 1–13 |
| `SCAN` | how busy is each channel | `LOG` | text |
| `NEW_PAIRING` | new key, forget all trackers | | |
| `OTA_BEGIN` size + sha256 | take new firmware | | |
| `OTA_CHUNK` offset + bytes | up to 1024 bytes, in order | | |
| `OTA_END` | check it and restart into it | | |
| `OTA_ABORT` | forget it | | |

Channel, key and the allowed list are kept in NVS. A new dongle makes a random
key and starts on channel 11.

### Updating the dongle itself (firmware 1.1.0 and later)

No cable swap and no download mode: the dongle takes new firmware over this
same port. The server sends `OTA_BEGIN` with the size and SHA-256 of the image,
then the image in chunks, then `OTA_END`. Every one of those is answered with
`RESULT` before the next is sent — USB does not lose bytes, so there is no
window to track and nothing to resend.

What arrives is the **app only**, not the whole-flash release image: the server
slices it out at `0x10000`. It goes into the app slot that is not running, so
NVS survives and trackers stay bound. It is armed only once every byte matches
the digest, and `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` means the bootloader
restores the old firmware unless the new one answers a `HELLO`.

Trackers lose the beacon for the few seconds this takes (the slot is erased
first) and find it again afterwards, the same as a channel change.

### Binding sequence

1. Server opens the dongle's port: `HELLO` → `MSG_HELLO`; `GET_KEY` → `MSG_KEY`.
2. Tracker plugged in with USB: `GET INFO` gives its address.
3. Server → dongle: `ALLOW <tracker>`.
4. Server → tracker: `SET DONGLE <dongle> <channel> <key>`.
5. The tracker restarts its radio, hears the beacon and sends its handshake;
   the dongle passes it on; the server answers through `SEND`.

## Building and releases

```
pio run -d dongle-firmware                    # dongle
pio run -e picovr_c3                          # tracker
cd dongle-firmware && python scripts/package.py dist
```

`package.py` merges bootloader, partition table, `boot_app0` and the app into
`picovr-dongle-esp32c3.bin` (flash at `0x0`) and writes `picovr-dongle.json`
(version, link version, size, SHA-256).

CI (`.github/workflows/dongle-firmware.yml`) builds both on every push. A
`dongle-vX.Y.Z` tag publishes a release with those two files; the version must
match `FIRMWARE_MAJOR/MINOR/PATCH` in `src/main.cpp`. The PicoVR server looks
for the newest `dongle-v*` release in this repository and downloads them from
`releases/download/<tag>/`.
