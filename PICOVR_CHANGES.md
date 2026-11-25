# PicoVR changes to the tracker firmware

Every change made to this firmware for PicoVR, where it is, and how far it was
tested. **Update this file in the same commit as any firmware change.**

Base: `16c5b36` "Update packets.h" (24 Oct 2026), the firmware as it was before
any PicoVR work. Everything below is relative to it.

`MBE-Thing` is the branch the web flasher builds for customers. Anything on it
is in customers' hands as soon as they flash.

## Rules before anything reaches MBE-Thing

These exist because each one was broken once, and a customer paid for it (see
"What went wrong" below).

1. **CI is green on the commit:** the native sensor test, both PicoVR builds,
   and the web flasher build (`.github/workflows/tracker-firmware.yml`).
2. **It has run on a real PicoVR tracker for at least 30 minutes**, starting
   cold so the tracker warms up, both at rest and moving. In the serial
   console, `GET PERF` every few minutes must show no I2C read failures, no
   FIFO overruns, and rotation packets flowing. Building and PC tests are not
   enough: the gyro freeze passed both.
3. **It has been used with stock SlimeVR Server**, not only PicoVR Server.
   Customers use SlimeVR Server until PicoVR Server is released.
4. **The owner has said yes** to that specific push.
5. **Only what was asked for.** A fix for a customer problem contains that fix
   and nothing else. Other work goes on its own branch.
6. **MBE-Thing only moves forward.** No force-pushes: undo with `git revert`, so
   every build a customer may be running can be found again.
7. **Write down what was tested and what was not**, in the commit message and in
   this file. Never describe behaviour that has not been seen working.

## On MBE-Thing now (`ecce373`, identical to `c217d06`)

| Commit | Date | Change | Tested on a tracker? |
|---|---|---|---|
| `a079d80` | 24 Oct | Wi-Fi, PicoVR Dongle and Auto link modes. Wi-Fi is the default. | Not recorded |
| `f4c3805` | 24 Oct | PicoVR Dongle firmware (`dongle-firmware/`), packaging and CI. Not part of the tracker image. | No |
| `54613c5` | 24 Oct | PicoVR link feature flag; `picovr_topaz` build env | Not recorded |
| `b8df138` | 25 Oct | Topaz: LSM6DSV rotation 0, BNO085 variant env | Not recorded |
| `a795e58` | 25 Oct | Release workflow and `scripts/package_tracker.py` | CI only |
| `73f8a0a` | 25 Oct | `BOARD_TOPAZESTROGEN` env in `platformio-tools.ini`, so the web flasher can build the board | Built locally |
| `66c41e7` | 30 Oct | Over-the-air firmware update (packets 201-205, `src/network/ota.*`) | No |
| `a308fd6` | 30 Oct | Dongle firmware takes its own update over its port | No |
| `344e822` | 2 Nov | Feature flag saying the firmware can be updated over the air | Not recorded |
| `d4c95ec` | 3 Nov | Report the real version; `GET WIFI` reads Wi-Fi settings back | Not recorded |
| `a25f870` | 9 Nov | Releases are drafts until someone writes the notes (CI only) | CI only |
| `be1ac25` `36bd70c` `1cd7af0` | 30-31 Oct | Gyro bias interpolated for temperature, updated mid-session, with serial reports | Short bench run only. **Reverted**, see below |
| `ef498a4` `6a3e694` `c374e59` | 24 Nov | Revert the three gyro bias commits. Sensor code back to `16c5b36`. | A customer's tracker that froze works since |
| `c217d06` | 24 Nov | IMU watchdog: no gyro data for 2 s, or gyro readings frozen for 3 s, means reset and reconfigure the IMU, retrying every 5 s. Sends one line with the IMU registers (`PACKET_SERIAL`). | Native test; running on a customer's tracker |
| `33f225d` | 24 Nov | Restored all files to `16c5b36` | Superseded by the next |
| `ecce373` | 24 Nov | Undid `33f225d`: files back to `c217d06` | Same files as `c217d06` |

## On local branch `fix/i2c-bus` only (not on MBE-Thing)

| Commit | Change |
|---|---|
| `4b8c828` | LSM6DSV: keep the I2C anti-spike filters on (`IF_CFG` 03h, `ASF_CTRL` bit 5) |
| `696cb7b` | LSM6DS FIFO reader: check every I2C read and drop a failed one instead of parsing leftover memory as samples; drain the FIFO with up to 4 reads per poll so a stall doesn't become an overrun; print the overrun message once per 100 |
| `428542c` | Remove the blocking 100 ms LED blink on every calibration save (upstream removed it too) |
| `30c5925` | `GET PERF` serial command: loop time per section, worst stall, fusion share, sample and packet rates, I2C failures, FIFO overruns |
| `19cf5cf` | CI runs on MBE-Thing (it only watched `picovr-dongle` before), runs the native test, and builds like the web flasher |

To ship any of these, follow the rules above first.

### Hardware tests of `fix/i2c-bus` (25 Nov 2026)

Tracker: PicoVR LSM6DSV board, MAC `90:DA:72:92:FF:14`, on USB on a desk, not
moved. Built as the web flasher does (`BOARD_TOPAZESTROGEN` with
`-DPICOVR_TOPAZ`), flashed as boot_app0 + app0 so calibration stayed.

| Run | Length | Result |
|---|---|---|
| Old build `a25f870` (control), after a reset | 13 min | No freeze. Its gyro change never switched on: it waits for 5 C of warming and the desk gave 3 C (23 to 26 C). A desk cannot reproduce the customer freeze. |
| `fix/i2c-bus`, USB only, no server | 35 min | Gyro 480/s and accel 240/s every minute. 0 I2C failures, 0 FIFO overruns. Rotation kept updating. Worst loop gap 7-9 ms, once 13 ms (FIFO holds about 0.8 s). |
| `fix/i2c-bus` through PicoVR Dongle 1.1.0 to PicoVR Server | 31 min | Connected in 5 s, followed the server's channel change 11 to 12 by itself. 0 failures, 0 overruns, 0 packets lost, signal -42 dBm throughout, no disconnects. One 0.1 s pause when switching from Wi-Fi to the dongle. |

Not tested yet: a cold start (tracker off, then on), Wi-Fi, stock SlimeVR
Server, and a tracker that is moving or worn.

Note for reading `GET PERF`: a still tracker sends rotation only when it
changes (`OPTIMIZE_UPDATES`), which on a desk is under once a second, so
"rotation packets 0/s" is normal there.

## What went wrong, 10-24 November 2026

- **The gyro bias change went to customers without a long run on a tracker.**
  On 10 Nov MBE-Thing was moved forward to include temperature-interpolated
  gyro bias that changed mid-session as a tracker warmed. A short bench run
  measured less drift, but nobody ran a tracker from cold for half an hour.
  Customers' trackers stopped showing rotation about 5 minutes after power-on.
  It was rolled back on 24 Nov, and a customer's affected tracker has worked
  since. The exact mechanism was never proven.
- **Nothing checked MBE-Thing automatically.** CI ran on `picovr-dongle` only,
  and never built what the web flasher builds. Fixed in `19cf5cf`.
- **A fix went to customers before it ran on a tracker** (`c217d06`). It turned
  out fine, but by luck, not by testing.
- **A diagnostic was described as working when it didn't.** The watchdog's
  register line goes out as `PACKET_SERIAL`, and both servers only print that
  to the app's developer console, not to any log file. Customers can't send it
  to us. The serial console (USB) is what actually shows it.
- **IMU reads were never checked**, in the original firmware too. A customer's
  ankle tracker with a failing I2C bus (`Error 263` / `-1` several times a
  second, on every firmware including `16c5b36`) fed leftover memory into the
  fusion. That tracker's bus fault is hardware; the unchecked reads are fixed
  on `fix/i2c-bus`.

## How to test

- Native sensor test (any PC with g++ 13 or newer, or WSL):
  `g++ -std=c++20 -I src -o sensor_test test/native/sensorwatchdog_test.cpp && ./sensor_test`
- PicoVR builds: `pio run -e picovr_topaz -e picovr_topaz_bno`
- What the web flasher builds:
  `PLATFORMIO_BUILD_FLAGS=-DPICOVR_TOPAZ pio run -c platformio-tools.ini -e BOARD_TOPAZESTROGEN`
- On a tracker: flash over USB, open the serial console, type `GET PERF` every
  few minutes for at least 30 minutes.
