# ESP32 MQTT IR Blaster

ESP-IDF firmware for an ESP32-based infrared controller intended as a lightweight replacement for dedicated network IR hardware.

The device connects to Wi-Fi and subscribes to MQTT topics. Incoming command names are translated into IR frames and transmitted using the ESP32 RMT peripheral. The current implementation supports RC5 and several NEC command tables and includes status-LED feedback for MQTT connectivity.

## Hardware

The original project was developed for an ESP32-based M5Atom Lite-style device with:

- IR transmitter on GPIO 12 by default
- SK6812 status LED on GPIO 27 by default
- Wi-Fi connectivity

Both GPIO assignments can be changed through ESP-IDF `menuconfig`.

## MQTT topics

The device identifier is configurable. With the default identifier `rig01`, the firmware subscribes to:

```text
irsender/rig01/rc5
irsender/rig01/nec/hisense
irsender/rig01/nec/pleio
irsender/rig01/nec/ugreen
```

Payloads are command names such as `Standby`, `Vol+`, `Ok`, `Back` or the values present in the relevant command table in `main/irsender_main.cpp`.

Example:

```bash
mosquitto_pub -h broker.example.com \
  -t irsender/rig01/rc5 \
  -m Standby
```

## Build

Requires ESP-IDF.

```bash
idf.py set-target esp32
idf.py menuconfig
```

Under **IR Sender Configuration**, configure:

- Wi-Fi SSID and password
- MQTT broker URI
- optional MQTT username/password
- device/rig identifier
- IR transmitter GPIO
- status LED GPIO

Then build and flash:

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## MQTT TLS

The broker URI is configurable and may use either `mqtt://` or `mqtts://`. TLS certificate verification requirements depend on the broker and ESP-IDF configuration. For a public deployment, configure an appropriate CA trust mechanism rather than embedding private certificates or credentials in source control.

## Repository hygiene

Generated ESP-IDF output (`build/`), local `sdkconfig`, managed component caches and IDE files are intentionally excluded from version control.

## Third-party component

`components/esp32-rmt-ir` is derived from the `junkfix/esp32-rmt-ir` project, as noted in the source headers. Review the upstream project's licence/redistribution terms before publishing or redistributing that component.

## Notes

This repository is a sanitised portfolio version of hardware-control firmware. Network credentials, broker details and deployment-specific identifiers are deliberately supplied through configuration rather than hard-coded in the source.

## Third-party components

`components/esp32-rmt-ir` is derived from the `junkfix/esp32-rmt-ir` project and is redistributed under the MIT License. The upstream copyright and licence text are preserved in `components/esp32-rmt-ir/LICENSE`.
