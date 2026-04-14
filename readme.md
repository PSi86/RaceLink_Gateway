# RaceLink Gateway

Firmware for the **RaceLink USB Gateway**.

This repository contains the embedded gateway firmware used within the **RaceLink** ecosystem. The gateway acts as the bridge between the wireless RaceLink nodes and the host software running on a computer or Raspberry Pi.

In a typical setup:

- **RaceLink_WLED** provides wireless nodes based on WLED
- **RaceLink_Gateway** sends commands to those nodes and receives their responses and telemetry
- **RaceLink_Host** communicates with the gateway over USB and provides higher-level control, logic, and user interfaces

This repository is focused on the **gateway firmware only**. It does **not** contain WLED node firmware and it does **not** contain the host-side application.

---

## What this repository provides

- firmware for the **RaceLink USB Gateway**
- a **wireless-to-USB bridge** for the RaceLink system
- radio transport handling for RaceLink packets
- USB framing for communication with the host software
- board-specific configuration for supported gateway hardware
- optional local status/debug output on the integrated OLED display

---

## Role in the RaceLink ecosystem

The RaceLink Gateway sits between the host and the wireless devices:

```text
RaceLink_Host  <--USB-->  RaceLink_Gateway  <--wireless-->  RaceLink nodes
                                                           ├─ RaceLink_WLED nodes
                                                           ├─ Startblocks
                                                           └─ Custom nodes
```

Typical responsibilities of the gateway include:

- receiving commands from **RaceLink_Host** over USB
- sending those commands over the wireless link
- opening and managing receive windows after transmissions
- receiving packets from wireless nodes
- forwarding received packets, RSSI and SNR metadata, and transport events back to the host over USB

---

## Current project structure

The current repository is organized as a PlatformIO project with the usual directories:

```text
RaceLink_Gateway/
├─ include/
├─ lib/
├─ src/
│  ├─ main.cpp
│  ├─ racelink_proto.h
│  └─ racelink_transport_core.h
├─ test/
└─ platformio.ini
```

---

## Hardware and build target

The current `platformio.ini` defines a single PlatformIO environment:

- `WirelessStickV3-ESP32S3`

The configuration is currently based on an **ESP32-S3** target and includes:

- SX1262 radio support
- OLED support via **U8g2**
- USB serial communication to the host
- board-specific pin definitions through compile-time build flags

The current build configuration also defines the gateway as:

- `DEV_TYPE=1`
- `DEV_TYPE_STR="RaceLink_Gateway_v4"`

---

## Communication overview

The gateway firmware handles two communication sides:

### USB side
On the USB side, the gateway exchanges framed data with the host software. The firmware comments describe a frame structure of:

```text
[0x00][LEN][TYPE][DATA...]
```

This is used for host-to-gateway and gateway-to-host communication.

### Wireless side
On the wireless side, the gateway uses a RaceLink packet protocol and radio transport layer to transmit commands and receive responses from wireless nodes.

The current source also defines radio defaults such as:

- frequency: `867700000 Hz`
- bandwidth: `125 kHz`
- spreading factor: `SF7`
- coding rate denominator: `5` (4/5)
- preamble: `8`
- sync word: `0x12`

---

## Display and local status output

The current firmware supports a small OLED display and can show status/debug information locally. The project uses **U8g2** for the display implementation.

Typical locally shown information includes transmission and reception counters as well as temporary packet debug output.

---

## Requirements

Before building, make sure you have:

- **PlatformIO** installed  
  Either through **VS Code + PlatformIO extension** or a standalone PlatformIO installation
- a supported gateway hardware platform compatible with the configured environment
- the required USB connection for flashing and host communication

---

## Quick start

### 1. Clone the repository

```bash
git clone https://github.com/PSi86/RaceLink_Gateway.git
```

### 2. Open it in PlatformIO

Open the project in **VS Code** with the **PlatformIO** extension installed.

### 3. Build the firmware

Build the current gateway firmware with:

```bash
pio run -e WirelessStickV3-ESP32S3
```

### 4. Flash the firmware

Flash the firmware to the target board using PlatformIO or your preferred upload workflow.

### 5. Connect it to the host

After flashing, connect the gateway to the machine running **RaceLink_Host** via USB.

---

## Dependencies

The current `platformio.ini` declares these external libraries:

- `jgromes/RadioLib`
- `olikraus/U8g2`

These are used for:

- radio communication
- OLED rendering

---

## Configuration notes

At the moment, the project uses compile-time definitions in `platformio.ini` for hardware-specific settings such as:

- radio pin mapping
- OLED pin mapping
- gateway device type
- board-level options
- selected radio module type

If you port the gateway firmware to different hardware, you will most likely need to adjust those build flags.

---

## Example hardware-specific settings currently defined

The current configuration includes compile-time definitions for:

- radio pins such as `RACELINK_CS`, `RACELINK_SCK`, `RACELINK_MOSI`, `RACELINK_MISO`, `RACELINK_RST`, `RACELINK_BUSY`, and `RACELINK_DIO1`
- OLED pins such as `OLED_SDA`, `OLED_SCL`, and `OLED_RST`
- `PIN_VEXT`
- `RX_STICKY`
- the gateway device type and device type string

The source also checks that these required definitions are present at build time.

---

## Integration with other RaceLink repositories

This repository is usually used together with:

### RaceLink_WLED
Contains WLED-based wireless node firmware used by RaceLink-compatible nodes.

Repository:
`https://github.com/PSi86/RaceLink_WLED`

### RaceLink_Host
Contains the host-side software that communicates with the gateway over USB and typically runs the higher-level control logic and web interface.

Repository:
`https://github.com/PSi86/RaceLink_Host`

---

## Customizing for your own hardware

If your gateway hardware differs from the currently configured target, you will most likely need to adjust:

- board type in `platformio.ini`
- radio module selection
- radio SPI pins
- OLED pins
- power enable pin definitions
- upload and monitor settings
- optional debug and display behavior

For derived boards, it is usually best to duplicate the current PlatformIO environment and modify only the necessary values.

---

## Troubleshooting

### Build fails because a pin definition is missing
Check the build flags in `platformio.ini`. The source explicitly requires several radio and OLED pin definitions at compile time.

### The gateway builds but cannot communicate with nodes
Verify the selected radio module, pin mapping, and radio parameters such as frequency, bandwidth, spreading factor, and sync word.

### The gateway works over radio but not over USB
Check the USB serial connection, baud rate, and host-side framing expectations.

### The OLED remains blank
Verify the OLED pin definitions and display wiring.

---

## Intended audience

This repository is mainly intended for:

- RaceLink gateway firmware development
- custom gateway hardware for the RaceLink ecosystem
- developers integrating RaceLink wireless nodes with host-side software

---

## Related repositories

- RaceLink Gateway: `https://github.com/PSi86/RaceLink_Gateway`
- RaceLink WLED nodes: `https://github.com/PSi86/RaceLink_WLED`
- RaceLink Host: `https://github.com/PSi86/RaceLink_Host`

---

## Notes

This repository provides the **gateway firmware layer** of the RaceLink project.

It is intended to be used together with the corresponding node firmware and host software, but it remains a separate repository so that gateway development, hardware adaptation, and firmware distribution can be managed independently.
