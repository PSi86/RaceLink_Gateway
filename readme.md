# RaceLink Gateway

ESP32-S3 firmware for the **RaceLink USB Gateway** — a
USB ↔ LoRa bridge that connects [RaceLink_Host](https://github.com/PSi86/RaceLink_Host)
to wireless RaceLink nodes.

## Documentation

📚 **Full documentation lives at
[RaceLink_Docs](https://github.com/PSi86/RaceLink_Docs)**:

* **Gateway operator setup** — connect, identify, OLED indicators
* **Wire protocol reference** — USB framing, gateway state machine
* **Build flags / hardware variants**

This README only covers what's specific to *this repository* —
build, flash, hardware targets. For the operator guide, the wire
protocol, and the system overview, follow the link above.

## Quick start

```bash
git clone https://github.com/PSi86/RaceLink_Gateway.git
cd RaceLink_Gateway
pio run -e WirelessStickV3-ESP32S3
```

Open in VS Code with the PlatformIO extension; flash via
PlatformIO's *Upload* action; then connect the USB cable to the
host machine.

## Hardware target

Two PlatformIO environments, both ESP32-S3 + SX1262 + SSD1306 OLED
sharing an identical radio / OLED-I2C / Vext / button pin map:

* `WirelessStickV3-ESP32S3` — Heltec Wireless Stick V3, 0.49" 64×32 OLED
* `HeltecWiFiLoRa32V4-ESP32S3` — Heltec WiFi LoRa 32 V4 (V4.3), 0.96" 128×64 OLED

The only per-board difference is the OLED panel geometry, selected at
compile time via the `OLED_GEOMETRY_64X32` / `OLED_GEOMETRY_128X64`
build flag (see [`platformio.ini`](platformio.ini)); `src/main.cpp`
switches the U8g2 constructor and status/debug layout accordingly.
Build the V4 with `pio run -e HeltecWiFiLoRa32V4-ESP32S3`.

Compile-time defines (shared by both environments):

* `DEV_TYPE=1`
* `DEV_TYPE_STR="RaceLink_Gateway_v4"`

For the full pin-mapping table and customisation guide, see the
[Gateway firmware page](https://psi86.github.io/RaceLink_Docs/RaceLink_Gateway/).

## Radio defaults

| Parameter | Value |
|---|---|
| Frequency | 867.7 MHz |
| Bandwidth | 125 kHz |
| Spreading factor | SF7 |
| Coding rate | 4/5 |
| Sync word | 0x12 |

These must match the WLED nodes' build profile. Operators outside
the EU 868 MHz ISM band must adjust the frequency in
`platformio.ini` and re-flash; nodes need a matching change.

## USB framing

Frames on the USB CDC link follow `[0x00][LEN][TYPE][DATA…]`. The
authoritative wire-protocol reference is at
[Wire protocol](https://psi86.github.io/RaceLink_Docs/reference/wire-protocol/);
the Gateway and WLED firmware mirror `racelink_proto.h`
byte-identically (verified by `tests/test_proto_header_drift.py`
in the Host repo).

## Repository structure

```text
RaceLink_Gateway/
├── include/
├── lib/
├── src/
│   ├── main.cpp                state machine, USB framing, LBT
│   ├── racelink_proto.h        ← byte-identical to Host + WLED
│   └── racelink_transport_core.h
├── test/
├── partitions/
│   └── racelink_gateway_8MB.csv  pinned flash layout, app0 @ 0x10000
├── platformio.ini
├── README.md
└── LICENSE
```

The platform version and the partition table are both pinned in
[`platformio.ini`](platformio.ini): releases publish images that name concrete
flash offsets, so neither may drift between two releases.
`tests/test_partition_table.py` fails if they do.

## Dependencies

* `jgromes/RadioLib` — SX1262 driver
* `olikraus/U8g2` — OLED rendering

## Licence

See [`LICENSE`](LICENSE).
