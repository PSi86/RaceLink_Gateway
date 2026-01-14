/*
 * USB LoRa Communicator (RadioLib + U8g2, Port deiner V4-Version)
 * - unveränderte State Machine (currentMode/requestMode)
 * - TX nur aus IDLE, RX-Fenster mit Timeout
 * - Serial-Layout identisch (6B MAC: 00 00 00 + Last3)
 * - OLED jetzt über U8g2, "Bold" = Doppelt zeichnen
 *
 * Basis: psi_GateCommunicator_V4.txt (dein Code). 1:1 Logikport. 
 */

#include <Arduino.h>
#include "lora_proto.h"
#include "lora_link_core.h"

static LoraLink::Core ll{};
static LoraLink::Callbacks cb{};

#include <SPI.h>
#include <Wire.h>
//#include <RadioLib.h>     // SX1262
#include <U8g2lib.h>

/************ LoRa PHY (wie vorher) ************/
#define RF_FREQUENCY_HZ             867700000UL //868000000UL
#define TX_OUTPUT_POWER             14           // dBm
#define LORA_BW_KHZ                 125.0        // 0 => 125 kHz -> 125.0 in RadioLib
#define LORA_SF                     7            // SF7
#define LORA_CR_DEN                 5            // 4/5 -> 5
#define LORA_PREAMBLE               8
#define LORA_SYNCWORD               0x12

/************ App/Proto ************/
#define IDENTIFY        1
#define ESPNOW_GATE     20
#define BASIC_IR_GATE   21
#define CUSTOM_IR_GATE  22
#define WIZMOTE_GATE    23
#define WLED_CUSTOM     24
#define GET_DEVICES     30
#define SET_GROUP       31

#define BAUDRATE        921600

#define RX_WINDOW_BROADCAST_MS  2000
#define RX_WINDOW_UNICAST_MS    1000

#define BUTTON_PIN      0   // User-Button (GPIO0)

/************ Button Handling (Short/Long) ************/
volatile bool btnFallingFlag = false;
unsigned long pressStartMs = 0;
bool longHandled = false;

void IRAM_ATTR isr_button() { btnFallingFlag = true; }

/************ Pins (aus platformio.ini via -D ...) ************/
#ifndef PIN_VEXT
  #error "Define PIN_VEXT in platformio.ini build_flags"
#endif
#ifndef LORA_CS
  #error "Define LORA_CS in platformio.ini build_flags"
#endif
#ifndef LORA_DIO1
  #error "Define LORA_DIO1 in platformio.ini build_flags"
#endif
#ifndef LORA_BUSY
  #error "Define LORA_BUSY in platformio.ini build_flags"
#endif
#ifndef LORA_RST
  #error "Define LORA_RST in platformio.ini build_flags"
#endif
#ifndef LORA_SCK
  #error "Define LORA_SCK in platformio.ini build_flags"
#endif
#ifndef LORA_MISO
  #error "Define LORA_MISO in platformio.ini build_flags"
#endif
#ifndef LORA_MOSI
  #error "Define LORA_MOSI in platformio.ini build_flags"
#endif
#ifndef OLED_SDA
  #error "Define OLED_SDA in platformio.ini build_flags"
#endif
#ifndef OLED_SCL
  #error "Define OLED_SCL in platformio.ini build_flags"
#endif
#ifndef OLED_RST
  #error "Define OLED_RST in platformio.ini build_flags"
#endif

/************ USB framing for Host <-> Device (gc_transport.py) ************/
// Frame: [0x00][LEN][TYPE][DATA...]; LEN = len(TYPE+DATA)
// Device->Host LoRa forward: TYPE = Header7.type (N2M), DATA = [Header7][Body][RSSI(LE16)][SNR(i8)]
// Device->Host Events:
static const uint8_t EV_ERROR            = 0xF0;
static const uint8_t EV_RX_WINDOW_OPEN   = 0xF1; // DATA: window_ms (LE16)
static const uint8_t EV_RX_WINDOW_CLOSED = 0xF2; // DATA: rx_count_delta (LE16)
static const uint8_t EV_TX_DONE          = 0xF3; // DATA: last_len (u8)
static const uint8_t EV_IDLE             = 0xF4; // DATA: 1

static inline void usb_send_frame(uint8_t type, const uint8_t* data, uint8_t len) {
  uint8_t L = (uint8_t)(1 + len); // TYPE + DATA
  Serial.write((uint8_t)0x00);
  Serial.write(L);
  Serial.write(type);
  if (len) Serial.write(data, len);
}

static inline void usb_send_event_u8(uint8_t evType, uint8_t v) {
  uint8_t d[1] = { v };
  usb_send_frame(evType, d, 1);
}
static inline void usb_send_event_u16(uint8_t evType, uint16_t v) {
  uint8_t d[2] = { (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF) };
  usb_send_frame(evType, d, 2);
}

/************ OLED (U8g2) ************/
// SSD1306 64x32 (UNIVISION) Full buffer, HW I2C
//SSD1306 64X32_NONAME: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp#ssd1306-64x32_1f-1
//Full Frame Buffer and Pin Definitions:
U8G2_SSD1306_64X32_1F_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ OLED_RST);

/************ MAC ************/
// lora_link_core.h enthält jetzt die MAC-bezogenen Elemente (ll.myMac6, ll.myLast3, ll.macReadOK)
/* uint8_t myMac6[6]  = {0};
uint8_t myLast3[3] = {0};
bool macReadOK = false;
 */
/************ Serial Handling (dein Parser) ************/
const byte maxBytes = 255;
byte receivedBytes[maxBytes];
byte numReceived = 0;
bool newSerialData = false;

// Robust serial frame receiver for framing: [0x00][LEN][TYPE][DATA...]
// LEN = number of bytes following LEN (TYPE + DATA)
const uint16_t FRAG_TIMEOUT_MS = 25;   // 10–30 ms sind praxisnah bei 921600 Baud

void recvSerialBytes() {
  static bool     recvInProgress = false;
  static uint8_t  ndx            = 0;     // how many bytes stored in receivedBytes (incl. LEN)
  static uint8_t  dataLen        = 0;     // LEN byte value (TYPE+DATA bytes expected)
  static uint32_t lastByteAtMs   = 0;

  // Timeout auch dann prüfen, wenn gerade keine Bytes anliegen:
  if (recvInProgress && !newSerialData) {
    uint32_t now = millis();
    if ((uint32_t)(now - lastByteAtMs) > FRAG_TIMEOUT_MS && Serial.available() == 0) {
      // Fragment verworfen (Timeout)
      recvInProgress = false;
      ndx = 0; dataLen = 0;
      // optionales Telemetrie-/Log-Event:
      // usb_send_event_u16(EV_ERROR, 3); // 3 = TIMEOUT
    }
  }

  while (Serial.available() > 0 && !newSerialData) {
    uint8_t rb = (uint8_t)Serial.read();
    lastByteAtMs = millis();

    // Noch kein Frame aktiv -> warte auf Header 0x00
    if (!recvInProgress) {
      if (rb == 0x00) {
        recvInProgress = true;
        ndx = 0;
        dataLen = 0;
      }
      continue;  // alles andere ignorieren bis zum nächsten 0x00
    }

    // Wir sind in einem Frame:
    if (ndx == 0) {
      // Erstes Byte nach 0x00 ist LEN (TYPE+DATA)
      dataLen = rb;
      if (dataLen == 0 || dataLen > (maxBytes - 1)) {
        // Ungültige Länge -> Frame verwerfen und auf nächsten Header warten
        recvInProgress = false;
        ndx = 0; dataLen = 0;
        // usb_send_event_u16(EV_ERROR, 1); // 1 = LEN_INVALID
        continue;
      }
      receivedBytes[ndx++] = dataLen;  // LEN in [0] ablegen; TYPE folgt als nächstes
      continue;
    }

    // TYPE/DATA aufnehmen
    if (ndx < maxBytes) {
      receivedBytes[ndx++] = rb;
    } else {
      // Pufferüberlauf -> Frame verwerfen
      recvInProgress = false;
      ndx = 0; dataLen = 0;
      // usb_send_event_u16(EV_ERROR, 2); // 2 = OVERRUN
      continue;
    }

    // Abschluss: genau wenn LEN + 1 Bytes (inkl. LEN) gelesen wurden
    if (ndx == (uint16_t)dataLen + 1) {
      numReceived    = ndx;        // enthält LEN + TYPE + DATA
      newSerialData  = true;       // signalisiert Main-Loop: Frame liegt in receivedBytes
      recvInProgress = false;
      ndx = 0; dataLen = 0;
      // while-Bedingung bricht hier ab (wegen !newSerialData), damit ein Frame pro Loop verarbeitet wird
    }
  }
}

/************ OLED Helpers (U8g2) ************/
static void drawStringBold(int16_t x, int16_t y, const char* s) {
  u8g2.setFont(u8g2_font_9x15B_mf);  // fette Schrift
  u8g2.drawStr(x, y, s);
  u8g2.setFont(u8g2_font_9x15_mf);
}

bool showDebug = false;
bool inhibitStatusDraw = false;

void drawStatus() {
  if (inhibitStatusDraw) return;

  char lineTX[16], lineRX[16];
  snprintf(lineTX, sizeof(lineTX), "TX:%u", ll.txCount);
  snprintf(lineRX, sizeof(lineRX), "RX:%u", ll.rxCountFiltered);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_9x15_mf);     // ~15px Zeilenhöhe -> 2 Zeilen passen in 32px

  // y-Koordinaten sind Baselines
  if (ll.rfMode == LoraLink::Mode::Tx) {
    drawStringBold(0, 14, lineTX);
    u8g2.drawStr(0, 31, lineRX);
  } else if (ll.rfMode == LoraLink::Mode::Rx) {
    u8g2.drawStr(0, 14, lineTX);
    drawStringBold(0, 31, lineRX);
  } else {
    u8g2.drawStr(0, 14, lineTX);
    u8g2.drawStr(0, 31, lineRX);
  }
  u8g2.sendBuffer();
}

// globals
uint8_t  dispbuffer[32];
uint8_t  dispLen = 0;

// 7 Bytes pro Zeile -> 21 Zeichen ("FF FF FF FF FF FF FF")
static const char HEXLUT[] = "0123456789ABCDEF";

void drawDebug(const uint8_t* buf, uint8_t len) {
  if (!showDebug) return;
  inhibitStatusDraw = true;                   // falls du die Statusanzeige temporär unterdrücken willst
  dispLen = (len > sizeof(dispbuffer)) ? sizeof(dispbuffer) : len;
  memcpy(dispbuffer, buf, dispLen);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_mf);

  // Kopfzeile: Länge
  char hdr[22];
  snprintf(hdr, sizeof(hdr), "LAST %uB%s", dispLen, (len > dispLen ? "+" : ""));
  //u8g2.drawStr(0, 10, hdr);

  const uint8_t perLine = 4;                  // passt in die Breite (21 Zeichen)
  char line[22];                               // max 21 chars + \0
  uint8_t y = 10; //22

  for (uint8_t i = 0; i < 3 && i * perLine < dispLen; ++i) {
    const uint8_t start = i * perLine;
    const uint8_t count = (dispLen - start > perLine) ? perLine : (dispLen - start);

    char* p = line;
    for (uint8_t j = 0; j < count; ++j) {
      if (j) *p++ = ' ';
      uint8_t b = dispbuffer[start + j];
      *p++ = HEXLUT[b >> 4];
      *p++ = HEXLUT[b & 0x0F];
    }
    *p = '\0';
    u8g2.drawStr(0, y, line);
    y += 10;                                   // Zeilenhöhe der 6x10-Schrift
  }

  // falls abgeschnitten: Ellipse unten rechts
  if (dispLen < len) u8g2.drawStr(110, 32, "...");

  u8g2.sendBuffer();
}

/************ RadioLib: Modul + ISR-Flags ************/
SX1262 radio = SX1262(new Module(
  LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, 
  SPI, SPISettings(8000000, MSBFIRST, SPI_MODE0)
));


/************ SYNC (Master-side) ************/
// Master keeps a "dumb" timebase derived from millis().
// Auto-SYNC is suppressed after an ARMED CONTROL until the host triggers one SYNC.
// SYNC frames are always broadcast and are queued without CAD/LBT and without jitter.

static constexpr uint8_t  GC_FLAG_ARM_ON_SYNC = (1u << 1);   // must match Node/Usermod

enum class SyncState : uint8_t {
  AUTO_ALLOWED = 0,
  WAIT_HOST_TRIGGER,   // after CONTROL(ARM), block auto sync until host triggers OPC_SYNC
  HOST_SYNC_RETRY      // host requested OPC_SYNC but it couldn't be queued yet (tx busy) -> retry ASAP
};

static SyncState syncState = SyncState::AUTO_ALLOWED;
static uint32_t      lastSyncQueuedMs = 0;   // when a sync was successfully queued (auto or host)
static uint8_t       hostSyncBri = 128;      // brightness of last host-triggered (pending) sync

// Queue a frame for immediate TX without CAD/LBT and without jitter.
static bool ll_queueTxNoCad(LoraLink::Core& ll, const uint8_t* buf, uint8_t len) {
  if (!buf || !len) return false;
  if (ll.txPending) return false;

  const bool prevLbt = ll.lbtEnable;
  ll.lbtEnable = false;
  const bool ok = LoraLink::scheduleSend(ll, buf, len, 0 /*no jitter*/);
  ll.lbtEnable = prevLbt;
  return ok;
}

// Build + queue a global SYNC frame (broadcast) using the current millis()-based timebase.
static bool sendSync(LoraLink::Core& ll, uint8_t brightness) {
  static const uint8_t BCAST3[3] = {0xFF, 0xFF, 0xFF};

  const uint32_t t = millis();

  LoraProto::P_Sync p{};
  p.ts24_0 = (uint8_t)(t);
  p.ts24_1 = (uint8_t)(t >> 8);
  p.ts24_2 = (uint8_t)(t >> 16);
  p.brightness = brightness;

  uint8_t out[sizeof(LoraProto::Header7) + sizeof(LoraProto::P_Sync)];
  const uint8_t type_full = LoraProto::make_type(LoraProto::DIR_M2N, LoraProto::OPC_SYNC);
  const uint8_t n = LoraProto::build(out, ll.myLast3, BCAST3, type_full, p);

  // no CAD/LBT, no jitter
  return ll_queueTxNoCad(ll, out, n);
}

// Auto-SYNC should only be sent when nothing else is happening.
static bool idleForAutoSync(const LoraLink::Core& ll) {
  // USB work pending?
  if (newSerialData) return false;
  if (Serial.available() > 0) return false;

  // Link busy?
  if (ll.txPending) return false;

  // Do not interfere with receive mode / rx windows
  if (ll.rfMode != LoraLink::Mode::Idle) return false;
  if (ll.reqRxKind != LoraLink::RxKind::None) return false;

  return true;
}

static void sync_service(LoraLink::Core& ll) {
  const uint32_t now = millis();

  // 1) Host-triggered SYNC retry: send ASAP once TX becomes available.
  if (syncState == SyncState::HOST_SYNC_RETRY) {
    if (!ll.txPending) {
      if (sendSync(ll, hostSyncBri)) {
        lastSyncQueuedMs = now;
        syncState = SyncState::AUTO_ALLOWED;
      }
    }
    return; // never auto-sync while host sync is pending
  }

  // 2) After CONTROL(ARM): block auto-sync until host triggers one SYNC.
  if (syncState == SyncState::WAIT_HOST_TRIGGER) {
    return;
  }

  // 3) Auto-SYNC: only when idle and at most every 30s.
  if (!idleForAutoSync(ll)) return;
  if (lastSyncQueuedMs != 0 && (uint32_t)(now - lastSyncQueuedMs) < 30000UL) return;

  if (sendSync(ll, hostSyncBri)) {
    lastSyncQueuedMs = now;
  }
}

/************ Host-Kommandos (1:1) ************/
void handleCommand() {
  if (!newSerialData) return;

  const uint8_t payloadLen = receivedBytes[0];
  if (payloadLen < 1) { newSerialData = false; return; }
  const uint8_t first = receivedBytes[1];

  // Legacy IDENTIFY (kept for port discovery)
  if (payloadLen == 1 && first == IDENTIFY) {
    char macstr[18];
    LoraLink::mac6ToStr(ll.myMac6, macstr);
    Serial.print(F("GateCommunicator_v4"));
    Serial.print(macstr);
    newSerialData = false;
    return;
  }

  // New framing: TYPE_FULL + recv3 + body...
  if ((first & 0x80) == LoraProto::DIR_M2N && payloadLen >= 4) {
    const uint8_t type_full = first;
    const uint8_t recv3[3] = { receivedBytes[2], receivedBytes[3], receivedBytes[4] };
    const uint8_t bodyLen = (uint8_t)(payloadLen - 4);
    const uint8_t* body   = &receivedBytes[5];

    uint8_t out[32]; uint8_t n = 0;

    switch (LoraProto::type_base(type_full)) {

      case LoraProto::OPC_DEVICES: {
        if (bodyLen == sizeof(LoraProto::P_GetDevices)) {
          LoraProto::P_GetDevices p{};
          memcpy(&p, body, sizeof(p));
          n = LoraProto::build(out, ll.myLast3, recv3, type_full, p);
          LoraLink::scheduleSend(ll, out, n);
          drawDebug(out, n);
          // decide on RX window depending on the sent frame being a broadcast or unicast
          if (LoraLink::isBroadcast3(recv3)) {
            // unbekannte Zahl von Empfängern -> längeres RX-Fenster
            LoraLink::requestRxTimed(ll, RX_WINDOW_BROADCAST_MS); // z.B. 2900ms
          } else {
            // unicast -> kürzeres RX-Fenster, nach nur einer Antwort RX beenden
            LoraLink::requestRxTimed(ll, RX_WINDOW_UNICAST_MS, 1); // z.B. 1000ms
          }
          //LoraLink::requestRxTimed(ll, RX_WINDOW_MS); // z.B. 2900ms
          //LoraLink::scheduleSendThenRxWindow(ll, out, n, RX_WINDOW_MS);
        }
      } break;

      case LoraProto::OPC_SET_GROUP: {
        if (bodyLen == sizeof(LoraProto::P_SetGroup)) {
          LoraProto::P_SetGroup p{};
          memcpy(&p, body, sizeof(p));
          n = LoraProto::build(out, ll.myLast3, recv3, type_full, p);
          LoraLink::scheduleSend(ll, out, n);
          drawDebug(out, n);

          if (LoraLink::isBroadcast3(recv3)) {
            // unbekannte Zahl von Empfängern -> längeres RX-Fenster
            LoraLink::requestRxTimed(ll, RX_WINDOW_BROADCAST_MS); // z.B. 2900ms
          } else {
            // unicast -> kürzeres RX-Fenster, nach nur einer Antwort RX beenden
            LoraLink::requestRxTimed(ll, RX_WINDOW_UNICAST_MS, 1); // z.B. 1000ms
          }
          //LoraLink::scheduleSendThenRxWindow(ll, out, n, RX_WINDOW_MS);
        }
      } break;

      case LoraProto::OPC_STATUS: {
        if (bodyLen == sizeof(LoraProto::P_GetStatus)) {
          LoraProto::P_GetStatus p{};
          memcpy(&p, body, sizeof(p));
          n = LoraProto::build(out, ll.myLast3, recv3, type_full, p);
          LoraLink::scheduleSend(ll, out, n);
          drawDebug(out, n);
          if (LoraLink::isBroadcast3(recv3)) {
            // unbekannte Zahl von Empfängern -> längeres RX-Fenster
            LoraLink::requestRxTimed(ll, RX_WINDOW_BROADCAST_MS); // z.B. 2900ms
          } else {
            // unicast -> kürzeres RX-Fenster, nach nur einer Antwort RX beenden
            LoraLink::requestRxTimed(ll, RX_WINDOW_UNICAST_MS, 1); // z.B. 1000ms
          }
          //LoraLink::scheduleSendThenRxWindow(ll, out, n, RX_WINDOW_MS);
        }
      } break;
      
      case LoraProto::OPC_CONTROL: {
        if (bodyLen == sizeof(LoraProto::P_Control)) {
          LoraProto::P_Control p{};
          memcpy(&p, body, sizeof(p));

          // remember brightness for auto-sync (host may choose to ignore it on the nodes)
          hostSyncBri = p.brightness;

          n = LoraProto::build(out, ll.myLast3, recv3, type_full, p);
          bool ok = LoraLink::scheduleSend(ll, out, n);
          drawDebug(out, n);

          // If this CONTROL arms a sync-start, block auto-sync until host triggers OPC_SYNC.
          if (ok && (p.flags & GC_FLAG_ARM_ON_SYNC)) {
            syncState = SyncState::WAIT_HOST_TRIGGER;
          }
        }
      } break;

      case LoraProto::OPC_CONFIG: {
        if (bodyLen == sizeof(LoraProto::P_Config)) {
          LoraProto::P_Config p{};
          memcpy(&p, body, sizeof(p));
          n = LoraProto::build(out, ll.myLast3, recv3, type_full, p);
          bool ok = LoraLink::scheduleSend(ll, out, n);
          drawDebug(out, n);
          if (ok) {
            if (LoraLink::isBroadcast3(recv3)) {
              // CONFIG should be unicast, but if broadcast, allow longer RX window
              LoraLink::requestRxTimed(ll, RX_WINDOW_BROADCAST_MS);
            } else {
              LoraLink::requestRxTimed(ll, RX_WINDOW_UNICAST_MS, 1);
            }
          }
        }
      } break;

      case LoraProto::OPC_SYNC: {
        // Host triggers a global SYNC NOW (broadcast). Body is fixed-size P_Sync (brightness is used).
        if (bodyLen == sizeof(LoraProto::P_Sync)) {
          LoraProto::P_Sync p{};
          memcpy(&p, body, sizeof(p));

          // brightness to use for this (possibly retried) host-triggered sync
          hostSyncBri = p.brightness;

          // Try to queue immediately (no CAD/LBT). If TX is busy, retry ASAP in sync_service().
          if (sendSync(ll, hostSyncBri)) {
            lastSyncQueuedMs = millis();
            syncState = SyncState::AUTO_ALLOWED;   // host sync done -> auto sync allowed again (30s gate applies)
          } else {
            syncState = SyncState::HOST_SYNC_RETRY;
          }
        }
      } break;

      case LoraProto::OPC_STREAM: {
        if (bodyLen == sizeof(LoraProto::P_Stream)) {
          LoraProto::P_Stream p{};
          memcpy(&p, body, sizeof(p));
          n = LoraProto::build(out, ll.myLast3, recv3, type_full, p);
          bool ok = LoraLink::scheduleSend(ll, out, n);
          drawDebug(out, n);
          if (ok) {
            if (LoraLink::isBroadcast3(recv3)) {
              LoraLink::requestRxTimed(ll, RX_WINDOW_BROADCAST_MS);
            } else {
              LoraLink::requestRxTimed(ll, RX_WINDOW_UNICAST_MS, 1);
            }
          }
        }
      } break;

    }
    newSerialData = false;
    return;
  }

  // Fallback: ignore
  newSerialData = false;
}

/************ LoRa to USB Forwarding ************/
static inline void usb_forward_lora(const uint8_t* pkt, uint8_t len, int16_t rssi, int8_t snr) {
  if (len < sizeof(LoraProto::Header7)) return;
  
  // TODO header filtern -> nun in lora_link_core.h
  //if (!LoraLink::receiverMatches(pkt.h.receiver3, myLast3)) return;  // broadcast ODER exakt meine 3B

  const uint8_t type = pkt[sizeof(LoraProto::Header7)-1]; // header.type
  uint8_t tmp[35];
  if (len > 32) len = 32;
  memcpy(tmp, pkt, len);
  uint16_t r = (uint16_t)rssi;
  tmp[len+0] = (uint8_t)(r & 0xFF);
  tmp[len+1] = (uint8_t)((r >> 8) & 0xFF);
  tmp[len+2] = (uint8_t)snr;
  usb_send_frame(type, tmp, (uint8_t)(len+3));
}

/************ LoRa Init (RadioLib) ************/
void lora_init() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  LoraLink::PhyCfg phy;
  phy.freqMHz   = (float)RF_FREQUENCY_HZ / 1e6f;
  phy.bwKHz     = LORA_BW_KHZ;
  phy.sf        = LORA_SF;
  phy.crDen     = LORA_CR_DEN;
  phy.syncWord  = LORA_SYNCWORD;
  phy.preamble  = LORA_PREAMBLE;
  phy.crcOn     = true;

  // Gerätespezifische Overrides (wie bisher im Master):
  phy.txPowerDbm   = TX_OUTPUT_POWER;     // z.B. 14
  phy.dio2RfSwitch = 1;                   // vorher: radio.setDio2AsRfSwitch(true);
  phy.rxBoost      = -1;                  // (lassen) oder 1/0 je nach Board

  if (!LoraLink::beginCommon(radio, ll, phy)) {
    // Fehlerhandling wie bisher
    radio = nullptr; // von node übernommen
  }

  //ll.radio = &radio; wird nun in der LoraLink::beginCommon gesetzt

  // LBT aktivieren oder deaktivieren:
  ll.lbtEnable = true;   // default: false
  
  LoraLink::attachDio1(radio, ll);
  LoraLink::setDefaultIdle(ll);
}

// --- benannte Callbacks (Master) ---
static void on_tx_start_cb(void* ctx) {
  // optional: eigenes Event, wenn du eins hast; sonst nur Anzeige
  // usb_send_event_u8(EV_TX_START, 1);
  drawStatus();                   // zeigt "TX" fett + erhöhten TX-Zähler (du erhöhst TX im Core)
}

static void on_tx_done_cb(void* ctx) {
  usb_send_event_u8(EV_TX_DONE, 1);
  drawStatus();                   // zurück auf IDLE oder direkt später RX-Open → drawStatus() wird erneut aufgerufen
}

static void on_rx_open_cb(uint16_t ms, void* ctx) {
  usb_send_event_u16(EV_RX_WINDOW_OPEN, ms);
  drawStatus();                   // RX fett + RX-Window-Time sichtbar
}

static void on_rx_packet_cb(const uint8_t* pkt, uint8_t len, int16_t rssi, int8_t snr, void* ctx) {
  usb_forward_lora(pkt, len, rssi, snr);
  drawStatus();                   // RX-Zähler inkrementiert + Anzeige
  drawDebug(pkt, len);
}

static void on_rx_closed_cb(uint16_t delta, void* ctx) {
  usb_send_event_u16(EV_RX_WINDOW_CLOSED, delta);
  drawStatus();                   // zurück auf IDLE fett
}

static void on_idle_cb(void* ctx) {
  //usb_send_event_u8(EV_IDLE, 1);
  drawStatus();                   // zurück auf IDLE
}

/************ Setup/Loop ************/
void setup() {
  Serial.begin(BAUDRATE);
  delay(50);

  // Vext einschalten
  pinMode(PIN_VEXT, OUTPUT); // Vext Pin als Output
  digitalWrite(PIN_VEXT, LOW);   // LOW => Vext ON
  
  // Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(BUTTON_PIN, isr_button, FALLING);
  delay(50);

  // I2C OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);

  // OLED sauber resetten (manche Busse hängen nach Reset)
/*   pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, 1);
  delay(1);
  digitalWrite(OLED_RST, 0);
  delay(20);
  digitalWrite(OLED_RST, 1);
  delay(10); */

  u8g2.begin();

  // kann weg, da in lora_link_core.h beginCommon integriert
/*   // MAC lesen
  if (LoraLink::readEfuseMac6(myMac6)) {
    macReadOK = true;
    LoraLink::last3FromMac6(myLast3, myMac6);
  } else {
    macReadOK = false;
    memset(myLast3, 0, 3);
  } */

  // LoRa
  lora_init();
  cb.onTxStart       = on_tx_start_cb;
  cb.onTxDone        = on_tx_done_cb;
  cb.onRxWindowOpen  = on_rx_open_cb;
  cb.onRxPacket      = on_rx_packet_cb;
  cb.onRxWindowClosed= on_rx_closed_cb;
  cb.onIdle          = on_idle_cb;
  cb.ctx             = nullptr;       // oder &deinContext

  // Startanzeige
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "LoRa Init...");
  u8g2.sendBuffer();
  delay(150);
  drawStatus();
}

void loop() {

  recvSerialBytes();
  handleCommand();
  LoraLink::service(ll, cb); // an erster stelle??
  sync_service(ll);
  
  // Button: Falling erkannt?
  if (btnFallingFlag) {
    btnFallingFlag = false;
    // Start der Messung – warten auf Loslassen in Polling
    pressStartMs = millis();
    longHandled = false;
  }

  // Loslassen erkennen & Dauer auswerten
  if (pressStartMs != 0) {
    bool pressed = (digitalRead(BUTTON_PIN) == LOW);
    unsigned long now = millis();

    // Long-Press Fire (einmalig)
    if (pressed && !longHandled && (now - pressStartMs) >= 1000) {
      longHandled = true;
      // Long: do something
      showDebug = !showDebug;
      if (!showDebug) {
        inhibitStatusDraw = false;
        drawStatus();
      }
    }
    // Release → Short, falls kein Long
    if (!pressed) {
      if (!longHandled && (now - pressStartMs) >= 30) {
        // LONG: CONTROL an lastRespondent
        inhibitStatusDraw = false;
        drawStatus();
      }
      pressStartMs = 0;
      longHandled  = false;
    }
  }
}
