// lora_link_core.h -- shared LoRa "link layer" for ESP32 + SX1262 (RadioLib)
// Header-only, Arduino-friendly. No heap allocations.
// - Unifies DIO1 ISR flagging + state machine (Idle/Tx/Rx)
// - Shared TX scheduler (with optional random backoff)
// - Shared RX window (finite like Master) OR continuous (like Node/WLED)
// - Small MAC / address helpers
//
// How to use (short):
//   1) Board code creates SX1262 radio with its own pins (Module(cs,dio1,rst,busy,spi)).
//   2) Call LoraLink::beginCommon(radio, cfg) with PHY config (freq, bw, sf, cr, ...).
//      Optionally leave device-specific options unset: txPowerDbm/dio2RfSwitch/rxBoost have defaults.
//   3) Keep one LoraLink::Core 'll' per device. Attach ISR once via LoraLink::attachDio1(radio, ll).
//   4) Define callbacks (onRxPacket, onTxDone, ...).
//   5) In loop(): LoraLink::service(ll, cb).
//   6) For Master-style RX window: LoraLink::requestRxWindow(ll, windowMs).
//      For Node continuous RX: LoraLink::requestRxWindow(ll, 0) once (or at setup()).
//   7) For sending: schedule with LoraLink::scheduleSend(...) or scheduleSendWithBackoff(...).
//
// Notes:
// - This file intentionally does NOT know the pinout. Keep Module() creation in each firmware.
// - Works with RadioLib SX1262. Requires <RadioLib.h> and <Arduino.h>.
// - Uses a single static ISR trampoline; ok because Master and Node are separate binaries.
// - Keep packets <= 32 bytes here to be safe.
//
// License: MIT
#pragma once

// --- shield RadioLib from global "#define random" in WLED -----------------
#ifdef random
  #undef random
  #define LL__RESTORE_RANDOM_MACRO_AFTER_RADIOLIB 1
#endif

#include <Arduino.h>

#include <RadioLib.h>

#include "lora_proto.h"
extern "C" {
  #include <esp_mac.h>
}

namespace LoraLink {

// -------------------- PHY config with device-specific overrides --------------------
struct PhyCfg {
  float   freqMHz      = 867.7f; //868.0f;
  float   bwKHz        = 125.0f;
  uint8_t sf           = 7;     // LoRa spreading factor
  uint8_t crDen        = 5;     // LoRa coding rate denominator (5 => 4/5)
  uint8_t syncWord     = 0x12;
  uint16_t preamble    = 8;
  bool    crcOn        = true;

  // Device-specific knobs (optional). If left "unset", defaults apply.
  // For txPowerDbm: INT8_MIN means "use default 14 dBm".
  int8_t  txPowerDbm   = INT8_MIN;

  // Tri-state flags: -1 = don't touch / leave RadioLib default, 0 = disable, 1 = enable
  int8_t  dio2RfSwitch = -1;    // radio.setDio2AsRfSwitch(...)
  int8_t  rxBoost      = -1;    // radio.setRxBoostedGainMode(...)
};

// Forward declaration
struct Core;

// -------------------- Callbacks --------------------
struct Callbacks {
  void (*onRxPacket)(const uint8_t* pkt, uint8_t len, int16_t rssi, int8_t snr, void* ctx) = nullptr;
  void (*onTxStart)(void* ctx) = nullptr;
  void (*onTxDone)(void* ctx) = nullptr;
  void (*onRxWindowOpen)(uint16_t ms, void* ctx) = nullptr;
  void (*onRxWindowClosed)(uint16_t rxCountDelta, void* ctx) = nullptr;
  void (*onIdle)(void* ctx) = nullptr;

  void* ctx = nullptr;
};


// -------------------- Mode --------------------
enum class Mode : uint8_t { Idle, Tx, Rx };
enum class RxKind : uint8_t { None, Timed, Continuous };
enum class TxArbiter : uint8_t { None, CadNeeded, CadPending };

// -------------------- Core state --------------------
struct Core {
  
  #if defined(GATE_LORA_SX1262)
    SX1262*   radio             = nullptr;
  #elif defined(GATE_LORA_LLCC68)
    LLCC68*   radio             = nullptr;
  #else
    #error "No LoRa radio module defined"
  #endif
  
  volatile bool dio1Flag      = false;
  volatile uint32_t irqFlags  = 0;

  uint8_t   myMac6[6]     = {0};            // eigene MAC-Adresse (EFUSE)  
  uint8_t   myLast3[3]    = {0};
  bool      macReadOK     = false;

  Mode      rfMode       = Mode::Idle;     // Idle, Tx, Rx
  bool      changeMode    = true;           // apply initial mode on first service() call

  // --- RX-Status ---
  RxKind    rxKind            = RxKind::None;   // aktueller RX-Typ (nur wenn rfMode==Rx)
  uint32_t  rxWindowEndMs     = 0;              // nur für RxTimed
  int8_t    rxNumWanted       = -1;             // Anzahl erwarteter Antworten im Timed-RX (-1=unbegrenzt)
  uint16_t  rxLbtTimeout      = 700;            // nur für RxTimed mit LBT (ms), maximale Zeit zwischen Paketen
  uint16_t  rxCountWinStart   = 0;

  // --- RX-Request (Wunsch) ---
  RxKind    reqRxKind     = RxKind::None;   // gewünschter RX-Typ
  uint16_t  reqRxMs       = 0;              // nur wenn reqRxKind==Timed

  // --- Default-Modus pro Gerät ---
  RxKind    defaultRxKind = RxKind::None;   // Master: None (Idle), Slave: Continuous
  uint16_t  defaultRxMs   = 500;              // für default Continuous=0 (echt kontinuierlich), für default Timed optional

  // --- TX-Queue ---
  bool      txPending       = false;
  uint8_t   txBuf[64];
  uint8_t   txLen           = 0;
  uint32_t  earliestTxAtMs  = 0;

  // Telemetrie
  int16_t   lastRssi      = 0;
  int8_t    lastSnr       = 0;
  uint16_t  rxCountTotal       = 0;
  uint16_t  rxCountFiltered    = 0;
  uint32_t  lastRxAtMs    = 0;
  uint16_t  txCount       = 0;
  uint32_t  lastTxAtMs    = 0;

  // --- Stream state (max 16 packets * 8 bytes = 128 bytes) ---
  enum class StreamMode : uint8_t { None, Rx, Tx };
  StreamMode streamMode          = StreamMode::None;
  bool      streamActive         = false;
  bool      streamReady          = false;
  bool      streamLastScheduled  = false;
  uint8_t   streamBuf[128]       = {0};
  uint8_t   streamLen            = 0;
  uint8_t   streamOffset         = 0;
  uint8_t   streamLastPacketsLeft = 0;
  uint8_t   streamTotalPackets   = 0;
  uint8_t   streamIndex          = 0;
  uint16_t  streamPostRxMs       = 0;
  int8_t    streamPostRxNumWanted = -1;
  uint8_t   streamDst3[3]        = {0};
  uint8_t   streamSrc3[3]        = {0};
  uint8_t   streamType           = 0;

  // --- LBT / Arbiter / ToA ---
  bool       lbtEnable   = true;                // im Setup setzen
  bool       lbtRxRelax  = true;              // LBT-Backoff (µs)
  TxArbiter  txArb       = TxArbiter::None;     // LBT-Status
  uint32_t   toaUsMax17  = 0;                   // ToA-Cache (µs) für 17-Byte-Paket // 51ms for 17B @ SF7BW125CR45

  uint16_t   debug = 0;

};

// -------------------- Static ISR trampoline --------------------
#if defined(ESP32)
  #define LL_ISR_ATTR IRAM_ATTR
#else
  #define LL_ISR_ATTR
#endif

// Each binary has at most one radio/Core pair ⇒ one trampoline is sufficient.
static Core* volatile g_ll = nullptr;

static void LL_ISR_ATTR onDio1ISR_trampoline() {
  if (g_ll) {
    g_ll->dio1Flag = true;
    //g_ll->irqFlags = g_ll->radio->getIrqFlags();
  }
}

// -------------------- Helpers --------------------
// -------------------- Address utilities --------------------
inline bool readEfuseMac6(uint8_t mac6[6]) {
#if defined(ESP_PLATFORM) || defined(ESP32)
  if (esp_read_mac(mac6, ESP_MAC_WIFI_STA) == ESP_OK) return true;
#else
  String m = WiFi.macAddress();
  if (m.length() == 17) {
    for (int i=0;i<6;i++) mac6[i] = strtoul(m.substring(3*i, 3*i+2).c_str(), nullptr, 16);
    return true;
  }
#endif
  return false;
}

inline void last3FromMac6(uint8_t out[3], const uint8_t mac6[6]) {
  out[0] = mac6[3]; out[1] = mac6[4]; out[2] = mac6[5];
}

/* inline void mac6_to_str(const uint8_t m[6], char out[18]) {
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
} */

inline void mac6ToStr(const uint8_t mac6[6], char out[18]) {
  static const char HEX_lookup[] = "0123456789ABCDEF";
  for (int i=0,j=0;i<6;i++) {
    uint8_t v = mac6[i];
    out[j++] = HEX_lookup[(v>>4)&0xF];
    out[j++] = HEX_lookup[v&0xF];
    if (i<5) out[j++] = ':';
  }
  out[17] = '\0';
}

// -------------------- Receiver/Broadcast helpers --------------------
inline bool same3(const uint8_t a[3], const uint8_t b[3]) {
  return a[0]==b[0] && a[1]==b[1] && a[2]==b[2];
}

inline bool isBroadcast3(const uint8_t last3[3]) {
  return last3[0]==0xFF && last3[1]==0xFF && last3[2]==0xFF;
}

inline bool receiverMatches(const uint8_t receiver3[3], const uint8_t myLast3[3]) {
  return isBroadcast3(receiver3) || same3(receiver3, myLast3);
}

// Maximaler LBT-Backoff in Millisekunden basierend auf time-on-air für das längste Paket
// (für 17-Byte-Paket ca. 51 ms bei SF7BW125CR45)
inline uint16_t lbtBackoffMaxMs(const Core& ll) {
  uint32_t ms = ll.toaUsMax17 / 1000U;   // floor(ToA/1000)
  return (ms < 5U) ? 5U : (uint16_t)ms;  // mind. 5 ms
}

// RadioLib RNG: random(minMs, maxMs). Inklusive oberer Rand => maxMs+1
inline uint16_t randMs(Core& ll, uint16_t minMs, uint16_t maxMs) {
  if (maxMs <= minMs) return minMs;
  const int32_t lo = (int32_t)minMs;
  const int32_t hiExclusive = (int32_t)maxMs + 1;
  int32_t r = ll.radio
                ? ll.radio->random(lo, hiExclusive)      // PhysicalLayer::random
                : (int32_t)::random((long)lo, (long)hiExclusive);
  return (uint16_t)r;
}

// -------------------- RX/TX mode helpers --------------------
inline void setDefaultIdle(Core& ll) {
  ll.defaultRxKind = RxKind::None;
  ll.defaultRxMs   = 0;
}
inline void setDefaultRxContinuous(Core& ll) {
  ll.defaultRxKind = RxKind::Continuous;
  ll.defaultRxMs   = 0; // echt kontinuierlich
  ll.reqRxKind = RxKind::Continuous;
  ll.reqRxMs = 0;
}
inline void requestRxTimed(Core& ll, uint16_t windowMs, int8_t rxNumWanted = -1) {
  ll.reqRxKind = RxKind::Timed;
  ll.reqRxMs = windowMs;
  ll.rxNumWanted = rxNumWanted;
  ll.changeMode = true; // force window (re)open even if already in Timed RX
}
inline void requestRxContinuous(Core& ll) {
  ll.reqRxKind = RxKind::Continuous;
  ll.reqRxMs = 0;
}
inline void cancelRxRequest(Core& ll) {
  ll.changeMode = true;
  ll.rfMode = Mode::Idle;
  ll.reqRxKind = RxKind::None;
  ll.reqRxMs = 0;
}

// One-slot TX scheduling (returns false if slot busy or oversize).
// with LBT enabled jitterMaxMs is overridden with 300ms -> do it based on ToA?!
// without LBT and jitterMaxMs>50 the given jitterMaxMs is used to delay the TX
// without LBT and jitterMaxMs=0 the TX is scheduled immediately
inline bool scheduleSend(Core& ll, const uint8_t* buf, uint8_t len, uint16_t jitterMaxMs = 2500) {

  if (ll.txPending || len == 0 || len > sizeof(ll.txBuf)) return false; // Check for pending TX or oversize
  memcpy(ll.txBuf, buf, len);
  ll.txLen = len;
  ll.earliestTxAtMs = millis();
  
  uint16_t jitterMinMs = 50; // default min jitter

  if (ll.lbtEnable) {
    //jitterMaxMs = lbtBackoffMaxMs(ll);
    jitterMaxMs = 300; // fixed max backoff for LBT
    uint16_t randDelayMs = randMs(ll, jitterMinMs, jitterMaxMs);
    //ll.debug = randDelayMs;
    ll.earliestTxAtMs += randDelayMs;
    ll.txArb = TxArbiter::CadNeeded;
  } 
  else {
    if (jitterMaxMs == 0) {
      ll.earliestTxAtMs += 0; // no delay
    }
    else if (jitterMaxMs > jitterMinMs) {
      ll.earliestTxAtMs += randMs(ll, jitterMinMs, jitterMaxMs);
    }
    else {
      ll.earliestTxAtMs += randMs(ll, jitterMinMs, 300); // at least some jitter
    }
    ll.txArb = TxArbiter::None;
  }

  ll.debug = 0;
  ll.txPending = true; // mark TX as pending
  return true;
}

inline bool scheduleSendThenRxWindow(Core& ll, const uint8_t* buf, uint8_t len, uint16_t rxMs) {
  // TODO: so noch ok oder muss an LBT angepasst werden? Wird aktuell nicht genutzt.
  if (!scheduleSend(ll, buf, len)) return false;
  requestRxTimed(ll, rxMs);
  return true;
}

// Small wrapper for building + scheduling a typed payload (using LoraProto).
template<typename PayloadT>
inline bool buildAndSchedule(Core& ll, const uint8_t my3[3], const uint8_t dst3[3],
                             uint8_t fullType, const PayloadT& p) {
  uint8_t out[sizeof(LoraProto::Header7) + sizeof(PayloadT)];
  uint8_t n = LoraProto::build(out, my3, dst3, fullType, p);
  return scheduleSend(ll, out, n);
}

inline bool buildEmptyAndSchedule(Core& ll, const uint8_t my3[3], const uint8_t dst3[3],
                                  uint8_t fullType) {
  uint8_t out[sizeof(LoraProto::Header7)];
  uint8_t n = LoraProto::build_empty(out, my3, dst3, fullType);
  return scheduleSend(ll, out, n);
}

// -------------------- Stream helpers --------------------
enum class StreamStatus : uint8_t { StreamStart, StreamContinue, StreamEnd, Error };

inline const uint8_t* streamBuffer(const Core& ll, uint8_t& len) {
  len = ll.streamLen;
  return ll.streamBuf;
}

inline void clearStreamReady(Core& ll) {
  ll.streamReady = false;
}

inline bool scheduleStreamSend(Core& ll, const uint8_t* data, uint8_t len,
                               const uint8_t src3[3], const uint8_t dst3[3],
                               uint8_t fullType, uint16_t rxMs, int8_t rxNumWanted = 1) {
  constexpr uint8_t kChunkLen = sizeof(LoraProto::P_Stream::data);
  constexpr uint8_t kMaxLen = 16 * kChunkLen;
  if (ll.streamActive || ll.txPending || ll.streamMode != Core::StreamMode::None) return false;
  if (!data || len == 0 || len > kMaxLen) return false;
  const uint8_t totalPackets = static_cast<uint8_t>((len + kChunkLen - 1U) / kChunkLen);
  if (totalPackets < 2 || totalPackets > 16) return false;

  const uint8_t paddedLen = static_cast<uint8_t>(totalPackets * kChunkLen);
  memset(ll.streamBuf, 0, paddedLen);
  memcpy(ll.streamBuf, data, len);
  ll.streamMode = Core::StreamMode::Tx;
  ll.streamActive = true;
  ll.streamReady = false;
  ll.streamLastScheduled = false;
  ll.streamLen = paddedLen;
  ll.streamOffset = 0;
  ll.streamTotalPackets = totalPackets;
  ll.streamIndex = 0;
  ll.streamPostRxMs = rxMs;
  ll.streamPostRxNumWanted = rxNumWanted;
  ll.streamType = fullType;
  memcpy(ll.streamDst3, dst3, sizeof(ll.streamDst3));
  memcpy(ll.streamSrc3, src3, sizeof(ll.streamSrc3));
  return true;
}

inline bool queueNextStreamPacket(Core& ll) {
  constexpr uint8_t kChunkLen = sizeof(LoraProto::P_Stream::data);
  if (ll.streamMode != Core::StreamMode::Tx || !ll.streamActive || ll.txPending) return false;
  if (ll.streamIndex >= ll.streamTotalPackets) return false;

  const bool isStart = (ll.streamIndex == 0);
  const bool isStop = (ll.streamIndex == static_cast<uint8_t>(ll.streamTotalPackets - 1));
  const uint8_t packetsLeft = static_cast<uint8_t>((ll.streamTotalPackets - 1) - ll.streamIndex);

  LoraProto::P_Stream p{};
  p.ctrl = LoraProto::encode_stream_ctrl(isStart, isStop, packetsLeft);
  memcpy(p.data, &ll.streamBuf[ll.streamOffset], kChunkLen);

  uint8_t out[sizeof(LoraProto::Header7) + sizeof(LoraProto::P_Stream)];
  uint8_t n = LoraProto::build(out, ll.streamSrc3, ll.streamDst3, ll.streamType, p);
  if (!scheduleSend(ll, out, n, 0)) return false;

  ll.streamOffset += kChunkLen;
  ++ll.streamIndex;
  ll.streamLastScheduled = isStop;
  return true;
}

inline StreamStatus handleStreamPacket(Core& ll, const LoraProto::P_Stream& pkt) {
  constexpr uint8_t kStreamDataLen = sizeof(pkt.data);
  constexpr uint8_t kMaxPackets = 16;
  const LoraProto::StreamCtrl ctrl = LoraProto::decode_stream_ctrl(pkt.ctrl);

  if (ctrl.packets_left >= kMaxPackets) return StreamStatus::Error;
  if (ll.streamMode == Core::StreamMode::Tx) return StreamStatus::Error;

  if (ctrl.start) {
    if (ctrl.stop || ctrl.packets_left == 0) return StreamStatus::Error;
    ll.streamMode = Core::StreamMode::Rx;
    ll.streamActive = true;
    ll.streamReady = false;
    ll.streamLen = 0;
    ll.streamOffset = 0;
    ll.streamLastPacketsLeft = ctrl.packets_left;
    if (ll.streamOffset + kStreamDataLen > sizeof(ll.streamBuf)) return StreamStatus::Error;
    memcpy(&ll.streamBuf[ll.streamOffset], pkt.data, kStreamDataLen);
    ll.streamOffset += kStreamDataLen;
    ll.streamLen = ll.streamOffset;
    return StreamStatus::StreamStart;
  }

  if (!ll.streamActive) return StreamStatus::Error;

  if (ctrl.stop) {
    if (ctrl.packets_left != 0 || ll.streamLastPacketsLeft != 1) return StreamStatus::Error;
    if (ll.streamOffset + kStreamDataLen > sizeof(ll.streamBuf)) return StreamStatus::Error;
    memcpy(&ll.streamBuf[ll.streamOffset], pkt.data, kStreamDataLen);
    ll.streamOffset += kStreamDataLen;
    ll.streamLen = ll.streamOffset;
    ll.streamActive = false;
    ll.streamReady = true;
    ll.streamLastPacketsLeft = 0;
    ll.streamMode = Core::StreamMode::None;
    return StreamStatus::StreamEnd;
  }

  if (ctrl.packets_left == 0) return StreamStatus::Error;
  if (ctrl.packets_left != static_cast<uint8_t>(ll.streamLastPacketsLeft - 1)) return StreamStatus::Error;
  if (ll.streamOffset + kStreamDataLen > sizeof(ll.streamBuf)) return StreamStatus::Error;
  memcpy(&ll.streamBuf[ll.streamOffset], pkt.data, kStreamDataLen);
  ll.streamOffset += kStreamDataLen;
  ll.streamLen = ll.streamOffset;
  ll.streamLastPacketsLeft = ctrl.packets_left;
  return StreamStatus::StreamContinue;
}

// -------------------- Radio initialization common code --------------------
#if defined(GATE_LORA_SX1262)
inline bool beginCommon(SX1262& radio, Core& ll, const PhyCfg& cfg) {
#elif defined(GATE_LORA_LLCC68)
inline bool beginCommon(LLCC68& radio, Core& ll, const PhyCfg& cfg) {
#else
  #error "No LoRa radio module defined"
#endif
//inline bool beginCommon(SX1262& radio, Core& ll, const PhyCfg& cfg) {
  const int8_t power = (cfg.txPowerDbm == INT8_MIN) ? 14 : cfg.txPowerDbm;

  int16_t st = radio.begin(cfg.freqMHz, cfg.bwKHz, cfg.sf, cfg.crDen,
                           cfg.syncWord, power, cfg.preamble);
  if (st != RADIOLIB_ERR_NONE) return false;

  if (cfg.crcOn) radio.setCRC(true); else radio.setCRC(false);
  if (cfg.dio2RfSwitch != -1) radio.setDio2AsRfSwitch(cfg.dio2RfSwitch == 1);
  if (cfg.rxBoost != -1)      radio.setRxBoostedGainMode(cfg.rxBoost == 1);

  radio.standby();        // ensure standby after init

  if(readEfuseMac6(ll.myMac6)) {
    ll.macReadOK = true;
    last3FromMac6(ll.myLast3, ll.myMac6);
  }
  
  ll.radio = &radio;
  ll.toaUsMax17 = radio.getTimeOnAir(16);   // µs // 51ms for 16B @ SF7BW125CR45
  g_ll = &ll;

  return true;
}

#if defined(GATE_LORA_SX1262)
inline void attachDio1(SX1262& radio, Core& ll) {
  radio.setDio1Action(onDio1ISR_trampoline);
}
#elif defined(GATE_LORA_LLCC68)
inline void attachDio1(LLCC68& radio, Core& ll) {
  radio.setDio1Action(onDio1ISR_trampoline);
}
#else
#error "No LoRa radio module defined"
#endif

// -------------------- The service pump (call in loop()) --------------------
inline void service(Core& ll, const Callbacks& cb) {
  const uint32_t now = millis();

  // (A) IRQ: TX-Done oder RX-Paket
  if (ll.dio1Flag) {
    ll.dio1Flag = false;

    if (ll.rfMode == Mode::Rx) {
      size_t len = ll.radio->getPacketLength();
      if (len >= sizeof(LoraProto::Header7)) {
        // TODO: ab hier nochmal checken! txBuf als maximale länge? -> besser hardcoden
        // können mehrere pakete im readData buffer enthalten sein?

        if (len > sizeof(ll.txBuf)) len = sizeof(ll.txBuf);
        uint8_t pkt[64]; if (len > sizeof(pkt)) len = sizeof(pkt);
        
        if (ll.radio->readData(pkt, len) == RADIOLIB_ERR_NONE) {
          ++ll.rxCountTotal;

          // Try filtering out unwanted packets early disabled for debugging
          LoraProto::Header7 h{};
          if (!LoraProto::parseHeader(pkt, (uint8_t)len, h)) return;
          if (!receiverMatches(h.receiver, ll.myLast3)) return;  // broadcast ODER exakt meine 3B
          //if (LoraProto::type_dir(h.type) != LoraProto::DIR_M2N) return; // muss je nach rolle im hauptcode geprüft werden

          ll.lastRssi = (int16_t)ll.radio->getRSSI(true);
          ll.lastSnr  = (int8_t) ll.radio->getSNR();

          if(ll.rxNumWanted > 0) --ll.rxNumWanted; // nur wenn begrenzte Anzahl erwartet
         
          ++ll.rxCountFiltered;
          ll.lastRxAtMs = now;
          if (cb.onRxPacket) cb.onRxPacket(pkt, (uint8_t)len, ll.lastRssi, ll.lastSnr, cb.ctx);
        }
      }
      // Rx fortsetzen nicht nötig (nutze immer continuous RX)
      /* if (!ll.txPending && ll.rxKind != RxKind::None) {
        ll.radio->startReceive(); // nicht nötig, startReceive löst continuous RX aus
      } */
    }
  }

  if (!ll.txPending && ll.streamMode == Core::StreamMode::Tx && ll.streamActive) {
    queueNextStreamPacket(ll);
  }

  // (B) Wenn im Idle, dann gewünschten Modus prüfen und wechseln
  if(ll.rfMode == Mode::Idle) {
    // Idle → gewünschten Modus prüfen

    if (ll.txPending) {
      // TX steht an
      ll.rfMode = Mode::Tx;
      ll.changeMode = true;
      //ll.reqRxKind = RxKind::None;
      //ll.reqRxMs = 0;
      return;
    }
    else if (ll.reqRxKind == RxKind::None && 
      (ll.reqRxKind != ll.rxKind || ll.changeMode)) {

      // kein RX gewünscht, Idle festigen
      ll.radio->standby();
      ll.rxKind = RxKind::None;
      ll.changeMode = false;
      //ll.reqRxMs = 0; //unnötig
      if (cb.onIdle) cb.onIdle(cb.ctx);
      return; // fertig
    }
    else if (ll.reqRxKind != RxKind::None) {

      // RX gewünscht
      ll.rfMode = Mode::Rx;
      ll.changeMode = true;
      return; // fertig
    }
  }

  // (B) TX pending starten, wenn sendezeit erreicht, aber kein timed RX läuft
  //if (ll.txPending && ll.rxKind != RxKind::Timed && (int32_t)(now - ll.earliestTxAtMs) >= 0) {
  if (ll.rfMode == Mode::Tx) {
    
    if (ll.changeMode) {
      ll.radio->standby(); // sicherstellen, dass nichts mehr empfangen wird bis earliestTxAtMs
      ll.changeMode = false; // TX jetzt durchziehen
      if (cb.onTxStart) cb.onTxStart(cb.ctx);
    }

    if((int32_t)(now - ll.earliestTxAtMs) < 0) {
      return; // noch nicht Zeit zum Senden
    }

    // LBT / CAD wenn nötig
    if (ll.txArb == TxArbiter::CadNeeded) {
      // zum senden in scheduleSend nur txArb auf CadNeeded und txPending auf true setzen
      ChannelScanConfig_t cfg = {
        .cad = {
          .symNum = RADIOLIB_SX126X_CAD_ON_4_SYMB,   // robuste Defaults RADIOLIB_SX126X_CAD_ON_4_SYMB RADIOLIB_SX126X_CAD_ON_2_SYMB // RADIOLIB_SX126X_CAD_PARAM_DEFAULT
          .detPeak = 18, //16, // RADIOLIB_SX126X_CAD_PARAM_DEFAULT // default: SF+13=20
          .detMin = 10, //12, //RADIOLIB_SX126X_CAD_PARAM_DET_MIN, // RADIOLIB_SX126X_CAD_PARAM_DEFAULT //default: 10 // 5 zu niedrig
          .exitMode = RADIOLIB_SX126X_CAD_GOTO_STDBY, // RADIOLIB_SX126X_CAD_PARAM_DEFAULT
          .timeout = 0,
          .irqFlags = RADIOLIB_IRQ_CAD_DEFAULT_FLAGS,
          .irqMask = RADIOLIB_IRQ_CAD_DEFAULT_MASK,
        },
      };

      //int16_t state = ll.radio->startChannelScan(cfg);
      int16_t state = ll.radio->scanChannel(cfg);
      
      if (state != RADIOLIB_CHANNEL_FREE) {
        // busy → kurzen Backoff und erneut CAD wenn Zeit erreicht
        ll.earliestTxAtMs = now + randMs(ll, 100, 200); // fixed backoff for LBT
        ll.txArb = TxArbiter::CadNeeded;
        ll.debug += 1; // debug counter für busy CAD
        //if(ll.debug <= 2) ll.debug = 2;
        return; // kein weiterer Service jetzt
      }
      else {
        ll.txArb = TxArbiter::None;
        // weiter zum senden
      }
    }
    if(ll.txArb == TxArbiter::None) {
      //if (cb.onTxStart) cb.onTxStart(cb.ctx); // schon bei CAD aufrufen?
      if (ll.radio->transmit(ll.txBuf, ll.txLen) == RADIOLIB_ERR_NONE) {
        ll.txPending = false;
        //ll.reqRxKind = ll.defaultRxKind; // nach TX wieder in default RX modus wechseln
        //ll.reqRxMs   = ll.defaultRxMs;
        
        ++ll.txCount;
        ll.lastTxAtMs = now;
        
        //ll.debug = 100;
        if (cb.onTxDone) cb.onTxDone(cb.ctx);        
        
        if (ll.streamMode == Core::StreamMode::Tx && ll.streamLastScheduled) {
          ll.streamMode = Core::StreamMode::None;
          ll.streamActive = false;
          ll.streamLastScheduled = false;
          ll.streamLen = 0;
          ll.streamOffset = 0;
          ll.streamTotalPackets = 0;
          ll.streamIndex = 0;
          ll.streamType = 0;
          if (ll.streamPostRxMs > 0) {
            requestRxTimed(ll, ll.streamPostRxMs, ll.streamPostRxNumWanted);
          }
          ll.streamPostRxMs = 0;
          ll.streamPostRxNumWanted = -1;
        }

        ll.rfMode = Mode::Idle;
        ll.changeMode = true;

        return; // gesendet, nun platz machen für restlichen code
      }
      else {
        ll.txArb = TxArbiter::CadNeeded;
        return;
      }
    }
  }
  // (D) RX-Requests bedienen
  if (ll.rfMode == Mode::Rx) {
    
    // check TX pending (especially for continuous RX)
    if (ll.txPending) {
      if (ll.rxKind == RxKind::Timed) {
        const uint16_t delta = ll.rxCountFiltered - ll.rxCountWinStart;
        if (cb.onRxWindowClosed) cb.onRxWindowClosed(delta, cb.ctx);
      }
      ll.rxKind = RxKind::None;
      ll.rxWindowEndMs = 0;
      ll.rxNumWanted = -1;
      ll.rfMode = Mode::Idle; // Wechsel zu Tx ermöglichen
      ll.radio->standby(); // Empfang sofort beenden, nicht erst wenn earliestTxAtMs erreicht ist
      ll.changeMode = true;
      return;
    }

    // RX-Window Timeout prüfen und ggf. beenden
    if (ll.rxKind == RxKind::Timed && ll.rxWindowEndMs > 0) {
      // RX-Timed: laufendes Fenster -> Fensterende prüfen
      
      if ((int32_t)(now - ll.rxWindowEndMs) >= 0 || ll.rxNumWanted == 0) {
        // Fensterende erreicht oder alle Antworten empfangen

        if(ll.lbtRxRelax && ll.rxNumWanted != 0) {
          // LBT Rx-Relax: RX fortsetzen bis seit dem letzten Paket länger als rxLbtTimeout ms vergangen sind
          // Ausnahme: wenn rxNumWanted==0 (alle Antworten empfangen)
          const uint32_t deltaSinceLastRx = now - ll.lastRxAtMs;
          if (deltaSinceLastRx < ll.rxLbtTimeout) {
            // noch nicht timeout
            return; // nichts tun, RX läuft weiter
          }
        }

        ll.rfMode = Mode::Idle;
        ll.reqRxKind = ll.defaultRxKind; // nach RX wieder in default modus wechseln
        ll.reqRxMs   = ll.defaultRxMs;
        ll.changeMode = true;
        ll.rxWindowEndMs = 0;
        ll.rxNumWanted = -1;
        
        const uint16_t delta = ll.rxCountFiltered - ll.rxCountWinStart;
        if (cb.onRxWindowClosed) cb.onRxWindowClosed(delta, cb.ctx);

        return; // fertig
      }

      return; // laufendes Fenster noch nicht beendet
    }

    if (ll.changeMode) {
      // komme von Idle: Continous RX starten
      if(ll.radio->startReceive() != RADIOLIB_ERR_NONE) {
        // RX nicht gestartet
        return; // fehler, wird im nächsten durchgang erneut versucht
      }
    }

    if (ll.changeMode || ll.rxKind != ll.reqRxKind) {
      // von einem rx modus in einen anderen wechseln

      if(ll.reqRxKind==RxKind::Timed) {
        // setze rxWindowEndMs reqRxMs
        // bei LBT automatisch kürzeres rxWindow? -> bei scheduleSend entsprechend anpassen
        ll.rxCountWinStart = ll.rxCountFiltered;
        const uint16_t w = (ll.reqRxMs ? ll.reqRxMs : ll.defaultRxMs);
        ll.rxWindowEndMs = now + w;
        if (cb.onRxWindowOpen) cb.onRxWindowOpen(w, cb.ctx);
      }
      else if(ll.reqRxKind==RxKind::Continuous) {
        ll.rxWindowEndMs = 0;
        if (cb.onRxWindowOpen) cb.onRxWindowOpen(0, cb.ctx);
      }
      ll.rxKind = ll.reqRxKind; // nach übernahme des neuen modus setzen
      ll.changeMode = false; // mode change done
    }
  }
}

} // namespace LoraLink

#ifdef LL__RESTORE_RANDOM_MACRO_AFTER_RADIOLIB
  #define random hw_random // replace arduino random()
  #undef LL__RESTORE_RANDOM_MACRO_AFTER_RADIOLIB
#endif
