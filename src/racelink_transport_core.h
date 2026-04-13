// racelink_transport_core.h -- shared RaceLink transport core for ESP32 + SX1262 (RadioLib)
// Header-only, Arduino-friendly. No heap allocations.
// - Unifies DIO1 ISR flagging + state machine (Idle/Tx/Rx)
// - Shared TX scheduler (with optional random backoff)
// - Shared RX window (finite like Master) OR continuous (like Node/WLED)
// - Small MAC / address helpers
//
// How to use (short):
//   1) Board code creates SX1262 radio with its own pins (Module(cs,dio1,rst,busy,spi)).
//   2) Call RaceLinkTransport::beginCommon(radio, cfg) with PHY config (freq, bw, sf, cr, ...).
//      Optionally leave device-specific options unset: txPowerDbm/dio2RfSwitch/rxBoost have defaults.
//   3) Keep one RaceLinkTransport::Core 'rl' per device. Attach ISR once via RaceLinkTransport::attachDio1(radio, rl).
//   4) Define callbacks (onRxPacket, onTxDone, ...).
//   5) In loop(): RaceLinkTransport::service(rl, cb).
//   6) For Master-style RX window: RaceLinkTransport::requestRxWindow(rl, windowMs).
//      For Node continuous RX: RaceLinkTransport::requestRxWindow(rl, 0) once (or at setup()).
//   7) For sending: schedule with RaceLinkTransport::scheduleSend(...) or scheduleSendWithBackoff(...).
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
  #define RL__RESTORE_RANDOM_MACRO_AFTER_RADIOLIB 1
#endif

#include <Arduino.h>

#include <RadioLib.h>

#include "racelink_proto.h"
extern "C" {
  #include <esp_mac.h>
}

namespace RaceLinkTransport {

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
    #error "No supported radio module defined"
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
  #define RL_ISR_ATTR IRAM_ATTR
#else
  #define RL_ISR_ATTR
#endif

// Each binary has at most one radio/Core pair ⇒ one trampoline is sufficient.
static Core* volatile g_rl = nullptr;

static void RL_ISR_ATTR onDio1ISR_trampoline() {
  if (g_rl) {
    g_rl->dio1Flag = true;
    //g_rl->irqFlags = g_rl->radio->getIrqFlags();
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
inline uint16_t lbtBackoffMaxMs(const Core& rl) {
  uint32_t ms = rl.toaUsMax17 / 1000U;   // floor(ToA/1000)
  return (ms < 5U) ? 5U : (uint16_t)ms;  // mind. 5 ms
}

// RadioLib RNG: random(minMs, maxMs). Inklusive oberer Rand => maxMs+1
inline uint16_t randMs(Core& rl, uint16_t minMs, uint16_t maxMs) {
  if (maxMs <= minMs) return minMs;
  const int32_t lo = (int32_t)minMs;
  const int32_t hiExclusive = (int32_t)maxMs + 1;
  int32_t r = rl.radio
                ? rl.radio->random(lo, hiExclusive)      // PhysicalLayer::random
                : (int32_t)::random((long)lo, (long)hiExclusive);
  return (uint16_t)r;
}

// -------------------- RX/TX mode helpers --------------------
inline void setDefaultIdle(Core& rl) {
  rl.defaultRxKind = RxKind::None;
  rl.defaultRxMs   = 0;
}
inline void setDefaultRxContinuous(Core& rl) {
  rl.defaultRxKind = RxKind::Continuous;
  rl.defaultRxMs   = 0; // echt kontinuierlich
  rl.reqRxKind = RxKind::Continuous;
  rl.reqRxMs = 0;
}
inline void requestRxTimed(Core& rl, uint16_t windowMs, int8_t rxNumWanted = -1) {
  rl.reqRxKind = RxKind::Timed;
  rl.reqRxMs = windowMs;
  rl.rxNumWanted = rxNumWanted;
  rl.changeMode = true; // force window (re)open even if already in Timed RX
}
inline void requestRxContinuous(Core& rl) {
  rl.reqRxKind = RxKind::Continuous;
  rl.reqRxMs = 0;
}
inline void cancelRxRequest(Core& rl) {
  rl.changeMode = true;
  rl.rfMode = Mode::Idle;
  rl.reqRxKind = RxKind::None;
  rl.reqRxMs = 0;
}

// One-slot TX scheduling (returns false if slot busy or oversize).
// with LBT enabled jitterMaxMs is overridden with 300ms -> do it based on ToA?!
// without LBT and jitterMaxMs>50 the given jitterMaxMs is used to delay the TX
// without LBT and jitterMaxMs=0 the TX is scheduled immediately
inline bool scheduleSend(Core& rl, const uint8_t* buf, uint8_t len, uint16_t jitterMaxMs = 2500) {

  if (rl.txPending || len == 0 || len > sizeof(rl.txBuf)) return false; // Check for pending TX or oversize
  memcpy(rl.txBuf, buf, len);
  rl.txLen = len;
  rl.earliestTxAtMs = millis();
  
  uint16_t jitterMinMs = 50; // default min jitter

  if (rl.lbtEnable) {
    //jitterMaxMs = lbtBackoffMaxMs(rl);
    jitterMaxMs = 300; // fixed max backoff for LBT
    uint16_t randDelayMs = randMs(rl, jitterMinMs, jitterMaxMs);
    //rl.debug = randDelayMs;
    rl.earliestTxAtMs += randDelayMs;
    rl.txArb = TxArbiter::CadNeeded;
  } 
  else {
    if (jitterMaxMs == 0) {
      rl.earliestTxAtMs += 0; // no delay
    }
    else if (jitterMaxMs > jitterMinMs) {
      rl.earliestTxAtMs += randMs(rl, jitterMinMs, jitterMaxMs);
    }
    else {
      rl.earliestTxAtMs += randMs(rl, jitterMinMs, 300); // at least some jitter
    }
    rl.txArb = TxArbiter::None;
  }

  rl.debug = 0;
  rl.txPending = true; // mark TX as pending
  return true;
}

inline bool scheduleSendThenRxWindow(Core& rl, const uint8_t* buf, uint8_t len, uint16_t rxMs) {
  // TODO: so noch ok oder muss an LBT angepasst werden? Wird aktuell nicht genutzt.
  if (!scheduleSend(rl, buf, len)) return false;
  requestRxTimed(rl, rxMs);
  return true;
}

// Small wrapper for building + scheduling a typed payload (using RaceLinkProto).
template<typename PayloadT>
inline bool buildAndSchedule(Core& rl, const uint8_t my3[3], const uint8_t dst3[3],
                             uint8_t fullType, const PayloadT& p) {
  uint8_t out[sizeof(RaceLinkProto::Header7) + sizeof(PayloadT)];
  uint8_t n = RaceLinkProto::build(out, my3, dst3, fullType, p);
  return scheduleSend(rl, out, n);
}

inline bool buildEmptyAndSchedule(Core& rl, const uint8_t my3[3], const uint8_t dst3[3],
                                  uint8_t fullType) {
  uint8_t out[sizeof(RaceLinkProto::Header7)];
  uint8_t n = RaceLinkProto::build_empty(out, my3, dst3, fullType);
  return scheduleSend(rl, out, n);
}

// -------------------- Stream helpers --------------------
enum class StreamStatus : uint8_t { StreamStart, StreamContinue, StreamEnd, Error };

inline const uint8_t* streamBuffer(const Core& rl, uint8_t& len) {
  len = rl.streamLen;
  return rl.streamBuf;
}

inline void clearStreamReady(Core& rl) {
  rl.streamReady = false;
}

inline bool scheduleStreamSend(Core& rl, const uint8_t* data, uint8_t len,
                               const uint8_t src3[3], const uint8_t dst3[3],
                               uint8_t fullType, uint16_t rxMs, int8_t rxNumWanted = 1) {
  constexpr uint8_t kChunkLen = sizeof(RaceLinkProto::P_Stream::data);
  constexpr uint8_t kMaxLen = 16 * kChunkLen;
  if (rl.streamActive || rl.txPending || rl.streamMode != Core::StreamMode::None) return false;
  if (!data || len == 0 || len > kMaxLen) return false;
  const uint8_t totalPackets = static_cast<uint8_t>((len + kChunkLen - 1U) / kChunkLen);
  if (totalPackets < 2 || totalPackets > 16) return false;

  const uint8_t paddedLen = static_cast<uint8_t>(totalPackets * kChunkLen);
  memset(rl.streamBuf, 0, paddedLen);
  memcpy(rl.streamBuf, data, len);
  rl.streamMode = Core::StreamMode::Tx;
  rl.streamActive = true;
  rl.streamReady = false;
  rl.streamLastScheduled = false;
  rl.streamLen = paddedLen;
  rl.streamOffset = 0;
  rl.streamTotalPackets = totalPackets;
  rl.streamIndex = 0;
  rl.streamPostRxMs = rxMs;
  rl.streamPostRxNumWanted = rxNumWanted;
  rl.streamType = fullType;
  memcpy(rl.streamDst3, dst3, sizeof(rl.streamDst3));
  memcpy(rl.streamSrc3, src3, sizeof(rl.streamSrc3));
  return true;
}

inline bool queueNextStreamPacket(Core& rl) {
  constexpr uint8_t kChunkLen = sizeof(RaceLinkProto::P_Stream::data);
  if (rl.streamMode != Core::StreamMode::Tx || !rl.streamActive || rl.txPending) return false;
  if (rl.streamIndex >= rl.streamTotalPackets) return false;

  const bool isStart = (rl.streamIndex == 0);
  const bool isStop = (rl.streamIndex == static_cast<uint8_t>(rl.streamTotalPackets - 1));
  const uint8_t packetsLeft = static_cast<uint8_t>((rl.streamTotalPackets - 1) - rl.streamIndex);

  RaceLinkProto::P_Stream p{};
  p.ctrl = RaceLinkProto::encode_stream_ctrl(isStart, isStop, packetsLeft);
  memcpy(p.data, &rl.streamBuf[rl.streamOffset], kChunkLen);

  uint8_t out[sizeof(RaceLinkProto::Header7) + sizeof(RaceLinkProto::P_Stream)];
  uint8_t n = RaceLinkProto::build(out, rl.streamSrc3, rl.streamDst3, rl.streamType, p);
  if (!scheduleSend(rl, out, n, 0)) return false;

  rl.streamOffset += kChunkLen;
  ++rl.streamIndex;
  rl.streamLastScheduled = isStop;
  return true;
}

inline StreamStatus handleStreamPacket(Core& rl, const RaceLinkProto::P_Stream& pkt) {
  constexpr uint8_t kStreamDataLen = sizeof(pkt.data);
  constexpr uint8_t kMaxPackets = 16;
  const RaceLinkProto::StreamCtrl ctrl = RaceLinkProto::decode_stream_ctrl(pkt.ctrl);

  if (ctrl.packets_left >= kMaxPackets) return StreamStatus::Error;
  if (rl.streamMode == Core::StreamMode::Tx) return StreamStatus::Error;

  if (ctrl.start) {
    if (ctrl.stop || ctrl.packets_left == 0) return StreamStatus::Error;
    rl.streamMode = Core::StreamMode::Rx;
    rl.streamActive = true;
    rl.streamReady = false;
    rl.streamLen = 0;
    rl.streamOffset = 0;
    rl.streamLastPacketsLeft = ctrl.packets_left;
    if (rl.streamOffset + kStreamDataLen > sizeof(rl.streamBuf)) return StreamStatus::Error;
    memcpy(&rl.streamBuf[rl.streamOffset], pkt.data, kStreamDataLen);
    rl.streamOffset += kStreamDataLen;
    rl.streamLen = rl.streamOffset;
    return StreamStatus::StreamStart;
  }

  if (!rl.streamActive) return StreamStatus::Error;

  if (ctrl.stop) {
    if (ctrl.packets_left != 0 || rl.streamLastPacketsLeft != 1) return StreamStatus::Error;
    if (rl.streamOffset + kStreamDataLen > sizeof(rl.streamBuf)) return StreamStatus::Error;
    memcpy(&rl.streamBuf[rl.streamOffset], pkt.data, kStreamDataLen);
    rl.streamOffset += kStreamDataLen;
    rl.streamLen = rl.streamOffset;
    rl.streamActive = false;
    rl.streamReady = true;
    rl.streamLastPacketsLeft = 0;
    rl.streamMode = Core::StreamMode::None;
    return StreamStatus::StreamEnd;
  }

  if (ctrl.packets_left == 0) return StreamStatus::Error;
  if (ctrl.packets_left != static_cast<uint8_t>(rl.streamLastPacketsLeft - 1)) return StreamStatus::Error;
  if (rl.streamOffset + kStreamDataLen > sizeof(rl.streamBuf)) return StreamStatus::Error;
  memcpy(&rl.streamBuf[rl.streamOffset], pkt.data, kStreamDataLen);
  rl.streamOffset += kStreamDataLen;
  rl.streamLen = rl.streamOffset;
  rl.streamLastPacketsLeft = ctrl.packets_left;
  return StreamStatus::StreamContinue;
}

// -------------------- Radio initialization common code --------------------
#if defined(GATE_LORA_SX1262)
inline bool beginCommon(SX1262& radio, Core& rl, const PhyCfg& cfg) {
#elif defined(GATE_LORA_LLCC68)
inline bool beginCommon(rlCC68& radio, Core& rl, const PhyCfg& cfg) {
#else
  #error "No supported radio module defined"
#endif
//inline bool beginCommon(SX1262& radio, Core& rl, const PhyCfg& cfg) {
  const int8_t power = (cfg.txPowerDbm == INT8_MIN) ? 14 : cfg.txPowerDbm;

  int16_t st = radio.begin(cfg.freqMHz, cfg.bwKHz, cfg.sf, cfg.crDen,
                           cfg.syncWord, power, cfg.preamble);
  if (st != RADIOLIB_ERR_NONE) return false;

  if (cfg.crcOn) radio.setCRC(true); else radio.setCRC(false);
  if (cfg.dio2RfSwitch != -1) radio.setDio2AsRfSwitch(cfg.dio2RfSwitch == 1);
  if (cfg.rxBoost != -1)      radio.setRxBoostedGainMode(cfg.rxBoost == 1);

  radio.standby();        // ensure standby after init

  if(readEfuseMac6(rl.myMac6)) {
    rl.macReadOK = true;
    last3FromMac6(rl.myLast3, rl.myMac6);
  }
  
  rl.radio = &radio;
  rl.toaUsMax17 = radio.getTimeOnAir(16);   // µs // 51ms for 16B @ SF7BW125CR45
  g_rl = &rl;

  return true;
}

#if defined(GATE_LORA_SX1262)
inline void attachDio1(SX1262& radio, Core& rl) {
  radio.setDio1Action(onDio1ISR_trampoline);
}
#elif defined(GATE_LORA_LLCC68)
inline void attachDio1(rlCC68& radio, Core& rl) {
  radio.setDio1Action(onDio1ISR_trampoline);
}
#else
#error "No LoRa radio module defined"
#endif

// -------------------- The service pump (call in loop()) --------------------
inline void service(Core& rl, const Callbacks& cb) {
  const uint32_t now = millis();

  // (A) IRQ: TX-Done oder RX-Paket
  if (rl.dio1Flag) {
    rl.dio1Flag = false;

    if (rl.rfMode == Mode::Rx) {
      size_t len = rl.radio->getPacketLength();
      if (len >= sizeof(RaceLinkProto::Header7)) {
        // TODO: ab hier nochmal checken! txBuf als maximale länge? -> besser hardcoden
        // können mehrere pakete im readData buffer enthalten sein?

        if (len > sizeof(rl.txBuf)) len = sizeof(rl.txBuf);
        uint8_t pkt[64]; if (len > sizeof(pkt)) len = sizeof(pkt);
        
        if (rl.radio->readData(pkt, len) == RADIOLIB_ERR_NONE) {
          ++rl.rxCountTotal;

          // Try filtering out unwanted packets early disabled for debugging
          RaceLinkProto::Header7 h{};
          if (!RaceLinkProto::parseHeader(pkt, (uint8_t)len, h)) return;
          if (!receiverMatches(h.receiver, rl.myLast3)) return;  // broadcast ODER exakt meine 3B
          //if (RaceLinkProto::type_dir(h.type) != RaceLinkProto::DIR_M2N) return; // muss je nach rolle im hauptcode geprüft werden

          rl.lastRssi = (int16_t)rl.radio->getRSSI(true);
          rl.lastSnr  = (int8_t) rl.radio->getSNR();

          if(rl.rxNumWanted > 0) --rl.rxNumWanted; // nur wenn begrenzte Anzahl erwartet
         
          ++rl.rxCountFiltered;
          rl.lastRxAtMs = now;
          if (cb.onRxPacket) cb.onRxPacket(pkt, (uint8_t)len, rl.lastRssi, rl.lastSnr, cb.ctx);
        }
      }
      // Rx fortsetzen nicht nötig (nutze immer continuous RX)
      /* if (!rl.txPending && rl.rxKind != RxKind::None) {
        rl.radio->startReceive(); // nicht nötig, startReceive löst continuous RX aus
      } */
    }
  }

  if (!rl.txPending && rl.streamMode == Core::StreamMode::Tx && rl.streamActive) {
    queueNextStreamPacket(rl);
  }

  // (B) Wenn im Idle, dann gewünschten Modus prüfen und wechseln
  if(rl.rfMode == Mode::Idle) {
    // Idle → gewünschten Modus prüfen

    if (rl.txPending) {
      // TX steht an
      rl.rfMode = Mode::Tx;
      rl.changeMode = true;
      //rl.reqRxKind = RxKind::None;
      //rl.reqRxMs = 0;
      return;
    }
    else if (rl.reqRxKind == RxKind::None && 
      (rl.reqRxKind != rl.rxKind || rl.changeMode)) {

      // kein RX gewünscht, Idle festigen
      rl.radio->standby();
      rl.rxKind = RxKind::None;
      rl.changeMode = false;
      //rl.reqRxMs = 0; //unnötig
      if (cb.onIdle) cb.onIdle(cb.ctx);
      return; // fertig
    }
    else if (rl.reqRxKind != RxKind::None) {

      // RX gewünscht
      rl.rfMode = Mode::Rx;
      rl.changeMode = true;
      return; // fertig
    }
  }

  // (B) TX pending starten, wenn sendezeit erreicht, aber kein timed RX läuft
  //if (rl.txPending && rl.rxKind != RxKind::Timed && (int32_t)(now - rl.earliestTxAtMs) >= 0) {
  if (rl.rfMode == Mode::Tx) {
    
    if (rl.changeMode) {
      rl.radio->standby(); // sicherstellen, dass nichts mehr empfangen wird bis earliestTxAtMs
      rl.changeMode = false; // TX jetzt durchziehen
      if (cb.onTxStart) cb.onTxStart(cb.ctx);
    }

    if((int32_t)(now - rl.earliestTxAtMs) < 0) {
      return; // noch nicht Zeit zum Senden
    }

    // LBT / CAD wenn nötig
    if (rl.txArb == TxArbiter::CadNeeded) {
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

      //int16_t state = rl.radio->startChannelScan(cfg);
      int16_t state = rl.radio->scanChannel(cfg);
      
      if (state != RADIOLIB_CHANNEL_FREE) {
        // busy → kurzen Backoff und erneut CAD wenn Zeit erreicht
        rl.earliestTxAtMs = now + randMs(rl, 100, 200); // fixed backoff for LBT
        rl.txArb = TxArbiter::CadNeeded;
        rl.debug += 1; // debug counter für busy CAD
        //if(rl.debug <= 2) rl.debug = 2;
        return; // kein weiterer Service jetzt
      }
      else {
        rl.txArb = TxArbiter::None;
        // weiter zum senden
      }
    }
    if(rl.txArb == TxArbiter::None) {
      //if (cb.onTxStart) cb.onTxStart(cb.ctx); // schon bei CAD aufrufen?
      if (rl.radio->transmit(rl.txBuf, rl.txLen) == RADIOLIB_ERR_NONE) {
        rl.txPending = false;
        //rl.reqRxKind = rl.defaultRxKind; // nach TX wieder in default RX modus wechseln
        //rl.reqRxMs   = rl.defaultRxMs;
        
        ++rl.txCount;
        rl.lastTxAtMs = now;
        
        //rl.debug = 100;
        if (cb.onTxDone) cb.onTxDone(cb.ctx);        
        
        if (rl.streamMode == Core::StreamMode::Tx && rl.streamLastScheduled) {
          rl.streamMode = Core::StreamMode::None;
          rl.streamActive = false;
          rl.streamLastScheduled = false;
          rl.streamLen = 0;
          rl.streamOffset = 0;
          rl.streamTotalPackets = 0;
          rl.streamIndex = 0;
          rl.streamType = 0;
          if (rl.streamPostRxMs > 0) {
            requestRxTimed(rl, rl.streamPostRxMs, rl.streamPostRxNumWanted);
          }
          rl.streamPostRxMs = 0;
          rl.streamPostRxNumWanted = -1;
        }

        rl.rfMode = Mode::Idle;
        rl.changeMode = true;

        return; // gesendet, nun platz machen für restlichen code
      }
      else {
        rl.txArb = TxArbiter::CadNeeded;
        return;
      }
    }
  }
  // (D) RX-Requests bedienen
  if (rl.rfMode == Mode::Rx) {
    
    // check TX pending (especially for continuous RX)
    if (rl.txPending) {
      if (rl.rxKind == RxKind::Timed) {
        const uint16_t delta = rl.rxCountFiltered - rl.rxCountWinStart;
        if (cb.onRxWindowClosed) cb.onRxWindowClosed(delta, cb.ctx);
      }
      rl.rxKind = RxKind::None;
      rl.rxWindowEndMs = 0;
      rl.rxNumWanted = -1;
      rl.rfMode = Mode::Idle; // Wechsel zu Tx ermöglichen
      rl.radio->standby(); // Empfang sofort beenden, nicht erst wenn earliestTxAtMs erreicht ist
      rl.changeMode = true;
      return;
    }

    // RX-Window Timeout prüfen und ggf. beenden
    if (rl.rxKind == RxKind::Timed && rl.rxWindowEndMs > 0) {
      // RX-Timed: laufendes Fenster -> Fensterende prüfen
      
      if ((int32_t)(now - rl.rxWindowEndMs) >= 0 || rl.rxNumWanted == 0) {
        // Fensterende erreicht oder alle Antworten empfangen

        if(rl.lbtRxRelax && rl.rxNumWanted != 0) {
          // LBT Rx-Relax: RX fortsetzen bis seit dem letzten Paket länger als rxLbtTimeout ms vergangen sind
          // Ausnahme: wenn rxNumWanted==0 (alle Antworten empfangen)
          const uint32_t deltaSinceLastRx = now - rl.lastRxAtMs;
          if (deltaSinceLastRx < rl.rxLbtTimeout) {
            // noch nicht timeout
            return; // nichts tun, RX läuft weiter
          }
        }

        rl.rfMode = Mode::Idle;
        rl.reqRxKind = rl.defaultRxKind; // nach RX wieder in default modus wechseln
        rl.reqRxMs   = rl.defaultRxMs;
        rl.changeMode = true;
        rl.rxWindowEndMs = 0;
        rl.rxNumWanted = -1;
        
        const uint16_t delta = rl.rxCountFiltered - rl.rxCountWinStart;
        if (cb.onRxWindowClosed) cb.onRxWindowClosed(delta, cb.ctx);

        return; // fertig
      }

      return; // laufendes Fenster noch nicht beendet
    }

    if (rl.changeMode) {
      // komme von Idle: Continous RX starten
      if(rl.radio->startReceive() != RADIOLIB_ERR_NONE) {
        // RX nicht gestartet
        return; // fehler, wird im nächsten durchgang erneut versucht
      }
    }

    if (rl.changeMode || rl.rxKind != rl.reqRxKind) {
      // von einem rx modus in einen anderen wechseln

      if(rl.reqRxKind==RxKind::Timed) {
        // setze rxWindowEndMs reqRxMs
        // bei LBT automatisch kürzeres rxWindow? -> bei scheduleSend entsprechend anpassen
        rl.rxCountWinStart = rl.rxCountFiltered;
        const uint16_t w = (rl.reqRxMs ? rl.reqRxMs : rl.defaultRxMs);
        rl.rxWindowEndMs = now + w;
        if (cb.onRxWindowOpen) cb.onRxWindowOpen(w, cb.ctx);
      }
      else if(rl.reqRxKind==RxKind::Continuous) {
        rl.rxWindowEndMs = 0;
        if (cb.onRxWindowOpen) cb.onRxWindowOpen(0, cb.ctx);
      }
      rl.rxKind = rl.reqRxKind; // nach übernahme des neuen modus setzen
      rl.changeMode = false; // mode change done
    }
  }
}

} // namespace RaceLinkTransport

#ifdef RL__RESTORE_RANDOM_MACRO_AFTER_RADIOLIB
  #define random hw_random // replace arduino random()
  #undef RL__RESTORE_RANDOM_MACRO_AFTER_RADIOLIB
#endif
