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

// P3: enables access to SX126x low-level commands (setBufferBaseAddress, writeBuffer,
//     setTx, setDioIrqParams, clearIrqStatus, setPacketParams). These are protected in
//     RadioLib by default; we define the flag BEFORE including RadioLib.h so the class
//     is compiled with them exposed as public. This is safe here because this header
//     is the only place RadioLib.h is included in the project.
#ifndef RADIOLIB_LOW_LEVEL
#define RADIOLIB_LOW_LEVEL 1
#endif

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
  uint8_t sf           = 7;     // RaceLink spreading factor
  uint8_t crDen        = 5;     // RaceLink coding rate denominator (5 => 4/5)
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
  
  #if defined(RACELINK_SX1262)
    SX1262*   radio             = nullptr;
  #elif defined(RACELINK_LLCC68)
    LLCC68*   radio             = nullptr;
  #else
    #error "No supported radio module defined"
  #endif
  
  volatile bool dio1Flag      = false;
  volatile uint32_t irqFlags  = 0;

  uint8_t   myMac6[6]     = {0};            // own MAC address (read from EFUSE)
  uint8_t   myLast3[3]    = {0};
  bool      macReadOK     = false;

  Mode      rfMode       = Mode::Idle;     // Idle, Tx, Rx
  bool      changeMode    = true;           // apply initial mode on first service() call
  bool      radioStandby  = true;           // P2: tracks that the chip is currently in Standby — used to skip redundant standby() calls before TX/CAD
  bool      txArmed       = false;          // P3: chip TX FIFO has been pre-loaded with txBuf and is ready for a setTx() command

  // --- RX state ---
  RxKind    rxKind            = RxKind::None;   // current RX type (only meaningful while rfMode==Rx)
  uint32_t  rxWindowEndMs     = 0;              // only used for RxTimed
  int8_t    rxNumWanted       = -1;             // expected reply count in Timed RX (-1 = unbounded)
  uint16_t  rxLbtTimeout      = 700;            // only for Timed RX with LBT (ms), max gap between packets before closing the window
  uint16_t  rxCountWinStart   = 0;

  // --- RX request (caller-requested mode) ---
  RxKind    reqRxKind     = RxKind::None;   // requested RX type
  uint16_t  reqRxMs       = 0;              // only used when reqRxKind==Timed

  // --- Default mode per device ---
  RxKind    defaultRxKind = RxKind::None;   // Master: None (Idle), Slave: Continuous
  uint16_t  defaultRxMs   = 500;            // for default Continuous=0 (truly continuous), for default Timed optional

  // --- TX-Queue ---
  bool      txPending       = false;
  uint8_t   txBuf[64];
  uint8_t   txLen           = 0;
  uint32_t  earliestTxAtMs  = 0;

  // --- TX packet parameters (P3: consumed by setPacketParams() in prepareTxBuffer) ---
  // Captured in beginCommon() from PhyCfg, then left unchanged for the lifetime of the radio.
  uint16_t  txPreamble      = 8;
  uint8_t   txCrcType       = 0x01; // RADIOLIB_SX126X_LORA_CRC_ON

  // --- IRQ-Robustness tracking (watchdog against deadlock from lost IRQs) ---
  // txSubmittedAtMs: millis() snapshot from scheduleSend() — anchors the "stuck in Tx mode
  //   without ever firing setTx" watchdog (covers e.g. persistent prepareTxBuffer failures
  //   or pathological CAD-busy loops). 0 means no TX is pending.
  // txStartedAtMs: millis() snapshot from fireArmedTx() — anchors the "TX-Done IRQ never
  //   arrived" watchdog (covers lost DIO1 IRQ / stuck chip). 0 means no TX in flight.
  uint32_t  txSubmittedAtMs = 0;
  uint32_t  txStartedAtMs   = 0;
  uint16_t  txWatchdogCount = 0;  // count of forced recoveries due to either watchdog firing
  uint16_t  rxErrCount      = 0;  // HEADER_ERR + CRC_ERR counter (dropped RX packets)

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

  // --- LBT / arbiter / ToA ---
  bool       lbtEnable   = true;                // configured at setup
  bool       lbtRxRelax  = true;                // if true, keep RX-Timed window open past its deadline as long as packets keep arriving (LBT relax)
  TxArbiter  txArb       = TxArbiter::None;     // LBT arbitration state
  uint32_t   toaUsMaxPkt = 0;                   // Cached Time-on-Air (µs) for the longest possible packet (sizeof(txBuf)); set in beginCommon().

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

// Maximum LBT backoff in milliseconds, based on time-on-air for the longest possible packet.
// (e.g. a 64-byte packet at SF7/BW125/CR45 takes ~150 ms)
inline uint16_t lbtBackoffMaxMs(const Core& rl) {
  uint32_t ms = rl.toaUsMaxPkt / 1000U;  // floor(ToA / 1000)
  return (ms < 5U) ? 5U : (uint16_t)ms;  // never below 5 ms
}

// RadioLib RNG: PhysicalLayer::random(lo, hiExclusive). Upper bound is exclusive, so
// we add 1 to maxMs to make this helper inclusive of maxMs.
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
  rl.defaultRxMs   = 0; // truly continuous (no window timeout)
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

// One-slot TX scheduling (returns false if the slot is busy or the payload is oversize).
// With LBT enabled: jitterMaxMs is overridden to a fixed 300 ms (TODO: derive from ToA?).
// Without LBT and jitterMaxMs > 50: the given jitterMaxMs is used to randomly delay the TX.
// Without LBT and jitterMaxMs == 0: TX is scheduled immediately.
inline bool scheduleSend(Core& rl, const uint8_t* buf, uint8_t len, uint16_t jitterMaxMs = 2500) {

  if (rl.txPending || len == 0 || len > sizeof(rl.txBuf)) return false; // reject if slot busy or oversize
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
  rl.txArmed = false;          // I6: defensive — txBuf content is new, any prior FIFO arming is now stale
  rl.txSubmittedAtMs = millis(); // I2: anchor for the "stuck in Tx mode without firing setTx" watchdog
  rl.txPending = true;          // mark TX as pending
  return true;
}

inline bool scheduleSendThenRxWindow(Core& rl, const uint8_t* buf, uint8_t len, uint16_t rxMs) {
  // TODO: revisit interaction with LBT (currently unused).
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
  // Single-packet streams (totalPackets == 1) are valid: the sole frame carries
  // start && stop with packets_left == 0. The len == 0 guard above already
  // excludes the totalPackets == 0 case; kept as defensive belt-and-braces.
  if (totalPackets == 0 || totalPackets > 16) return false;

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
    // Single-packet stream: start && stop with packets_left == 0 is valid.
    // Multi-packet start: stop must be unset and packets_left > 0.
    if (ctrl.stop && ctrl.packets_left != 0) return StreamStatus::Error;
    if (!ctrl.stop && ctrl.packets_left == 0) return StreamStatus::Error;
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
    if (ctrl.stop) {
      // Single-packet stream is already complete in this one frame.
      rl.streamActive = false;
      rl.streamReady = true;
      rl.streamLastPacketsLeft = 0;
      rl.streamMode = Core::StreamMode::None;
      return StreamStatus::StreamEnd;
    }
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
#if defined(RACELINK_SX1262)
inline bool beginCommon(SX1262& radio, Core& rl, const PhyCfg& cfg) {
#elif defined(RACELINK_LLCC68)
inline bool beginCommon(LLCC68& radio, Core& rl, const PhyCfg& cfg) {
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
  rl.radioStandby = true;

  // P3: cache TX packet parameters for use by prepareTxBuffer()
  rl.txPreamble = cfg.preamble;
  rl.txCrcType  = cfg.crcOn ? RADIOLIB_SX126X_LORA_CRC_ON : RADIOLIB_SX126X_LORA_CRC_OFF;

  if(readEfuseMac6(rl.myMac6)) {
    rl.macReadOK = true;
    last3FromMac6(rl.myLast3, rl.myMac6);
  }
  
  rl.radio = &radio;
  // ToA cache for the longest possible packet (sizeof(rl.txBuf), currently 64 bytes).
  // Used by lbtBackoffMaxMs() and as the basis of the TX-Done watchdog timeout.
  // Computing on max size ensures the watchdog never fires during a legitimate max-length TX.
  rl.toaUsMaxPkt = radio.getTimeOnAir(sizeof(rl.txBuf));
  g_rl = &rl;

  return true;
}

#if defined(RACELINK_SX1262)
inline void attachDio1(SX1262& radio, Core& rl) {
  radio.setDio1Action(onDio1ISR_trampoline);
}
#elif defined(RACELINK_LLCC68)
inline void attachDio1(LLCC68& radio, Core& rl) {
  radio.setDio1Action(onDio1ISR_trampoline);
}
#else
#error "No RaceLink radio module defined"
#endif

// -------------------- IRQ robustness: recovery --------------------
// recoverToIdle(): force the radio back to Standby, clear all chip IRQ flags, and reset
// the state machine to Idle. Prevents a permanent RF deadlock when an expected IRQ
// (especially TX-Done) is lost or when an unexpected IRQ/state combination is detected.
//
// Any TX currently in flight is dropped (txPending=false). The caller can resubmit via
// scheduleSend() if needed. An in-progress multi-packet stream is aborted entirely
// (streamMode/streamActive cleared) — otherwise the receiver would see a gap mid-stream
// and reject the rest, while the sender would think it had sent successfully.
//
// Important: reqRxKind is intentionally left untouched, so the next service() iteration
// will detect any standing RX request (e.g. Continuous on a Slave) in the Idle block and
// re-enter Rx automatically. The device is therefore guaranteed to be receive-capable
// again immediately after recovery.
inline void recoverToIdle(Core& rl, uint16_t debugCode = 0) {
  if (rl.radio) {
    rl.radio->standby();
    rl.radio->clearIrqStatus(RADIOLIB_SX126X_IRQ_ALL);
  }
  rl.dio1Flag        = false;
  rl.radioStandby    = true;
  rl.txArmed         = false;
  rl.txPending       = false;
  rl.txSubmittedAtMs = 0;
  rl.txStartedAtMs   = 0;
  rl.txArb           = TxArbiter::None;
  rl.rxKind          = RxKind::None;
  rl.rxWindowEndMs   = 0;
  rl.rxNumWanted     = -1;
  rl.rfMode          = Mode::Idle;
  rl.changeMode      = true;

  // I5: abort any in-progress TX stream — keeping streamMode==Tx with cleared txPending
  // would otherwise cause queueNextStreamPacket() to schedule the *next* fragment,
  // producing a packets_left gap that the receiver would reject.
  if (rl.streamMode == Core::StreamMode::Tx) {
    rl.streamMode             = Core::StreamMode::None;
    rl.streamActive           = false;
    rl.streamReady            = false;
    rl.streamLastScheduled    = false;
    rl.streamLen              = 0;
    rl.streamOffset           = 0;
    rl.streamTotalPackets     = 0;
    rl.streamIndex            = 0;
    rl.streamPostRxMs         = 0;
    rl.streamPostRxNumWanted  = -1;
  }

  if (debugCode) rl.debug = debugCode; // surface the trigger via WebUI / debug output
}

// -------------------- P3: TX pre-load helpers --------------------
// prepareTxBuffer(): pre-loads rl.txBuf into the chip's packet RAM and configures
// per-packet parameters WHILE the radio is still in Standby. Once this has succeeded,
// fireArmedTx() can start the TX with a single setTx() command (~5–15 µs of SPI) instead
// of the full transmit()/startTransmit() sequence (~200–500 µs).
// Preconditions: rl.radioStandby == true, rl.txLen > 0.
// CAD operations do NOT touch packet RAM (SX1262 DS §8.2), so the pre-loaded content
// survives CAD-busy backoff retries.
inline bool prepareTxBuffer(Core& rl) {
  if (!rl.radio) return false;
  if (rl.txLen == 0 || rl.txLen > sizeof(rl.txBuf)) return false;
  if (!rl.radioStandby) return false;

  // setPacketParams: idempotent for the values that come from begin(); only payloadLen varies per packet.
  int16_t st = rl.radio->setPacketParams(
    rl.txPreamble,
    rl.txCrcType,
    rl.txLen,
    RADIOLIB_SX126X_LORA_HEADER_EXPLICIT,
    RADIOLIB_SX126X_LORA_IQ_STANDARD
  );
  if (st != RADIOLIB_ERR_NONE) return false;

  // Write the payload at offset 0. We deliberately do NOT call setBufferBaseAddress
  // here: RadioLib's begin() and startReceive() both reset the TX base to 0, and
  // nothing else in our state machine moves it. Between prepareTxBuffer and
  // fireArmedTx the radio is always in Standby (CAD does not touch packet RAM and
  // never enters RX), so a "stray RX overwriting the FIFO" cannot happen. Saving
  // the SPI round-trip shaves ~10–20 µs from the pre-load.
  st = rl.radio->writeBuffer(rl.txBuf, rl.txLen, 0x00);
  if (st != RADIOLIB_ERR_NONE) return false;

  rl.txArmed = true;
  return true;
}

// fireArmedTx(): starts the TX using the FIFO content previously loaded by prepareTxBuffer().
// Non-blocking; TX-Done is signalled via DIO1 and handled in service()'s IRQ block.
// Post-CAD-free sequence is: clearIrqStatus + setDioIrqParams(TX_DONE|TIMEOUT) + setTx.
// This adds ~30–60 µs of SPI overhead — the bulk of the historical transmit() latency
// (writeBuffer, setPacketParams) was already paid in prepareTxBuffer() before CAD.
inline bool fireArmedTx(Core& rl) {
  if (!rl.radio || !rl.txArmed) return false;

  // Clear any residual dio1Flag set by the CAD-Done ISR, so the next service() entry
  // does not misinterpret it as a TX-Done event.
  rl.dio1Flag = false;

  // Fully clear the chip IRQ status register (CAD flags etc.).
  int16_t st = rl.radio->clearIrqStatus(RADIOLIB_SX126X_IRQ_ALL);
  if (st != RADIOLIB_ERR_NONE) return false;

  // Remap DIO1 to TX-Done and TIMEOUT (scanChannel had it pointing at CAD events).
  st = rl.radio->setDioIrqParams(
    RADIOLIB_SX126X_IRQ_TX_DONE | RADIOLIB_SX126X_IRQ_TIMEOUT, // global IRQ enable
    RADIOLIB_SX126X_IRQ_TX_DONE | RADIOLIB_SX126X_IRQ_TIMEOUT  // DIO1 mapping
  );
  if (st != RADIOLIB_ERR_NONE) return false;

  // Actual TX start: non-blocking, returns immediately.
  st = rl.radio->setTx(0); // 0 = no internal SX1262 timeout
  rl.txArmed = false;
  if (st == RADIOLIB_ERR_NONE) {
    rl.radioStandby  = false;
    rl.txStartedAtMs = millis(); // arm the in-flight watchdog only on confirmed start
    return true;
  }
  // setTx failed — radio is most likely still in standby; do not arm the in-flight
  // watchdog. The caller's failure path will reset txArb to CadNeeded and the next
  // service() iteration re-runs prepareTxBuffer + CAD + fireArmedTx.
  return false;
}

// -------------------- The service pump (call in loop()) --------------------
inline void service(Core& rl, const Callbacks& cb) {
  const uint32_t now = millis();

  // (A) IRQ handling: dispatch based on the actual IRQ flag bits read from the chip.
  // Older code dispatched based on rfMode (assumption-based), which works fine for
  // blocking transmit() but can deadlock with non-blocking setTx() if an expected IRQ
  // is lost, or misinterpret a stale CAD/TIMEOUT IRQ as TX-Done. Now: flag-based with
  // sanity checks against rfMode, plus an explicit recovery path via recoverToIdle().
  if (rl.dio1Flag) {
    rl.dio1Flag = false;

    // getIrqFlags() is public (no RADIOLIB_LOW_LEVEL flag needed); returns a uint32_t
    // mirror of the SX126x IRQ status register. Clear immediately to avoid re-entry.
    uint32_t flags = rl.radio->getIrqFlags();
    rl.radio->clearIrqStatus(RADIOLIB_SX126X_IRQ_ALL);

    if (flags & RADIOLIB_SX126X_IRQ_TX_DONE) {
      // Plausibility check: TX_DONE should only arrive while we believe we are in Tx mode.
      // A common benign case is a late TX_DONE arriving AFTER a watchdog recovery already
      // moved us back to Idle (or even past it into a new operation). In that case the
      // radio finished a TX we already gave up on — neutralize the chip back to standby
      // and consume the IRQ without disturbing whatever the state machine is doing now.
      if (rl.rfMode != Mode::Tx) {
        if (rl.radio) rl.radio->standby();
        rl.radioStandby = true;
        rl.debug = 0xA1;
        return;
      }
      // The SX126x automatically returns to STDBY_RC after TX_DONE (datasheet §13.1.4).
      // We trust that contract and skip an explicit standby() call here — saves one
      // SPI round-trip in the post-TX cleanup path. If the assumption ever turned out
      // to be wrong, the next operation that depends on radioStandby would fail and
      // the watchdog in the Tx block would recover the device automatically.
      rl.radioStandby    = true;
      rl.txArmed         = false;
      rl.txPending       = false;
      rl.txStartedAtMs   = 0; // clear in-flight watchdog anchor
      rl.txSubmittedAtMs = 0; // clear submission watchdog anchor

      ++rl.txCount;
      rl.lastTxAtMs = now;
      if (cb.onTxDone) cb.onTxDone(cb.ctx);

      // Multi-packet stream completion: only fire when the LAST fragment was just sent.
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
      return;
    }

    if (flags & RADIOLIB_SX126X_IRQ_RX_DONE) {
      // Same rationale as the TX_DONE mismatch above: a stale RX_DONE arriving after we
      // already transitioned away from Rx (e.g. into Tx for a reply) should not disturb
      // the new operation. Neutralize the chip and consume the IRQ.
      if (rl.rfMode != Mode::Rx) {
        if (rl.radio) rl.radio->standby();
        rl.radioStandby = true;
        rl.debug = 0xA2;
        return;
      }
      size_t len = rl.radio->getPacketLength();
      if (len >= sizeof(RaceLinkProto::Header7)) {
        // TODO: revisit max-length handling — currently bounded by sizeof(txBuf) and a
        // local 64-byte buffer. Investigate whether multiple packets can be queued in
        // the radio buffer between IRQs (unlikely with single-packet RX).
        if (len > sizeof(rl.txBuf)) len = sizeof(rl.txBuf);
        uint8_t pkt[64]; if (len > sizeof(pkt)) len = sizeof(pkt);

        if (rl.radio->readData(pkt, len) == RADIOLIB_ERR_NONE) {
          ++rl.rxCountTotal;

          // Early-filter unwanted packets — header parse + receiver-match
          RaceLinkProto::Header7 h{};
          if (!RaceLinkProto::parseHeader(pkt, (uint8_t)len, h)) return;
          if (!receiverMatches(h.receiver, rl.myLast3)) return;  // broadcast OR exact match on my 3-byte address
          //if (RaceLinkProto::type_dir(h.type) != RaceLinkProto::DIR_M2N) return; // role-specific direction check (left to the main code)

          rl.lastRssi = (int16_t)rl.radio->getRSSI(true);
          rl.lastSnr  = (int8_t) rl.radio->getSNR();

          if(rl.rxNumWanted > 0) --rl.rxNumWanted; // only when a bounded reply count was requested

          ++rl.rxCountFiltered;
          rl.lastRxAtMs = now;
          if (cb.onRxPacket) cb.onRxPacket(pkt, (uint8_t)len, rl.lastRssi, rl.lastSnr, cb.ctx);
        }
      }
      // No need to restart RX — we always use continuous-RX mode.
      return;
    }

    if (flags & RADIOLIB_SX126X_IRQ_TIMEOUT) {
      // RX-Timeout or TX-Timeout — either way, drop back to a safe Idle state.
      recoverToIdle(rl, /*debugCode*/ 0xA3);
      return;
    }

    if (flags & (RADIOLIB_SX126X_IRQ_HEADER_ERR | RADIOLIB_SX126X_IRQ_CRC_ERR)) {
      // RX error during continuous-RX: the chip auto-resumes RX; just bump telemetry.
      ++rl.rxErrCount;
      return;
    }

    // Spurious / unknown IRQ (e.g. a residual from scanChannel even though we clear
    // dio1Flag there): silently consume — no recovery action needed.
  }

  if (!rl.txPending && rl.streamMode == Core::StreamMode::Tx && rl.streamActive) {
    queueNextStreamPacket(rl);
  }

  // (B) While in Idle, evaluate the requested mode and transition if needed.
  if(rl.rfMode == Mode::Idle) {

    if (rl.txPending) {
      // TX pending: P1 — fall through to the Tx block within the SAME service() call,
      // instead of returning here and losing a service-tick before TX can start.
      rl.rfMode = Mode::Tx;
      rl.changeMode = true;
      // NO return here — execution continues into the `if (rfMode == Tx)` block below.
    }
    else if (rl.reqRxKind == RxKind::None &&
      (rl.reqRxKind != rl.rxKind || rl.changeMode)) {

      // No RX requested — confirm Idle. Same P2 pattern as in the Tx block: skip the
      // standby SPI command when our tracking already says the chip is in standby
      // (typical post-TX-Done path where the chip just returned to STDBY_RC).
      if (!rl.radioStandby) {
        rl.radio->standby();
        rl.radioStandby = true;
      }
      rl.rxKind = RxKind::None;
      rl.changeMode = false;
      if (cb.onIdle) cb.onIdle(cb.ctx);
      return;
    }
    else if (rl.reqRxKind != RxKind::None) {

      // RX requested — switch to Rx mode (actual startReceive happens in the Rx block).
      rl.rfMode = Mode::Rx;
      rl.changeMode = true;
      return;
    }
  }

  // (C) Tx block — drive the pending TX through standby + prepare + CAD + setTx.
  if (rl.rfMode == Mode::Tx) {

    if (rl.changeMode) {
      if (!rl.radioStandby) {
        // P2: only call standby() if the chip is not already there — saves ~100–500 µs of SPI per Tx entry.
        rl.radio->standby();
        rl.radioStandby = true;
      }
      rl.changeMode = false;
      if (cb.onTxStart) cb.onTxStart(cb.ctx);
    }

    // I2: submission watchdog — guards against being stuck in Tx mode without ever
    // firing setTx (e.g. persistent prepareTxBuffer SPI failures or pathological CAD-busy
    // loops). 5 s easily accommodates a heavily congested channel (CAD-busy backoffs of
    // 100–200 ms each give >25 retries inside the window) — anything beyond that is a
    // genuine stuck-state requiring recovery.
    constexpr uint32_t TX_SUBMIT_STUCK_MS = 5000;
    if (rl.txSubmittedAtMs != 0 && (int32_t)(now - rl.txSubmittedAtMs) > (int32_t)TX_SUBMIT_STUCK_MS) {
      ++rl.txWatchdogCount;
      recoverToIdle(rl, /*debugCode*/ 0xB1);
      return;
    }

    // P3: if TX is currently in flight (radio not in Standby), wait for the DIO1 TX-Done
    // IRQ — cleanup happens in the IRQ path. The in-flight watchdog catches the case
    // where the IRQ is lost or the chip is stuck: after ToA + 100 ms safety margin we
    // force-recover via recoverToIdle() so the device remains receive-capable.
    if (!rl.radioStandby) {
      const uint32_t txWatchdogMs = (rl.toaUsMaxPkt / 1000U) + 100U;
      if (rl.txStartedAtMs != 0 && (int32_t)(now - rl.txStartedAtMs) > (int32_t)txWatchdogMs) {
        ++rl.txWatchdogCount;
        recoverToIdle(rl, /*debugCode*/ 0xB0);
      }
      return;
    }

    // P3: pre-load the TX FIFO before CAD. This happens during the LBT jitter wait
    // (50–300 ms) which is otherwise idle time. After CAD-free, the actual TX start
    // is then just a setTx command (~30–60 µs SPI instead of ~200–500 µs).
    if (!rl.txArmed) {
      if (!prepareTxBuffer(rl)) {
        // setPacketParams / writeBuffer failed — very rare. Retry via the CAD path.
        rl.txArb = TxArbiter::CadNeeded;
        return;
      }
    }

    if((int32_t)(now - rl.earliestTxAtMs) < 0) {
      return; // not yet time to transmit (jitter / backoff still pending)
    }

    // LBT / CAD if required.
    if (rl.txArb == TxArbiter::CadNeeded) {
      ChannelScanConfig_t cfg = {
        .cad = {
          .symNum   = RADIOLIB_SX126X_CAD_ON_4_SYMB,    // robust default; 2-symb is faster but flakier
          .detPeak  = 18,                                // default rule of thumb is SF+13 = 20; 18 is slightly more permissive
          .detMin   = 10,                                // RadioLib default; 5 was found too low to be reliable
          .exitMode = RADIOLIB_SX126X_CAD_GOTO_STDBY,    // return to standby after CAD so we control the next mode
          .timeout  = 0,
          .irqFlags = RADIOLIB_IRQ_CAD_DEFAULT_FLAGS,
          .irqMask  = RADIOLIB_IRQ_CAD_DEFAULT_MASK,
        },
      };

      int16_t state = rl.radio->scanChannel(cfg);
      rl.dio1Flag    = false; // P3: CAD-Done ISR set dio1Flag — clear so it cannot be misread as TX-Done later
      rl.radioStandby = true; // P2: with CAD exitMode==CAD_GOTO_STDBY the chip is guaranteed to be in Standby now (whether FREE or BUSY)

      if (state != RADIOLIB_CHANNEL_FREE) {
        // Busy → short randomized backoff then re-attempt CAD when the new time hits.
        // The pre-loaded FIFO (txArmed==true) survives, because CAD does not touch packet RAM.
        rl.earliestTxAtMs = now + randMs(rl, 100, 200);
        rl.txArb = TxArbiter::CadNeeded;
        rl.debug += 1; // CAD-busy occurrence counter (visible via debug telemetry)
        return;
      }
      else {
        rl.txArb = TxArbiter::None;
        // fall through to fire setTx
      }
    }
    if(rl.txArb == TxArbiter::None) {
      // P3: start TX from the pre-loaded FIFO. fireArmedTx() is non-blocking; the
      // TX-Done cleanup runs in the IRQ handler above when DIO1 fires.
      if (fireArmedTx(rl)) {
        // TX in flight, main loop free. We remain in rfMode=Tx with txPending=true
        // until the DIO1 TX-Done IRQ is processed.
        return;
      }
      else {
        // setTx failed — retry via the CAD path on the next service() iteration.
        rl.txArmed = false;
        rl.txArb = TxArbiter::CadNeeded;
        return;
      }
    }
  }

  // (D) Service RX requests.
  if (rl.rfMode == Mode::Rx) {
    
    // Check for a pending TX (especially relevant while in continuous RX).
    if (rl.txPending) {
      if (rl.rxKind == RxKind::Timed) {
        const uint16_t delta = rl.rxCountFiltered - rl.rxCountWinStart;
        if (cb.onRxWindowClosed) cb.onRxWindowClosed(delta, cb.ctx);
      }
      rl.rxKind = RxKind::None;
      rl.rxWindowEndMs = 0;
      rl.rxNumWanted = -1;
      rl.rfMode = Mode::Idle;       // allow the Idle block to transition to Tx on next service()
      rl.radio->standby();          // stop RX immediately rather than waiting for earliestTxAtMs
      rl.radioStandby = true;       // P2: the chip is now guaranteed to be in Standby
      rl.changeMode = true;
      return;
    }

    // Check whether a Timed-RX window has expired and close it if so.
    if (rl.rxKind == RxKind::Timed && rl.rxWindowEndMs > 0) {

      if ((int32_t)(now - rl.rxWindowEndMs) >= 0 || rl.rxNumWanted == 0) {
        // Window deadline reached or all expected replies received.

        if(rl.lbtRxRelax && rl.rxNumWanted != 0) {
          // LBT RX-relax: extend the window as long as packets keep arriving — i.e. only
          // close it once `rxLbtTimeout` ms have passed since the last received packet.
          // Exception: if rxNumWanted==0 the bounded count is satisfied — close immediately.
          const uint32_t deltaSinceLastRx = now - rl.lastRxAtMs;
          if (deltaSinceLastRx < rl.rxLbtTimeout) {
            return; // not yet relax-timeout — let RX keep running
          }
        }

        rl.rfMode = Mode::Idle;
        rl.reqRxKind = rl.defaultRxKind; // return to the device's default mode after the window closes
        rl.reqRxMs   = rl.defaultRxMs;
        rl.changeMode = true;
        rl.rxWindowEndMs = 0;
        rl.rxNumWanted = -1;

        const uint16_t delta = rl.rxCountFiltered - rl.rxCountWinStart;
        if (cb.onRxWindowClosed) cb.onRxWindowClosed(delta, cb.ctx);

        return;
      }

      return; // window still running
    }

    if (rl.changeMode) {
      // Coming from Idle: start continuous RX.
      if(rl.radio->startReceive() != RADIOLIB_ERR_NONE) {
        return; // startReceive failed — will be retried next service() iteration
      }
      rl.radioStandby = false; // P2: chip is now in RX
    }

    if (rl.changeMode || rl.rxKind != rl.reqRxKind) {
      // Transitioning between RX modes (e.g. Continuous → Timed).

      if(rl.reqRxKind==RxKind::Timed) {
        // Open a new Timed window.
        // TODO: under LBT, consider shortening the window automatically — adjust at scheduleSend instead.
        rl.rxCountWinStart = rl.rxCountFiltered;
        const uint16_t w = (rl.reqRxMs ? rl.reqRxMs : rl.defaultRxMs);
        rl.rxWindowEndMs = now + w;
        if (cb.onRxWindowOpen) cb.onRxWindowOpen(w, cb.ctx);
      }
      else if(rl.reqRxKind==RxKind::Continuous) {
        rl.rxWindowEndMs = 0;
        if (cb.onRxWindowOpen) cb.onRxWindowOpen(0, cb.ctx);
      }
      rl.rxKind = rl.reqRxKind;
      rl.changeMode = false;
    }
  }
}

} // namespace RaceLinkTransport

#ifdef RL__RESTORE_RANDOM_MACRO_AFTER_RADIOLIB
  #define random hw_random // replace arduino random()
  #undef RL__RESTORE_RANDOM_MACRO_AFTER_RADIOLIB
#endif
