// rf_config_nvs.h -- Persistent storage for the LoRa PHY config
// (P_RfConfig wire struct). Backs the runtime reconfiguration paths
// on both sides of the protocol:
//   * Gateway:   GW_CMD_SET_RF_CONFIG / GW_CMD_GET_RF_CONFIG (USB-CDC)
//   * WLED node: OPC_RF_CONFIG / OPC_GET_RF_CONFIG (LoRa wire)
// plus the compile-default fallback used by setup() / radioInit()
// before radio.begin().
//
// Design:
//   * Header-only / inline -- avoids a separate translation unit and
//     keeps the NVS surface localised to a single file. ESP32
//     Preferences (NVS) does its own CRC, but we add a schema CRC16
//     over the P_RfConfig bytes so a corrupted slot drops cleanly to
//     compile-time defaults instead of feeding random bytes into
//     radio.begin().
//   * Boot-loop recovery: bootCounterIncrement() ticks BEFORE
//     radio.begin(); bootCounterClear() runs AFTER a successful init.
//     If the counter ever reaches BOOT_COUNTER_MAX the persisted slot
//     is wiped on the next boot -- breaking a brick loop caused by a
//     valid-looking-but-unflashable config (e.g. SF too high for the
//     environment + radio.begin() hanging the firmware).
//   * Validation lives here so the GW_CMD handler, the LoRa-side
//     OPC_RF_CONFIG handler, and load() all share one source of truth
//     for range checks.
//   * The compile-default helper lives at the call site (Gateway:
//     main.cpp; WLED node: racelink_wled.cpp) to keep this header
//     side-agnostic. The only per-side knowledge that has to live
//     somewhere is the mapping of compile-default defines
//     (RACELINK_CR vs RACELINK_CR_DEN, RACELINK_SYNC_WORD vs
//     RACELINK_SYNCWORD) -- both call sites resolve those locally.
//
// This file is NOT one of the four shared protocol headers
// (racelink_proto.h, racelink_headless.h, racelink_indicators.h,
// racelink_transport_core.h). Drift-test coverage is not required;
// byte-identical sync across the WLED/Gateway repos is a code-hygiene
// convention, not a wire-protocol contract.
//
// Added 2026-05-20 for Stage 1 of the multi-gateway plan
// (Gateway PR-2 + WLED node PR-3); audience-neutralised on 2026-05-23
// so a single file ships to both sides byte-identically.

#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "racelink_proto.h"

namespace RfConfigNvs {

// Magic marker stored alongside the config blob. A bump here (e.g.
// after a wire-format change to P_RfConfig) invalidates every existing
// NVS slot -- load() then returns false and the caller falls back to
// compile-time defaults. Schema-version semantics, baked into NVS.
static const uint16_t NVS_MAGIC = 0xA5C3;

// Bootloop strike count before the slot is wiped on the next boot.
// 3 is a balance between "robust against transient init failures" and
// "operator doesn't have to power-cycle 8 times to recover".
static const uint8_t BOOT_COUNTER_MAX = 3;

static const char* NVS_NAMESPACE = "rl_rf";
static const char* KEY_MAGIC     = "magic";
static const char* KEY_CONFIG    = "cfg";
static const char* KEY_CRC       = "crc";
static const char* KEY_BOOTCNT   = "bootcnt";

// CRC16-CCITT over the P_RfConfig payload. NVS already CRCs each blob
// internally; this is the schema-level guard that catches "blob was
// stored under an older P_RfConfig layout" cases the NVS CRC cannot
// detect (sizeof match + field shuffle).
inline uint16_t crc16(const uint8_t* data, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; ++j) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// Range validator. Returns RF_CHANGE_OK on accept,
// RF_CHANGE_REJECTED_RANGE otherwise. BW set mirrors the SX1262 / LLCC68
// hardware-supported list (all values fit a uint16 x10 representation).
inline RaceLinkProto::RfChangeReason validate(const RaceLinkProto::P_RfConfig& c) {
  // ISM bands: EU868 (863..870) and US915 (902..928) -- both fit in a
  // single 863..928 MHz window. The FW does not enforce per-region
  // sub-band rules; that is a host responsibility.
  if (c.freq_hz < 863000000UL || c.freq_hz > 928000000UL) {
    return RaceLinkProto::RF_CHANGE_REJECTED_RANGE;
  }
  switch (c.bw_khz_x10) {
    case 78:   // 7.81 kHz
    case 104:  // 10.42
    case 156:  // 15.63
    case 208:  // 20.83
    case 313:  // 31.25
    case 417:  // 41.67
    case 625:  // 62.5
    case 1250: // 125
    case 2500: // 250
    case 5000: // 500
      break;
    default:
      return RaceLinkProto::RF_CHANGE_REJECTED_RANGE;
  }
  if (c.sf < 5 || c.sf > 12) return RaceLinkProto::RF_CHANGE_REJECTED_RANGE;
  if (c.cr_den < 5 || c.cr_den > 8) return RaceLinkProto::RF_CHANGE_REJECTED_RANGE;
  if (c.tx_power_dbm < -9 || c.tx_power_dbm > 22) return RaceLinkProto::RF_CHANGE_REJECTED_RANGE;
  if (c.preamble < 6) return RaceLinkProto::RF_CHANGE_REJECTED_RANGE;
  return RaceLinkProto::RF_CHANGE_OK;
}

// Persist a validated config to NVS. Re-runs validate() defensively;
// returns RF_CHANGE_REJECTED_RANGE if the caller skipped that step.
inline RaceLinkProto::RfChangeReason store(const RaceLinkProto::P_RfConfig& c) {
  auto r = validate(c);
  if (r != RaceLinkProto::RF_CHANGE_OK) return r;
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/false)) {
    return RaceLinkProto::RF_CHANGE_NVS_FAIL;
  }
  bool ok = true;
  ok &= (prefs.putUShort(KEY_MAGIC, NVS_MAGIC) == sizeof(uint16_t));
  ok &= (prefs.putBytes(KEY_CONFIG, &c, sizeof(c)) == sizeof(c));
  const uint16_t crc = crc16(reinterpret_cast<const uint8_t*>(&c), sizeof(c));
  ok &= (prefs.putUShort(KEY_CRC, crc) == sizeof(uint16_t));
  prefs.end();
  return ok ? RaceLinkProto::RF_CHANGE_OK : RaceLinkProto::RF_CHANGE_NVS_FAIL;
}

// Load a previously persisted config. Returns true iff the slot is
// present, magic matches, size matches, CRC verifies, and validate()
// passes. Caller falls back to compile defaults on false.
inline bool load(RaceLinkProto::P_RfConfig& out) {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/true)) return false;
  bool good = false;
  do {
    if (!prefs.isKey(KEY_MAGIC) || !prefs.isKey(KEY_CONFIG) || !prefs.isKey(KEY_CRC)) break;
    if (prefs.getUShort(KEY_MAGIC, 0) != NVS_MAGIC) break;
    if (prefs.getBytesLength(KEY_CONFIG) != sizeof(RaceLinkProto::P_RfConfig)) break;
    RaceLinkProto::P_RfConfig tmp{};
    if (prefs.getBytes(KEY_CONFIG, &tmp, sizeof(tmp)) != sizeof(tmp)) break;
    const uint16_t expected = prefs.getUShort(KEY_CRC, 0);
    const uint16_t actual   = crc16(reinterpret_cast<const uint8_t*>(&tmp), sizeof(tmp));
    if (expected != actual) break;
    if (validate(tmp) != RaceLinkProto::RF_CHANGE_OK) break;
    out = tmp;
    good = true;
  } while (false);
  prefs.end();
  return good;
}

// Increment-and-read the boot-loop counter. Call BEFORE radio.begin().
// Returns the new counter value; if it exceeds BOOT_COUNTER_MAX the
// caller should wipe() the persisted config so the next boot reaches
// compile defaults.
inline uint8_t bootCounterIncrement() {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/false)) return 0;
  uint8_t c = prefs.getUChar(KEY_BOOTCNT, 0);
  if (c < 255) ++c;  // saturate
  prefs.putUChar(KEY_BOOTCNT, c);
  prefs.end();
  return c;
}

// Clear the boot-loop counter. Call AFTER a successful radio.begin().
inline void bootCounterClear() {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/false)) return;
  prefs.putUChar(KEY_BOOTCNT, 0);
  prefs.end();
}

// Wipe the persisted RF config slot (used by boot-loop recovery and by
// the "factory reset" path the host can trigger). Also resets the boot
// counter so the next boot starts fresh. The next load() returns false.
inline void wipe() {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/false)) return;
  prefs.remove(KEY_MAGIC);
  prefs.remove(KEY_CONFIG);
  prefs.remove(KEY_CRC);
  prefs.putUChar(KEY_BOOTCNT, 0);
  prefs.end();
}

} // namespace RfConfigNvs
