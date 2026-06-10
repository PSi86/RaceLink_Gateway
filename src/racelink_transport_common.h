// racelink_transport_common.h -- transport-agnostic RaceLink helpers
// Header-only, Arduino-friendly. No heap allocations.
//
// This header holds the parts of the RaceLink transport layer that have NO
// dependency on the physical medium (no RadioLib, no W5500/UDP). Both the LoRa
// backend (racelink_transport_core.h) and the Ethernet backend
// (racelink_transport_eth.h) include this file and reuse these byte-for-byte:
//   - MAC / 3-byte address helpers (readEfuseMac6, last3FromMac6, mac6ToStr,
//     same3, isBroadcast3, receiverMatches)
//   - the StreamStatus result type
//   - the RX-side multi-packet stream reassembly state machine
//     (handleStreamPacket / streamBuffer / clearStreamReady)
//
// The stream functions are templates on the transport Core type (CoreT). Every
// backend Core exposes the same stream-state field names plus a nested
// `StreamMode` enum, so a single definition serves both transports while the
// TX-side stream scheduling (which must call the backend-specific scheduleSend)
// stays in each backend header.
//
// License: MIT
#pragma once

#include <Arduino.h>
#include "racelink_proto.h"

extern "C" {
  #include <esp_mac.h>
}

namespace RaceLinkTransport {

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

// -------------------- Stream helpers (RX reassembly) --------------------
enum class StreamStatus : uint8_t { StreamStart, StreamContinue, StreamEnd, Error };

template<typename CoreT>
inline const uint8_t* streamBuffer(const CoreT& rl, uint8_t& len) {
  len = rl.streamLen;
  return rl.streamBuf;
}

template<typename CoreT>
inline void clearStreamReady(CoreT& rl) {
  rl.streamReady = false;
}

template<typename CoreT>
inline StreamStatus handleStreamPacket(CoreT& rl, const RaceLinkProto::P_Stream& pkt) {
  constexpr uint8_t kStreamDataLen = sizeof(pkt.data);
  constexpr uint8_t kMaxPackets = 16;
  const RaceLinkProto::StreamCtrl ctrl = RaceLinkProto::decode_stream_ctrl(pkt.ctrl);

  if (ctrl.packets_left >= kMaxPackets) return StreamStatus::Error;
  if (rl.streamMode == CoreT::StreamMode::Tx) return StreamStatus::Error;

  if (ctrl.start) {
    // Single-packet stream: start && stop with packets_left == 0 is valid.
    // Multi-packet start: stop must be unset and packets_left > 0.
    if (ctrl.stop && ctrl.packets_left != 0) return StreamStatus::Error;
    if (!ctrl.stop && ctrl.packets_left == 0) return StreamStatus::Error;
    rl.streamMode = CoreT::StreamMode::Rx;
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
      rl.streamMode = CoreT::StreamMode::None;
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
    rl.streamMode = CoreT::StreamMode::None;
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

} // namespace RaceLinkTransport
