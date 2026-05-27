#pragma once
// RaceLink Indicators — central status-notification catalog.
//
// Wire-stable enum of short-lived visual status indicators that overlay the
// segment for a caller-specified duration and then restore the previous
// state. Wire format: OPC_INDICATE (racelink_proto.h, 0x0C) carrying
// P_Indicate { type: u8, durationSec: u8 }. The same catalog is consumed
// by both ends — receivers expand a numeric type via findIndicator() and
// senders (Gateway, Host, FPVGate, Headless master) can build the packet
// via buildIndicatePacket() without needing the WLED side present.
//
// Design rule from the project owner: indicators MUST be animated
// (BREATH / STROBE / similar), never STATIC — solid colors are too
// inconspicuous as a notification.
//
// Header is intentionally WLED-neutral; only depends on stdint and the
// shared wire-format header.

#include <stdint.h>

#include "racelink_proto.h"

namespace RaceLinkIndicators {

// Wire-stable indicator IDs. APPEND-ONLY — existing IDs must never be
// renumbered or repurposed; older firmware silently drops unknown types
// via findIndicator() returning nullptr.
enum IndicatorType : uint8_t {
  IND_PAIR_CONFIRMED = 0,   // white breathe — slave acknowledges an OPC_SET_GROUP
  IND_PROBE_REJECTED = 1,   // red strobe — headless probe answered by an existing master
  IND_HEADLESS_ENTER = 2,   // blue breathe — promotion to headless master succeeded
  IND_HEADLESS_EXIT  = 3,   // magenta breathe — step-down from headless (5-click or gateway takeover)
  IND_IDENTIFY       = 4,   // magenta strobe — operator-triggered "where is this device?" from the host UI
  IND_PAIRING_TX     = 5,   // green-cyan strobe — Headless master sent a SET_GROUP packet (pairing TX)
  // Add new rows below in append-only fashion. Always animated.
};

// One row drives ALL visual aspects of the indicator. fxMode is the WLED
// effect index — should be FX_MODE_BREATH (3), FX_MODE_STROBE (23), or
// similar animated mode; STATIC (0) violates the project's animation rule.
struct IndicatorDef {
  uint8_t  type;
  const char* label;
  uint8_t  fxMode;      // WLED effect mode index (animated)
  uint8_t  speed;       // 0..255
  uint8_t  intensity;   // 0..255
  uint32_t color1;      // 0xRRGGBB foreground (color2 stays untouched by the overlay)
  uint8_t  brightness;  // 0..255 — overrides bri for the indicator's duration
};

// Default catalog. Order does NOT need to match enum order — findIndicator
// is a linear scan by ``type`` field, not array index.
//
// Design spec (2026-05-17 standardisation):
//   * fxMode    = 23 STROBE for every entry. BREATH was used historically
//                 but read as too subtle in a race environment; the
//                 operator missed positive events.
//   * speed     = 3-tier urgency code within the WLED-effective STROBE
//                 range 235..252 (outside that range the effect feels
//                 either too slow or too fast):
//                   - 235  slow   = positive event (calm)
//                   - 245  medium = informational / state-change /
//                                   operator-initiated action
//                   - 250  fast   = error / blocking state (alert)
//   * intensity = 128 uniformly (balanced strobe duty cycle)
//   * brightness= 230 uniformly (full visibility with limiter headroom)
//   * color1    = encodes the EVENT CATEGORY. Operator decodes
//                 "what happened" from the dominant RGB channel:
//                   - green-dominant  = success
//                   - blue-dominant   = promotion (role-up)
//                   - red-dominant    = error / reject
//                   - red+blue        = operator-locate
//                   - mixed warm      = demotion (role-down)
//                 Project rule "kein pures rot, grün, blau oder weiß"
//                 applies — every color must mix ≥ 2 RGB channels at
//                 value ≥ 0x33.
//
// When adding a new row, pick the speed tier by urgency and a color
// that fits one of the category families above (or reserves a new
// family with a distinct dominance pattern). Speeds 235 and 252 are
// kept free as headroom for "very calm" and "panic" tiers respectively.
static const IndicatorDef INDICATOR_CATALOG[] = {
  // type,                label,             fxMode,         spd, int, color1,    bri
  { IND_PAIR_CONFIRMED,   "Pair Confirmed",  23 /*STROBE*/,  235, 128, 0x00FFAAu, 230 },  // bright teal — success
  { IND_PROBE_REJECTED,   "Probe Rejected",  23 /*STROBE*/,  250, 128, 0xFF3300u, 230 },  // red-orange — error/reject
  { IND_HEADLESS_ENTER,   "Headless Enter",  23 /*STROBE*/,  245, 128, 0x00CCFFu, 230 },  // ice cyan   — promotion
  { IND_HEADLESS_EXIT,    "Headless Exit",   23 /*STROBE*/,  245, 128, 0xFFAA00u, 230 },  // amber      — demotion
  { IND_IDENTIFY,         "Identify",        23 /*STROBE*/,  245, 128, 0xFF00CCu, 230 },  // magenta    — operator locate
  // PAIRING_TX: throttled per-SET_GROUP flash for the Headless master. Fires
  // only when the master actually transmits a SET_GROUP packet (new device
  // pairing OR post-reboot re-bind sweep over the persistent slave registry),
  // never for routine scene/sync/brightness broadcasts. Lower intensity (96)
  // shortens the on-pulse so a quick blip reads as "I just paired a device"
  // rather than a sustained state overlay. Brightness 200 trades a small
  // amount of visibility for less eye fatigue during 40-slave re-bind bursts.
  { IND_PAIRING_TX,       "Pairing TX",      23 /*STROBE*/,  248,  96, 0x00FF40u, 200 },  // green-cyan — SET_GROUP send
};

static const uint8_t INDICATOR_CATALOG_SIZE =
    (uint8_t)(sizeof(INDICATOR_CATALOG) / sizeof(INDICATOR_CATALOG[0]));

// Look up a row by wire-stable type ID. Returns nullptr if the firmware
// does not know the requested indicator — forward compatible.
inline const IndicatorDef* findIndicator(uint8_t type) {
  for (uint8_t i = 0; i < INDICATOR_CATALOG_SIZE; ++i) {
    if (INDICATOR_CATALOG[i].type == type) return &INDICATOR_CATALOG[i];
  }
  return nullptr;
}

// Runtime state — caller-owned. The WLED usermod instantiates this once
// inside its class.
//
// Rendering model (2026-05-17 rewrite): indicators are rendered as a
// frame-buffer overlay via the usermod's handleOverlayDraw() hook
// instead of switching the segment's effect mode. The hook is invoked
// by setShowCallback() after every segment effect has been rendered
// and blended into the strip frame-buffer, immediately before
// strip.show() pushes the pixels to hardware. The underlying effect
// (Traffic Light, Palette, Fireworks, …) keeps running for the
// entire indicator duration — its SEGENV state, palette, colour
// slots, and any heap allocated via SEGENV.data are never touched.
// The strobe is purely a per-frame pixel-level overwrite that
// disappears the moment ``active`` flips back to false. Fleet phase
// sync is therefore preserved automatically, with zero catch-up
// cycling. No snapshot of pre-indicator segment state is required.
struct IndicatorState {
  bool     active           = false;
  uint32_t expiresAtMs      = 0;
  // Catalog values copied at applyLocalIndicator() time so the overlay
  // renderer doesn't need to re-resolve the catalog row every frame.
  uint32_t activeColor1     = 0;   // packed 0xRRGGBB
  uint8_t  activeSpeed      = 0;   // 0..255 — drives strobe cycle period (matches FX.cpp blink() semantics)
  uint8_t  activeBrightness = 0;   // 0..255 — pre-multiplied into the on-frame paint color
};

// Build an OPC_INDICATE wire packet. ``dstLast3`` may be the FF:FF:FF
// broadcast address (notify whole fleet) or a specific node's 3-byte MAC
// suffix (unicast notification, e.g. Host pinging a specific device with
// IDENTIFY). ``durationSec == 0`` is a cancel signal on the receiver
// (clears any active indicator without showing a new one).
inline uint8_t buildIndicatePacket(uint8_t* out,
                                   const uint8_t myLast3[3],
                                   const uint8_t dstLast3[3],
                                   uint8_t indicatorType,
                                   uint8_t durationSec) {
  using namespace RaceLinkProto;
  P_Indicate p{ indicatorType, durationSec };
  return build(out, myLast3, dstLast3, make_type(DIR_M2N, OPC_INDICATE), p);
}

} // namespace RaceLinkIndicators
