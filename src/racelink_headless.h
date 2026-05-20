#pragma once
// RaceLink Headless Mode — shared catalog + packet builders.
//
// This header is intentionally WLED-neutral: it depends only on
// ``racelink_proto.h`` (which itself only needs <stdint.h>/<stddef.h>).
// External Gateway-side software (e.g. FPVGate, https://github.com/
// LouisHitchcock/FPVGate) can ``#include`` it unchanged and reuse the
// same scene catalog + packet builders that the WLED Headless master
// uses. Adding a scene = appending one row to ``SCENE_CATALOG[]``.
//
// All packet builders emit the wire bytes via ``RaceLinkProto::build`` /
// ``build_empty`` so the byte layout cannot drift from the rest of the
// protocol; the caller hands the resulting buffer + length to its own
// LoRa transport (no transport coupling here).

#include <stdint.h>
#include <string.h>

#include "racelink_proto.h"

namespace RaceLinkHeadless {

// -------------------- Probe / activation constants --------------------
// All times in milliseconds; tuned for SF7/125k EU868 latency budget.

// Window in which a Gateway / other Headless Master may answer our
// probe-IDENTIFY_REPLY with OPC_SET_GROUP. Measured from the moment the
// first probe is actually sent.
static const uint32_t HEADLESS_PROBE_TIMEOUT_MS    = 1500;

// Random pre-probe delay applied to both the manual 5-click activation
// and the boot-time auto-resume. Decorrelates two simultaneously powered
// devices so one finishes its probe first and claims master, then
// answers the other's probe with OPC_SET_GROUP. Values [500..2000].
static const uint32_t HEADLESS_PROBE_JITTER_MIN_MS = 500;
static const uint32_t HEADLESS_PROBE_JITTER_MAX_MS = 2000;

// Second probe broadcast offset (relative to the first) for single-loss
// coverage at SF7 over EU868. Two probes inside HEADLESS_PROBE_TIMEOUT_MS
// give >99.9% catch probability against a single dropped frame.
static const uint32_t HEADLESS_PROBE_RETRY_OFFSET_MS = 600;

// Keepalive cadence — every HEADLESS_SYNC_KEEPALIVE_MS the headless master
// broadcasts an OPC_SYNC (4B autosync form) carrying its local millis()
// timestamp. This (a) keeps slaves' masterContactedRecently() true so the
// local color-cycle button stays gated, and (b) anchors slaves'
// strip.timebase against the master clock so cyclic effects (Breathe etc.)
// stay phase-stable across the fleet. Mirrors Gateway autosync semantics.
static const uint32_t HEADLESS_SYNC_KEEPALIVE_MS   = 30000;

// Group ID assignment range. Group 0 means "unpaired", 255 is the broadcast
// pseudo-group used by OPC_CONTROL / OPC_OFFSET. Group 1 is reserved for the
// Headless master itself, so slave assignment starts at 2 and runs through 254.
static const uint8_t HEADLESS_MASTER_GROUP_ID = 1;
static const uint8_t HEADLESS_FIRST_GROUP_ID  = 2;
static const uint8_t HEADLESS_MAX_GROUP_ID    = 254;

// Persistent slave registry capacity. Each record costs 4 bytes RAM; the
// cfg.json representation runs ~340 bytes when full.
static const uint8_t HEADLESS_MAX_SLAVES = 40;

// Spacing between proactive SET_GROUP sends during the post-reboot re-bind
// burst. 250 ms leaves enough channel-free time between consecutive master
// TXs for the addressed slave to run CAD + send its OPC_ACK back without
// colliding with the next master SET_GROUP. The earlier value (50 ms) caused
// CAD-busy backoffs on every slave ACK attempt — visible as ``rl.debug``
// counting up on the slaves. Slave-side processing of SET_GROUP itself was
// unaffected (apply runs before the ACK queue), but the master had no way to
// observe ACKs and the channel filled with retried ACK attempts.
static const uint32_t HEADLESS_REASSIGN_INTERVAL_MS = 500;

// Debounce window for the slave-table persistence pump. A 40-slave pairing
// burst (~30 s) collapses to a single cfg.json write 5 s after the last
// table mutation, instead of 40 writes back-to-back (flash wear mitigation).
static const uint32_t HEADLESS_PERSIST_DEBOUNCE_MS = 5000;

// Pairing-TX indicator throttling and duration. The indicator fires only on
// SET_GROUP sends (new-device pairing AND post-reboot re-bind sweep). The
// throttle prevents successive SET_GROUPs in a re-bind sweep from re-
// extending the deadline into a continuous overlay; the 1500 ms duration is
// long enough to bridge the 50 ms re-assign spacing into one visible sweep
// but short enough that an isolated single pairing reads as a single blip.
static const uint32_t HEADLESS_PAIRING_INDICATOR_THROTTLE_MS = 200;
static const uint32_t HEADLESS_PAIRING_INDICATOR_DURATION_MS = 1500;

// Debounce window for the post-pairing scene rebroadcast. After a successful
// SET_GROUP send (proactive boot-burst OR individual reactive pairing) the
// master schedules a single OPC_HEADLESS broadcast so the freshly-bound
// slaves snap to the current visual state. Successive pairings within the
// window collapse to one rebroadcast — a 10-slave boot burst produces one
// scene packet at the end, not ten. 1000 ms gives the last slave's ACK time
// to clear the channel before the scene broadcast contends for it.
static const uint32_t HEADLESS_SCENE_REBROADCAST_DEBOUNCE_MS = 1000;

// Persistent per-master record of "this slave belongs to me at this group".
// Stored as a flat dense array, ``addr3 == {0,0,0}`` flags an empty slot.
struct HeadlessSlaveRec {
  uint8_t addr3[3];
  uint8_t groupId;
};

// -------------------- Slave-registry helpers (WLED-neutral) --------------------
// Pure data operations on a caller-owned ``HeadlessSlaveRec[]`` array. No
// dependency on Arduino, WLED, or the radio transport — the WLED Headless
// master and an external Gateway-side driver use identical semantics so the
// registry round-trip (in-RAM <-> cfg.json) stays interchangeable.

// Linear scan; returns -1 when ``a3`` is not present.
inline int8_t findSlaveIdx(const HeadlessSlaveRec* table, uint8_t count,
                           const uint8_t a3[3]) {
  for (uint8_t i = 0; i < count; ++i) {
    const HeadlessSlaveRec& r = table[i];
    if (r.addr3[0] == a3[0] && r.addr3[1] == a3[1] && r.addr3[2] == a3[2]) {
      return (int8_t)i;
    }
  }
  return -1;
}

// Insert-or-update. ``count`` is updated in-place when an append happens.
// Returns false only when the table is full AND ``a3`` is not already
// present (caller refuses the pairing in that case).
inline bool upsertSlave(HeadlessSlaveRec* table, uint8_t& count, uint8_t max,
                        const uint8_t a3[3], uint8_t groupId) {
  int8_t idx = findSlaveIdx(table, count, a3);
  if (idx >= 0) {
    table[idx].groupId = groupId;
    return true;
  }
  if (count >= max) return false;
  HeadlessSlaveRec& r = table[count];
  r.addr3[0] = a3[0];
  r.addr3[1] = a3[1];
  r.addr3[2] = a3[2];
  r.groupId  = groupId;
  ++count;
  return true;
}

// Drop every record. ``count`` reset to 0, all slot bytes zeroed. ``max``
// argument required because the array length is caller-owned.
inline void clearSlaveTable(HeadlessSlaveRec* table, uint8_t& count, uint8_t max) {
  for (uint8_t i = 0; i < max; ++i) {
    table[i].addr3[0] = 0;
    table[i].addr3[1] = 0;
    table[i].addr3[2] = 0;
    table[i].groupId  = 0;
  }
  count = 0;
}

// -------------------- Persist-debounce state (WLED-neutral) --------------------
// Decouples "the registry mutated" from "trigger a flash write". A pairing
// burst calls markPersistDirty() many times in succession; the loop pump
// fires the actual write (caller-side: configNeedsWrite = true) only once
// persistDebounceElapsed() returns true. Side-effect free — caller owns
// the time source and the persistence trigger.
struct PersistState {
  bool     dirty       = false;
  uint32_t lastMutAtMs = 0;
};

inline void markPersistDirty(PersistState& s, uint32_t now) {
  s.dirty       = true;
  s.lastMutAtMs = now;
}

inline bool persistDebounceElapsed(const PersistState& s, uint32_t now,
                                   uint32_t debounceMs) {
  if (!s.dirty) return false;
  return (now - s.lastMutAtMs) >= debounceMs;
}

inline void persistConsumed(PersistState& s) {
  s.dirty = false;
}

// -------------------- Scene-rebroadcast-debounce state --------------------
// One-shot debounced scheduler for the post-pairing OPC_HEADLESS rebroadcast.
// Caller arms via scheduleSceneRebroadcast() after a successful SET_GROUP
// (proactive boot-burst or individual reactive pairing); the loop pump fires
// the actual OPC_HEADLESS once sceneRebroadcastReady() returns true. Multiple
// arms within the debounce window collapse to a single rebroadcast.
struct SceneRebroadcastState {
  uint32_t pendingAtMs = 0;   // 0 = idle (no pending rebroadcast)
};

inline void scheduleSceneRebroadcast(SceneRebroadcastState& s, uint32_t now,
                                     uint32_t debounceMs) {
  s.pendingAtMs = now + debounceMs;
}

inline bool sceneRebroadcastReady(const SceneRebroadcastState& s, uint32_t now) {
  if (s.pendingAtMs == 0) return false;
  return (int32_t)(now - s.pendingAtMs) >= 0;
}

inline void sceneRebroadcastConsumed(SceneRebroadcastState& s) {
  s.pendingAtMs = 0;
}

// -------------------- Re-bind sweep cursor state --------------------
// Drives the post-promotion proactive SET_GROUP burst over the persistent
// slave registry. Side-effect free — caller looks up the indexed entry,
// builds the packet, attempts TX, and reports back via confirmReassignSent
// (advance cursor) or deferReassignRetry (retry same slot, e.g. on a busy
// TX queue). pickReassignTarget returns 0xFF for "no work this tick".
struct ReassignState {
  uint8_t  cursor       = 0xFF;   // 0xFF = idle / no sweep active
  uint32_t nextSendAtMs = 0;
};

inline void startReassign(ReassignState& s, uint8_t slaveCount,
                          uint32_t firstSendAtMs) {
  if (slaveCount == 0) {
    s.cursor = 0xFF;
    return;
  }
  s.cursor       = 0;
  s.nextSendAtMs = firstSendAtMs;
}

// Returns the slave index to send on this tick, or 0xFF if (a) sweep idle,
// (b) sweep already complete, or (c) interval has not elapsed yet.
inline uint8_t pickReassignTarget(const ReassignState& s, uint8_t slaveCount,
                                  uint32_t now) {
  if (s.cursor == 0xFF) return 0xFF;
  if (s.cursor >= slaveCount) return 0xFF;
  if ((int32_t)(now - s.nextSendAtMs) < 0) return 0xFF;
  return s.cursor;
}

// True once the cursor has advanced past the end of the registry — used by
// the caller to fire the post-sweep scene rebroadcast exactly once.
inline bool reassignSweepCompleted(const ReassignState& s, uint8_t slaveCount) {
  return s.cursor != 0xFF && s.cursor >= slaveCount;
}

inline void confirmReassignSent(ReassignState& s, uint32_t now,
                                uint32_t intervalMs) {
  s.cursor++;
  s.nextSendAtMs = now + intervalMs;
}

// TX queue was busy — keep the cursor, just reschedule the next attempt.
inline void deferReassignRetry(ReassignState& s, uint32_t now,
                               uint32_t intervalMs) {
  s.nextSendAtMs = now + intervalMs;
}

inline void abortReassign(ReassignState& s) {
  s.cursor = 0xFF;
}

// -------------------- Pairing-blip throttle --------------------
// Decision-only helper for IND_PAIRING_TX firing: returns true when the
// caller should fire the indicator (and updates lastBlipAtMs as a side
// effect on the reference). Returns false when the previous blip is too
// recent — keeps the 1500 ms indicator overlay from being re-extended
// into a continuous strobe during a back-to-back SET_GROUP sweep.
inline bool shouldFirePairingBlip(uint32_t& lastBlipAtMs, uint32_t now,
                                  uint32_t throttleMs) {
  if ((now - lastBlipAtMs) < throttleMs) return false;
  lastBlipAtMs = now;
  return true;
}

// -------------------- Group-id reservation --------------------
// Case-B decision logic from headlessAssignGroupTo(): returns the next free
// group ID and bumps the counter. Returns 0 if the counter is exhausted
// (caller refuses the assignment). Clamps below-range counters up to
// HEADLESS_FIRST_GROUP_ID before checking exhaustion, so a freshly-reset
// counter (0) correctly yields HEADLESS_FIRST_GROUP_ID (2) on the next
// reservation. The bump is automatic — caller does NOT need to re-add 1.
inline uint8_t reserveNextGroupId(uint8_t& counter) {
  if (counter < HEADLESS_FIRST_GROUP_ID) counter = HEADLESS_FIRST_GROUP_ID;
  if (counter > HEADLESS_MAX_GROUP_ID) return 0;
  const uint8_t id = counter;
  counter = (uint8_t)(counter + 1);
  return id;
}

// -------------------- Scene catalog (data-driven) --------------------

// Symbolic scene IDs travel over the wire in ``P_Headless.sceneId``. Add
// rows at the bottom; existing IDs are wire-stable.
enum HeadlessSceneId : uint8_t {
  SCENE_OFFSET_BREATHE      = 0,
  SCENE_SOLID_RED           = 1,
  SCENE_SOLID_GREEN         = 2,
  SCENE_ALL_OFF             = 3,
  SCENE_RESTORE_BOOT_COLOR  = 4,
};

// Scene-row behaviour flags (catalog-only, never on the wire).
static const uint8_t SCENE_FLAG_USE_OFFSET     = 0x01; // emit OPC_OFFSET before the scene
static const uint8_t SCENE_FLAG_RESTORE_LOCAL  = 0x02; // receiver re-runs its local boot-random color
static const uint8_t SCENE_FLAG_ALL_OFF        = 0x04; // receiver forces brightness=0, ignores other fields

// One row of the scene catalog. ``fxMode`` is the WLED effect index the
// receiver applies (0 = FX_MODE_STATIC). ``color1`` is RGB in 0xRRGGBB
// packing. ``offsetMode`` is one of RaceLinkProto::OffsetMode (only
// honored when ``flags & SCENE_FLAG_USE_OFFSET``).
struct HeadlessScene {
  uint8_t  sceneId;
  const char* label;
  uint8_t  fxMode;
  uint8_t  speed;
  uint8_t  intensity;
  uint32_t color1;       // 0xRRGGBB
  uint8_t  offsetMode;   // RaceLinkProto::OffsetMode
  int16_t  offsetBase;
  int16_t  offsetStep;
  uint8_t  flags;        // SCENE_FLAG_*
};

// Default catalog rows. Five entries cover the user-locked feature set
// (offset-breathe, solid red, solid green, all off, restore boot color).
static const HeadlessScene SCENE_CATALOG[] = {
  // sceneId,                   label,              fxMode,         spd, int, color1,      offsetMode,                     base, step, flags
  { SCENE_OFFSET_BREATHE,       "Offset Breathe",   3 /*BREATH*/,   128, 128, 0xFFFFFFu,   RaceLinkProto::OFFSET_MODE_LINEAR, 0,   400,  SCENE_FLAG_USE_OFFSET },
  { SCENE_SOLID_RED,            "Solid Red",        0 /*STATIC*/,   0,   0,   0xFF0000u,   RaceLinkProto::OFFSET_MODE_NONE,    0,   0,    0 },
  { SCENE_SOLID_GREEN,          "Solid Green",      0 /*STATIC*/,   0,   0,   0x00FF00u,   RaceLinkProto::OFFSET_MODE_NONE,    0,   0,    0 },
  { SCENE_ALL_OFF,              "All Off",          0 /*STATIC*/,   0,   0,   0x000000u,   RaceLinkProto::OFFSET_MODE_NONE,    0,   0,    SCENE_FLAG_ALL_OFF },
  { SCENE_RESTORE_BOOT_COLOR,   "Restore Boot",     0 /*STATIC*/,   0,   0,   0x000000u,   RaceLinkProto::OFFSET_MODE_NONE,    0,   0,    SCENE_FLAG_RESTORE_LOCAL },
};

static const uint8_t SCENE_CATALOG_SIZE =
    (uint8_t)(sizeof(SCENE_CATALOG) / sizeof(SCENE_CATALOG[0]));

// Look up a scene row by wire-stable ID. Returns nullptr if the ID is
// unknown to this build (forward-compat: older firmware silently drops).
inline const HeadlessScene* findSceneById(uint8_t sceneId) {
  for (uint8_t i = 0; i < SCENE_CATALOG_SIZE; ++i) {
    if (SCENE_CATALOG[i].sceneId == sceneId) return &SCENE_CATALOG[i];
  }
  return nullptr;
}

// Advance to the next catalog index (wraps). Used by the headless master
// to step through the catalog on each single-click while active.
inline uint8_t nextSceneIdx(uint8_t currentIdx) {
  if (currentIdx >= SCENE_CATALOG_SIZE) return 0;
  uint8_t n = (uint8_t)(currentIdx + 1);
  if (n >= SCENE_CATALOG_SIZE) n = 0;
  return n;
}

// -------------------- Headless-master runtime state --------------------

// Fields are caller-owned. The WLED usermod instantiates this inside its
// class; an external Gateway-side driver would manage its own copy.
struct HeadlessState {
  bool     active             = false;   // currently running as Headless Master
  bool     probing            = false;   // probe window open, decision pending
  bool     probeAborted       = false;   // an OPC_SET_GROUP arrived during the probe — abort
  uint8_t  currentSceneIdx    = 0xFF;    // index into SCENE_CATALOG (0xFF = none)
  uint8_t  broadcastBri       = 128;     // last-known fleet brightness
  uint32_t probeStartedAtMs   = 0;       // millis() when probe window opened (first send)
  uint32_t probeFirstSendAtMs = 0;       // scheduled local time for the first probe send
  uint32_t probeSecondSendAtMs= 0;       // scheduled local time for the second probe send
  bool     probeFirstSent     = false;
  bool     probeSecondSent    = false;
  uint32_t lastBroadcastAtMs  = 0;       // last time we emitted ANY headless packet (for keepalive)
};

// -------------------- Wire-stable flag mirrors --------------------
// Local mirror of the racelink_wled.h flag bits the WLED-neutral packet
// builders need. Values are wire-stable: GateCfgFlags in racelink_wled.h
// must keep these identical (single host-side source of truth is the
// project's flags.py; do not redefine the bit positions).
static const uint8_t RACELINK_FLAG_POWER_ON_BIT    = 1u << 0;
static const uint8_t RACELINK_FLAG_HAS_BRI_BIT     = 1u << 2;

// -------------------- Packet builders --------------------

// Build IDENTIFY_REPLY broadcast that probes for an active master. The
// receiver is hard-wired to FF:FF:FF; payload mirrors what an unpaired
// node would send (``groupId = 0``) so any Gateway treats it identically
// to a normal post-boot recovery announcement.
inline uint8_t buildIdentifyProbe(uint8_t* out,
                                  const uint8_t myLast3[3],
                                  const uint8_t myMac6[6] /* may be all-zero */,
                                  uint8_t fw,
                                  uint8_t devType) {
  using namespace RaceLinkProto;
  const uint8_t bcast[3] = { 0xFF, 0xFF, 0xFF };
  P_IdentifyReply p{};
  p.fw      = fw;
  p.caps    = devType;
  p.groupId = 0;   // signal "unpaired" so a master responds with SET_GROUP
  for (int i = 0; i < 6; ++i) p.mac6[i] = myMac6 ? myMac6[i] : 0;
  return build(out, myLast3, bcast, make_type(DIR_N2M, OPC_DEVICES), p);
}

// Unicast OPC_SET_GROUP to ``dstLast3``. Used by the headless master to
// pair an arriving unpaired node into the next free group.
inline uint8_t buildSetGroupPacket(uint8_t* out,
                                   const uint8_t myLast3[3],
                                   const uint8_t dstLast3[3],
                                   uint8_t groupId) {
  using namespace RaceLinkProto;
  P_SetGroup p{ groupId };
  return build(out, myLast3, dstLast3, make_type(DIR_M2N, OPC_SET_GROUP), p);
}

// Broadcast OPC_HEADLESS to all paired nodes. Body is {sceneId, brightness};
// receivers expand via their local Headless catalog. Receiver = FF:FF:FF.
// (Function and parameter names keep "scene" terminology because each
// catalog row is internally called a "Headless scene"; only the wire
// opcode itself was renamed away from OPC_SCENE on 2026-05-17 so that
// name can be reserved for a future host-level RaceLink-Scene opcode.)
inline uint8_t buildHeadlessPacket(uint8_t* out,
                                   const uint8_t myLast3[3],
                                   uint8_t sceneId,
                                   uint8_t brightness) {
  using namespace RaceLinkProto;
  const uint8_t bcast[3] = { 0xFF, 0xFF, 0xFF };
  P_Headless p{ sceneId, brightness };
  return build(out, myLast3, bcast, make_type(DIR_M2N, OPC_HEADLESS), p);
}

// Build the OPC_OFFSET packet a scene with SCENE_FLAG_USE_OFFSET requires
// to be sent BEFORE its OPC_HEADLESS. groupId=255 (broadcast). The wire
// layout matches the LINEAR/NONE/EXPLICIT modes in racelink_proto.h.
//
// Returns the total wire length, or 0 if the offset mode in the row is
// not yet supported by this builder (defensive — currently only NONE and
// LINEAR are emitted by the default catalog).
inline uint8_t buildOffsetPacketForScene(uint8_t* out,
                                         const uint8_t myLast3[3],
                                         const HeadlessScene& s) {
  using namespace RaceLinkProto;
  const uint8_t bcast[3] = { 0xFF, 0xFF, 0xFF };
  uint8_t bodyLen = 0;
  uint8_t body[8];
  body[0] = 255;          // groupId broadcast
  body[1] = s.offsetMode; // OffsetMode
  switch (s.offsetMode) {
    case OFFSET_MODE_NONE:
      bodyLen = 2;
      break;
    case OFFSET_MODE_EXPLICIT: {
      const uint16_t v = (uint16_t)s.offsetBase;
      body[2] = (uint8_t)(v & 0xFF);
      body[3] = (uint8_t)(v >> 8);
      bodyLen = 4;
    } break;
    case OFFSET_MODE_LINEAR: {
      const uint16_t base = (uint16_t)s.offsetBase;
      const uint16_t step = (uint16_t)s.offsetStep;
      body[2] = (uint8_t)(base & 0xFF);
      body[3] = (uint8_t)(base >> 8);
      body[4] = (uint8_t)(step & 0xFF);
      body[5] = (uint8_t)(step >> 8);
      bodyLen = 6;
    } break;
    default:
      return 0; // unsupported mode for this builder
  }

  // Header7 + body. ``build_empty`` writes the header; we append the body
  // manually so we can carry the variable-length OPC_OFFSET payload.
  uint8_t n = build_empty(out, myLast3, bcast, make_type(DIR_M2N, OPC_OFFSET));
  for (uint8_t i = 0; i < bodyLen; ++i) out[n + i] = body[i];
  return (uint8_t)(n + bodyLen);
}

// Broadcast brightness-only OPC_CONTROL: fieldMask = BRIGHTNESS, no
// other fields. groupId=255 (broadcast), flags = POWER_ON | HAS_BRI so
// the receiver actually applies it. Used by the headless master's long-
// press fade and by FPVGate's brightness slider.
inline uint8_t buildBrightnessPacket(uint8_t* out,
                                     const uint8_t myLast3[3],
                                     uint8_t brightness) {
  using namespace RaceLinkProto;
  const uint8_t bcast[3] = { 0xFF, 0xFF, 0xFF };
  uint8_t n = build_empty(out, myLast3, bcast, make_type(DIR_M2N, OPC_CONTROL));
  // OPC_CONTROL body (variable length): groupId, flags, fieldMask, value
  out[n++] = 255;                                    // groupId broadcast
  out[n++] = (uint8_t)(RACELINK_FLAG_POWER_ON_BIT | RACELINK_FLAG_HAS_BRI_BIT); // see below
  out[n++] = RL_CTRL_F_BRIGHTNESS;                   // fieldMask
  out[n++] = brightness;                             // value
  return n;
}

} // namespace RaceLinkHeadless
