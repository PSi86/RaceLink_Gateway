#pragma once
#include <stdint.h>
#include <stddef.h>

// RaceLink protocol v2.0 -- shared, header-only protocol for SX1262 / LLCC68 based nodes
// Packet = Header7 (3B sender + 3B receiver + 1B type) + Body (0..BODY_MAX B, currently 22)
// Phase-D rename (2026-04-25): OPC_CONTROL / P_Control refer to the variable-length
// direct-effect packet (0x08). The old fixed-length preset packet (0x04) is now
// OPC_PRESET / P_Preset. Opcode values are unchanged — byte-wire-compatible.
// Direction bit (0x80): 0 = Master->Node, 1 = Node->Master
// Broadcast: receiver3 == FF:FF:FF
// NOTE: All multi-byte fields are little-endian.

// Device Type Lookup (unused, only for reference)
enum RL_Dev_Type : uint8_t {
  GATEWAY_REV1 = 1,
  NODE_WLED_REV1 = 10,
  NODE_WLED_REV3 = 11,
  NODE_WLED_REV4 = 12,
  NODE_WLED_REV5 = 13,
  NODE_WLED_STARTBLOCK_REV3 = 50
  // Add more device types as needed
};

namespace RaceLinkProto {

// -------------------- Versioning --------------------
static const uint8_t PROTO_VER_MAJOR = 2;
static const uint8_t PROTO_VER_MINOR = 0;

// -------------------- Direction/Type helpers --------------------
static const uint8_t DIR_M2N = 0x00;  // Master -> Node
static const uint8_t DIR_N2M = 0x80;  // Node   -> Master
// BODY_MAX: max. Body-Länge aller RaceLink-Pakete.
// Historisch 20; 2026-04-24 auf 22 angehoben für OPC_CONTROL (post-rename;
// pre-rename OPC_CONTROL_ADV — das erste variable-length Paket, 3..21 B Body).
// +1 B Reserve. Fallback auf fixe Paketgröße: siehe Plan-Doku
// (plane-f-r-mich-ein-refactored-boole.md, Abschnitt "Fallback zu fixed-length").
static const uint8_t BODY_MAX = 22;

inline uint8_t type_dir(uint8_t t)  { return t & 0x80; }
inline uint8_t type_base(uint8_t t) { return t & 0x7F; }
inline uint8_t flip_dir(uint8_t t)  { return t ^ 0x80; }
inline uint8_t make_type(uint8_t dir, uint8_t opcode7) { return (dir | (opcode7 & 0x7F)); }

// -------------------- Header --------------------
struct __attribute__((packed)) Header7 {
  uint8_t sender[3];
  uint8_t receiver[3];
  uint8_t type; // DIR | opcode7
};

// -------------------- Opcodes (7-bit, shared) --------------------
// Use with make_type(DIR_*, OPC_*)
enum Opcode7 : uint8_t {
  OPC_DEVICES       = 0x01, // GET_DEVICES (M2N) / IDENTIFY_REPLY (N2M)
  OPC_SET_GROUP     = 0x02, // SET_GROUP (M2N)
  OPC_STATUS        = 0x03, // GET_STATUS (M2N) / STATUS_REPLY (N2M)
  OPC_PRESET        = 0x04, // PRESET (M2N) -- apply a WLED preset id (see P_Preset)
  OPC_CONFIG        = 0x05, // CONFIG (M2N)
  OPC_SYNC          = 0x06, // SYNC Pulse (M2N)
  OPC_STREAM        = 0x07, // STREAM_M2N (M2N)
  OPC_CONTROL       = 0x08, // CONTROL (M2N) -- variable-length direct effect params (see layout below)
  OPC_OFFSET        = 0x09, // OFFSET (M2N) -- per-group offset_ms snapshot used by ARM_ON_SYNC + OFFSET_MODE controls
  OPC_GET_CONFIG    = 0x0A, // GET_CONFIG (M2N, 1B body=option) -> reply: same opcode N2M, body=P_Config (5B). Read-back of OPC_CONFIG-style options so the host can detect device-vs-host divergence on dialog open.
  OPC_ACK           = 0x7E, // ACK (both directions, as response only)
  // Phase-D rename (2026-04-25): opcode values are invariant, only the
  // identifiers shifted: OPC_CONTROL -> OPC_PRESET (0x04), OPC_CONTROL_ADV
  // -> OPC_CONTROL (0x08). Byte-wire-compatible with older gateway/WLED
  // builds that still use the pre-rename names.
};

// -------------------- Payloads --------------------
// Master -> Node
struct __attribute__((packed)) P_GetDevices  { uint8_t groupId; uint8_t flags;        }; // 2B
struct __attribute__((packed)) P_SetGroup    { uint8_t groupId;                       }; // 1B
struct __attribute__((packed)) P_GetStatus   { uint8_t groupId; uint8_t flags;        }; // 2B
struct __attribute__((packed)) P_Preset      { uint8_t groupId; uint8_t flags; uint8_t presetId; uint8_t brightness; }; // 4B (OPC_PRESET; add palette later?)
struct __attribute__((packed)) P_Config      { uint8_t option; uint8_t data0; uint8_t data1; uint8_t data2; uint8_t data3; }; // 5B
// OPC_GET_CONFIG body — request to read one option's current device-side
// value. Reply uses the same opcode (N2M direction bit flipped) with a
// P_Config-shaped 5B body; ``data0..3`` carry the live value packed
// per-option in the same little-endian layout as the OPC_CONFIG write
// commands (uint8 / uint16 LE / uint16-pair). Unknown / write-only
// options (0x02 ClearMaster, 0x0F ClearAll, 0x81 Reboot, etc.) produce
// no reply; the host treats that as a timeout.
struct __attribute__((packed)) P_GetConfig   { uint8_t option;                       }; // 1B
// OPC_SYNC body. Wire length is 4B (legacy clock-tick form) or 5B
// (flag-bearing form). The 5th byte is `flags`; bit 0 =
// SYNC_FLAG_TRIGGER_ARMED. The device adjusts its timebase on every SYNC
// regardless of length; pending arm-on-sync state materialises ONLY when
// the trigger bit is set. Autosync (whether gateway- or host-driven)
// emits the 4B form so it cannot accidentally fire armed effects ahead of
// a deliberate sync. RULES[].req_len must be 0 (variable) for OPC_SYNC.
struct __attribute__((packed)) P_Sync        { uint8_t ts24_0; uint8_t ts24_1; uint8_t ts24_2; uint8_t brightness; uint8_t flags; }; // 5B (4B legacy w/o flags) // 24-bit timestamp LSB first + bri + flags
// SYNC flags. Trigger bit gates pending arm-on-sync materialisation; bit 1
// reserved for a future HAS_GROUP_MASK extension carrying selective fire.
static const uint8_t SYNC_FLAG_TRIGGER_ARMED = 0x01;
struct __attribute__((packed)) P_Stream      { uint8_t ctrl; uint8_t data[8];         }; // 9B
// OPC_OFFSET (Master -> Node, RESP_NONE) — variable-length 2..7 B body.
// First two bytes are always present:
//   Byte 0: groupId (0..254 = filter; 255 = broadcast to all groups)
//   Byte 1: mode    (OffsetMode enum below)
// The remaining bytes depend on mode (see OffsetMode comments). The receiver
// stores the parsed config in pending_change; it materialises into active
// at the next accepted OPC_CONTROL/OPC_PRESET (immediate-apply path) or at
// the OPC_SYNC that fires the queued effect (arm-on-sync path).
//
// Receivers compute their per-device offset_ms by evaluating the stored
// formula against their own current.groupId at the moment a CONTROL arms
// with OFFSET_MODE flag set. The per-device snapshot is then used by the
// SYNC handler's deferred-apply path. A subsequent OPC_OFFSET cannot
// retroactively shift an already-armed effect.
//
// Acceptance gate (strict symmetric, 2026-04-30):
//   The packet's OFFSET_MODE flag MUST match the receiver's effective
//   offset state. Both directions strict:
//     * F=1 + E=1 -> ACCEPT (use stored offset)
//     * F=0 + E=0 -> ACCEPT (normal immediate apply)
//     * F=1 + E=0 -> DROP   (use-offset request without configured offset)
//     * F=0 + E=1 -> DROP   (the device "stays in offset mode" until
//                            OPC_OFFSET(NONE) + materialisation transitions
//                            it out)
//   State transitions between "in offset mode" and "not in offset mode"
//   happen ONLY via OPC_OFFSET. CONTROL/PRESET packets dispatch effects
//   within the current state; they never transition the device's offset
//   configuration. The strict gate also gives Strategy A its broadcast
//   targeting: a single OPC_CONTROL with F=1 lands on exactly the
//   offset-configured devices.
// To leave offset mode the operator sends OPC_OFFSET(NONE) (sets
// pending=NONE so subsequent F=0 packets accept) followed by a
// materialisation event (OPC_PRESET, or ARM_ON_SYNC OPC_CONTROL with
// F=0 followed by OPC_SYNC). The scene_runner's ``offset_group(mode=none)``
// container performs both steps in one operator action.
//
// Variable length is signalled by req_len=0 in the rules table (same as
// OPC_CONTROL); body bounds are enforced in the receiver based on mode.
enum OffsetMode : uint8_t {
  OFFSET_MODE_NONE     = 0x00,  // no further payload (clears stored config; offset = 0)
  OFFSET_MODE_EXPLICIT = 0x01,  // +2 B: uint16 offset_ms (LE)
  OFFSET_MODE_LINEAR   = 0x02,  // +4 B: int16 base_ms (LE), int16 step_ms (LE)
                                //       offset = clamp(base + groupId * step, 0..65535)
  OFFSET_MODE_VSHAPE   = 0x03,  // +5 B: int16 base, int16 step, uint8 center
                                //       offset = clamp(base + |groupId - center| * step, 0..65535)
  OFFSET_MODE_MODULO   = 0x04,  // +5 B: int16 base, int16 step, uint8 cycle (0 -> 1)
                                //       offset = clamp(base + (groupId % cycle) * step, 0..65535)
  // 0x05..0xFE reserved for future modes (LOG2, STEP_BIN, RANDOM, ...)
};

// Worst-case OPC_OFFSET body size (mode header + largest payload). Used
// only for buffer sizing; the actual length is mode-driven.
static const uint8_t MAX_P_OFFSET = 7;
static_assert(MAX_P_OFFSET <= BODY_MAX, "MAX_P_OFFSET exceeds BODY_MAX");

// -------------------- Gateway USB events + commands (Master <-> Host) --------------------
// These constants do NOT travel over LoRa. They are USB-only event types
// emitted by the gateway firmware to the host (and command bytes from
// host to gateway) over the [0x00][LEN][TYPE][DATA] USB framing.
// They live in this shared header so the firmware and the host transport
// (racelink/transport/gateway_events.py) can never drift on the byte
// values — the same drift test that pins the wire protocol pins these.
//
// Gateway state-machine refactor (Batch B, 2026-04-28):
//   * EV_RX_WINDOW_OPEN (0xF1)  -> repurposed as EV_STATE_CHANGED.
//   * EV_RX_WINDOW_CLOSED (0xF2) -> retired; subsumed by EV_STATE_CHANGED(IDLE).
//   * EV_IDLE (0xF4)             -> repurposed as EV_TX_REJECTED.
//   * EV_STATE_REPORT (0xF5)     -> new; reply to GW_CMD_STATE_REQUEST.
//   * GW_CMD_STATE_REQUEST       -> new; host asks gateway for current state.
// The wire-vocabulary table in docs/PROTOCOL.md mirrors this set.
static const uint8_t EV_ERROR         = 0xF0;  // body: UTF-8 reason or reason code(s)
static const uint8_t EV_STATE_CHANGED = 0xF1;  // body: [state_byte, [metadata...]]
// 0xF2 retired (was EV_RX_WINDOW_CLOSED); subsumed by EV_STATE_CHANGED(IDLE).
static const uint8_t EV_TX_DONE       = 0xF3;  // body: 1 byte (last_len; legacy)
static const uint8_t EV_TX_REJECTED   = 0xF4;  // body: [type_full, reason_byte]
static const uint8_t EV_STATE_REPORT  = 0xF5;  // body: [state_byte, [metadata...]]

// Gateway state machine bytes carried inside EV_STATE_CHANGED /
// EV_STATE_REPORT body[0]. The state set depends on the gateway's
// default RX mode at boot:
//   * setDefaultRxContinuous (current setup): IDLE / TX / RX_WINDOW / ERROR.
//     IDLE means "in continuous RX, ready for the next host TX".
//   * setDefaultRxNone: RX / TX / RX_WINDOW / ERROR. RX means "actively
//     receiving"; there is no resting "doing nothing" state.
// The two sets are mutually exclusive at runtime (the gateway picks one
// at boot based on its mode and uses only that subset).
enum GatewayState : uint8_t {
  GW_STATE_IDLE      = 0x00,  // continuous RX, ready
  GW_STATE_TX        = 0x01,  // transmitting
  GW_STATE_RX_WINDOW = 0x02,  // bounded RX window open; metadata = uint16 min_ms LE
  GW_STATE_RX        = 0x03,  // active receive only (setDefaultRxNone mode)
  GW_STATE_ERROR     = 0xFE,  // fault; metadata = reason byte(s) or empty
};

// EV_TX_REJECTED reason codes carried in body[1]. body[0] echoes the
// rejected packet's type_full so the host can match the NACK to the
// offending send. The reason set is small on purpose — most rejections
// are txPending (single-slot scheduler busy); the others guard against
// host-side framing bugs.
static const uint8_t TX_REJECT_TXPENDING = 0x01;  // gateway already transmitting
static const uint8_t TX_REJECT_OVERSIZE  = 0x02;  // body too large for txBuf
static const uint8_t TX_REJECT_ZEROLEN   = 0x03;  // body empty / zero-length
static const uint8_t TX_REJECT_UNKNOWN   = 0xFF;

// Host -> Gateway USB-only command bytes (NOT LoRa opcodes). Sent as the
// TYPE byte in the [0x00][LEN][TYPE][DATA] framing. Gateway dispatches
// these in handleCommand() ahead of the wire-protocol DIR_M2N branch.
//
// IDENTIFY is the legacy port-discovery ping; STATE_REQUEST is the new
// (Batch B) gateway-state query that replies via EV_STATE_REPORT.
static const uint8_t GW_CMD_IDENTIFY      = 0x01;  // 1-byte payload [0x01]
static const uint8_t GW_CMD_STATE_REQUEST = 0x7F;  // 1-byte payload [0x7F] -> EV_STATE_REPORT

// -------------------- P_Control (variable-length, 3..21 B) --------------------
// Master -> Node, OPC_CONTROL. Direct effect-parameter packet (pre-rename:
// OPC_CONTROL_ADV / P_ControlAdv). First variable-length packet in RaceLink.
// Layout:
//   Byte 0   : groupId               (always)
//   Byte 1   : flags                 (always)  -- identical semantics/bit layout to OPC_PRESET
//                                                 (POWER_ON, ARM_ON_SYNC, HAS_BRI, FORCE_TT0,
//                                                  FORCE_REAPPLY, OFFSET_MODE); bits 6-7 reserved.
//                                                 Single host-side source of truth: racelink/domain/flags.py.
//   Byte 2   : fieldMask             (always)  -- which single-byte fields follow, in fixed order:
//                bit 0 RL_CTRL_F_BRIGHTNESS     -> +1 B u8
//                bit 1 RL_CTRL_F_MODE           -> +1 B u8   (WLED effect index)
//                bit 2 RL_CTRL_F_SPEED          -> +1 B u8
//                bit 3 RL_CTRL_F_INTENSITY      -> +1 B u8
//                bit 4 RL_CTRL_F_CUSTOM1        -> +1 B u8
//                bit 5 RL_CTRL_F_CUSTOM2        -> +1 B u8
//                bit 6 RL_CTRL_F_CUSTOM3_CHECKS -> +1 B (bits 0-4 custom3, bits 5-7 check1/2/3)
//                bit 7 RL_CTRL_F_EXT            -> extMask byte + extended payload follows
//   Byte X   : extMask               (only if RL_CTRL_F_EXT set) -- extended fields in fixed order:
//                bit 0 RL_CTRL_E_PALETTE        -> +1 B u8
//                bit 1 RL_CTRL_E_COLOR1         -> +3 B RGB
//                bit 2 RL_CTRL_E_COLOR2         -> +3 B RGB
//                bit 3 RL_CTRL_E_COLOR3         -> +3 B RGB
//                bits 4-7 reserved
// Max body when all fields present: 3 + 7 + 1 + 1 + 9 = 21 bytes  (<= BODY_MAX=22, 1 B reserve).
// Variable length: RULES[] uses req_len=0 -> decide_response() skips length check.
// Fallback to fixed-length struct: see project plan doc, section "Fallback zu fixed-length".

static const uint8_t RL_CTRL_F_BRIGHTNESS     = 0x01;
static const uint8_t RL_CTRL_F_MODE           = 0x02;
static const uint8_t RL_CTRL_F_SPEED          = 0x04;
static const uint8_t RL_CTRL_F_INTENSITY      = 0x08;
static const uint8_t RL_CTRL_F_CUSTOM1        = 0x10;
static const uint8_t RL_CTRL_F_CUSTOM2        = 0x20;
static const uint8_t RL_CTRL_F_CUSTOM3_CHECKS = 0x40;
static const uint8_t RL_CTRL_F_EXT            = 0x80;

static const uint8_t RL_CTRL_E_PALETTE        = 0x01;
static const uint8_t RL_CTRL_E_COLOR1         = 0x02;
static const uint8_t RL_CTRL_E_COLOR2         = 0x04;
static const uint8_t RL_CTRL_E_COLOR3         = 0x08;

// Packing of custom3_checks byte
static const uint8_t RL_CTRL_C3_MASK     = 0x1F; // bits 0-4
static const uint8_t RL_CTRL_CHECK1_BIT  = 0x20; // bit 5
static const uint8_t RL_CTRL_CHECK2_BIT  = 0x40; // bit 6
static const uint8_t RL_CTRL_CHECK3_BIT  = 0x80; // bit 7

// Worst-case body size for OPC_CONTROL (all fields present incl. extMask + all 3 colors).
static const uint8_t MAX_P_CONTROL = 21;
static_assert(MAX_P_CONTROL <= BODY_MAX, "MAX_P_CONTROL exceeds BODY_MAX");

// Node -> Master
//struct __attribute__((packed)) P_IdentifyReply { uint8_t proto_ver_major; uint8_t proto_ver_minor; uint8_t caps; uint8_t groupId; uint8_t mac6[6]; }; // 10B
struct __attribute__((packed)) P_IdentifyReply { uint8_t fw; uint8_t caps; uint8_t groupId; uint8_t mac6[6]; }; // 10B // fw, caps, groupId, mac6[6] // 9B
//struct __attribute__((packed)) P_StatusReply   { uint8_t fw_major; uint8_t fw_minor; uint8_t fw_patch; uint16_t vbat_mV; int8_t rssi; int8_t snr; };   // 7B
// effectId carries the active WLED segment mode index (renamed from presetId
// 2026-04-25 — wire format unchanged, semantics shift). A future OPC_STATUS_EXT
// will carry the full DeviceState snapshot (mode/speed/customs/colors/etc.).
struct __attribute__((packed)) P_StatusReply   { uint8_t flags; uint8_t configByte; uint8_t effectId; uint8_t brightness; uint16_t vbat_mV; int8_t rssi; int8_t snr; };   // 8B

// ACK (both directions, response only)
enum AckStatus : uint8_t { ACK_OK=0, ACK_BAD_TYPE=1, ACK_BAD_LEN=2, ACK_UNAUTHORIZED=3, ACK_BUSY=4, ACK_ERROR=5 };
struct __attribute__((packed)) P_Ack { uint8_t echo_opcode7; uint8_t status; uint8_t seq; }; // seq currently 0

static_assert(sizeof(P_Preset) <= BODY_MAX, "P_Preset too large");
static_assert(sizeof(P_Config) <= BODY_MAX, "P_Config too large");
static_assert(sizeof(P_GetConfig) <= BODY_MAX, "P_GetConfig too large");
static_assert(sizeof(P_Sync) <= BODY_MAX, "P_Sync too large");
static_assert(sizeof(P_Stream) <= BODY_MAX, "P_Stream too large");
static_assert(sizeof(P_IdentifyReply) <= BODY_MAX, "P_IdentifyReply too large");
static_assert(sizeof(P_StatusReply) <= BODY_MAX, "P_StatusReply too large");
static_assert(sizeof(P_Ack) <= BODY_MAX, "P_Ack too large");

// -------------------- Response policy registry --------------------
enum RespPolicy : uint8_t { RESP_NONE=0, RESP_ACK=1, RESP_SPECIFIC=2 };

struct PacketRule {
  uint8_t  opcode7;     // shared opcode (7-bit)
  uint8_t  req_dir;     // DIR_M2N or DIR_N2M: who normally sends the request
  RespPolicy policy;    // how to answer a request
  uint8_t  rsp_opcode7; // for SPECIFIC: opcode7 of the reply (usually identical to request opcode7)
  uint8_t  req_len;     // expected body length for request
  uint8_t  rsp_len;     // expected body length for reply (0 if RESP_NONE / RESP_ACK)
  const char* name;     // debug label
};

// Forward decl of size helpers
template<typename T> constexpr uint8_t SZ() { return (uint8_t)sizeof(T); }

// Rules (keep small and constexpr-friendly)
static constexpr PacketRule RULES[] = {
  // OPC_DEVICES: GET_DEVICES (M2N, 2B) -> IDENTIFY_REPLY (N2M, 10B)
  { OPC_DEVICES,    DIR_M2N, RESP_SPECIFIC, OPC_DEVICES, SZ<P_GetDevices>(),  SZ<P_IdentifyReply>(), "DEVICES/IDENTIFY" },
  // SET_GROUP (M2N, 1B) -> ACK
  { OPC_SET_GROUP,  DIR_M2N, RESP_ACK,      OPC_ACK,     SZ<P_SetGroup>(),    SZ<P_Ack>(),           "SET_GROUP" },
  // OPC_STATUS: GET_STATUS (M2N, 2B) -> STATUS_REPLY (N2M, 8B)
  { OPC_STATUS,     DIR_M2N, RESP_SPECIFIC, OPC_STATUS,  SZ<P_GetStatus>(),   SZ<P_StatusReply>(),   "STATUS" },
  // OPC_PRESET: PRESET (M2N, 4B) -> no response
  { OPC_PRESET,     DIR_M2N, RESP_NONE,     0,           SZ<P_Preset>(),      0,                     "PRESET" },
  // OPC_CONTROL: direct effect-parameter packet (M2N, variable length 3..21B) -> no response.
  // req_len = 0 signals variable length; decide_response() at the check below skips the
  // length comparison when req_len == 0. Body layout is documented above near P_Control.
  { OPC_CONTROL,    DIR_M2N, RESP_NONE,     0,           0 /*variable*/,      0,                     "CONTROL" },
  // CONFIG (M2N, 5B) -> ACK
  { OPC_CONFIG,     DIR_M2N, RESP_ACK,      OPC_ACK,     SZ<P_Config>(),      SZ<P_Ack>(),           "CONFIG" },
  // OPC_SYNC: SYNC (M2N, variable 4..5B) -> no response.
  // req_len = 0 signals variable length; legacy 4B form has no flags byte
  // (clock-tick only), 5B form carries SYNC_FLAG_* in the trailing byte.
  { OPC_SYNC,       DIR_M2N, RESP_NONE,     0,           0 /*variable*/,      0,                     "SYNC" },
  // OPC_OFFSET: variable-length offset config (M2N, 2..7B) -> no response.
  // req_len = 0 signals variable length; receiver decodes mode-specific
  // payload sizing — see OffsetMode enum above.
  { OPC_OFFSET,     DIR_M2N, RESP_NONE,     0,           0 /*variable*/,      0,                     "OFFSET" },
  // OPC_STREAM: STREAM_M2N (M2N, 9B) -> ACK (only last packet in stream)
  { OPC_STREAM,     DIR_M2N, RESP_ACK,      OPC_ACK,     SZ<P_Stream>(),      SZ<P_Ack>(),           "STREAM_M2N" },
  // OPC_GET_CONFIG: read-back of an OPC_CONFIG-style option (M2N, 1B body=option)
  // -> reply: same opcode with N2M direction bit, body=P_Config (5B) carrying
  // option + data0..3 packed per the matching write command's layout. Used by
  // the host's "Device Options" dialog to detect host-vs-device divergence on
  // open; absent reply means "device does not expose this option for read".
  { OPC_GET_CONFIG, DIR_M2N, RESP_SPECIFIC, OPC_GET_CONFIG, SZ<P_GetConfig>(), SZ<P_Config>(),       "GET_CONFIG" },
};

// Lookup by 7-bit opcode
inline const PacketRule* find_rule(uint8_t opcode7) {
  for (size_t i=0; i<sizeof(RULES)/sizeof(RULES[0]); ++i) {
    if (RULES[i].opcode7 == opcode7) return &RULES[i];
  }
  return nullptr;
}

// Decide the response for an incoming packet header + body length.
// Returns policy and the response type (full type including flipped dir & opcode) if applicable.
struct RespDecision {
  RespPolicy policy;
  uint8_t    resp_type; // valid if policy != RESP_NONE
};
inline RespDecision decide_response(uint8_t in_type, uint8_t in_body_len) {
  RespDecision d{ RESP_NONE, 0 };
  const uint8_t dir = type_dir(in_type);
  const uint8_t opc = type_base(in_type);
  const PacketRule* r = find_rule(opc);
  if (!r) return d;
  // only requests (from req_dir side) trigger a response
  if (r->req_dir != dir) return d;
  // basic body length sanity (optional: relax in production)
  // NOTE: req_len == 0 means "variable length" (e.g. OPC_CONTROL_ADV) -- length check is skipped.
  if (r->req_len && in_body_len != r->req_len) {
    // we could return NACK here, but NACK not requested now
    return d;
  }
  if (r->policy == RESP_ACK) {
    d.policy = RESP_ACK;
    d.resp_type = make_type(flip_dir(dir), OPC_ACK);
  } else if (r->policy == RESP_SPECIFIC) {
    d.policy = RESP_SPECIFIC;
    d.resp_type = make_type(flip_dir(dir), r->rsp_opcode7);
  }
  return d;
}

// -------------------- Pack/Unpack helpers --------------------
inline void put3(uint8_t dst[3], const uint8_t src[3]) { dst[0]=src[0]; dst[1]=src[1]; dst[2]=src[2]; }
inline bool isBroadcast3(const uint8_t r3[3]) { return r3[0]==0xFF && r3[1]==0xFF && r3[2]==0xFF; } // TODO: put all helpers in racelink_transport_core.h or collect all here?

inline bool parseHeader(const uint8_t* buf, uint8_t len, Header7& h) {
  if (len < sizeof(Header7)) return false;
  const Header7* src = reinterpret_cast<const Header7*>(buf);
  for (int i=0;i<3;i++){ h.sender[i]=src->sender[i]; h.receiver[i]=src->receiver[i]; }
  h.type = src->type;
  return true;
}

template<typename PayloadT>
inline bool parseBody(const uint8_t* buf, uint8_t len, PayloadT& out) {
  const uint8_t need = (uint8_t)(sizeof(Header7) + sizeof(PayloadT));
  if (len != need) return false;                      // exakter Match ist hier safer
  const uint8_t* body = buf + sizeof(Header7);
  memcpy(&out, body, sizeof(PayloadT));               // direkter memcpy reicht (PayloadT packed!)
  return true;
}

template<typename PayloadT>
inline uint8_t build(uint8_t* out, const uint8_t s3[3], const uint8_t r3[3], uint8_t full_type, const PayloadT& p) {
  Header7* h = reinterpret_cast<Header7*>(out);
  put3(h->sender, s3); put3(h->receiver, r3); h->type = full_type;
  uint8_t* body = out + sizeof(Header7);
  const uint8_t bodyLen = (uint8_t)sizeof(PayloadT);
  for (uint8_t i=0;i<bodyLen;i++) body[i] = reinterpret_cast<const uint8_t*>(&p)[i];
  return (uint8_t)(sizeof(Header7) + bodyLen);
}

inline uint8_t build_empty(uint8_t* out, const uint8_t s3[3], const uint8_t r3[3], uint8_t full_type) {
  Header7* h = reinterpret_cast<Header7*>(out);
  put3(h->sender, s3); put3(h->receiver, r3); h->type = full_type;
  return (uint8_t)sizeof(Header7);
}

struct StreamCtrl {
  bool start;
  bool stop;
  uint8_t packets_left;
};

inline uint8_t encode_stream_ctrl(bool start, bool stop, uint8_t packets_left) {
  uint8_t ctrl = (start ? 0x80U : 0x00U) | (stop ? 0x40U : 0x00U);
  return static_cast<uint8_t>(ctrl | (packets_left & 0x3FU));
}

inline StreamCtrl decode_stream_ctrl(uint8_t ctrl) {
  StreamCtrl decoded{};
  decoded.start = (ctrl & 0x80U) != 0U;
  decoded.stop = (ctrl & 0x40U) != 0U;
  decoded.packets_left = static_cast<uint8_t>(ctrl & 0x3FU);
  return decoded;
}

} // namespace RaceLinkProto
