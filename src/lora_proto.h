#pragma once
#include <stdint.h>
#include <stddef.h>

// LoRaProto v1.1 -- shared, header-only protocol for ESP32 + SX1262
// Packet = Header7 (3B sender + 3B receiver + 1B type) + Body (0..20B)
// Direction bit (0x80): 0 = Master->Node, 1 = Node->Master
// Broadcast: receiver3 == FF:FF:FF
// NOTE: All multi-byte fields are little-endian.

namespace LoraProto {

// -------------------- Versioning --------------------
static const uint8_t PROTO_VER_MAJOR = 1;
static const uint8_t PROTO_VER_MINOR = 4;

// -------------------- Direction/Type helpers --------------------
static const uint8_t DIR_M2N = 0x00;  // Master -> Node
static const uint8_t DIR_N2M = 0x80;  // Node   -> Master
static const uint8_t BODY_MAX = 20;

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
  OPC_CONTROL       = 0x04, // CONTROL (M2N)
  OPC_CONFIG        = 0x05, // CONFIG (M2N)
  OPC_SYNC          = 0x06, // SYNC Pulse (M2N)
  OPC_STREAM        = 0x07, // STREAM_M2N (M2N)
  OPC_ACK           = 0x7E, // ACK (both directions, as response only)
  // (optional future: 0x05 STATUS_UPDATE N2M unrequested telemetry)
};

// -------------------- Payloads --------------------
// Master -> Node
struct __attribute__((packed)) P_GetDevices  { uint8_t groupId; uint8_t flags;        }; // 2B
struct __attribute__((packed)) P_SetGroup    { uint8_t groupId;                       }; // 1B
struct __attribute__((packed)) P_GetStatus   { uint8_t groupId; uint8_t flags;        }; // 2B
struct __attribute__((packed)) P_Control     { uint8_t groupId; uint8_t flags; uint8_t presetId; uint8_t brightness; }; // 4B // add palette later?
struct __attribute__((packed)) P_Config      { uint8_t option; uint8_t data0; uint8_t data1; uint8_t data2; uint8_t data3; }; // 5B
struct __attribute__((packed)) P_Sync        { uint8_t ts24_0; uint8_t ts24_1; uint8_t ts24_2; uint8_t brightness; }; // 4B // 24-bit timestamp LSB first + bri
struct __attribute__((packed)) P_Stream      { uint8_t ctrl; uint8_t data[8];         }; // 9B

// Node -> Master
//struct __attribute__((packed)) P_IdentifyReply { uint8_t proto_ver_major; uint8_t proto_ver_minor; uint8_t caps; uint8_t groupId; uint8_t mac6[6]; }; // 10B
struct __attribute__((packed)) P_IdentifyReply { uint8_t fw; uint8_t caps; uint8_t groupId; uint8_t mac6[6]; }; // 10B // fw, caps, groupId, mac6[6] // 9B
//struct __attribute__((packed)) P_StatusReply   { uint8_t fw_major; uint8_t fw_minor; uint8_t fw_patch; uint16_t vbat_mV; int8_t rssi; int8_t snr; };   // 7B
struct __attribute__((packed)) P_StatusReply   { uint8_t flags; uint8_t configByte; uint8_t presetId; uint8_t brightness; uint16_t vbat_mV; int8_t rssi; int8_t snr; };   // 8B

// ACK (both directions, response only)
enum AckStatus : uint8_t { ACK_OK=0, ACK_BAD_TYPE=1, ACK_BAD_LEN=2, ACK_UNAUTHORIZED=3, ACK_BUSY=4, ACK_ERROR=5 };
struct __attribute__((packed)) P_Ack { uint8_t echo_opcode7; uint8_t status; uint8_t seq; }; // seq currently 0

static_assert(sizeof(P_Control) <= BODY_MAX, "P_Control too large");
static_assert(sizeof(P_Config) <= BODY_MAX, "P_Config too large");
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
  // OPC_CONTROL: CONTROL (M2N, 4B) -> no response
  { OPC_CONTROL,    DIR_M2N, RESP_NONE,     0,           SZ<P_Control>(),     0,                     "CONTROL" },
  // CONFIG (M2N, 5B) -> ACK
  { OPC_CONFIG,     DIR_M2N, RESP_ACK,      OPC_ACK,     SZ<P_Config>(),      SZ<P_Ack>(),           "CONFIG" },
  // OPC_SYNC: SYNC (M2N, 4B) -> no response
  { OPC_SYNC,       DIR_M2N, RESP_NONE,     0,           SZ<P_Sync>(),        0,                     "SYNC" },
  // OPC_STREAM: STREAM_M2N (M2N, 9B) -> ACK (only last packet in stream)
  { OPC_STREAM,     DIR_M2N, RESP_ACK,      OPC_ACK,     SZ<P_Stream>(),      SZ<P_Ack>(),           "STREAM_M2N" },
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
inline bool isBroadcast3(const uint8_t r3[3]) { return r3[0]==0xFF && r3[1]==0xFF && r3[2]==0xFF; } // TODO: auch weitere helper aus lora_link_core.h hierher oder alle helper nach lora_link_core.h verschieben?

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

} // namespace LoraProto
