/*
 * RaceLink USB Gateway v4
 * - TX nur aus IDLE, RX-Fenster mit Timeout
 * - Streaming (max 16 Pakete * 8 Bytes = 128 Bytes) mit Post-TX-RX-Fenster
 * - OLED jetzt über U8g2, "Bold" = Doppelt zeichnen
 */

#include <Arduino.h>
#include "racelink_proto.h"
#include "racelink_transport_core.h"

static RaceLinkTransport::Core rl{};
static RaceLinkTransport::Callbacks cb{};

// -------------------- Multithreading globals (Batch C, 2026-05-10) --------------------
// The gateway runs four FreeRTOS tasks:
//   * rl-display  (Core 0) — OLED renderer; pre-existing.
//   * rl-usb-tx   (Core 0) — drains g_usbTxQ → Serial.write. Single writer to Serial.
//   * rl-usb-rx   (Core 0) — recvSerialBytes() + button polling. Pushes complete
//                            host frames onto g_hostCmdQ.
//   * rl-radio    (Core 1) — sole owner of `rl`, SX1262 SPI, the TX/RX state
//                            machine, command processing, and auto-SYNC. Woken
//                            by DIO1 ISR notification and by cross-task notifies
//                            from rl-usb-rx when a host frame lands.
// Single-owner discipline replaces every mutex: rl-radio is the only task that
// touches `rl`. Inter-task data crosses via two FreeRTOS queues. The display
// path retains its existing portMUX + notify primitives.
struct UsbTxItem   { uint8_t buf[40]; uint8_t len; };
struct HostCmdItem { uint8_t buf[36]; uint8_t len; };

static QueueHandle_t g_usbTxQ      = nullptr;   // radio/cmd → rl-usb-tx, 16 slots
static QueueHandle_t g_hostCmdQ    = nullptr;   // rl-usb-rx → rl-radio,  4 slots
static TaskHandle_t  g_radioTask   = nullptr;   // DIO1 ISR / rl-usb-rx wake target
static TaskHandle_t  g_usbTxTask   = nullptr;
static TaskHandle_t  g_usbRxTask   = nullptr;

// Drop counters: queue overflow means we lost telemetry, not radio work. Kept
// for future EV_ERROR telemetry; not currently surfaced.
static volatile uint32_t g_usbTxDropCount   = 0;
static volatile uint32_t g_hostCmdDropCount = 0;

#include <SPI.h>
#include <Wire.h>
//#include <RadioLib.h>     // SX1262
#include <U8g2lib.h>

/************ LoRa PHY ************/
#define RACELINK_FREQ_HZ             867700000UL //868000000UL
#define RACELINK_TX_POWER             14           // dBm
#define RACELINK_BW_KHZ                 125.0        // 0 => 125 kHz -> 125.0 in RadioLib
#define RACELINK_SF                     7            // SF7
#define RACELINK_CR_DEN                 5            // 4/5 -> 5
#define RACELINK_PREAMBLE               8
#define RACELINK_SYNCWORD               0x12

/************ USB Link ************/
#define BAUDRATE        921600

/************ RX Wait times ************/
#define RX_WINDOW_BROADCAST_MS  2000
#define RX_WINDOW_UNICAST_MS    1000

/************ Button Pin ************/
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
#ifndef RACELINK_CS
  #error "Define RACELINK_CS in platformio.ini build_flags"
#endif
#ifndef RACELINK_DIO1
  #error "Define RACELINK_DIO1 in platformio.ini build_flags"
#endif
#ifndef RACELINK_BUSY
  #error "Define RACELINK_BUSY in platformio.ini build_flags"
#endif
#ifndef RACELINK_RST
  #error "Define RACELINK_RST in platformio.ini build_flags"
#endif
#ifndef RACELINK_SCK
  #error "Define RACELINK_SCK in platformio.ini build_flags"
#endif
#ifndef RACELINK_MISO
  #error "Define RACELINK_MISO in platformio.ini build_flags"
#endif
#ifndef RACELINK_MOSI
  #error "Define RACELINK_MOSI in platformio.ini build_flags"
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
// Device->Host RaceLink transport forward: TYPE = Header7.type (N2M), DATA = [Header7][Body][RSSI(LE16)][SNR(i8)]
// Device->Host Events: see racelink_proto.h ("Gateway USB events + commands"):
//   EV_ERROR (0xF0), EV_STATE_CHANGED (0xF1), EV_TX_DONE (0xF3),
//   EV_TX_REJECTED (0xF4), EV_STATE_REPORT (0xF5).
// Host->Device commands: GW_CMD_IDENTIFY (0x01), GW_CMD_STATE_REQUEST (0x7F).

// Push raw bytes onto g_usbTxQ for rl-usb-tx to write to Serial. Non-blocking:
// on a full queue we drop the item and bump g_usbTxDropCount — telemetry loss
// is preferable to stalling the RF callback path on USB-CDC throughput. Pre-
// queue this was a direct Serial.write() in radio-callback context on Core 1,
// which could block ~10 ms per coalesced frame and back-pressure the loop into
// missing the next DIO1 IRQ.
static inline void usb_tx_enqueue_raw(const uint8_t* data, uint8_t len) {
  if (!g_usbTxQ || !data || !len) return;
  UsbTxItem it;
  if (len > sizeof(it.buf)) len = sizeof(it.buf);
  memcpy(it.buf, data, len);
  it.len = len;
  if (xQueueSend(g_usbTxQ, &it, 0) != pdTRUE) {
    g_usbTxDropCount++;
  }
}

static inline void usb_send_frame(uint8_t type, const uint8_t* data, uint8_t len) {
  // Build the framed packet [0x00][LEN][TYPE][DATA...] into a stack buffer and
  // hand it to rl-usb-tx via g_usbTxQ. The coalescing rationale (one USB-CDC
  // transaction per frame, avoid the host bridge's 16 ms latency_timer) is
  // preserved: rl-usb-tx issues exactly one Serial.write(it.buf, it.len) per
  // queue item.
  //
  // Frame size is bounded: max event body in this gateway is ~32 bytes
  // (a forwarded N2M packet with RSSI/SNR), so a 40-byte queue slot is
  // generous. Oversized callers are dropped + counted — none today.
  if (len > 40 - 3) {
    g_usbTxDropCount++;
    return;
  }
  uint8_t buf[40];
  buf[0] = 0x00;               // SOF
  buf[1] = (uint8_t)(1 + len); // LEN = TYPE + DATA bytes
  buf[2] = type;
  if (len) memcpy(&buf[3], data, len);
  usb_tx_enqueue_raw(buf, (uint8_t)(3 + len));
}

static inline void usb_send_event_u8(uint8_t evType, uint8_t v) {
  uint8_t d[1] = { v };
  usb_send_frame(evType, d, 1);
}
static inline void usb_send_event_u16(uint8_t evType, uint16_t v) {
  uint8_t d[2] = { (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF) };
  usb_send_frame(evType, d, 2);
}
static inline void usb_send_event_buf(uint8_t evType, const uint8_t* data, uint8_t len) {
  usb_send_frame(evType, data, len);
}

// -------------------- Gateway state machine (Batch B, 2026-04-28) --------------------
// The gateway is the single source of truth for its own state. setGatewayState()
// emits one EV_STATE_CHANGED on every transition (deduplicates idempotent sets
// so we don't spam the host on every continuous-RX re-entry that didn't actually
// change the state byte). The host mirrors this 1:1 — no derived state machine.
// EV_STATE_CHANGED body shape:
//   * 1 byte  (state_byte) for IDLE / TX / RX / ERROR.
//   * 3 bytes (state_byte, min_ms_LE16) for RX_WINDOW.

static uint8_t  gw_currentState   = 0xFF; // sentinel "uninitialised" so the first set always emits
static uint16_t gw_currentMetaU16 = 0;

static inline void emit_state_event(uint8_t evType, uint8_t state_byte, uint16_t meta_u16, bool has_meta) {
  if (has_meta) {
    uint8_t d[3] = { state_byte, (uint8_t)(meta_u16 & 0xFF), (uint8_t)((meta_u16 >> 8) & 0xFF) };
    usb_send_event_buf(evType, d, 3);
  } else {
    uint8_t d[1] = { state_byte };
    usb_send_event_buf(evType, d, 1);
  }
}

static inline void setGatewayState(uint8_t state_byte, uint16_t meta_u16 = 0, bool has_meta = false) {
  if (state_byte == gw_currentState && meta_u16 == gw_currentMetaU16) {
    return; // no-op; the host already has this exact state
  }
  gw_currentState   = state_byte;
  gw_currentMetaU16 = meta_u16;
  emit_state_event(RaceLinkProto::EV_STATE_CHANGED, state_byte, meta_u16, has_meta);
}

static inline void emit_state_report() {
  // Reply to GW_CMD_STATE_REQUEST. Always emit even if state hasn't changed —
  // this is a query-response, not a transition notification. RX_WINDOW carries
  // its min_ms metadata; everything else is a 1-byte body.
  bool has_meta = (gw_currentState == RaceLinkProto::GW_STATE_RX_WINDOW);
  emit_state_event(RaceLinkProto::EV_STATE_REPORT, gw_currentState, gw_currentMetaU16, has_meta);
}

// NACK helper: emit EV_TX_REJECTED(type_full, reason) so the host can match
// the rejection to the offending send. Reason codes are defined in
// racelink_proto.h (TX_REJECT_TXPENDING / OVERSIZE / ZEROLEN / UNKNOWN).
static inline void usb_send_tx_rejected(uint8_t type_full, uint8_t reason) {
  uint8_t d[2] = { type_full, reason };
  usb_send_event_buf(RaceLinkProto::EV_TX_REJECTED, d, 2);
}

// scheduleSend wrapper: pre-checks the same rejection conditions the core
// uses (txPending, oversize, zerolen) and emits a typed NACK on rejection.
// Returns true iff the frame was accepted into the single-slot scheduler.
//
// Why jitterMaxMs defaults to 0 here (Batch B follow-up, 2026-04-28):
// The host's synchronous _send_m2n already serialises USB writes via
// _tx_lock — only one frame is ever in flight at a time. There is no
// burst-from-the-host scenario for which gateway-side pre-TX jitter
// would help; it just adds latency.
//
// scheduleSend's own ``jitterMaxMs=2500`` parameter default is a footgun
// for any caller that forgets to override it: when the gateway runs with
// LBT off (its intentional master-side default — see transport_init()),
// the call lands in randMs(50, 2500) and every host command waits a
// random 50-2500 ms before the radio actually transmits. Empirically a
// single small WLED preset send measured ~1097 ms wall-clock under that
// default — well past the host's 2 s deadlock guard for an unlucky draw.
//
// When LBT is on the same scheduleSend silently overwrites jitterMaxMs
// to 300 ms (transport_core.h line 318), which is why toggling LBT
// "fixes" the symptom — the cap is hidden inside the LBT branch and has
// nothing to do with CAD itself. Passing 0 here makes the wrapper opt
// out of jitter entirely, so observed latency reduces to airtime + USB
// regardless of the LBT setting.
static inline bool try_schedule_or_nack(RaceLinkTransport::Core& rl,
                                        const uint8_t* buf, uint8_t len,
                                        uint8_t type_full,
                                        uint16_t jitterMaxMs = 0) {
  if (len == 0) {
    usb_send_tx_rejected(type_full, RaceLinkProto::TX_REJECT_ZEROLEN);
    return false;
  }
  if (len > sizeof(rl.txBuf)) {
    usb_send_tx_rejected(type_full, RaceLinkProto::TX_REJECT_OVERSIZE);
    return false;
  }
  if (rl.txPending) {
    usb_send_tx_rejected(type_full, RaceLinkProto::TX_REJECT_TXPENDING);
    return false;
  }
  bool ok = RaceLinkTransport::scheduleSend(rl, buf, len, jitterMaxMs);
  if (!ok) {
    // Defence in depth: pre-checks passed but the core still refused. The
    // race window is narrow (no preemptive scheduling here) but the NACK
    // keeps the host's outcome registry honest under all conditions.
    usb_send_tx_rejected(type_full, RaceLinkProto::TX_REJECT_UNKNOWN);
  }
  return ok;
}

/************ OLED (U8g2) ************/
// SSD1306 64x32 (UNIVISION) Full buffer, HW I2C
//SSD1306 64X32_NONAME: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp#ssd1306-64x32_1f-1
//Full Frame Buffer and Pin Definitions:
U8G2_SSD1306_64X32_1F_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ OLED_RST);

/************ MAC ************/
// racelink_transport_core.h enthält jetzt die MAC-bezogenen Elemente (rl.myMac6, rl.myLast3, rl.macReadOK)
/* uint8_t myMac6[6]  = {0};
uint8_t myLast3[3] = {0};
bool macReadOK = false;
 */
/************ Serial Handling (dein Parser) ************/
// Robust serial frame receiver for framing: [0x00][LEN][TYPE][DATA...]
// LEN = number of bytes following LEN (TYPE + DATA). On a complete frame the
// receiver pushes a HostCmdItem onto g_hostCmdQ and notifies rl-radio.
// Pre-refactor this set the global newSerialData flag for the Arduino loop()
// to drain — that mechanism is gone; the queue + notify is the sole signal.
const uint16_t FRAG_TIMEOUT_MS = 25;   // 10–30 ms sind praxisnah bei 921600 Baud

void recvSerialBytes() {
  static bool     recvInProgress = false;
  static uint8_t  ndx            = 0;                       // bytes stored in frameBuf incl. LEN
  static uint8_t  dataLen        = 0;                       // LEN byte value (TYPE+DATA expected)
  static uint32_t lastByteAtMs   = 0;
  static uint8_t  frameBuf[sizeof(HostCmdItem::buf)];        // local accumulator → queue slot

  // Fragment-Timeout auch ohne ankommende Bytes prüfen.
  if (recvInProgress) {
    uint32_t now = millis();
    if ((uint32_t)(now - lastByteAtMs) > FRAG_TIMEOUT_MS && Serial.available() == 0) {
      recvInProgress = false;
      ndx = 0; dataLen = 0;
      // usb_send_event_u16(EV_ERROR, 3); // 3 = TIMEOUT (optional)
    }
  }

  while (Serial.available() > 0) {
    uint8_t rb = (uint8_t)Serial.read();
    lastByteAtMs = millis();

    if (!recvInProgress) {
      if (rb == 0x00) {
        recvInProgress = true;
        ndx = 0;
        dataLen = 0;
      }
      continue;  // alles andere ignorieren bis zum nächsten 0x00
    }

    if (ndx == 0) {
      dataLen = rb;
      if (dataLen == 0 || dataLen > (uint8_t)(sizeof(frameBuf) - 1)) {
        recvInProgress = false;
        ndx = 0; dataLen = 0;
        // usb_send_event_u16(EV_ERROR, 1); // 1 = LEN_INVALID
        continue;
      }
      frameBuf[ndx++] = dataLen;
      continue;
    }

    if (ndx < sizeof(frameBuf)) {
      frameBuf[ndx++] = rb;
    } else {
      recvInProgress = false;
      ndx = 0; dataLen = 0;
      // usb_send_event_u16(EV_ERROR, 2); // 2 = OVERRUN
      continue;
    }

    if (ndx == (uint16_t)dataLen + 1) {
      // Komplettes Frame: in HostCmdItem packen und an rl-radio übergeben.
      HostCmdItem item;
      item.len = ndx;
      memcpy(item.buf, frameBuf, ndx);
      if (g_hostCmdQ) {
        if (xQueueSend(g_hostCmdQ, &item, 0) != pdTRUE) {
          g_hostCmdDropCount++;
        } else if (g_radioTask) {
          // Radio-Task sofort wecken statt auf die 10 ms-Periode zu warten.
          xTaskNotifyGive(g_radioTask);
        }
      }
      recvInProgress = false;
      ndx = 0; dataLen = 0;
      return;  // ein Frame pro Aufruf — Aufrufer ruft uns periodisch (5 ms-Tick).
    }
  }
}

/************ OLED Helpers (U8g2) ************/
static void drawStringBold(int16_t x, int16_t y, const char* s) {
  u8g2.setFont(u8g2_font_9x15B_mf);  // fette Schrift
  u8g2.drawStr(x, y, s);
  u8g2.setFont(u8g2_font_9x15_mf);
}

// volatile: read by the Core-0 display task; written by the Core-1 button
// handler (showDebug, inhibitStatusDraw) or drawDebugInternal itself
// (inhibitStatusDraw). Single-byte aligned reads/writes are atomic on
// ESP32; volatile keeps the compiler from caching them in registers
// across the task loop body.
volatile bool showDebug = false;
volatile bool inhibitStatusDraw = false;

// drawStatusInternal: must only be called from the display task on Core 0.
// Use requestDisplayRedraw() from Core 1 (callbacks, loop, button handler)
// to schedule a redraw via the FreeRTOS notification.
void drawStatusInternal() {
  if (inhibitStatusDraw) return;

  char lineTX[16], lineRX[16];
  snprintf(lineTX, sizeof(lineTX), "TX:%u", rl.txCount);
  snprintf(lineRX, sizeof(lineRX), "RX:%u", rl.rxCountFiltered);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_9x15_mf);     // ~15px Zeilenhöhe -> 2 Zeilen passen in 32px

  // y-Koordinaten sind Baselines
  if (rl.rfMode == RaceLinkTransport::Mode::Tx) {
    drawStringBold(0, 14, lineTX);
    u8g2.drawStr(0, 31, lineRX);
  } else if (rl.rfMode == RaceLinkTransport::Mode::Rx) {
    u8g2.drawStr(0, 14, lineTX);
    drawStringBold(0, 31, lineRX);
  } else {
    u8g2.drawStr(0, 14, lineTX);
    u8g2.drawStr(0, 31, lineRX);
  }
  u8g2.sendBuffer();
}

// dispbuffer + dispLen are written by Core 1 inside requestDebugRedraw()
// (under g_displayMux) and read by Core 0 inside drawDebugInternal()
// (also under g_displayMux). The mux ensures the display task always
// sees a consistent (length, bytes) pair even if a new RX packet
// arrives mid-render.
uint8_t  dispbuffer[32];
uint8_t  dispLen = 0;

// 7 Bytes pro Zeile -> 21 Zeichen ("FF FF FF FF FF FF FF")
static const char HEXLUT[] = "0123456789ABCDEF";

// portMUX spinlock guarding dispbuffer/dispLen + the dirty flags. Defined
// here (above drawDebugInternal which uses it) so we don't need a
// forward declaration. The Core-0 display task globals (g_displayTask,
// g_displayDirty, g_debugDirty, DISPLAY_MIN_INTERVAL_TICKS) live in the
// same block right after this — they share the same lifetime + scope.
static portMUX_TYPE  g_displayMux  = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t  g_displayTask = nullptr;
static volatile bool g_displayDirty = false;
static volatile bool g_debugDirty   = false;

// 30 Hz rate cap for the display task. Notifications arriving inside the
// window are coalesced; dirty flags are preserved so the next render
// reflects the latest state.
static const TickType_t DISPLAY_MIN_INTERVAL_TICKS = pdMS_TO_TICKS(33);

// drawDebugInternal: must only be called from the display task on Core 0.
// Reads the already-snapshotted dispbuffer / dispLen under g_displayMux.
// Pre-offload this took a (buf, len) arg and copied inside; that copy now
// happens in requestDebugRedraw() on the Core 1 caller side, so the
// renderer is decoupled from the ephemeral RX-packet pointer lifetime.
void drawDebugInternal() {
  if (!showDebug) return;
  inhibitStatusDraw = true;                   // falls du die Statusanzeige temporär unterdrücken willst

  // Snapshot length + bytes into a stack-local copy under the mux so the
  // I²C send doesn't hold the lock during the slow OLED transfer.
  uint8_t local_len;
  uint8_t local_buf[sizeof(dispbuffer)];
  portENTER_CRITICAL(&g_displayMux);
  local_len = dispLen;
  if (local_len > sizeof(local_buf)) local_len = sizeof(local_buf);
  memcpy(local_buf, dispbuffer, local_len);
  portEXIT_CRITICAL(&g_displayMux);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_mf);

  // Kopfzeile: Länge (currently disabled visually, kept for future)
  char hdr[22];
  snprintf(hdr, sizeof(hdr), "LAST %uB", local_len);
  //u8g2.drawStr(0, 10, hdr);

  const uint8_t perLine = 4;                  // passt in die Breite (21 Zeichen)
  char line[22];                               // max 21 chars + \0
  uint8_t y = 10; //22

  for (uint8_t i = 0; i < 3 && i * perLine < local_len; ++i) {
    const uint8_t start = i * perLine;
    const uint8_t count = (local_len - start > perLine) ? perLine : (local_len - start);

    char* p = line;
    for (uint8_t j = 0; j < count; ++j) {
      if (j) *p++ = ' ';
      uint8_t b = local_buf[start + j];
      *p++ = HEXLUT[b >> 4];
      *p++ = HEXLUT[b & 0x0F];
    }
    *p = '\0';
    u8g2.drawStr(0, y, line);
    y += 10;                                   // Zeilenhöhe der 6x10-Schrift
  }

  u8g2.sendBuffer();
}

// -------------------- Display task on Core 0 (PRO_CPU) --------------------
// Pre-offload the OLED render ran in the Arduino loop on Core 1 and blocked
// for ~7-10 ms per u8g2.sendBuffer() on the I²C bus. With drawStatus called
// from the radio callbacks this directly delayed back-to-back host TX
// processing by ~5 ms/pkt. The display now lives on Core 0; Core-1 callers
// signal it via xTaskNotifyGive (sub-µs), and the task renders at most
// every 33 ms (≈30 Hz cap).
//
// Single-task ownership rule (post-setup): u8g2 + Wire are not thread-safe.
// After xTaskCreatePinnedToCore returns in setup(), Core 1 must NEVER touch
// u8g2.* or Wire.* directly — only through requestDisplayRedraw() /
// requestDebugRedraw().
//
// (g_displayTask, g_displayDirty, g_debugDirty, g_displayMux,
//  DISPLAY_MIN_INTERVAL_TICKS are defined further up — right above
//  drawDebugInternal, which uses g_displayMux. Defining them once there
//  avoids a forward-declaration / redefinition conflict.)

static void displayTaskFn(void*) {
  TickType_t lastDrawTick = xTaskGetTickCount();

  // First render so the operator sees the seeded IDLE status without
  // waiting for the first state-change notification.
  drawStatusInternal();
  lastDrawTick = xTaskGetTickCount();

  for (;;) {
    // Block forever until a Core-1 caller signals via xTaskNotifyGive.
    // pdTRUE clears the notification count to zero on receipt — the
    // dirty flags carry the "what to draw" information.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Rate cap: ensure at least 33 ms elapsed since the last render.
    // Notifications arriving during this delay are coalesced (binary
    // semaphore semantic) and dirty flags are preserved, so the next
    // render still reflects the latest state.
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed = now - lastDrawTick;
    if (elapsed < DISPLAY_MIN_INTERVAL_TICKS) {
      vTaskDelay(DISPLAY_MIN_INTERVAL_TICKS - elapsed);
    }

    bool needStatus, needDebug;
    portENTER_CRITICAL(&g_displayMux);
    needStatus = g_displayDirty; g_displayDirty = false;
    needDebug  = g_debugDirty;   g_debugDirty   = false;
    portEXIT_CRITICAL(&g_displayMux);

    // Render order: debug first (sets inhibitStatusDraw = true so a
    // pending status-redraw becomes a no-op until the user toggles
    // showDebug off via the button). drawDebugInternal() bails out
    // immediately when showDebug is false, so the call is cheap.
    if (needDebug)  drawDebugInternal();
    if (needStatus) drawStatusInternal();

    lastDrawTick = xTaskGetTickCount();
  }
}

// Core-1 callers schedule a status redraw. Sub-µs cost; never blocks.
// Safe to call from any non-ISR Core-1 context (callbacks, loop, button
// handler). Calling before the task is created (early in setup) is
// silently dropped; the task's first render in setup() covers boot state.
static inline void requestDisplayRedraw() {
  portENTER_CRITICAL(&g_displayMux);
  g_displayDirty = true;
  portEXIT_CRITICAL(&g_displayMux);
  if (g_displayTask) xTaskNotifyGive(g_displayTask);
}

// Core-1 callers schedule a debug-bytes redraw. Snapshots the byte
// buffer into the shared dispbuffer under g_displayMux so the display
// task always reads a consistent (length, bytes) pair even if a new
// RX packet lands mid-render.
static inline void requestDebugRedraw(const uint8_t* buf, uint8_t len) {
  portENTER_CRITICAL(&g_displayMux);
  uint8_t copyLen = (len > sizeof(dispbuffer)) ? sizeof(dispbuffer) : len;
  memcpy(dispbuffer, buf, copyLen);
  dispLen = copyLen;
  g_debugDirty = true;
  portEXIT_CRITICAL(&g_displayMux);
  if (g_displayTask) xTaskNotifyGive(g_displayTask);
}

/************ RadioLib: Modul + ISR-Flags ************/
SX1262 radio = SX1262(new Module(
  RACELINK_CS, RACELINK_DIO1, RACELINK_RST, RACELINK_BUSY,
  SPI, SPISettings(8000000, MSBFIRST, SPI_MODE0)
));

// Gateway-eigene DIO1-ISR: setzt zusätzlich zur (Bestand-)dio1Flag eine
// Task-Notification an rl-radio ab, damit der Radio-Task ohne 10 ms-Tick-
// Latenz auf TX-Done / RX-Pakete reagiert. Bewusst nicht in
// racelink_transport_core.h verdrahtet — der Header wird mit der WLED-Node-
// Firmware geteilt und bleibt unverändert. Statt RaceLinkTransport::attachDio1
// ruft transport_init() radio.setDio1Action(gateway_dio1_isr) direkt auf.
static void IRAM_ATTR gateway_dio1_isr() {
  rl.dio1Flag = true;
  BaseType_t hpw = pdFALSE;
  if (g_radioTask) vTaskNotifyGiveFromISR(g_radioTask, &hpw);
  portYIELD_FROM_ISR(hpw);
}


/************ SYNC (Master-side) ************/
// Master keeps a "dumb" timebase derived from millis() and broadcasts an
// autosync every 30 s while idle so nodes can correct drift. Autosync
// emits the legacy 4-byte SYNC body (no flags); arm-on-sync gating is
// now handled device-side via SYNC_FLAG_TRIGGER_ARMED on the host's
// deliberate-fire SYNC, so the gateway does NOT inhibit autosync
// during armed windows any more. The idle gate (``idleForAutoSync``)
// stays — autosync still mustn't preempt other in-flight TX work.
// SYNC frames are always broadcast and are queued without CAD/LBT and without jitter.

static constexpr uint8_t  GC_FLAG_ARM_ON_SYNC = (1u << 1);   // must match Node/Usermod

// After the SYNC_FLAG_TRIGGER_ARMED protocol change, the gateway no
// longer needs to inhibit autosync between an arm and the host's
// trigger SYNC: nodes now only materialise pending arm-on-sync state
// when the SYNC packet carries the trigger flag, and autosync
// deliberately leaves it 0. ``WAIT_HOST_TRIGGER`` is therefore commented
// out (rather than deleted) so the inhibit can be reintroduced quickly
// if a future protocol regression demands it; ``HOST_SYNC_RETRY`` stays
// — that's the unrelated retry path for when a host-initiated SYNC hits
// a busy TX slot.
enum class SyncState : uint8_t {
  AUTO_ALLOWED = 0,
  // WAIT_HOST_TRIGGER, // disabled: arm-on-sync is now gated device-side via SYNC_FLAG_TRIGGER_ARMED
  HOST_SYNC_RETRY      // host requested OPC_SYNC but it couldn't be queued yet (tx busy) -> retry ASAP
};

static SyncState syncState = SyncState::AUTO_ALLOWED;
static uint32_t      lastSyncQueuedMs = 0;   // when a sync was successfully queued (auto or host)
static uint8_t       hostSyncBri = 128;      // brightness of last host-triggered (pending) sync
// Captured SYNC_FLAG_* byte from the host's deliberate sync. The receive
// handler stashes it here so a HOST_SYNC_RETRY re-emission still carries the
// trigger bit; autosync paths leave it 0 (clock-tick only). Preserves the
// SYNC_FLAG_TRIGGER_ARMED gate end-to-end so nodes only fire armed effects
// on a deliberate-fire pulse.
static uint8_t       hostSyncFlags = 0;

// Queue a frame for immediate TX without CAD/LBT and without jitter.
// Used for SYNC pulses (auto + host-triggered). On rejection emits a typed
// NACK so the host's pending-send outcome registry can fail the matching
// _send_m2n call (or, for auto-sync, the host harmlessly drops the orphan
// NACK because no outcome slot is registered).
static bool rl_queueTxNoCad(RaceLinkTransport::Core& rl, const uint8_t* buf, uint8_t len, uint8_t type_full) {
  if (!buf || !len) {
    usb_send_tx_rejected(type_full, RaceLinkProto::TX_REJECT_ZEROLEN);
    return false;
  }
  if (rl.txPending) {
    usb_send_tx_rejected(type_full, RaceLinkProto::TX_REJECT_TXPENDING);
    return false;
  }

  const bool prevLbt = rl.lbtEnable;
  rl.lbtEnable = false;
  const bool ok = RaceLinkTransport::scheduleSend(rl, buf, len, 0 /*no jitter*/);
  rl.lbtEnable = prevLbt;
  if (!ok) {
    usb_send_tx_rejected(type_full, RaceLinkProto::TX_REJECT_UNKNOWN);
  }
  return ok;
}

// Build + queue a global SYNC frame (broadcast) using the current millis()-based timebase.
// ``flags`` is the SYNC_FLAG_* byte to pass through to nodes. Host-triggered
// sync uses the host's flags (so SYNC_FLAG_TRIGGER_ARMED reaches the nodes
// and materialises pending arm-on-sync state); autosync passes 0 so its
// pulse is clock-tick only and cannot fire armed effects.
//
// Wire-format split: emit the legacy 4-byte body (no flags byte) when
// flags == 0, 5-byte body when a flag is set. Old node firmware has
// ``req_len = SZ<P_Sync>() = 4`` (strict) and rejects any body that
// isn't exactly 4 bytes — so autosync MUST stay 4-byte to keep
// pre-trigger-flag fleets in sync. New firmware has ``req_len = 0``
// (variable) and accepts both 4 and 5 byte forms. RaceLinkProto::build
// writes the struct linearly (Header7 + ts24_0..2 + brightness +
// flags), so truncating by one byte cleanly drops only the trailing
// ``flags`` field — same legacy bytes pre-update host emit.
static bool sendSync(RaceLinkTransport::Core& rl, uint8_t brightness, uint8_t flags) {
  static const uint8_t BCAST3[3] = {0xFF, 0xFF, 0xFF};

  const uint32_t t = millis();

  RaceLinkProto::P_Sync p{};
  p.ts24_0 = (uint8_t)(t);
  p.ts24_1 = (uint8_t)(t >> 8);
  p.ts24_2 = (uint8_t)(t >> 16);
  p.brightness = brightness;
  p.flags = flags;

  uint8_t out[sizeof(RaceLinkProto::Header7) + sizeof(RaceLinkProto::P_Sync)];
  const uint8_t type_full = RaceLinkProto::make_type(RaceLinkProto::DIR_M2N, RaceLinkProto::OPC_SYNC);
  uint8_t n = RaceLinkProto::build(out, rl.myLast3, BCAST3, type_full, p);
  if (flags == 0) {
    // Drop the trailing flags byte → 4-byte legacy body. Compatible
    // with both old (req_len=4 strict) and new (req_len=0 variable)
    // node firmware.
    n -= 1;
  }

  // no CAD/LBT, no jitter; emits NACK on rejection for host-side correlation
  return rl_queueTxNoCad(rl, out, n, type_full);
}

static bool transportInDefaultIdleState(const RaceLinkTransport::Core& rl) {
  if (rl.changeMode) return false;
  if (rl.reqRxKind != rl.defaultRxKind) return false;
  if (rl.reqRxMs != rl.defaultRxMs) return false;

  switch (rl.defaultRxKind) {
    case RaceLinkTransport::RxKind::None:
      return rl.rfMode == RaceLinkTransport::Mode::Idle;
    case RaceLinkTransport::RxKind::Continuous:
      return rl.rfMode == RaceLinkTransport::Mode::Rx &&
             rl.rxKind == RaceLinkTransport::RxKind::Continuous;
    case RaceLinkTransport::RxKind::Timed:
    default:
      return false;
  }
}

// Auto-SYNC should only be sent when nothing else is happening.
// Pre-refactor this checked newSerialData + Serial.available() to gate against
// in-flight host work on Core 1. Post-refactor rl-usb-rx (Core 0) drains
// Serial into g_hostCmdQ, so a non-empty queue is the right idle gate.
// Serial.available() can't be reliably read from rl-radio's core; the queue
// check covers all host-side intent without cross-core peripheral access.
static bool idleForAutoSync(const RaceLinkTransport::Core& rl) {
  if (g_hostCmdQ && uxQueueMessagesWaiting(g_hostCmdQ) > 0) return false;

  if (rl.txPending) return false;
  return transportInDefaultIdleState(rl);
}

static void sync_service(RaceLinkTransport::Core& rl) {
  const uint32_t now = millis();

  // 1) Host-triggered SYNC retry: send ASAP once TX becomes available.
  // Uses the captured hostSyncFlags so SYNC_FLAG_TRIGGER_ARMED still reaches
  // nodes after a TX-busy retry.
  if (syncState == SyncState::HOST_SYNC_RETRY) {
    if (!rl.txPending) {
      if (sendSync(rl, hostSyncBri, hostSyncFlags)) {
        lastSyncQueuedMs = now;
        syncState = SyncState::AUTO_ALLOWED;
      }
    }
    return; // never auto-sync while host sync is pending
  }

  // 2) (disabled) After CONTROL(ARM): block auto-sync until host triggers one SYNC.
  // Removed when SYNC_FLAG_TRIGGER_ARMED was introduced: nodes now only
  // fire armed effects on a SYNC that carries the trigger flag, and
  // autosync deliberately leaves it 0 — so this inhibit is no longer
  // needed. The idle gate below is still load-bearing (don't autosync
  // mid-TX or while USB work is queued); only the arm-aware gate goes.
  // if (syncState == SyncState::WAIT_HOST_TRIGGER) {
  //   return;
  // }

  // 3) Auto-SYNC: only when idle and at most every 30s. flags=0 = clock tick
  // only, never fires armed effects.
  if (!idleForAutoSync(rl)) return;
  if (lastSyncQueuedMs != 0 && (uint32_t)(now - lastSyncQueuedMs) < 30000UL) return;

  if (sendSync(rl, hostSyncBri, /*flags=*/0)) {
    lastSyncQueuedMs = now;
  }
}

/************ Host-Kommandos (1:1) ************/
// Pre-refactor this read the global receivedBytes / newSerialData pair set by
// the Arduino loop()'s recvSerialBytes() pass. Now rl-radio drains g_hostCmdQ
// and calls handleCommand(frame, len) directly — the globals are gone.
// `frame` layout: [0]=LEN (= TYPE+DATA byte count), [1]=TYPE, [2..]=DATA.
// `len` is total bytes (= LEN + 1).
void handleCommand(const uint8_t* frame, uint8_t len) {
  if (len < 2) return;

  const uint8_t payloadLen = frame[0];
  if (payloadLen < 1) return;
  if ((uint16_t)payloadLen + 1u > (uint16_t)len) return;
  const uint8_t firstByte = frame[1];

  // IDENTIFY (for port discovery). Reply is raw text (DEV_TYPE_STR + MAC) —
  // not a framed event — so it goes through usb_tx_enqueue_raw rather than
  // usb_send_frame. The host parser keys on the SOF (0x00) of subsequent
  // events to resync, so a leading non-framed string is fine.
  if (payloadLen == 1 && firstByte == RaceLinkProto::GW_CMD_IDENTIFY) {
    char macstr[18];
    RaceLinkTransport::mac6ToStr(rl.myMac6, macstr);
    uint8_t out[40];
    size_t n = 0;
    const char* dev = DEV_TYPE_STR;
    while (*dev && n < sizeof(out)) out[n++] = (uint8_t)*dev++;
    for (size_t i = 0; i < 17 && n < sizeof(out); ++i) out[n++] = (uint8_t)macstr[i];
    usb_tx_enqueue_raw(out, (uint8_t)n);
    return;
  }

  // STATE_REQUEST: host asks for the current gateway state (Batch B). Replies
  // with EV_STATE_REPORT(state_byte, [metadata]). Used at startup, after USB
  // reconnect, and from the master-pill ↻ refresh button.
  if (payloadLen == 1 && firstByte == RaceLinkProto::GW_CMD_STATE_REQUEST) {
    emit_state_report();
    return;
  }

  // New framing: TYPE_FULL + recv3 + body...
  if ((firstByte & 0x80) == RaceLinkProto::DIR_M2N && payloadLen >= 4) {
    const uint8_t type_full = firstByte;
    const uint8_t recv3[3] = { frame[2], frame[3], frame[4] };
    const uint8_t bodyLen = (uint8_t)(payloadLen - 4);
    const uint8_t* body   = &frame[5];

    uint8_t out[32]; uint8_t n = 0;

    switch (RaceLinkProto::type_base(type_full)) {

      // Transport redesign (Plan Phase A): the dongle now stays in Continuous
      // RX (setDefaultRxContinuous in transport_init()). We no longer request
      // a Timed RX window after unicast/broadcast transmits -- the Core drops
      // back to Continuous RX automatically after TX, and the Host owns the
      // request-matching + timeout logic. This removes the double-clocking
      // that caused "No ACK_OK ... (timeout)" warnings on the Host even when
      // the node's ACK had been received and forwarded promptly.

      case RaceLinkProto::OPC_DEVICES: {
        if (bodyLen == sizeof(RaceLinkProto::P_GetDevices)) {
          RaceLinkProto::P_GetDevices p{};
          memcpy(&p, body, sizeof(p));
          n = RaceLinkProto::build(out, rl.myLast3, recv3, type_full, p);
          try_schedule_or_nack(rl, out, n, type_full);
          requestDebugRedraw(out, n);
        }
      } break;

      case RaceLinkProto::OPC_SET_GROUP: {
        if (bodyLen == sizeof(RaceLinkProto::P_SetGroup)) {
          RaceLinkProto::P_SetGroup p{};
          memcpy(&p, body, sizeof(p));
          n = RaceLinkProto::build(out, rl.myLast3, recv3, type_full, p);
          try_schedule_or_nack(rl, out, n, type_full);
          requestDebugRedraw(out, n);
        }
      } break;

      case RaceLinkProto::OPC_STATUS: {
        if (bodyLen == sizeof(RaceLinkProto::P_GetStatus)) {
          RaceLinkProto::P_GetStatus p{};
          memcpy(&p, body, sizeof(p));
          n = RaceLinkProto::build(out, rl.myLast3, recv3, type_full, p);
          try_schedule_or_nack(rl, out, n, type_full);
          requestDebugRedraw(out, n);
        }
      } break;

      case RaceLinkProto::OPC_PRESET: {
        if (bodyLen == sizeof(RaceLinkProto::P_Preset)) {
          RaceLinkProto::P_Preset p{};
          memcpy(&p, body, sizeof(p));

          // remember brightness for auto-sync (host may choose to ignore it on the nodes)
          hostSyncBri = p.brightness;

          n = RaceLinkProto::build(out, rl.myLast3, recv3, type_full, p);
          bool ok = try_schedule_or_nack(rl, out, n, type_full);
          requestDebugRedraw(out, n);

          // (disabled) If this PRESET arms a sync-start, block auto-sync until host triggers OPC_SYNC.
          // Removed when SYNC_FLAG_TRIGGER_ARMED was introduced — see the
          // SyncState comment block. Kept commented out (rather than
          // deleted) so the inhibit can be re-enabled without recreating
          // it from scratch if a future protocol regression demands it.
          // if (ok && (p.flags & GC_FLAG_ARM_ON_SYNC)) {
          //   syncState = SyncState::WAIT_HOST_TRIGGER;
          // }
          (void)ok;
        }
      } break;

      case RaceLinkProto::OPC_CONFIG: {
        if (bodyLen == sizeof(RaceLinkProto::P_Config)) {
          RaceLinkProto::P_Config p{};
          memcpy(&p, body, sizeof(p));
          n = RaceLinkProto::build(out, rl.myLast3, recv3, type_full, p);
          try_schedule_or_nack(rl, out, n, type_full);
          requestDebugRedraw(out, n);
        }
      } break;

      case RaceLinkProto::OPC_GET_CONFIG: {
        // Read-back of an OPC_CONFIG-style option. Body is 1 byte (the
        // option to read). Reply (N2M, opcode 0x0A) reuses the P_Config
        // 5-byte body and is forwarded to USB by the standard transport
        // path (usb_forward_transport) without any opcode-specific code
        // here -- the gateway treats every N2M frame the same way.
        if (bodyLen == sizeof(RaceLinkProto::P_GetConfig)) {
          RaceLinkProto::P_GetConfig p{};
          memcpy(&p, body, sizeof(p));
          n = RaceLinkProto::build(out, rl.myLast3, recv3, type_full, p);
          try_schedule_or_nack(rl, out, n, type_full);
          requestDebugRedraw(out, n);
        }
      } break;

      case RaceLinkProto::OPC_SYNC: {
        // Host triggers a global SYNC NOW (broadcast). Body is variable: 4 B
        // legacy clock-tick form, or 5 B with a trailing SYNC_FLAG_* byte.
        // Bit 0 (SYNC_FLAG_TRIGGER_ARMED) gates pending arm-on-sync
        // materialisation device-side; we capture and pass it through so the
        // node sees the host's intent end-to-end.
        if (bodyLen >= 4 && bodyLen <= sizeof(RaceLinkProto::P_Sync)) {
          // brightness to use for this (possibly retried) host-triggered sync
          hostSyncBri = body[3];
          hostSyncFlags = (bodyLen >= 5) ? body[4] : 0;

          // Try to queue immediately (no CAD/LBT). If TX is busy, retry ASAP in sync_service().
          if (sendSync(rl, hostSyncBri, hostSyncFlags)) {
            lastSyncQueuedMs = millis();
            syncState = SyncState::AUTO_ALLOWED;   // host sync done -> auto sync allowed again (30s gate applies)
          } else {
            syncState = SyncState::HOST_SYNC_RETRY;
          }
        }
      } break;

      case RaceLinkProto::OPC_STREAM: {
        if (bodyLen > 0) {
          const bool isBroadcast = RaceLinkTransport::isBroadcast3(recv3);
          const uint16_t rxWindowMs = isBroadcast ? RX_WINDOW_BROADCAST_MS : RX_WINDOW_UNICAST_MS;
          const int8_t rxNumWanted = isBroadcast ? -1 : 1;
          // scheduleStreamSend takes the streaming setup or refuses it because
          // a stream is already active or the slot is busy. Per-chunk sends
          // happen later in service() / queueNextStreamPacket. NACK only the
          // setup-rejection here — the host's outcome registry lines up with
          // the first chunk's EV_TX_DONE.
          bool ok = RaceLinkTransport::scheduleStreamSend(rl, body, bodyLen, rl.myLast3, recv3, type_full,
                                                          rxWindowMs, rxNumWanted);
          if (!ok) {
            uint8_t reason = rl.txPending ? RaceLinkProto::TX_REJECT_TXPENDING
                                          : RaceLinkProto::TX_REJECT_UNKNOWN;
            usb_send_tx_rejected(type_full, reason);
          }
        } else {
          usb_send_tx_rejected(type_full, RaceLinkProto::TX_REJECT_ZEROLEN);
        }
      } break;

      case RaceLinkProto::OPC_CONTROL: {
        // Variable-length body (3..MAX_P_CONTROL=21 B). Layout: see P_Control in racelink_proto.h.
        // build<T>() is not usable here (template needs fixed PayloadT); assemble header + raw body manually.
        if (bodyLen >= 3 && bodyLen <= RaceLinkProto::MAX_P_CONTROL) {
          RaceLinkProto::Header7* h = reinterpret_cast<RaceLinkProto::Header7*>(out);
          RaceLinkProto::put3(h->sender, rl.myLast3);
          RaceLinkProto::put3(h->receiver, recv3);
          h->type = type_full;
          memcpy(out + sizeof(RaceLinkProto::Header7), body, bodyLen);
          n = (uint8_t)(sizeof(RaceLinkProto::Header7) + bodyLen);
          try_schedule_or_nack(rl, out, n, type_full);
          requestDebugRedraw(out, n);
        }
      } break;

      case RaceLinkProto::OPC_OFFSET: {
        // Variable-length offset config (2..7 B body). First two bytes are
        // groupId + mode; the rest depends on mode (see OffsetMode enum).
        // Forwarded transparently; the gateway does not interpret the body.
        if (bodyLen >= 2 && bodyLen <= RaceLinkProto::MAX_P_OFFSET) {
          RaceLinkProto::Header7* h = reinterpret_cast<RaceLinkProto::Header7*>(out);
          RaceLinkProto::put3(h->sender, rl.myLast3);
          RaceLinkProto::put3(h->receiver, recv3);
          h->type = type_full;
          memcpy(out + sizeof(RaceLinkProto::Header7), body, bodyLen);
          n = (uint8_t)(sizeof(RaceLinkProto::Header7) + bodyLen);
          try_schedule_or_nack(rl, out, n, type_full);
          requestDebugRedraw(out, n);
        }
      } break;

    }
    return;
  }

  // Fallback: ignore
}

/************ RaceLink transport to USB forwarding ************/
static inline void usb_forward_transport(const uint8_t* pkt, uint8_t len, int16_t rssi, int8_t snr) {
  if (len < sizeof(RaceLinkProto::Header7)) return;
  
  // TODO header filtern -> nun in racelink_transport_core.h
  //if (!RaceLinkTransport::receiverMatches(pkt.h.receiver3, myLast3)) return;  // broadcast ODER exakt meine 3B

  const uint8_t type = pkt[sizeof(RaceLinkProto::Header7)-1]; // header.type
  uint8_t tmp[35];
  if (len > 32) len = 32;
  memcpy(tmp, pkt, len);
  uint16_t r = (uint16_t)rssi;
  tmp[len+0] = (uint8_t)(r & 0xFF);
  tmp[len+1] = (uint8_t)((r >> 8) & 0xFF);
  tmp[len+2] = (uint8_t)snr;
  usb_send_frame(type, tmp, (uint8_t)(len+3));
}

/************ RaceLink transport init (RadioLib) ************/
void transport_init() {
  SPI.begin(RACELINK_SCK, RACELINK_MISO, RACELINK_MOSI, RACELINK_CS);

  RaceLinkTransport::PhyCfg phy;
  phy.freqMHz   = (float)RACELINK_FREQ_HZ / 1e6f;
  phy.bwKHz     = RACELINK_BW_KHZ;
  phy.sf        = RACELINK_SF;
  phy.crDen     = RACELINK_CR_DEN;
  phy.syncWord  = RACELINK_SYNCWORD;
  phy.preamble  = RACELINK_PREAMBLE;
  phy.crcOn     = true;

  // Gerätespezifische Overrides (wie bisher im Master):
  phy.txPowerDbm   = RACELINK_TX_POWER;     // z.B. 14
  phy.dio2RfSwitch = 1;                   // vorher: radio.setDio2AsRfSwitch(true);
  phy.rxBoost      = -1;                  // (lassen) oder 1/0 je nach Board

  if (!RaceLinkTransport::beginCommon(radio, rl, phy)) {
    // Fehlerhandling wie bisher
    radio = nullptr; // von node übernommen
  }

  //rl.radio = &radio; wird nun in der RaceLinkTransport::beginCommon gesetzt

  // LBT (Listen-Before-Talk) — adds a CAD scan + 50-300 ms backoff before
  // every TX. Enabled (true) by default in the Core struct (used by WLED
  // and other nodes); the gateway intentionally turns it OFF because it's
  // the only host-side TX-er and doesn't need spectrum-sharing politeness
  // with itself. Per-TX jitter for host commands is governed instead by
  // try_schedule_or_nack (jitterMaxMs=0 since the Batch B follow-up,
  // 2026-04-28); SYNC and stream chunks already pass jitterMaxMs=0
  // explicitly via rl_queueTxNoCad / queueNextStreamPacket.
  //
  // GOTCHA for future maintainers: scheduleSend's LBT branch hard-overwrites
  // `jitterMaxMs` to 300 ms (transport_core.h line 318) — that hidden cap
  // is inside the LBT branch, not next to LBT's CAD-scan logic. So toggling
  // lbtEnable here ALSO changes the effective jitter cap, not just CAD.
  // This is the reason "enable LBT" used to look like a fix for the
  // ~1 s host-command latency: the cap was masking the 50-2500 ms jitter
  // default that try_schedule_or_nack used to inherit.
  rl.lbtEnable = false;   // gateway-side default, distinct from the Core struct's default of true

  // DIO1-ISR wird NICHT hier verdrahtet — setup() ruft radio.setDio1Action(
  // gateway_dio1_isr) erst NACH dem Spawnen der Tasks auf, damit die ISR ab
  // dem ersten Edge eine gültige g_radioTask-Handle vorfindet. Bewusst NICHT
  // RaceLinkTransport::attachDio1(): die Default-Trampoline würde nur
  // rl.dio1Flag setzen, ohne den Task zu notifizieren — das wäre der alte
  // Poll-Pfad. Unsere ISR macht beides.
  RaceLinkTransport::setDefaultRxContinuous(rl);
}

// --- benannte Callbacks (Master) ---
// State machine emit policy (Batch B):
//   * on_tx_start_cb  -> setGatewayState(TX). Pairs with EV_TX_DONE which the
//                        host's _send_m2n waits on as the outcome event. The
//                        STATE_CHANGED(TX) is for pill rendering only.
//   * on_tx_done_cb   -> emits EV_TX_DONE (the outcome event). State stays TX
//                        until the next on_rx_open_cb fires (which sets IDLE
//                        under setDefaultRxContinuous, the gateway's mode).
//   * on_rx_open_cb   -> ms == 0 means "entering continuous RX" = IDLE under
//                        setDefaultRxContinuous (the gateway's resting state).
//                        ms > 0 means a bounded RX_WINDOW; min_ms is metadata.
//   * on_rx_closed_cb -> no state emit; the next on_rx_open_cb covers it.
//                        (timed RX → continuous RX → IDLE happens within the
//                        same service() loop pass.)
//   * on_idle_cb      -> setGatewayState(IDLE). Only fires under
//                        setDefaultRxNone, which the gateway doesn't currently
//                        use. Wired up so a future mode switch needs no edit.
static void on_tx_start_cb(void* ctx) {
  setGatewayState(RaceLinkProto::GW_STATE_TX);
  // Schedule a status redraw on Core 0; sub-µs cost in the RF hot path.
  // Pre-offload this called drawStatus() directly which blocked ~7-10 ms
  // on I²C — measurable overhead on multi-packet sends.
  requestDisplayRedraw();
}

static void on_tx_done_cb(void* ctx) {
  // EV_TX_DONE is the *outcome* event the host's _send_m2n waits on. The
  // matching state transition (TX → IDLE under setDefaultRxContinuous, or
  // TX → RX_WINDOW for unicast streams) is emitted by the next callback the
  // service loop fires (on_rx_open_cb). Keeping outcome and state separate
  // matches the v4 design: outcome and state are orthogonal events.
  usb_send_event_u8(RaceLinkProto::EV_TX_DONE, 1);
  // Status redraw scheduled — display task picks the latest rfMode value.
  requestDisplayRedraw();
}

static void on_rx_open_cb(uint16_t ms, void* ctx) {
  if (ms > 0) {
    setGatewayState(RaceLinkProto::GW_STATE_RX_WINDOW, ms, /*has_meta=*/true);
  } else {
    // Continuous RX = the gateway's resting state under setDefaultRxContinuous
    setGatewayState(RaceLinkProto::GW_STATE_IDLE);
  }
  requestDisplayRedraw();
}

static void on_rx_packet_cb(const uint8_t* pkt, uint8_t len, int16_t rssi, int8_t snr, void* ctx) {
  usb_forward_transport(pkt, len, rssi, snr);
  // Snapshot the bytes into the shared dispbuffer + signal the task.
  // RX-counter changes are visible too — schedule a status redraw.
  requestDisplayRedraw();
  requestDebugRedraw(pkt, len);
}

static void on_rx_closed_cb(uint16_t delta, void* ctx) {
  // No state emit — the subsequent on_rx_open_cb in the same service pass
  // sets the new state (continuous RX = IDLE, or another RX_WINDOW for a
  // chained operation). The delta byte is no longer surfaced; per-window
  // RX counts are visible via the per-packet forwarding path.
  requestDisplayRedraw();
}

static void on_idle_cb(void* ctx) {
  // Reachable only under setDefaultRxNone (not the current gateway mode). The
  // setGatewayState() dedup guards make this a no-op when the state is already
  // IDLE.
  setGatewayState(RaceLinkProto::GW_STATE_IDLE);
  requestDisplayRedraw();
}

/************ Task bodies (Batch C, 2026-05-10) ************/

// -------------------- rl-usb-tx (Core 0) --------------------
// Sole writer to `Serial`. Pre-refactor every event/forward call
// Serial.write() inline from radio-callback context on Core 1 — that could
// block ~10 ms on USB-CDC throughput per coalesced frame and stall the next
// DIO1 service. Now those callers enqueue 40-byte items into g_usbTxQ and
// this task drains them on Core 0, decoupled from the RF hot path.
//
// Disconnect handling: `if (Serial)` checks the USB-CDC DTR state. When the
// host detaches we silently drop draining items (the queue fills upstream
// and increments g_usbTxDropCount). On reattach we resume immediately —
// future events go through; we deliberately don't replay buffered items.
static void usbTxTaskFn(void*) {
  UsbTxItem it;
  for (;;) {
    if (xQueueReceive(g_usbTxQ, &it, portMAX_DELAY) == pdTRUE) {
      if (Serial) {
        Serial.write(it.buf, it.len);
      }
      // Bei nicht verbundenem Host: stillschweigend droppen.
    }
  }
}

// -------------------- Button polling helper (used by rl-usb-rx) --------------
// 1:1 the logic of the pre-refactor loop() button block (lines 1136-1171).
// Runs on Core 0 in the rl-usb-rx 5 ms tick. The ISR (isr_button) still fires
// on whichever core's GPIO peripheral edge-triggers — `digitalRead()` and
// `attachInterrupt()` are core-agnostic on ESP32-S3.
// showDebug/inhibitStatusDraw: pre-refactor both were written from Core 1 and
// read by the display task on Core 0. Now writes happen on Core 0 too
// (single-core single-writer for the toggle), strictly safer than before.
static void button_poll_step() {
  if (btnFallingFlag) {
    btnFallingFlag = false;
    pressStartMs = millis();
    longHandled = false;
  }
  if (pressStartMs != 0) {
    bool pressed = (digitalRead(BUTTON_PIN) == LOW);
    unsigned long now = millis();
    if (pressed && !longHandled && (now - pressStartMs) >= 1000) {
      longHandled = true;
      showDebug = !showDebug;
      if (!showDebug) {
        inhibitStatusDraw = false;
        requestDisplayRedraw();
      }
    }
    if (!pressed) {
      if (!longHandled && (now - pressStartMs) >= 30) {
        inhibitStatusDraw = false;
        requestDisplayRedraw();
      }
      pressStartMs = 0;
      longHandled  = false;
    }
  }
}

// -------------------- rl-usb-rx (Core 0) --------------------
// Drains incoming USB-CDC bytes into g_hostCmdQ and runs the button-poll
// step. Arduino HardwareSerial has no blocking-on-byte primitive, so we
// poll at a 1 ms cadence (= FreeRTOS tick). Pre-refactor loop() polled
// every iteration with no sleep — effectively continuous. A 1 ms tick is
// close enough that host-side latency regressions are <1 ms (vs. the
// initial 5 ms tick which added ~2.5 ms avg to every host command).
// Button thresholds (30 ms short / 1000 ms long) are well above 1 ms.
// CPU cost: ~1000 wakes/s × ~µs each = negligible on Core 0.
static void usbRxTaskFn(void*) {
  for (;;) {
    recvSerialBytes();        // pushes complete frames + notifies rl-radio
    button_poll_step();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// -------------------- rl-radio (Core 1) --------------------
// Sole owner of `rl`, the SX1262 SPI bus, and all radio state. Woken by
// (a) the DIO1 ISR notify on TX-done / RX-packet-ready, and (b) rl-usb-rx
// when it pushes a host frame onto g_hostCmdQ. The 10 ms timeout drives
// time-gated state transitions that have no IRQ source — LBT backoff
// (`earliestTxAtMs`), RX-window timeouts (`rxWindowEndMs`), and the
// auto-SYNC 30 s gate.
//
// **Inner service loop (Batch C latency fix, 2026-05-10):**
// RaceLinkTransport::service() returns early after every internal state
// transition (Rx→Idle, Idle→Tx, Tx→standby→transmit→Idle, Idle→Rx).
// Pre-refactor Arduino loop() pumped these back-to-back within microseconds
// because loop() had no sleep. Calling service() exactly once per wake
// here added a 10 ms notification-wait between every transition,
// accumulating 20–30 ms per host-command round-trip (host observed
// ~20 ms regression). Loop service() while `rl.changeMode` says more
// transitions are pending; service() is cheap (a few register reads)
// when nothing is happening, and rl.changeMode is the natural fixed-point
// signal from the state machine itself. The cap of 8 is well above the
// longest legitimate chain (Rx→Idle→Tx→standby→transmit→Idle→Rx ≈ 6).
static void radioTaskFn(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

    // 1) Hostkommandos drainieren. Mehrere Frames pro Wake OK —
    //    handleCommand ist günstig; scheduleSend serialisiert sich selbst
    //    über rl.txPending. Vor service(), damit ein gerade angekommener
    //    Befehl in dieser Wake gleich die State-Machine treibt.
    HostCmdItem cmd;
    while (xQueueReceive(g_hostCmdQ, &cmd, 0) == pdTRUE) {
      handleCommand(cmd.buf, cmd.len);
    }

    // 2) Radio state machine bis zum Fixpunkt pumpen (siehe oben).
    for (int i = 0; i < 8; ++i) {
      RaceLinkTransport::service(rl, cb);
      if (!rl.changeMode) break;
    }

    // 3) Auto-SYNC: 30 s-Tor, gated über idleForAutoSync(rl).
    sync_service(rl);
  }
}

/************ Setup/Loop ************/
void setup() {
  Serial.begin(BAUDRATE);
  delay(50);

  // Queues VOR jedem Aufruf von usb_send_frame/setGatewayState anlegen, sonst
  // gehen frühe Boot-Events (z. B. das initiale EV_STATE_CHANGED(IDLE)) ins
  // Leere. Consumer-Task wird später gespawnt; bis dahin füllt sich die Queue
  // unkritisch auf — 16 Slots reichen für jede plausible Boot-Sequenz.
  g_usbTxQ   = xQueueCreate(16, sizeof(UsbTxItem));
  g_hostCmdQ = xQueueCreate(4,  sizeof(HostCmdItem));

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

  // kann weg, da in racelink_transport_core.h beginCommon integriert
/*   // MAC lesen
  if (RaceLinkTransport::readEfuseMac6(myMac6)) {
    macReadOK = true;
    RaceLinkTransport::last3FromMac6(myLast3, myMac6);
  } else {
    macReadOK = false;
    memset(myLast3, 0, 3);
  } */

  // RaceLink transport
  transport_init();
  cb.onTxStart       = on_tx_start_cb;
  cb.onTxDone        = on_tx_done_cb;
  cb.onRxWindowOpen  = on_rx_open_cb;
  cb.onRxPacket      = on_rx_packet_cb;
  cb.onRxWindowClosed= on_rx_closed_cb;
  cb.onIdle          = on_idle_cb;
  cb.ctx             = nullptr;       // oder &deinContext

  // Startanzeige — runs on Core 1 BEFORE the display task is created,
  // so direct u8g2 calls are safe here (no other task touches u8g2 yet).
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "RaceLink Init...");
  u8g2.sendBuffer();
  delay(150);
  drawStatusInternal();

  // Seed the gateway state machine. Under setDefaultRxContinuous (the gateway's
  // mode) the resting state is IDLE = "in continuous RX, ready". This emits the
  // first EV_STATE_CHANGED(IDLE) so a host that connects after boot sees a
  // valid pill state without waiting for the next service() pass. A host that
  // missed this initial event re-syncs via GW_CMD_STATE_REQUEST anyway.
  setGatewayState(RaceLinkProto::GW_STATE_IDLE);

  // Hand display ownership to a Core-0 FreeRTOS task. After this point the
  // Arduino loop on Core 1 must NEVER touch u8g2.* or Wire.* directly —
  // only through requestDisplayRedraw() / requestDebugRedraw(), which
  // signal the task via xTaskNotifyGive (sub-µs, no I²C blocking on the
  // RF/USB hot path).
  //
  // Stack: 4 KB is generous for u8g2's draw buffers + snprintf locals.
  // Priority: tskIDLE_PRIORITY + 1 — low; cooperates with the FreeRTOS
  // idle task on Core 0.
  // Pinned to Core 0 (PRO_CPU): WiFi/BLE are not enabled in this firmware
  // so Core 0 is otherwise idle.
  xTaskCreatePinnedToCore(
    displayTaskFn,           // task function
    "rl-display",            // name (visible in `vTaskList` debug output)
    4096,                    // stack bytes
    nullptr,                 // params
    tskIDLE_PRIORITY + 1,    // priority (low)
    &g_displayTask,          // handle (used by request* helpers)
    0                        // pin to Core 0 (PRO_CPU)
  );

  // -------------------- I/O tasks on Core 0 (PRO_CPU) --------------------
  // Priorität 4: bewusst eine Stufe unter TinyUSB (Prio 5 auf Arduino-ESP32),
  // damit der USB-Stack-Pumpe immer Vorrang hat. Stack 2 KB reicht — beide
  // Tasks haben minimale Lokal-Locals (HardwareSerial-Aufruftiefe ~600 B).
  xTaskCreatePinnedToCore(usbTxTaskFn, "rl-usb-tx", 2048, nullptr, 4, &g_usbTxTask, 0);
  xTaskCreatePinnedToCore(usbRxTaskFn, "rl-usb-rx", 2048, nullptr, 4, &g_usbRxTask, 0);

  // -------------------- Radio task on Core 1 (APP_CPU) -------------------
  // Hohe Priorität (configMAX_PRIORITIES-2 = 23 bei Arduino-ESP32-Default 25)
  // damit der RF-State-Machine-Tick deterministisch reagiert. Stack 8 KB
  // deckt RadioLib-Aufrufketten (scanChannel → transmit → SPItransfer) plus
  // handleCommand-Locals (out[32], P_*-Structs) mit Sicherheitsmarge ab.
  xTaskCreatePinnedToCore(radioTaskFn, "rl-radio", 8192, nullptr,
                          configMAX_PRIORITIES - 2, &g_radioTask, 1);

  // DIO1-ISR JETZT verdrahten — g_radioTask ist gesetzt, ab der ersten
  // RX/TX-Flanke notifiziert die ISR ohne Race. Bewusst nach den
  // Task-Creates und nicht in transport_init().
  radio.setDio1Action(gateway_dio1_isr);
}

// Arduino-Default-Loop ist nach Batch C inaktiv. recvSerialBytes / handleCommand
// / RaceLinkTransport::service / sync_service / Button-Polling laufen jetzt in
// dedizierten FreeRTOS-Tasks (rl-usb-rx, rl-radio). Den loopTask löschen wir
// NICHT — das Arduino-ESP32-Framework macht implizite Annahmen über dessen
// Existenz. Stattdessen schläft er für immer.
void loop() {
  vTaskDelay(portMAX_DELAY);
}
