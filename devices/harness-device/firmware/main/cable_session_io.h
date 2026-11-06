// The single owner of the session callbacks (on_frame, session_tick) cable_client.c registers.
//
// cable_link.c's reader task and cable_ble.c's NimBLE event handling run on two different FreeRTOS
// tasks, but cable_client.c's session state (s_agents, s_machines, s_last_rx_us, and so on) has
// always been touched from exactly one task and has no lock — cable_link.h says so directly: "The
// decoder has one reader and no lock; another task must not reset it during a feed." Adding BLE as
// a second transport cannot mean a second task calling on_frame/session_tick directly.
//
// So neither transport calls the real handlers. Both are started with cable_session_io_feed and
// cable_session_io_tick as their cb/tick — these only enqueue — and this file owns the one task
// that drains the queue and calls the real on_frame/session_tick. cable_link.c itself needed no
// change for this: its cb/tick parameters were always a generic callback, never hardcoded to call
// on_frame, so pointing them at a queueing adapter instead of the final handler is a wiring change
// at the call site (cable_client_start), not a change to cable_link.c's own code.
//
// Deliberately out of scope: CABLE_TYPE_FW and CABLE_TYPE_PET (bulk transfers whose backpressure
// is tied to cable_link.c's USB-specific RX_BUF sizing — see cable_ble.c, which rejects both types
// before they ever reach this queue). Only cable_link.c's USB reader keeps feeding every frame type.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cable_frame.h"
#include "cable_link.h" // cable_tick_cb

// Starts the session task and prepares the queue. Call once, before cable_link_start/cable_ble_start
// — cable_session_io_feed/cable_session_io_tick below are what those two are given instead of `cb`/
// `tick` directly. Returns false if the queue or task could not be created (no memory for the
// session — same "fails without crashing the boot" posture as cable_link_start).
bool cable_session_io_start(cable_frame_cb cb, cable_tick_cb tick, void *ctx);

// Matches cable_frame_cb's signature exactly so it can be passed anywhere a transport wants a
// frame callback. Copies payload_len bytes (PSRAM-preferred) and enqueues them; the real on_frame
// runs later, on the session task, not on the calling task. A queue that is full, or a payload too
// large for this path (bulk-transfer frame types are rejected earlier, by the caller — see
// cable_ble.c), is logged and dropped, never blocks the caller.
void cable_session_io_feed(uint8_t version, uint8_t type, const uint8_t *payload, size_t payload_len, void *ctx);

// Matches cable_tick_cb's signature. A tick means "check for timeouts", not a value to preserve,
// so several requests before the session task next wakes collapse into a single real tick call.
void cable_session_io_tick(void *ctx);
