// The single owner of the session callbacks (on_frame, session_tick) cable_client.c registers.
//
// cable_link.c's reader task and cable_ble.c's NimBLE event handling run on two different FreeRTOS
// tasks, but cable_client.c's session state (s_agents, s_machines, s_last_rx_us, and so on) has
// always been touched from exactly one task and has no lock — cable_link.h says so directly: "The
// decoder has one reader and no lock; another task must not reset it during a feed." Adding BLE as
// a second transport cannot mean a second task calling on_frame/session_tick directly.
//
// Neither transport calls the real handlers directly. Both are started with cable_session_io_feed
// and cable_session_io_tick as their cb/tick — these take a shared mutex and then call the real
// handler SYNCHRONOUSLY, on the calling task, exactly as cable_link.c's reader task always has.
// This is deliberately a mutex around a direct call, not a queue plus a dedicated consumer task:
// an earlier queue-based design capped the payload size to keep a bounded queue cheap, which
// silently broke USB firmware/pet-pack transfers (up to CABLE_MAX_PAYLOAD, routed through the same
// path) — see this plan's ledger for the finding. A mutex has no size to pick, drops nothing, and
// preserves the exact backpressure cable_link.c's reader task already depended on (a slow on_frame,
// such as an OTA flash write, blocks that task for precisely as long as it always has); a transport
// on a DIFFERENT task simply waits for the lock first. cable_link.c itself needed no change for
// this either way: its cb/tick parameters were always a generic callback, never hardcoded to call
// on_frame, so pointing them at this adapter instead of the final handler is a wiring change at the
// call site (cable_client_start), not a change to cable_link.c's own code.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cable_frame.h"
#include "cable_link.h" // cable_tick_cb

// Starts the session and creates its mutex. Call once, before cable_link_start/cable_ble_start —
// cable_session_io_feed/cable_session_io_tick below are what those two are given instead of `cb`/
// `tick` directly. Returns false if the mutex could not be created (no memory for the session —
// same "fails without crashing the boot" posture as cable_link_start).
bool cable_session_io_start(cable_frame_cb cb, cable_tick_cb tick, void *ctx);

// Matches cable_frame_cb's signature exactly so it can be passed anywhere a transport wants a
// frame callback. Takes the session mutex, then calls the real on_frame synchronously (the
// calling task blocks for exactly as long as on_frame takes — same as before BLE existed), then
// releases it.
void cable_session_io_feed(uint8_t version, uint8_t type, const uint8_t *payload, size_t payload_len, void *ctx);

// Matches cable_tick_cb's signature. Takes the session mutex, calls the real session_tick
// synchronously, releases it.
void cable_session_io_tick(void *ctx);

// Runs fn(arg) under the same session mutex as cable_session_io_feed/tick, so a transport can
// safely touch cable_client.c's session state from an event that isn't itself a frame or a tick —
// for example, clearing which transport is "active" when that transport disconnects.
void cable_session_io_run(void (*fn)(void *arg), void *arg);
