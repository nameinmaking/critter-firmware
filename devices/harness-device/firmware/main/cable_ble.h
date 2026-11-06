// The BLE transport under cable_frame: a NimBLE GATT service (one write characteristic for bytes
// in, one notify characteristic for bytes out) standing in for cable_link.c's USB port on machines
// that block USB serial access to the dial. See docs/ble.md for the GATT shape, pairing and
// stale-bond recovery, and docs/superpowers/specs/2026-11-05-dial-wireless-transport-design.md (in
// the companion critter-harness repo) for why this exists at all.
//
// Task 1 scope only: build enablement, advertising, and a working RX/TX path. Bonding is NOT yet
// enforced here — see cable_ble.c's own startup log line — that lands with ble_policy.c (Task 2/3).
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cable_frame.h"
#include "cable_link.h" // cable_tick_cb

// Starts NimBLE, registers the GATT service, and begins advertising. Call once, alongside
// cable_link_start (see cable_client_start) — `cb`/`tick` should be cable_session_io_feed/
// cable_session_io_tick, not the real session handlers directly (see cable_session_io.h); this
// file does not enforce that, it is a wiring contract documented at the call site.
//
// CABLE_TYPE_FW and CABLE_TYPE_PET are never forwarded to `cb`: BLE does not carry bulk transfers
// (firmware/pet-pack updates stay USB-only, physical-access-gated — see cable_session_io.h). Any
// such frame arriving over BLE is logged and dropped before `cb` ever sees it.
//
// Returns false if NimBLE failed to initialize (logged; the dial still works over USB).
bool cable_ble_start(cable_frame_cb cb, cable_tick_cb tick, void *ctx);

// Frame `payload` and notify it to the subscribed central. Same contract as cable_link_send: a
// false return means no central is connected and subscribed right now — a normal state for this
// transport, not an error. Callers should treat it as "not connected", never retry in a tight loop.
bool cable_ble_send(uint8_t type, const uint8_t *payload, size_t payload_len);
