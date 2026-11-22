// Pure policy for the BLE transport: no NimBLE or ESP-IDF headers, so this compiles with a plain
// host compiler like board_table.c does. cable_ble.c calls into this; this file calls nothing back.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum { BLE_DISCONNECT_CENTRAL, BLE_DISCONNECT_PERIPHERAL, BLE_DISCONNECT_LINK_LOSS } ble_disconnect_reason_t;

// Always true today (we always want to be connectable again) — a real function, not a stub,
// because a later firmware revision (e.g. a deep-sleep mode) is exactly the kind of change that
// would make this conditional, and the decision belongs here, not scattered at every call site.
bool ble_policy_should_readvertise(ble_disconnect_reason_t reason);

typedef enum { BOND_NONE, BOND_VALID, BOND_STALE } ble_bond_state_t;

// peer_has_bond: true if the dial's own bond store has a record for this peer's address.
// peer_claims_bonded: true if the peer's connection attempt asserts it already has a bond (i.e.
// it skipped pairing). Mirrors the bridge-side plan's stale-bond detection: a peer that claims a
// bond the dial doesn't have (or vice versa) is BOND_STALE, not BOND_NONE.
ble_bond_state_t ble_policy_classify_bond(bool peer_has_bond, bool peer_claims_bonded);

// A small, dependency-free function so this task's test doesn't need cable_client.c's full
// include graph: given a raw power_battery_pct() reading, return what belongs on the wire.
// raw_pct < 0 means unknown (power.h's own convention) — has_pct is false in that case and pct
// is meaningless.
typedef struct { bool has_pct; int pct; bool charging; } ble_battery_wire_t;
ble_battery_wire_t ble_battery_for_wire(int raw_pct, bool charging);

// Whether a BLE link should become the dial's active transport now. The dial's protocol is
// dial-speaks-first (hello), and replies go out on whichever transport is active, so a BLE link
// must be made active explicitly — nothing arrives over it until the dial has said hello there.
// trusted: passkey-authenticated and encrypted. subscribed: the central enabled TX notifications.
// us_since_usb_rx: microseconds since USB last delivered a frame, or < 0 if it never has; a USB
// session heard from within the last 15 s (the dial's own silence timeout) keeps the link.
bool ble_policy_should_activate(bool trusted, bool subscribed, int64_t us_since_usb_rx);

// How long cable_ble_send waits before retrying a notification that failed. NimBLE reports a full
// buffer pool as "no memory"; buffers free up as the radio drains, so that one is worth waiting
// for, briefly. Returns the wait in ms, or -1 to give up (the send then reports "not connected").
// attempt counts from 0.
int ble_policy_notify_retry_ms(bool out_of_buffers, int attempt);

// True iff work started for connection generation `started_gen` may continue: that connection is
// still the current one (generation unchanged; 0 means "never connected"), trusted and subscribed.
// cable_ble.c bumps the generation on every connect and disconnect, so a stale activation or the
// tail of a frame can never reach a later central, even one NimBLE gave the same handle number.
bool ble_policy_link_still_current(unsigned started_gen, unsigned current_gen, bool trusted, bool subscribed);
