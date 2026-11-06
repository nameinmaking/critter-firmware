// Pure policy for the BLE transport: no NimBLE or ESP-IDF headers, so this compiles with a plain
// host compiler like board_table.c does. cable_ble.c calls into this; this file calls nothing back.
#pragma once

#include <stdbool.h>

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
