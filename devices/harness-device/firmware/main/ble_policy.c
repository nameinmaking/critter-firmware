#include "ble_policy.h"

bool ble_policy_should_readvertise(ble_disconnect_reason_t reason)
{
    (void)reason;
    return true;
}

ble_bond_state_t ble_policy_classify_bond(bool peer_has_bond, bool peer_claims_bonded)
{
    if (peer_has_bond && peer_claims_bonded) return BOND_VALID;
    if (!peer_has_bond && !peer_claims_bonded) return BOND_NONE;
    return BOND_STALE;
}

ble_battery_wire_t ble_battery_for_wire(int raw_pct, bool charging)
{
    if (raw_pct < 0) return (ble_battery_wire_t){ .has_pct = false, .pct = 0, .charging = charging };
    return (ble_battery_wire_t){ .has_pct = true, .pct = raw_pct, .charging = charging };
}
