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
