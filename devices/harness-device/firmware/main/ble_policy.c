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

#define BLE_POLICY_USB_QUIET_US (15 * 1000 * 1000)

bool ble_policy_should_activate(bool trusted, bool subscribed, int64_t us_since_usb_rx)
{
    if (!trusted || !subscribed) return false;
    return us_since_usb_rx < 0 || us_since_usb_rx > BLE_POLICY_USB_QUIET_US;
}

#define BLE_POLICY_RETRY_WAIT_MS 5
#define BLE_POLICY_RETRY_ATTEMPTS 200 // ~1 s in total: long enough to drain, short enough to never wedge a sender

int ble_policy_notify_retry_ms(bool out_of_buffers, int attempt)
{
    if (!out_of_buffers || attempt >= BLE_POLICY_RETRY_ATTEMPTS) return -1;
    return BLE_POLICY_RETRY_WAIT_MS;
}
