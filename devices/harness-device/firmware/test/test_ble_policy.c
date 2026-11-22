#include "../main/ble_policy.h"
#include <stdio.h>
static int fails;
#define CHECK(cond) do { if (!(cond)) { fails++; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); } } while (0)

int main(void)
{
    CHECK(ble_policy_should_readvertise(BLE_DISCONNECT_CENTRAL));
    CHECK(ble_policy_should_readvertise(BLE_DISCONNECT_PERIPHERAL));
    CHECK(ble_policy_should_readvertise(BLE_DISCONNECT_LINK_LOSS));

    CHECK(ble_policy_classify_bond(true, true) == BOND_VALID);
    CHECK(ble_policy_classify_bond(false, false) == BOND_NONE);
    CHECK(ble_policy_classify_bond(false, true) == BOND_STALE);
    CHECK(ble_policy_classify_bond(true, false) == BOND_STALE);

    // BLE becomes the active link only once it is trusted AND subscribed, and never steals a USB
    // session that has spoken in the last 15 s (-1 = USB has never delivered a frame).
    CHECK(ble_policy_should_activate(true, true, -1));
    CHECK(ble_policy_should_activate(true, true, 15000001));
    CHECK(!ble_policy_should_activate(true, true, 5000000));
    CHECK(!ble_policy_should_activate(true, true, 15000000));
    CHECK(!ble_policy_should_activate(false, true, -1));
    CHECK(!ble_policy_should_activate(true, false, -1));

    // A notify that hit "out of buffers" waits 5 ms and retries, for about a second, then gives up;
    // any other failure gives up at once.
    CHECK(ble_policy_notify_retry_ms(true, 0) == 5);
    CHECK(ble_policy_notify_retry_ms(true, 199) == 5);
    CHECK(ble_policy_notify_retry_ms(true, 200) == -1);
    CHECK(ble_policy_notify_retry_ms(false, 0) == -1);

    // Work started for one connection (an activation, or the rest of a frame) only continues while
    // that same connection is still up, trusted and subscribed. A disconnect or a new connection
    // bumps the generation, even when NimBLE reuses the same handle number.
    CHECK(ble_policy_link_still_current(7, 7, true, true));
    CHECK(!ble_policy_link_still_current(7, 8, true, true));
    CHECK(!ble_policy_link_still_current(7, 9, true, true));
    CHECK(!ble_policy_link_still_current(7, 7, false, true));
    CHECK(!ble_policy_link_still_current(7, 7, true, false));
    CHECK(!ble_policy_link_still_current(0, 0, true, true)); // generation 0 = no connection yet

    printf("ble_policy: %s\n", fails ? "FAILED" : "ok");
    return fails ? 1 : 0;
}
