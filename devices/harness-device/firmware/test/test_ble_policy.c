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

    printf("ble_policy: %s\n", fails ? "FAILED" : "ok");
    return fails ? 1 : 0;
}
