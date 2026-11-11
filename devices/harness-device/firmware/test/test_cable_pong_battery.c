#include "../main/ble_policy.h"
#include <stdio.h>
static int fails;
#define CHECK(cond) do { if (!(cond)) { fails++; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); } } while (0)

int main(void)
{
    ble_battery_wire_t ok = ble_battery_for_wire(87, false);
    CHECK(ok.has_pct && ok.pct == 87 && !ok.charging);

    ble_battery_wire_t unknown = ble_battery_for_wire(-1, false);
    CHECK(!unknown.has_pct);

    printf("ble_battery_for_wire: %s\n", fails ? "FAILED" : "ok");
    return fails ? 1 : 0;
}
