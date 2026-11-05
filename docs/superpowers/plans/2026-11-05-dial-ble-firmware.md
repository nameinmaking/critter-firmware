# Dial BLE Transport — Firmware Side Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the dial a BLE peripheral role that speaks the existing cable protocol over a
Nordic-UART-shaped GATT service, with mandatory bonding and a battery field on `pong`, alongside
(not replacing) the existing USB transport.

**Architecture:** Split in two, mirroring this codebase's own existing split between
`cable_frame.c` (portable, no ESP-IDF headers, natively testable) and `cable_link.c` (the hardware
transport boundary, not natively tested): `ble_policy.c`/`.h` holds every *decision* this feature
makes (restart advertising? treat this peer as bonded, stale-bonded, or unknown?) as plain
functions with no NimBLE types in their signatures, tested the same way `test_board.c` tests
`board_from_acks` — compiled directly on the host, no ESP-IDF. `cable_ble.c`/`.h` holds the actual
NimBLE glue (GATT service registration, advertising, pairing callbacks) and calls into
`ble_policy.c` for every decision; it is exercised by real hardware, not by this plan's test suite,
same as `cable_link.c` today.

**Tech Stack:** ESP-IDF >= 5.5 (per this project's own `CMakeLists.txt` header comment), NimBLE —
IDF 5.x's standard BLE host stack, confirmed present in this exact version (NimBLE has been the
default BLE host since IDF 5.0; this project is already on 5.5, well past that). C11, the existing
`cc -fsanitize=undefined,bounds` host-test convention in `test/run.sh`.

**Spec:** `docs/superpowers/specs/2026-11-05-dial-wireless-transport-design.md` (critter-harness
repo — read it there; this repo does not have its own copy). The bridge-side plan implementing the
other half of this same spec is `docs/superpowers/plans/2026-11-05-dial-ble-bridge.md` in the
critter-harness repo.

## Global Constraints

- GATT shape, exact values, shared verbatim with the bridge-side plan — **do not change these
  without updating both plans**:
  - Service UUID: `af5786b4-caf1-426d-9c97-5e2615542a1a`
  - RX characteristic (central writes in): `93aa83a4-5e59-40dd-9780-a6798b8a876d`
  - TX characteristic (peripheral notifies out): `db369d45-0d6a-42ef-8785-4423fce86e90`
- The dial is BLE peripheral only, never central — it advertises and accepts one connection, it
  never scans or connects out.
- Bonding is mandatory before the RX/TX characteristics may be used — no "just works" pairing;
  use a passkey displayed on the dial's own screen (design doc's Security section).
- Advertising must resume after **every** kind of disconnect, including one the central initiates
  — a documented bug in a *different* ESP32 BLE stack (Arduino's) makes this worth testing for
  explicitly rather than assuming NimBLE just does the right thing.
- Battery fields on the wire, shared verbatim with the bridge-side plan: the existing `pong`
  message (built by `msg("pong")` in `main/cable_client.c:1401`) gains `battPct` (integer 0–100, or
  JSON `null` if `power_battery_pct()` returns -1) and `charging` (boolean, from
  `power_is_charging()`).
- USB stays exactly as it is — this plan adds BLE alongside it, never behind a flag that disables
  USB, and touches no line of `cable_link.c`.
- No task in this plan flashes or runs against the physical dial. `ble_policy.c`'s tests run
  natively on the host, same as `test_board.c`.

## Review Focus

- Advertising must resume after a disconnect the *central* initiates, not only one the peripheral
  initiates or a radio-level link loss — this is exactly the case the Arduino-BLE-stack bug (cited
  in the design doc's prior art) got wrong on a different stack.
- A connection from a peer with no bond at all (first-ever contact) must require pairing before
  either characteristic is usable — never silently allow unauthenticated reads/writes "just this
  once."
- A connection from a peer presenting a bond the dial no longer recognizes (dial was reflashed,
  bond storage cleared) must be classified distinctly from "a peer with no bond" — this is the
  stale-bond case the bridge-side plan also has to detect on its end; the two sides need compatible
  behavior (the dial should not treat a stale-bond peer as already trusted).
- `power_battery_pct()` returning -1 (no driver, read error) must produce a JSON `null`
  `battPct`, never a literal `-1` leaking onto the wire where a consumer might print "-1%".
  Battery fields must also not be attempted before `power_init()` has succeeded.
- The passkey displayed on-screen must not be dismissible or overwritten by an unrelated toast
  (an agent event, a voice error) arriving mid-pairing — the owner needs to actually read and use
  it.

---

### Task 1: NimBLE build enablement and a minimal advertising GATT service

**Files:**
- Modify: `sdkconfig.defaults.esp32s3` (add `CONFIG_BT_ENABLED=y`, `CONFIG_BT_NIMBLE_ENABLED=y`,
  and whichever further NimBLE-peripheral-only options the chosen component needs — consult
  NimBLE's own Kconfig help text for this exact IDF version rather than guessing values)
- Modify: `main/CMakeLists.txt:79-81` (add the BT component, e.g. `bt`, to `BOARD_REQUIRES`
  alongside the existing `esp_wifi` entry)
- Create: `main/cable_ble.c`, `main/cable_ble.h`
- Modify: `main/CMakeLists.txt`'s `BOARD_SRCS` list (add `cable_ble.c` next to `cable_link.c`)
- Modify: `main/app_main.c` (call this task's init function alongside the existing USB/cable init)

**Interfaces:**
- Produces:
  ```c
  // cable_ble.h
  // Starts advertising the GATT service below. Call once at boot, alongside cable_link_start().
  // Returns false if NimBLE failed to initialize (logged; the dial still works over USB).
  bool cable_ble_start(void);
  ```
  Internally, `cable_ble.c` registers one GATT service (the Service UUID above) with the RX
  (write) and TX (notify) characteristics (the RX/TX UUIDs above), wires incoming RX writes into
  the *same* frame-decoding path `cable_link.c` already uses for USB bytes (read
  `main/cable_link.c` for its exact `cable_link_start`/data-in entry point before wiring this —
  the goal is one shared decoder, two transports feeding it, not a second decoder), and sends
  outgoing frames as TX notifications. This task does not yet handle bonding or the
  advertising-resume policy — those are Tasks 2 and 3. For now, advertise openly and log a
  warning that bonding isn't enforced yet, so the task has an independently verifiable deliverable
  (it builds, it advertises) before the security-critical pieces land.

- [ ] **Step 1: Enable NimBLE in `sdkconfig.defaults.esp32s3` and `main/CMakeLists.txt`.**

- [ ] **Step 2: Build with `idf.py build` (or this project's existing build entry point — check**
  for a Makefile/CI script target before typing `idf.py` directly). **Expected: clean build, no
  new warnings.** (No test framework covers a build step; this is the step's own verification.)

- [ ] **Step 3: Implement `cable_ble_start()` and the GATT service/advertising registration in**
  `cable_ble.c`, wired into `app_main.c`.

- [ ] **Step 4: Build again. Expected: clean build.**

- [ ] **Step 5: Commit**

```bash
git add sdkconfig.defaults.esp32s3 main/CMakeLists.txt main/cable_ble.c main/cable_ble.h main/app_main.c
git commit -m "feat(ble): advertise a Nordic-UART-shaped GATT service alongside USB"
```

---

### Task 2: `ble_policy` — advertising-resume and bond classification, natively tested

**Files:**
- Create: `main/ble_policy.c`, `main/ble_policy.h`
- Test: `test/test_ble_policy.c`
- Modify: `test/run.sh` (add the compile-and-run lines for this new test, following the exact
  pattern already used for `test/test_board.c` at the lines shown above)

**Interfaces:**
- Consumes: nothing from Task 1 — this file depends on no NimBLE or ESP-IDF headers at all, by
  design, so it compiles with a plain host compiler exactly like `board_table.c` does.
- Produces:
  ```c
  // ble_policy.h
  typedef enum { BLE_DISCONNECT_CENTRAL, BLE_DISCONNECT_PERIPHERAL, BLE_DISCONNECT_LINK_LOSS } ble_disconnect_reason_t;

  // Always true today (we always want to be connectable again) — a real function, not a stub,
  // because a later firmware revision (e.g. a deep-sleep mode) is exactly the kind of change that
  // would make this conditional, and the decision belongs here, not scattered at every call site.
  bool ble_policy_should_readvertise(ble_disconnect_reason_t reason);

  typedef enum { BOND_NONE, BOND_VALID, BOND_STALE } ble_bond_state_t;

  // peer_has_bond: true if the dial's own bond store has a record for this peer's address.
  // peer_claims_bonded: true if the peer's connection attempt asserts it already has a bond
  // (i.e. it skipped pairing). Mirrors the bridge-side plan's stale-bond detection: a peer that
  // claims a bond the dial doesn't have (or vice versa) is BOND_STALE, not BOND_NONE.
  ble_bond_state_t ble_policy_classify_bond(bool peer_has_bond, bool peer_claims_bonded);
  ```

- [ ] **Step 1: Write failing tests for `ble_policy_should_readvertise`**

```c
// test/test_ble_policy.c
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
```

- [ ] **Step 2: Add to `test/run.sh`, following the exact `test_board` block's shape:**

```bash
cc -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize="${SANITIZERS:-undefined,bounds}" \
   -I "$here/../main" -o "$out/test_ble_policy" "$here/test_ble_policy.c" "$here/../main/ble_policy.c"
"$out/test_ble_policy"
```

- [ ] **Step 3: Run `test/run.sh`. Expected: FAIL to compile — `ble_policy.c`/`.h` don't exist.**

- [ ] **Step 4: Implement `ble_policy.c`/`.h`** per the signatures above. The four
  `ble_policy_classify_bond` cases given in the test are the complete truth table for two
  booleans — no further cases exist to enumerate.

- [ ] **Step 5: Run `test/run.sh`. Expected: `ble_policy: ok`, and the rest of the suite still**
  passes (this step runs the whole script, not just the new block — it's one script, one pass/fail).

- [ ] **Step 6: Commit**

```bash
git add main/ble_policy.c main/ble_policy.h test/test_ble_policy.c test/run.sh
git commit -m "feat(ble): pure advertising-resume and bond-classification policy, host-tested"
```

---

### Task 3: Wire the policy into `cable_ble.c` — bonding, passkey, advertising resume

**Files:**
- Modify: `main/cable_ble.c` (from Task 1 — add the NimBLE pairing/bonding callbacks and the
  disconnect-event handler)
- Modify: `main/ui/habitat/ui_habitat.c` or wherever `ui_cable_toast`'s declaration lives (check —
  it's defined around line 5890 of `ui_habitat.c`; confirm its header before using it from
  `cable_ble.c`)

**Interfaces:**
- Consumes: `ble_policy_should_readvertise`, `ble_policy_classify_bond`, `ble_disconnect_reason_t`,
  `ble_bond_state_t` (Task 2); `ui_cable_toast(const char *message)` (existing, declared in
  whichever header `ui_habitat.c`'s other externally-called functions are declared in — read
  `main/cable_client.c`'s existing `#include` of that header before adding a new one).
- Produces: nothing further — this task completes the feature; Task 4 (battery) and Task 5
  (docs) are independent additions.

- [ ] **Step 1: Wire NimBLE's GAP disconnect event into `ble_policy_should_readvertise` and**
  `ble_gap_adv_start()` (or this IDF version's equivalent call) — on every disconnect event NimBLE
  reports, classify its reason into a `ble_disconnect_reason_t` and only call the real advertising
  restart if the policy function says yes. There is no new test here: Task 2 already proved the
  policy is always `true` today; this step is the untested NimBLE-glue wiring, consistent with this
  plan's Architecture section.

- [ ] **Step 2: Wire NimBLE's pairing/bonding callbacks: on a connection attempt, look up whether**
  the peer has an existing bond and whether its connection parameters assert prior bonding, call
  `ble_policy_classify_bond`, and: `BOND_VALID` → proceed; `BOND_NONE` → begin pairing (passkey
  method, not just-works) and call `ui_cable_toast()` with the passkey formatted for the owner to
  read and confirm; `BOND_STALE` → reject the connection and call `ui_cable_toast()` with a message
  naming the mismatch (e.g. `"BLE: a device claims a bond I don't have — re-pair it"`) rather than
  silently dropping it.

- [ ] **Step 3: Confirm that the RX/TX characteristics reject access before `BOND_VALID` is**
  reached for a given connection (NimBLE's own characteristic permission flags, set at GATT
  registration time in Task 1, should enforce this — confirm it actually does by inspecting the
  permission flags you set in Task 1's `cable_ble.c`, not by assuming).

- [ ] **Step 4: Guard the passkey toast against being overwritten mid-pairing (Review Focus item**
  5) — add a `pairing_active` state flag in `cable_ble.c`, set it before calling `ui_cable_toast()`
  with the passkey and clear it once pairing completes or times out; mirror `ui_cable_toast()`'s
  own existing `s.voice_open` guard in `main/ui/habitat/ui_habitat.c` by adding the same style of
  early-return check there for this new flag, so an unrelated toast (a voice error, an agent
  event) arriving mid-pairing is dropped rather than replacing the passkey the owner is reading.
  No native test here — this touches `ui_habitat.c`'s own display-lock state, which (like the rest
  of that file) is exercised on real hardware, not natively; verify by code inspection that the new
  guard's shape matches the existing `voice_open` one exactly.

- [ ] **Step 5: Build. Expected: clean.**

- [ ] **Step 6: Commit**

```bash
git add main/cable_ble.c
git commit -m "feat(ble): enforce bonding before data flow, passkey on screen, advertise after every disconnect"
```

---

### Task 4: Battery fields on `pong`

**Files:**
- Modify: `main/cable_client.c:1401` (the `ping` → `pong` handler)
- Test: `test/test_cable_pong_battery.c` (new; follow `test/test_cable_frame.c`'s plain-C,
  no-ESP-IDF pattern if `msg()`/`cJSON` compile standalone — check `cable_client.c`'s own
  `#include` list first; if `msg()` pulls in anything ESP-IDF-specific, test the smaller
  battery-field-formatting logic as its own extracted function instead, named below, rather than
  fighting the whole file's dependencies)

**Interfaces:**
- Consumes: `power_battery_pct()`, `power_is_charging()` (existing, `main/board/power.h`).
- Produces:
  ```c
  // A small, dependency-free function so this task's test doesn't need cable_client.c's full
  // include graph: given a raw power_battery_pct() reading, return what belongs on the wire.
  // In cable_ble.h or a new tiny header — wherever Task 1/3 left room; this plan leaves the exact
  // file to whichever of cable_ble.h/.c has the lightest existing dependency footprint.
  typedef struct { bool has_pct; int pct; bool charging; } ble_battery_wire_t;
  ble_battery_wire_t ble_battery_for_wire(int raw_pct, bool charging);
  ```
  `cable_client.c`'s `ping` handler calls `ble_battery_for_wire(power_battery_pct(),
  power_is_charging())` and adds `battPct` (the number, or JSON null if `!has_pct`) and `charging`
  to the existing `msg("pong")` object before sending.

- [ ] **Step 1: Write a failing test for `ble_battery_for_wire`**

```c
#include "../main/ble_policy.h" // or wherever Task 3 placed ble_battery_for_wire — update this include to match
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
```

- [ ] **Step 2: Add the matching block to `test/run.sh`. Run it. Expected: FAIL to compile.**

- [ ] **Step 3: Implement `ble_battery_for_wire`.**

- [ ] **Step 4: Run `test/run.sh`. Expected: all green, including this new block.**

- [ ] **Step 5: Wire it into `cable_client.c:1401`'s `ping` handler** (add `battPct`/`charging` to
  the `msg("pong")` object via `cJSON_AddNumberToObject`/`cJSON_AddNullToObject`/
  `cJSON_AddBoolToObject` as appropriate — no test covers this one line of wiring directly, since
  `cable_client.c` is not natively compiled in isolation here; it's covered by the firmware's
  existing on-device acceptance testing, same as the rest of `cable_client.c`'s message building).

- [ ] **Step 6: Build. Expected: clean. Run `test/run.sh` once more. Expected: all green.**

- [ ] **Step 7: Commit**

```bash
git add main/cable_client.c test/test_cable_pong_battery.c test/run.sh main/ble_policy.h
git commit -m "feat(ble): surface battery percent and charging state on the existing pong"
```

---

### Task 5: Documentation (public-facing)

This repo is going public soon — this task documents the BLE peripheral for a reader who has
never seen this firmware, not just an internal changelog line.

**Files:**
- Modify: `devices/harness-device/README.md` if one exists at that level (check — otherwise the
  nearest existing README covering this firmware) — add a short "Transports" note pointing at the
  fuller doc below.
- Create: `devices/harness-device/firmware/docs/ble.md` — what the GATT service looks like (the
  three UUIDs, as constants a reader can grep for in both repos), the pairing/bonding flow from
  the owner's perspective (what appears on the dial's screen, what "stale bond" means and how to
  clear it so re-pairing is possible), and that advertising always resumes after a disconnect —
  the owner never has to power-cycle the dial to make it reachable again.

**Interfaces:** none.

- [ ] **Step 1: Read the nearest existing top-level doc for this firmware** (check for a README at
  `devices/harness-device/` or `devices/harness-device/firmware/`) to match this project's existing
  documentation voice before writing.

- [ ] **Step 2: Write `devices/harness-device/firmware/docs/ble.md`** covering the GATT shape,
  the pairing flow and what the owner sees/does, the stale-bond recovery path, and a
  cross-reference to the bridge-side repo's `docs/ble-transport.md` (critter-harness) so a reader
  of either repo's docs can find the other half.

- [ ] **Step 3: Add the "Transports" pointer to the nearest README.**

- [ ] **Step 4: Run `test/run.sh` once more to confirm nothing broke (docs-only changes, but**
  verify before every commit per this project's own discipline). **Expected: all passing.**

- [ ] **Step 5: Commit**

```bash
git add devices/harness-device/firmware/docs/ble.md devices/harness-device/README.md
git commit -m "docs: document the BLE peripheral for public readers"
```

---

## Before the first real-hardware run

**None of the tasks above flash this firmware to the physical dial or test it over real
Bluetooth.** `ble_policy.c` and `ble_battery_for_wire` are proven on the host, exactly like
`board_table.c` is today; `cable_ble.c`'s actual NimBLE glue is unverified by this plan's own test
suite, same as `cable_link.c`'s USB glue always has been. The first time this firmware is actually
flashed to the physical dial and connected to over real BLE — after this plan's branch is
otherwise complete and reviewed — needs the owner's explicit go-ahead first, per the standing
hardware-safety rule from this session: this is not a step any task in this plan performs on its
own, and the executor should stop and ask before attempting it, even once all tasks above show
green.
