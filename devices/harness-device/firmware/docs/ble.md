# BLE transport

The firmware's primary transport is USB — see the top-level [README](../README.md). A second,
optional transport runs over Bluetooth Low Energy, for machines where USB serial access to the
device is blocked (some corporate endpoint security policies do this). It is a second way to reach
the same cable protocol, not a second protocol: the message catalog, frame format and the host-side
daemon's behavior are unchanged either way. See the bridge-side repo's
[`docs/ble-transport.md`](https://github.com/nameinmaking/critter-harness/blob/main/docs/ble-transport.md)
for the host/daemon half of this picture (how the daemon finds, pairs with, and pins the device).

## The GATT service

One primary service, one write characteristic (host → device) and one notify characteristic
(device → host). The three UUIDs are fixed and identical in both this firmware and the bridge
daemon — grep for them in either repo:

| Role | UUID |
|---|---|
| Service | `af5786b4-caf1-426d-9c97-5e2615542a1a` |
| RX (host writes) | `93aa83a4-5e59-40dd-9780-a6798b8a876d` |
| TX (device notifies) | `db369d45-0d6a-42ef-8785-4423fce86e90` |

Bytes on either characteristic are the same framed messages USB carries (see
[`main/cable_frame.h`](../main/cable_frame.h)) — chunked to fit a GATT write/notify payload, not a
different encoding.

## Pairing, from the owner's side

The first time a given host connects, the device has no bond for it yet, so pairing runs
automatically: the device shows a 6-digit code on its own screen, and the host's operating system
shows its own Bluetooth pairing prompt asking the owner to confirm or type that same code in. This
is standard OS-level Bluetooth pairing, not anything the firmware or the bridge daemon draws
itself. Once paired, every later connection is automatic — no code, no prompt — until the bond is
lost on one side (see below).

While the code is on screen, nothing else can overwrite it: an unrelated notice (a voice error, an
agent event) arriving mid-pairing is dropped rather than replacing the code the owner is reading.

## When the two sides disagree about the bond

If the device still has a bond on file for an address that is now trying to pair fresh, or a host
tries to skip pairing and resume a bond the device doesn't actually have, the device rejects the
connection and shows `BLE: a device claims a bond I don't have — re-pair it`. This happens most
often after one side's Bluetooth state was reset independently of the other — for example the
firmware was reflashed, which clears the device's own bond store, while the host still remembers
the old one. Forgetting the device in the host's own Bluetooth settings and reconnecting runs
pairing again, same as "Pairing" above.

## Reconnecting

Advertising always resumes after a disconnect — however it happened: the host walking out of
range, closing its lid, or deliberately disconnecting. The owner never has to power-cycle the
device to make it reachable again.

## What BLE does not carry

Firmware updates and custom pet-pack (`.hpet`) installs stay USB-only. Physical access to the
cable remains the only path to flashing this device — a deliberate choice, not a gap: those are
large, multi-chunk transfers whose flow control is tied to the USB link's own buffering, and
extending that safely to a radio link is a different piece of work than this transport. A
firmware-update or pet-pack frame that somehow arrives over BLE is logged and dropped.

## Battery

Every `pong` reply (sent on the existing ping/pong cadence, any transport) carries the device's
battery percent and charging state, read from the PMIC where present. This is most useful over
BLE, where the device isn't tethered by a cable that would otherwise make "is it charging"
obvious.

## Status

This transport is build-verified (`idf.py build`) and its policy logic (advertising-resume
decisions, bond classification, battery-field formatting) is proven on the host alongside the rest
of this firmware's native test suite — see [`test/run.sh`](../test/run.sh). The NimBLE GATT,
pairing and advertising glue itself (`main/cable_ble.c`) has not yet been exercised against a real
central or a real device, the same honest caveat this firmware has always carried for
`main/cable_link.c`'s USB glue.
