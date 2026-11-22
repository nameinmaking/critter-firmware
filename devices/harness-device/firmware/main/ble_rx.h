// One FreeRTOS task that owns everything the BLE transport does with received bytes and with its
// periodic tick, so neither runs where it can do harm:
// - GATT writes arrive on the NimBLE host task. Running the session's frame handlers there (which
//   send replies) meant a reply waiting for free notification buffers would wait on the very task
//   that frees them. Writes are now copied into a stream buffer and handled here instead.
// - The 250 ms tick used to run on the esp_timer task and block on the session lock, stalling every
//   other esp_timer callback on the dial. It runs here now.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Starts the task. on_bytes gets received bytes in order; on_tick runs about every 250 ms; on_reset
// runs (on this task, before any later bytes) after ble_rx_reset — the caller resets its frame
// decoder there. Returns false if the task or buffer could not be created.
bool ble_rx_start(void (*on_bytes)(const uint8_t *data, size_t len), void (*on_tick)(void), void (*on_reset)(void));

// Queues received bytes. Called from the NimBLE host task; never blocks. False means the buffer
// was full and the bytes were dropped (logged) — the frame decoder resyncs past the gap.
bool ble_rx_push(const uint8_t *data, size_t len);

// Discards bytes not yet handled and has on_reset run before anything pushed afterwards. Called on
// every BLE connect and disconnect, so nothing from one connection reaches the next.
void ble_rx_reset(void);
