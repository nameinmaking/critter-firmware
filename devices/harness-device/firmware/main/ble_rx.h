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

// The largest write ble_rx_push accepts: the largest attribute value ATT allows.
#define BLE_RX_MAX_WRITE 512

// Starts the task. on_bytes gets each queued write, whole and in order, with the tag it was queued
// with; on_tick runs about every 250 ms. Returns false if the task or buffer could not be created.
//
// The tag is the BLE connection generation the write arrived on (see cable_ble.c). Carrying it with
// the bytes, instead of resetting the queue on connect/disconnect, means the receiver can tell
// exactly which connection every byte came from: it resets its frame decoder whenever the tag
// changes and ignores frames whose connection has ended. Nothing is discarded by timing.
bool ble_rx_start(void (*on_bytes)(uint32_t tag, const uint8_t *data, size_t len), void (*on_tick)(void));

// Queues one received write (at most BLE_RX_MAX_WRITE bytes). Called from the NimBLE host task;
// never blocks. A write is queued whole or not at all; false means the buffer was full and it was
// dropped (logged and counted) — the frame decoder resyncs past the gap.
bool ble_rx_push(uint32_t tag, const uint8_t *data, size_t len);
