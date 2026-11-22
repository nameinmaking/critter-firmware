#include "ble_rx.h"

#include <stdatomic.h>
#include <string.h>

#include "cable_frame.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/message_buffer.h"
#include "freertos/task.h"

static const char *TAG = "ble_rx";

// Room for two largest frames: writes arrive without response, so this buffer is the only slack
// while ble_rx is busy in a frame handler or a send retry. It lives in PSRAM (see ble_rx_start).
#define BLE_RX_BUFFER_BYTES (2 * CABLE_MAX_FRAME)
#define BLE_RX_STACK        6144   // matches cable_link.c's READER_STACK: both run the same frame handlers
#define BLE_RX_PRIORITY     5      // matches cable_link.c's reader task
#define BLE_RX_TICK_MS      250

// One queued write: its tag, then its bytes.
typedef struct {
    uint32_t tag;
    uint8_t  data[BLE_RX_MAX_WRITE];
} ble_rx_msg_t;

static MessageBufferHandle_t s_msgs;
static StaticMessageBuffer_t s_msgs_struct;
static ble_rx_msg_t s_in;   // only the ble_rx task reads into it; too big for its stack alongside the handlers
static ble_rx_msg_t s_out;  // only the NimBLE host task pushes (single writer), whose 4 KB stack has no room for it
static void (*s_on_bytes)(uint32_t, const uint8_t *, size_t);
static void (*s_on_tick)(void);
static atomic_uint s_dropped_writes;

static void ble_rx_task(void *arg)
{
    (void)arg;
    int64_t next_tick_us = esp_timer_get_time() + BLE_RX_TICK_MS * 1000;
    while (1) {
        int64_t now = esp_timer_get_time();
        TickType_t wait = 0;
        if (next_tick_us > now) wait = pdMS_TO_TICKS((next_tick_us - now) / 1000) + 1;
        size_t n = xMessageBufferReceive(s_msgs, &s_in, sizeof s_in, wait);
        if (n >= sizeof s_in.tag && s_on_bytes) s_on_bytes(s_in.tag, s_in.data, n - sizeof s_in.tag);
        if (esp_timer_get_time() >= next_tick_us) {
            if (s_on_tick) s_on_tick();
            next_tick_us = esp_timer_get_time() + BLE_RX_TICK_MS * 1000;
        }
    }
}

bool ble_rx_start(void (*on_bytes)(uint32_t tag, const uint8_t *data, size_t len), void (*on_tick)(void))
{
    s_on_bytes = on_bytes;
    s_on_tick = on_tick;
    // The storage lives in PSRAM: only tasks touch it, never an ISR, and internal RAM is the dial's
    // scarcest resource (see sdkconfig.defaults.esp32s3's NimBLE PSRAM note).
    uint8_t *storage = heap_caps_malloc(BLE_RX_BUFFER_BYTES + 1, MALLOC_CAP_SPIRAM);
    if (!storage) {
        ESP_LOGE(TAG, "no PSRAM for the BLE receive buffer");
        return false;
    }
    s_msgs = xMessageBufferCreateStatic(BLE_RX_BUFFER_BYTES, storage, &s_msgs_struct);
    if (!s_msgs) {
        heap_caps_free(storage);
        return false;
    }
    if (xTaskCreate(ble_rx_task, "ble_rx", BLE_RX_STACK, NULL, BLE_RX_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "ble_rx task create failed");
        vMessageBufferDelete(s_msgs);
        s_msgs = NULL;
        heap_caps_free(storage);
        return false;
    }
    return true;
}

bool ble_rx_push(uint32_t tag, const uint8_t *data, size_t len)
{
    if (!s_msgs || len > BLE_RX_MAX_WRITE) return false;
    // Sent as one message, so it is queued whole or not at all.
    s_out.tag = tag;
    memcpy(s_out.data, data, len);
    if (xMessageBufferSend(s_msgs, &s_out, sizeof s_out.tag + len, 0) == 0) {
        unsigned drops = atomic_fetch_add(&s_dropped_writes, 1) + 1;
        ESP_LOGW(TAG, "BLE receive buffer full — dropped a %u-byte write (%u writes dropped since boot)",
                 (unsigned)len, drops);
        return false;
    }
    return true;
}
