#include "ble_rx.h"

#include <stdatomic.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

static const char *TAG = "ble_rx";

#define BLE_RX_BUFFER_BYTES 4096
#define BLE_RX_STACK        6144   // matches cable_link.c's READER_STACK: both run the same frame handlers
#define BLE_RX_PRIORITY     5      // matches cable_link.c's reader task
#define BLE_RX_TICK_MS      250

static StreamBufferHandle_t s_stream;
static StaticStreamBuffer_t s_stream_struct;
static void (*s_on_bytes)(const uint8_t *, size_t);
static void (*s_on_tick)(void);
static void (*s_on_reset)(void);
static atomic_bool s_reset_pending;

static void drain_for_reset(void)
{
    uint8_t scratch[128];
    while (xStreamBufferReceive(s_stream, scratch, sizeof scratch, 0) > 0) {}
    if (s_on_reset) s_on_reset();
}

static void ble_rx_task(void *arg)
{
    (void)arg;
    uint8_t chunk[256];
    int64_t next_tick_us = esp_timer_get_time() + BLE_RX_TICK_MS * 1000;
    while (1) {
        int64_t now = esp_timer_get_time();
        TickType_t wait = 0;
        if (next_tick_us > now) wait = pdMS_TO_TICKS((next_tick_us - now) / 1000) + 1;
        size_t n = xStreamBufferReceive(s_stream, chunk, sizeof chunk, wait);
        // A reset requested before (or while) these bytes were read wins: they may belong to the
        // connection that just ended.
        if (atomic_exchange(&s_reset_pending, false)) {
            drain_for_reset();
            n = 0;
        }
        if (n > 0 && s_on_bytes) s_on_bytes(chunk, n);
        if (esp_timer_get_time() >= next_tick_us) {
            if (s_on_tick) s_on_tick();
            next_tick_us = esp_timer_get_time() + BLE_RX_TICK_MS * 1000;
        }
    }
}

bool ble_rx_start(void (*on_bytes)(const uint8_t *, size_t), void (*on_tick)(void), void (*on_reset)(void))
{
    s_on_bytes = on_bytes;
    s_on_tick = on_tick;
    s_on_reset = on_reset;
    // The storage lives in PSRAM: only tasks touch it, never an ISR, and internal RAM is the dial's
    // scarcest resource (see sdkconfig.defaults.esp32s3's NimBLE PSRAM note).
    uint8_t *storage = heap_caps_malloc(BLE_RX_BUFFER_BYTES + 1, MALLOC_CAP_SPIRAM);
    if (!storage) {
        ESP_LOGE(TAG, "no PSRAM for the BLE receive buffer");
        return false;
    }
    s_stream = xStreamBufferCreateStatic(BLE_RX_BUFFER_BYTES, 1, storage, &s_stream_struct);
    if (!s_stream) {
        heap_caps_free(storage);
        return false;
    }
    if (xTaskCreate(ble_rx_task, "ble_rx", BLE_RX_STACK, NULL, BLE_RX_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "ble_rx task create failed");
        vStreamBufferDelete(s_stream);
        s_stream = NULL;
        heap_caps_free(storage);
        return false;
    }
    return true;
}

bool ble_rx_push(const uint8_t *data, size_t len)
{
    if (!s_stream) return false;
    size_t sent = xStreamBufferSend(s_stream, data, len, 0);
    if (sent != len) {
        ESP_LOGW(TAG, "BLE receive buffer full — dropped %u of %u bytes", (unsigned)(len - sent), (unsigned)len);
        return false;
    }
    return true;
}

void ble_rx_reset(void)
{
    atomic_store(&s_reset_pending, true);
}
