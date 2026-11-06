#include "cable_session_io.h"

#include <stdatomic.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "session_io";

// Bounds the queue to ordinary JSON control-message sizes. CABLE_TYPE_FW and CABLE_TYPE_PET (bulk
// transfers, up to CABLE_MAX_PAYLOAD=8192) are rejected by their transport before reaching here —
// see cable_ble.c — so this only ever needs to hold small, low-rate control traffic.
#define SESSION_QUEUE_DEPTH   4
#define SESSION_MAX_PAYLOAD   4096
#define SESSION_TASK_STACK    4096

typedef struct {
    uint8_t  version;
    uint8_t  type;
    uint8_t *payload; // heap-allocated by the producer, freed by the session task after the callback returns
    size_t   payload_len;
    void    *ctx;
} session_item_t;

static QueueHandle_t    s_queue;
static cable_frame_cb    s_cb;
static cable_tick_cb     s_tick;
static void             *s_ctx;
static atomic_bool       s_tick_requested;

static void *session_payload_alloc(size_t n)
{
    void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
    if (p) return p;
    return heap_caps_malloc(n, MALLOC_CAP_INTERNAL); // small enough to fall back without much risk
}

static void session_task(void *arg)
{
    (void)arg;
    while (1) {
        session_item_t item;
        if (xQueueReceive(s_queue, &item, pdMS_TO_TICKS(100)) == pdTRUE) {
            s_cb(item.version, item.type, item.payload, item.payload_len, item.ctx);
            heap_caps_free(item.payload);
        }
        if (atomic_exchange(&s_tick_requested, false)) {
            if (s_tick) s_tick(s_ctx);
        }
    }
}

bool cable_session_io_start(cable_frame_cb cb, cable_tick_cb tick, void *ctx)
{
    s_cb = cb;
    s_tick = tick;
    s_ctx = ctx;
    atomic_init(&s_tick_requested, false);

    s_queue = xQueueCreate(SESSION_QUEUE_DEPTH, sizeof(session_item_t));
    if (!s_queue) {
        ESP_LOGE(TAG, "no memory for the session queue — no transport can reach the session");
        return false;
    }

    if (xTaskCreate(session_task, "cable_session_io", SESSION_TASK_STACK, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "session task create failed — no transport can reach the session");
        vQueueDelete(s_queue);
        s_queue = NULL;
        return false;
    }
    return true;
}

void cable_session_io_feed(uint8_t version, uint8_t type, const uint8_t *payload, size_t payload_len, void *ctx)
{
    if (!s_queue) return;
    if (payload_len > SESSION_MAX_PAYLOAD) {
        ESP_LOGW(TAG, "dropping a %u-byte frame (type %u) — over the %d-byte session queue bound",
                 (unsigned)payload_len, (unsigned)type, SESSION_MAX_PAYLOAD);
        return;
    }
    uint8_t *copy = NULL;
    if (payload_len > 0) {
        copy = session_payload_alloc(payload_len);
        if (!copy) {
            ESP_LOGW(TAG, "dropping a %u-byte frame (type %u) — no memory to copy it", (unsigned)payload_len, (unsigned)type);
            return;
        }
        memcpy(copy, payload, payload_len);
    }
    session_item_t item = { .version = version, .type = type, .payload = copy, .payload_len = payload_len, .ctx = ctx };
    if (xQueueSend(s_queue, &item, 0) != pdTRUE) {
        // The queue is a backlog bound, not a buffer to grow: a session that cannot keep up with a
        // few outstanding control messages has a worse problem than one dropped frame.
        ESP_LOGW(TAG, "session queue full — dropping a %u-byte frame (type %u)", (unsigned)payload_len, (unsigned)type);
        if (copy) heap_caps_free(copy);
    }
}

void cable_session_io_tick(void *ctx)
{
    (void)ctx; // the real ctx was fixed when cable_session_io_start was called; a tick carries no data of its own
    atomic_store(&s_tick_requested, true);
}
