#include "cable_session_io.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "session_io";

static SemaphoreHandle_t s_lock;
static cable_frame_cb    s_cb;
static cable_tick_cb     s_tick;
static void             *s_ctx;

bool cable_session_io_start(cable_frame_cb cb, cable_tick_cb tick, void *ctx)
{
    s_cb = cb;
    s_tick = tick;
    s_ctx = ctx;

    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) {
        ESP_LOGE(TAG, "no memory for the session lock — no transport can reach the session");
        return false;
    }
    return true;
}

void cable_session_io_feed(uint8_t version, uint8_t type, const uint8_t *payload, size_t payload_len, void *ctx)
{
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cb(version, type, payload, payload_len, ctx);
    xSemaphoreGive(s_lock);
}

void cable_session_io_tick(void *ctx)
{
    (void)ctx; // the real ctx was fixed when cable_session_io_start was called; a tick carries no data of its own
    if (!s_lock || !s_tick) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_tick(s_ctx);
    xSemaphoreGive(s_lock);
}

void cable_session_io_run(void (*fn)(void *arg), void *arg)
{
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    fn(arg);
    xSemaphoreGive(s_lock);
}
