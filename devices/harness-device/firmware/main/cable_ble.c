#include "cable_ble.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"

// Not declared by any IDF/NimBLE header (ble_store_config.h only declares read/write/delete) —
// every NimBLE example forward-declares it itself in the file that calls it; bleprph's main.c
// does the same thing.
void ble_store_config_init(void);

static const char *TAG = "cable_ble";

// Fixed verbatim across this repo and the bridge-side critter-harness repo — see this file's own
// header comment and docs/ble.md. BLE_UUID128_INIT takes the UUID's bytes little-endian (reversed
// from how the dashed string reads left to right).
static const ble_uuid128_t s_service_uuid =
    BLE_UUID128_INIT(0x1a, 0x2a, 0x54, 0x15, 0x26, 0x5e, 0x97, 0x9c, 0x6d, 0x42, 0xf1, 0xca, 0xb4, 0x86, 0x57, 0xaf);
static const ble_uuid128_t s_rx_uuid =
    BLE_UUID128_INIT(0x6d, 0x87, 0x8a, 0x8b, 0x79, 0xa6, 0x80, 0x97, 0xdd, 0x40, 0x59, 0x5e, 0xa4, 0x83, 0xaa, 0x93);
static const ble_uuid128_t s_tx_uuid =
    BLE_UUID128_INIT(0x90, 0x6e, 0xe8, 0xfc, 0x23, 0x44, 0x85, 0x87, 0xef, 0x42, 0x6a, 0x0d, 0x45, 0x9d, 0x36, 0xdb);

static cable_frame_cb  s_cb;
static cable_tick_cb   s_tick;
static void           *s_ctx;
static cable_decoder_t s_decoder; // this transport's own decoder instance — see cable_session_io.h

static uint8_t own_addr_type;
static uint16_t s_tx_val_handle;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool     s_tx_subscribed;
static esp_timer_handle_t s_tick_timer;

// Every 250ms, matching cable_client.c's own session_task hello-loop cadence — NimBLE is
// event-driven and has no polling loop of its own to piggyback a periodic tick on, the way
// cable_link.c's reader task does.
#define BLE_TICK_PERIOD_US (250 * 1000)

static void tick_timer_cb(void *arg)
{
    (void)arg;
    if (s_tick) s_tick(s_ctx);
}

// Wraps the given `cb` with the BLE-only bulk-transfer exclusion documented in cable_ble.h.
static void ble_frame_cb(uint8_t version, uint8_t type, const uint8_t *payload, size_t payload_len, void *ctx)
{
    if (type == CABLE_TYPE_FW || type == CABLE_TYPE_PET) {
        ESP_LOGW(TAG, "dropping a type-%u frame over BLE — bulk transfers stay USB-only", (unsigned)type);
        return;
    }
    s_cb(version, type, payload, payload_len, ctx);
}

static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0; // the TX characteristic is notify-only; nothing reads or writes it from the central side
    uint8_t chunk[256];
    uint16_t out_len = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, chunk, sizeof(chunk), &out_len);
    if (rc != 0) {
        ESP_LOGW(TAG, "oversized or malformed GATT write (rc=%d) — dropped", rc);
        return 0; // never fails the GATT operation itself; the frame decoder resyncs past bad bytes
    }
    cable_decoder_feed(&s_decoder, chunk, out_len, ble_frame_cb, s_ctx);
    return 0;
}

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &s_rx_uuid.u,
                .access_cb = gatt_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &s_tx_uuid.u,
                .access_cb = gatt_access_cb,
                .val_handle = &s_tx_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }, // no more characteristics
        },
    },
    { 0 }, // no more services
};

static int cable_ble_gap_event(struct ble_gap_event *event, void *arg);

static void ble_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)&s_service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, cable_ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
    }
}

int cable_ble_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "connection %s; status=%d", event->connect.status == 0 ? "established" : "failed", event->connect.status);
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
        } else {
            ble_advertise(); // a failed connection attempt leaves us not advertising — resume it
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnect; reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_tx_subscribed = false;
        // Advertising must resume after EVERY kind of disconnect, including one the central
        // itself initiated — a documented bug in a different ESP32 BLE stack (Arduino's) is
        // exactly this failing to happen; see the design doc's prior-art section.
        ble_advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_tx_val_handle) {
            s_tx_subscribed = event->subscribe.cur_notify;
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGD(TAG, "mtu update; conn_handle=%d mtu=%d", event->mtu.conn_handle, event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE host reset; reason=%d", reason);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "no usable BLE address; rc=%d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "error determining address type; rc=%d", rc);
        return;
    }
    ble_advertise();
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run(); // returns only when nimble_port_stop() is called — never, for this dial
    nimble_port_freertos_deinit();
}

bool cable_ble_start(cable_frame_cb cb, cable_tick_cb tick, void *ctx)
{
    s_cb = cb;
    s_tick = tick;
    s_ctx = ctx;
    cable_decoder_init(&s_decoder);

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed (%s) — the dial still works over USB", esp_err_to_name(err));
        return false;
    }

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.gatts_register_cb = NULL;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    // Task 1 scope only: no bonding enforcement yet. Every central that finds this advertisement
    // can write/subscribe today — Task 2/3 (ble_policy.c, wired into this file's gap/bonding
    // callbacks) is what turns this on before the device trusts anything it receives.
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ESP_LOGW(TAG, "BLE advertising with no bonding enforced yet (Task 1 of the BLE transport plan) — do not treat this build as secure");

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed; rc=%d", rc);
        return false;
    }
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed; rc=%d", rc);
        return false;
    }

    rc = ble_svc_gap_device_name_set("harness-dial");
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_svc_gap_device_name_set failed; rc=%d", rc);
    }

    ble_store_config_init();

    const esp_timer_create_args_t tick_args = {
        .callback = tick_timer_cb,
        .name = "cable_ble_tick",
    };
    if (esp_timer_create(&tick_args, &s_tick_timer) == ESP_OK) {
        esp_timer_start_periodic(s_tick_timer, BLE_TICK_PERIOD_US);
    } else {
        ESP_LOGW(TAG, "no periodic tick timer — BLE sessions will not notice an idle host as promptly");
    }

    nimble_port_freertos_init(host_task);
    return true;
}

bool cable_ble_send(uint8_t type, const uint8_t *payload, size_t payload_len)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_tx_subscribed) return false;

    uint8_t frame[CABLE_MAX_FRAME];
    int n = cable_frame_encode(type, payload, payload_len, frame, sizeof(frame));
    if (n < 0) {
        ESP_LOGE(TAG, "refusing to send %u-byte payload (type %u) — too large to frame", (unsigned)payload_len, (unsigned)type);
        return false;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(frame, (uint16_t)n);
    if (!om) return false; // no memory for this notification right now — same "not connected" treatment as any other send failure
    int rc = ble_gatts_notify_custom(s_conn_handle, s_tx_val_handle, om);
    return rc == 0;
}
