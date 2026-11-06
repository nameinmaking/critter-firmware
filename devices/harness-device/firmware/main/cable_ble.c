#include "cable_ble.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "ble_policy.h"
#include "cable_session_io.h" // cable_session_io_run — serializes on_disconnect against on_frame/session_tick
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"
#include "ui/ui_screens.h" // ui_cable_toast

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
static void          (*s_on_disconnect)(void);
static cable_decoder_t s_decoder; // this transport's own decoder instance — see cable_session_io.h

static uint8_t own_addr_type;
static uint16_t s_tx_val_handle;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool     s_tx_subscribed;
static esp_timer_handle_t s_tick_timer;
static esp_timer_handle_t s_link_timeout_timer;

// Bonding/pairing state for the one connection this dial ever has at a time (undirected
// single-peripheral advertising — see ble_advertise). See this plan's ledger (Task 3 Ruling, and
// the Final Review fix-pass Rulings) for how these drive ble_policy_classify_bond's two call sites.
static bool s_peer_has_bond;   // the dial's own store already had a record for this peer at connect time
static bool s_pairing_active;  // true from PASSKEY_ACTION until ENC_CHANGE/DISCONNECT — also Step 4's toast guard

// Set only once BLE_GAP_EVENT_ENC_CHANGE reports both `encrypted` and `authenticated` on the
// connection descriptor — i.e. the passkey exchange actually happened, not merely an encrypted
// Just Works link. cable_ble_send refuses to notify anything until this is true, independent of
// the RX characteristic's own WRITE_AUTHEN enforcement: the two are separate layers (inbound ATT
// permission vs. outbound application gate) and neither substitutes for the other. Cleared on
// every CONNECT and DISCONNECT.
static bool s_link_trusted;

// A connection that never becomes trusted (ignores the security request, stalls mid-pairing, or
// completes only a Just Works encryption) would otherwise hold the single connection slot forever
// — nothing else disconnects it. This bounds that: it is armed on every CONNECT and disarmed the
// moment the link is trusted or the peer disconnects on its own.
#define BLE_LINK_TRUST_TIMEOUT_US (30 * 1000 * 1000)

#define BOND_MAX_PEERS 8 // this dial only ever bonds to the one bridge it's paired with; generous headroom for re-pairing without deleting the old entry first

static bool peer_already_bonded(const ble_addr_t *peer_id_addr)
{
    ble_addr_t peers[BOND_MAX_PEERS];
    int count = 0;
    int rc = ble_store_util_bonded_peers(peers, &count, BOND_MAX_PEERS);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_store_util_bonded_peers failed; rc=%d — treating as no bond on file", rc);
        return false;
    }
    for (int i = 0; i < count; i++) {
        if (ble_addr_cmp(&peers[i], peer_id_addr) == 0) return true;
    }
    return false;
}

// Every 250ms, matching cable_client.c's own session_task hello-loop cadence — NimBLE is
// event-driven and has no polling loop of its own to piggyback a periodic tick on, the way
// cable_link.c's reader task does.
#define BLE_TICK_PERIOD_US (250 * 1000)

static void tick_timer_cb(void *arg)
{
    (void)arg;
    if (s_tick) s_tick(s_ctx);
}

static void link_timeout_cb(void *arg)
{
    (void)arg;
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE && !s_link_trusted) {
        ESP_LOGW(TAG, "connection %d never became trusted within %ds — terminating it so advertising can resume for someone else",
                 s_conn_handle, BLE_LINK_TRUST_TIMEOUT_US / 1000000);
        ble_gap_terminate(s_conn_handle, BLE_ERR_AUTH_FAIL);
    }
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
                // _AUTHEN (not just _ENC) makes NimBLE itself reject a write at the ATT layer —
                // before gatt_access_cb ever runs — on a link that hasn't completed the passkey
                // exchange, including one that merely encrypted via Just Works. _NO_RSP matches
                // how the bridge actually writes this characteristic (write-without-response);
                // plain _WRITE stays too so a central that prefers write-with-response still works.
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_AUTHEN,
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
    // The name does NOT go here: flags (3B) + a complete 128-bit UUID list (2+16=18B) already
    // total 21 of the legacy advertising packet's 31-byte budget, and adding "harness-dial" (2+12)
    // would overflow it — ble_gap_adv_set_fields would then fail outright and this dial would never
    // advertise at all. The name goes in the scan response instead, a second, separate 31-byte
    // packet a central only gets if it actively scans, which costs this packet nothing.

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof rsp_fields);
    const char *name = ble_svc_gap_device_name();
    rsp_fields.name = (uint8_t *)name;
    rsp_fields.name_len = strlen(name);
    rsp_fields.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGW(TAG, "error setting scan response data; rc=%d — advertising without a name", rc);
    }

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, cable_ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
    }
}

static void run_on_disconnect(void *arg)
{
    (void)arg;
    if (s_on_disconnect) s_on_disconnect();
}

int cable_ble_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "connection %s; status=%d", event->connect.status == 0 ? "established" : "failed", event->connect.status);
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_pairing_active = false;
            s_link_trusted = false;
            cable_decoder_reset(&s_decoder); // a half-received frame from a prior connection must not bleed into this one
            struct ble_gap_conn_desc desc;
            if (ble_gap_conn_find(s_conn_handle, &desc) == 0) {
                s_peer_has_bond = peer_already_bonded(&desc.peer_id_addr);
            } else {
                s_peer_has_bond = false;
            }
            int rc = ble_gap_security_initiate(s_conn_handle);
            if (rc != 0) {
                ESP_LOGW(TAG, "ble_gap_security_initiate failed; rc=%d", rc);
            }
            if (s_link_timeout_timer) esp_timer_start_once(s_link_timeout_timer, BLE_LINK_TRUST_TIMEOUT_US);
        } else {
            ble_advertise(); // a failed connection attempt leaves us not advertising — resume it
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT: {
        ESP_LOGI(TAG, "disconnect; reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_tx_subscribed = false;
        s_link_trusted = false;
        if (s_link_timeout_timer) esp_timer_stop(s_link_timeout_timer);
        cable_decoder_reset(&s_decoder); // whatever was mid-frame when the link dropped is not valid for the next connection either
        if (s_pairing_active) ui_set_ble_pairing(false, NULL); // a disconnect mid-pairing still clears the guard
        s_pairing_active = false;
        cable_session_io_run(run_on_disconnect, NULL); // tells cable_client.c BLE is no longer the active transport, serialized against on_frame
        ble_disconnect_reason_t reason;
        // event->disconnect.reason is a host status code (BLE_HS_HCI_ERR(hci_code)), not a raw HCI
        // reason — comparing against bare BLE_ERR_* constants here never matched either branch.
        switch (event->disconnect.reason) {
        case BLE_HS_HCI_ERR(BLE_ERR_REM_USER_CONN_TERM): reason = BLE_DISCONNECT_CENTRAL; break;
        case BLE_HS_HCI_ERR(BLE_ERR_CONN_TERM_LOCAL):    reason = BLE_DISCONNECT_PERIPHERAL; break;
        default:                                         reason = BLE_DISCONNECT_LINK_LOSS; break;
        }
        // Advertising must resume after EVERY kind of disconnect, including one the central
        // itself initiated — a documented bug in a different ESP32 BLE stack (Arduino's) is
        // exactly this failing to happen; see the design doc's prior-art section.
        if (ble_policy_should_readvertise(reason)) {
            ble_advertise();
        }
        return 0;
    }

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_tx_val_handle) {
            s_tx_subscribed = event->subscribe.cur_notify;
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGD(TAG, "mtu update; conn_handle=%d mtu=%d", event->mtu.conn_handle, event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        // The peer is attempting to pair fresh while this dial still has a bond on file for it —
        // the mirror image of PASSKEY_ACTION's stale-bond check (there, the dial has no bond but
        // the peer claims one; here, the dial has one but the peer is pairing as if it had none,
        // e.g. because the peer forgot it). Delete this dial's half and let the stack retry the
        // pairing from scratch, which lands back on PASSKEY_ACTION with a clean slate — the owner
        // still has to confirm the passkey either way, so this never trusts a peer on its say-so.
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ESP_LOGW(TAG, "peer re-pairing despite an existing bond — deleting the old bond and retrying");
            ui_cable_toast("BLE: re-pairing a device I already knew — confirm the new code");
            ble_gap_unpair(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
            // Fresh pairing is starting (no resumption) — this call site always passes
            // peer_claims_bonded=false; see this plan's ledger (Task 3 Ruling) for why.
            ble_bond_state_t bond = ble_policy_classify_bond(s_peer_has_bond, false);
            if (bond == BOND_STALE) {
                ESP_LOGW(TAG, "rejecting a fresh pairing attempt from a peer this dial already has a bond for");
                ui_cable_toast("BLE: a device claims a bond I don't have — re-pair it");
                ble_gap_terminate(event->passkey.conn_handle, BLE_ERR_AUTH_FAIL);
                return 0;
            }
            s_pairing_active = true;
            uint32_t passkey = esp_random() % 1000000u;
            char msg[48];
            snprintf(msg, sizeof msg, "BLE pairing code: %06" PRIu32, passkey);
            ui_set_ble_pairing(true, msg); // raises the toast guard AND shows the passkey (Step 4)
            struct ble_sm_io pkey = { .action = BLE_SM_IOACT_DISP, .passkey = passkey };
            int rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            if (rc != 0) {
                ESP_LOGE(TAG, "ble_sm_inject_io failed; rc=%d", rc);
            }
        }
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE: {
        bool fresh_pairing_just_ran = s_pairing_active;
        s_pairing_active = false;
        if (fresh_pairing_just_ran) ui_set_ble_pairing(false, NULL); // pairing concluded either way — clear the toast guard
        if (!fresh_pairing_just_ran) {
            // No PASSKEY_ACTION preceded this: the peer attempted to resume directly —
            // peer_claims_bonded=true at this call site; see this plan's ledger (Task 3 Ruling).
            // This also catches a Just Works central that encrypted without ever triggering a
            // passkey prompt: s_peer_has_bond is false for it too, so it lands here as STALE. It
            // is left connected but untrusted rather than torn down on the spot — cable_ble_send's
            // own trust gate and the RX characteristic's WRITE_AUTHEN flag already make such a
            // link harmless, and BLE_LINK_TRUST_TIMEOUT_US bounds how long it can occupy the slot.
            ble_bond_state_t bond = ble_policy_classify_bond(s_peer_has_bond, true);
            if (bond == BOND_STALE) {
                ESP_LOGW(TAG, "a peer resumed without pairing but this dial has no bond for it (enc status=%d)", event->enc_change.status);
                ui_cable_toast("BLE: a device claims a bond I don't have — re-pair it");
            } else if (event->enc_change.status != 0) {
                ESP_LOGE(TAG, "encryption resumption failed despite a matching bond on file; status=%d", event->enc_change.status);
            }
        } else if (event->enc_change.status != 0) {
            ESP_LOGW(TAG, "pairing's encryption step failed; status=%d", event->enc_change.status);
        }
        if (event->enc_change.status == 0) {
            struct ble_gap_conn_desc desc;
            if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0 &&
                desc.sec_state.encrypted && desc.sec_state.authenticated) {
                s_link_trusted = true;
                if (s_link_timeout_timer) esp_timer_stop(s_link_timeout_timer);
            }
        }
        return 0;
    }

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

bool cable_ble_start(cable_frame_cb cb, cable_tick_cb tick, void *ctx, void (*on_disconnect)(void))
{
    s_cb = cb;
    s_tick = tick;
    s_ctx = ctx;
    s_on_disconnect = on_disconnect;
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
    // Passkey-on-the-dial's-own-screen pairing: the dial has a display and no input, so it
    // generates and shows the passkey (BLE_GAP_EVENT_PASSKEY_ACTION, BLE_SM_IOACT_DISP case below);
    // the central's own OS prompts the owner to type it in. sm_mitm alone only REQUESTS
    // authenticated pairing — sm_sec_lvl=4 is what makes NimBLE actually require LE Secure
    // Connections with an authenticated key for GATT access, and sm_sc_only=1 refuses to fall back
    // to legacy (passively crackable) pairing. Bonding stores the result so later connects skip
    // pairing.
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_DISP_ONLY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_sc_only = 1;
    ble_hs_cfg.sm_sec_lvl = 4;
    ble_hs_cfg.sm_our_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC;
    ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC;

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

    const esp_timer_create_args_t timeout_args = {
        .callback = link_timeout_cb,
        .name = "cable_ble_trust_to",
    };
    if (esp_timer_create(&timeout_args, &s_link_timeout_timer) != ESP_OK) {
        ESP_LOGW(TAG, "no link-trust timeout timer — an unauthenticated connection could hold the slot indefinitely");
        s_link_timeout_timer = NULL;
    }

    nimble_port_freertos_init(host_task);
    return true;
}

// Guards s_tx_frame exactly as cable_link.c's s_tx_lock guards its own static frame buffer:
// CABLE_MAX_FRAME (8200 bytes) does not fit on any of this dial's task stacks (cable_link.c itself
// says so), and cable_ble_send is called from several of them (the session task, UI, audio).
static SemaphoreHandle_t s_tx_lock;
static uint8_t s_tx_frame[CABLE_MAX_FRAME];

bool cable_ble_send(uint8_t type, const uint8_t *payload, size_t payload_len)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_tx_subscribed || !s_link_trusted) return false;

    if (!s_tx_lock) {
        s_tx_lock = xSemaphoreCreateMutex();
        if (!s_tx_lock) return false;
    }
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);

    int n = cable_frame_encode(type, payload, payload_len, s_tx_frame, sizeof(s_tx_frame));
    if (n < 0) {
        ESP_LOGE(TAG, "refusing to send %u-byte payload (type %u) — too large to frame", (unsigned)payload_len, (unsigned)type);
        xSemaphoreGive(s_tx_lock);
        return false;
    }

    // A notification is truncated silently at the connection's negotiated ATT MTU minus the
    // 3-byte ATT notify header — NimBLE never reports this as an error, it just cuts the payload.
    // CABLE_MAX_FRAME is far larger than any realistic MTU, so every frame above a couple hundred
    // bytes (which is most of them — hello alone is) has to go out as several notifications. The
    // decoder on both ends resyncs across chunk boundaries the same way it already does across
    // separate GATT writes, so chunk boundaries need not align with frame boundaries.
    uint16_t mtu = ble_att_mtu(s_conn_handle);
    if (mtu <= 3) mtu = BLE_ATT_MTU_DFLT;
    const size_t chunk_cap = (size_t)mtu - 3;

    bool ok = true;
    for (size_t off = 0; off < (size_t)n; off += chunk_cap) {
        size_t len = (size_t)n - off;
        if (len > chunk_cap) len = chunk_cap;
        struct os_mbuf *om = ble_hs_mbuf_from_flat(s_tx_frame + off, (uint16_t)len);
        if (!om) { ok = false; break; } // no memory for this notification right now — same "not connected" treatment as any other send failure
        int rc = ble_gatts_notify_custom(s_conn_handle, s_tx_val_handle, om);
        if (rc != 0) { ok = false; break; }
    }

    xSemaphoreGive(s_tx_lock);
    return ok;
}
