#include "jk_bms_ble.h"

static const char *tag = "ESP32-BMS";

/* Function definitions */
// --- Initialization & Startup ---
static void blecent_on_reset(int reason);
static void blecent_on_sync(void);
static void blecent_host_task(void *param);

// --- Scanning & Connecting ---
static void ble_scan(void);
static int is_target_device(const struct ble_gap_disc_desc *disc, const std::string &peer_addr_str);
static void ble_connect(void *disc);

// --- GAP Event Handling ---
static int blecent_gap_event(struct ble_gap_event *event, void *arg);

/* ------------------------ BLE Handling ------------------------ */

/**
 * GAP event handler
 */
static int blecent_gap_event(struct ble_gap_event *event, void *arg)
{
    // struct ble_gap_conn_desc desc;
    struct ble_hs_adv_fields fields;
    int rc;
    printf("GAP Event: %d\n", event->type);

    switch (event->type)
    {
    case BLE_GAP_EVENT_DISC:
        rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        if (rc != 0)
            return 0;

        // Print MAC address of discovered device
        ESP_LOGI(tag, "Device found: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->disc.addr.val[5],
                 event->disc.addr.val[4],
                 event->disc.addr.val[3],
                 event->disc.addr.val[2],
                 event->disc.addr.val[1],
                 event->disc.addr.val[0]);

        ble_connect(&event->disc);
        break;

    case BLE_GAP_EVENT_CONNECT:
    {
        if (event->connect.status != 0)
        {
            ESP_LOGE(tag, "Error: Connection failed; status=%d", event->connect.status);
            return 0;
        }

        ESP_LOGI(tag, "CONNECTION COMPLETE; status=%d; conn_handle=%d\n",
                 event->connect.status,
                 event->connect.conn_handle);

        /* Wait 500 ms to balance the connection */
        vTaskDelay(pdMS_TO_TICKS(500));

        break;
    }

    case BLE_GAP_EVENT_NOTIFY_RX:
    {
        if (!event->notify_rx.om)
        {
            ESP_LOGI(tag, "Notification received, but om is NULL!");
            break;
        }

        /* Attribute data is contained in event->notify_rx.om. Use
         * `os_mbuf_copydata` to copy the data received in notification mbuf */

        /* Get the received data */
        int len = OS_MBUF_PKTLEN(event->notify_rx.om);
        std::vector<uint8_t> data(len);
        os_mbuf_copydata(event->notify_rx.om, 0, len, data.data());

        /* Ignore if the data is the AT response */
        if (len == 4 && data[0] == 0x41 && data[1] == 0x54 && data[2] == 0x0D && data[3] == 0x0A)
        {
            break;
        }

        /* Peer sent us a notification or indication. */
        ESP_LOGI(tag, "Received %s; conn_handle=%d attr_handle=%d "
                      "attr_len=%d",
                 event->notify_rx.indication ? "indication" : "notification",
                 event->notify_rx.conn_handle,
                 event->notify_rx.attr_handle,
                 OS_MBUF_PKTLEN(event->notify_rx.om));
        ESP_LOG_BUFFER_HEX(tag, data.data(), len);
        break;
    }

    // case BLE_GAP_EVENT_MTU:
    //     MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
    //                 event->mtu.conn_handle,
    //                 event->mtu.channel_id,
    //                 event->mtu.value);
    //     return 0;

    default:
        break;
    }

    return 0;
}

/**
 * Determine if the device is the target device
 */
static int is_target_device(const struct ble_gap_disc_desc *disc, const std::string &peer_addr_str)
{
    struct ble_hs_adv_fields fields;
    int rc;

    /* The device has to be advertising connectability. */
    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND && disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND)
    {
        return 0;
    }

    rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
    if (rc != 0)
    {
        return 0;
    }

    uint8_t peer_addr[6] = {0};

    // Parse MAC address in reverse byte order (BLE style)
    int parsed = sscanf(peer_addr_str.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                        &peer_addr[5], &peer_addr[4], &peer_addr[3],
                        &peer_addr[2], &peer_addr[1], &peer_addr[0]);

    if (parsed != 6)
    {
        ESP_LOGE("BLE", "Invalid address format: %s", peer_addr_str.c_str());
        return 0;
    }

    // Compare parsed address with discovered device
    if (memcmp(peer_addr, disc->addr.val, sizeof(disc->addr.val)) != 0)
    {
        return 0;
    }

    return 1;
}

/**
 * BLE connect
 */
static void ble_connect(void *disc)
{
    uint8_t own_addr_type;
    int rc;
    ble_addr_t *addr;

    /* Don't do anything if we don't care about this advertiser. */
    if (!is_target_device((struct ble_gap_disc_desc *)disc, "c8:47:80:1f:05:b6"))
    {
        return;
    }

    /* Scanning must be stopped before a connection can be initiated. */
    rc = ble_gap_disc_cancel();
    if (rc != 0)
    {
        ESP_LOGE(tag, "Failed to cancel scan; rc=%d\n", rc);
        return;
    }

    /* Figure out address to use for connect (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(tag, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for
     * timeout.
     */
    addr = &((struct ble_gap_disc_desc *)disc)->addr;

    rc = ble_gap_connect(own_addr_type, addr, 30000, NULL,
                         blecent_gap_event, NULL);
    if (rc != 0)
    {
        ESP_LOGE(tag, "Error: Failed to connect to device; addr_type=%d; rc=%d\n", addr->type, rc);
        return;
    }
}

void ble_scan()
{
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Tell the controller to filter duplicates; we don't want to process
     * repeated advertisements from the same device.
     */
    disc_params.filter_duplicates = 1;

    /**
     * Perform a passive scan.  I.e., don't send follow-up scan requests to
     * each advertiser.
     */
    disc_params.passive = 1;

    /* Use defaults for the rest of the parameters. */
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                      blecent_gap_event, NULL);
    if (rc != 0)
    {
        MODLOG_DFLT(ERROR, "Error initiating GAP discovery procedure; rc=%d\n",
                    rc);
    }
}

static void blecent_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void blecent_on_sync(void)
{
    int rc;

    /* Make sure we have proper identity address set (public preferred) */
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

#if !CONFIG_EXAMPLE_INIT_DEINIT_LOOP
    /* Begin scanning for a peripheral to connect to. */
    ble_scan();
#endif
}

void blecent_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

uint8_t crc(const uint8_t data[], const uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        crc = crc + data[i];
    }
    return crc;
}

JkBmsBle::JkBmsBle()
{
    init_ble();
    // TODO Auto-generated constructor stub
}

JkBmsBle::~JkBmsBle()
{
    // TODO Auto-generated destructor stub
}

void JkBmsBle::init_ble()
{
    int rc;
    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(tag, "Failed to init nimble %d ", ret);
        return;
    }

    /* Configure the host. */
    ble_hs_cfg.reset_cb = blecent_on_reset;
    ble_hs_cfg.sync_cb = blecent_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("esp32");
    assert(rc == 0);

    nimble_port_freertos_init(blecent_host_task);
}

bool JkBmsBle::write_register(uint8_t address, uint32_t value, uint8_t length)
{
    uint8_t frame[20];
    frame[0] = 0xAA;    // start sequence
    frame[1] = 0x55;    // start sequence
    frame[2] = 0x90;    // start sequence
    frame[3] = 0xEB;    // start sequence
    frame[4] = address; // holding register
    frame[5] = length;  // size of the value in byte
    frame[6] = value >> 0;
    frame[7] = value >> 8;
    frame[8] = value >> 16;
    frame[9] = value >> 24;
    frame[10] = 0x00;
    frame[11] = 0x00;
    frame[12] = 0x00;
    frame[13] = 0x00;
    frame[14] = 0x00;
    frame[15] = 0x00;
    frame[16] = 0x00;
    frame[17] = 0x00;
    frame[18] = 0x00;
    frame[19] = crc(frame, sizeof(frame) - 1);

    // ESP_LOGI(tag, "Write register: %s", format_hex_pretty(frame, sizeof(frame)).c_str());
    // Send via NimBLE
    int rc = ble_gattc_write_flat(0, 18, frame, sizeof(frame), NULL, NULL);
    if (rc != 0)
    {
        ESP_LOGW(tag, "ble_gattc_write_flat failed; rc=%d", rc);
    }

    return rc == 0;
}