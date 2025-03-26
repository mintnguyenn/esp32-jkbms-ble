#include "jk_bms_ble.h"

static const char *tag = "ESP32-BMS";
static const uint8_t COMMAND_CELL_INFO = 0x96;
static const uint8_t COMMAND_DEVICE_INFO = 0x97;

static const uint16_t MIN_RESPONSE_SIZE = 300;
static const uint16_t MAX_RESPONSE_SIZE = 330;

std::vector<uint8_t> JkBmsBle::frame_buffer_;

// -----------------------------------------------------------------------------
// BLE Lifecycle
// -----------------------------------------------------------------------------

static void ble_on_reset(int reason)
{
    ESP_LOGE(tag, "Resetting state; reason=%d", reason);
}

static void ble_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    // Start scanning after BLE stack is ready
    JkBmsBle::ble_start_scanning();
}

void ble_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// -----------------------------------------------------------------------------
// Scanning / Connecting
// -----------------------------------------------------------------------------

static int is_target_device(const struct ble_gap_disc_desc *disc, const std::string &peer_addr_str)
{
    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
        disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND)
        return 0;

    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data) != 0)
        return 0;

    uint8_t peer_addr[6];
    if (sscanf(peer_addr_str.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
               &peer_addr[5], &peer_addr[4], &peer_addr[3],
               &peer_addr[2], &peer_addr[1], &peer_addr[0]) != 6)
    {
        ESP_LOGE(tag, "Invalid address format: %s", peer_addr_str.c_str());
        return 0;
    }
    return memcmp(peer_addr, disc->addr.val, sizeof(disc->addr.val)) == 0;
}

static void ble_connect(void *disc)
{
    if (!is_target_device((struct ble_gap_disc_desc *)disc, "c8:47:80:1f:05:b6"))
        return;

    if (ble_gap_disc_cancel() != 0)
    {
        ESP_LOGE(tag, "Failed to cancel scan");
        return;
    }

    uint8_t own_addr_type;
    if (ble_hs_id_infer_auto(0, &own_addr_type) != 0)
    {
        ESP_LOGE(tag, "Error determining address type");
        return;
    }

    ble_addr_t *addr = &((struct ble_gap_disc_desc *)disc)->addr;
    int rc = ble_gap_connect(own_addr_type, addr, 30000, NULL, ble_gap_event_handler, NULL);
    if (rc != 0)
    {
        ESP_LOGE(tag, "Connection failed; rc=%d", rc);
    }
}

void JkBmsBle::ble_start_scanning()
{
    uint8_t own_addr_type;
    if (ble_hs_id_infer_auto(0, &own_addr_type) != 0)
    {
        ESP_LOGE(tag, "Error determining address type");
        return;
    }

    struct ble_gap_disc_params disc_params = {
        .itvl = 0,
        .window = 0,
        .filter_policy = 0,
        .limited = 0,
        .passive = 1,
        .filter_duplicates = 1};

    int rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event_handler, NULL);
    if (rc != 0)
    {
        ESP_LOGE(tag, "Failed to start scanning; rc=%d", rc);
    }
}

// -----------------------------------------------------------------------------
// GAP Event Handling
// -----------------------------------------------------------------------------

int ble_gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_DISC:
        ESP_LOGI(tag, "Device found: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->disc.addr.val[5], event->disc.addr.val[4], event->disc.addr.val[3],
                 event->disc.addr.val[2], event->disc.addr.val[1], event->disc.addr.val[0]);
        ble_connect(&event->disc);
        break;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0)
        {
            ESP_LOGE(tag, "Connection failed; status=%d", event->connect.status);
            return 0;
        }
        ESP_LOGI(tag, "CONNECTION COMPLETE; conn_handle=%d", event->connect.conn_handle);

        // Delay before sending the first command
        vTaskDelay(pdMS_TO_TICKS(500));

        // Request Device info
        JkBmsBle::write_register(COMMAND_DEVICE_INFO, 0x00000000, 0x00);
        break;

    case BLE_GAP_EVENT_NOTIFY_RX:
    {
        if (!event->notify_rx.om)
            break;

        int len = OS_MBUF_PKTLEN(event->notify_rx.om);
        std::vector<uint8_t> data(len);
        os_mbuf_copydata(event->notify_rx.om, 0, len, data.data());
        if (len == 4 && memcmp(data.data(), "AT\r\n", 4) == 0)
            break;
        ESP_LOGI(tag, "Notification received; conn_handle=%d, attr_handle=%d, len=%d",
                 event->notify_rx.conn_handle, event->notify_rx.attr_handle, len);
        ESP_LOG_BUFFER_HEX(tag, data.data(), len);

        // Assemble the frame
        JkBmsBle::assemble(data.data(), len);
        break;
    }

    default:
        break;
    }
    return 0;
}

// -----------------------------------------------------------------------------
// CRC & BLE Utilities
// -----------------------------------------------------------------------------

uint8_t crc(const uint8_t data[], const uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++)
        crc += data[i];
    return crc;
}

bool JkBmsBle::write_register(uint8_t address, uint32_t value, uint8_t length)
{
    uint8_t frame[20] = {
        0xAA, 0x55, 0x90, 0xEB,
        address,
        length,
        (uint8_t)(value >> 0),
        (uint8_t)(value >> 8),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 24),
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00};

    frame[19] = crc(frame, sizeof(frame) - 1);
    int rc = ble_gattc_write_flat(0, 18, frame, sizeof(frame), NULL, NULL);
    if (rc != 0)
    {
        ESP_LOGW(tag, "write_register failed; rc=%d", rc);
    }
    return rc == 0;
}

JkBmsBle::JkBmsBle()
{
    ble_initialize();
}

JkBmsBle::~JkBmsBle() {}

void JkBmsBle::ble_initialize()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (nimble_port_init() != ESP_OK)
    {
        ESP_LOGE(tag, "Failed to init NimBLE");
        return;
    }

    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_svc_gap_device_name_set("esp32");
    nimble_port_freertos_init(ble_host_task);
}

void JkBmsBle::assemble(const uint8_t *data, uint16_t length)
{
    if (frame_buffer_.size() > MAX_RESPONSE_SIZE)
    {
        ESP_LOGW(tag, "Frame dropped because of invalid length");
        frame_buffer_.clear();
    }

    int offset = 0;
    // Detect and skip "AT\r\n" junk at the start
    if (data[0] == 0x41 && data[1] == 0x54 && data[2] == 0x0D && data[3] == 0x0A)
    {
        offset = 4;
        ESP_LOGW(tag, "Skipping 'AT\\r\\n' junk");
    }

    // Flush buffer on every preamble
    if (data[0 + offset] == 0x55 && data[1 + offset] == 0xAA && data[2 + offset] == 0xEB && data[3 + offset] == 0x90)
    {
        frame_buffer_.clear();
    }

    // Append data to the buffer
    frame_buffer_.insert(frame_buffer_.end(), data + offset, data + length);

    if (frame_buffer_.size() >= MIN_RESPONSE_SIZE)
    {
        const uint8_t *raw = &frame_buffer_[0];
        const uint16_t frame_size = 300;

        uint8_t computed_crc = crc(raw, frame_size - 1);
        uint8_t remote_crc = raw[frame_size - 1];

        if (computed_crc != remote_crc)
        {
            ESP_LOGW(tag, "CRC check failed! 0x%02X != 0x%02X", computed_crc, remote_crc);
            frame_buffer_.clear();
            return;
        }

        std::vector<uint8_t> full_data(frame_buffer_.begin(), frame_buffer_.end());
        ESP_LOGI(tag, "Raw data frame (%d bytes) received", full_data.size());
        ESP_LOG_BUFFER_HEX(tag, full_data.data(), full_data.size());
        ESP_LOGI(tag, "\n");

        JkBmsBle::decode_device_info_(full_data);
        frame_buffer_.clear();
    }
}

void JkBmsBle::decode_device_info_(const std::vector<uint8_t> &data)
{
    auto jk_get_16bit = [&](size_t i) -> uint16_t
    { return (uint16_t(data[i + 1]) << 8) | (uint16_t(data[i + 0]) << 0); };
    auto jk_get_32bit = [&](size_t i) -> uint32_t
    {
        return (uint32_t(jk_get_16bit(i + 2)) << 16) | (uint32_t(jk_get_16bit(i + 0)) << 0);
    };

    ESP_LOGI(tag, "Device info frame (%d bytes) received", data.size());
    // ESP_LOGVV(tag, "  %s", format_hex_pretty(&data.front(), 160).c_str());
    // ESP_LOGVV(tag, "  %s", format_hex_pretty(&data.front() + 160, data.size() - 160).c_str());
    ESP_LOGI(tag, "  Vendor ID:          %s", std::string(data.begin() + 6, data.begin() + 6 + 16).c_str());
    ESP_LOGI(tag, "  Hardware version:   %s", std::string(data.begin() + 22, data.begin() + 22 + 8).c_str());
    ESP_LOGI(tag, "  Software version:   %s", std::string(data.begin() + 30, data.begin() + 30 + 8).c_str());
    ESP_LOGI(tag, "  Uptime:             %lus", (unsigned long)jk_get_32bit(38));
    ESP_LOGI(tag, "  Power on count:     %lu", (unsigned long)jk_get_32bit(42));
    ESP_LOGI(tag, "  Device name:        %s", std::string(data.begin() + 46, data.begin() + 46 + 16).c_str());
    ESP_LOGI(tag, "  Device passcode:    %s", std::string(data.begin() + 62, data.begin() + 62 + 16).c_str());
    ESP_LOGI(tag, "  Manufacturing date: %s", std::string(data.begin() + 78, data.begin() + 78 + 8).c_str());
    ESP_LOGI(tag, "  Serial number:      %s", std::string(data.begin() + 86, data.begin() + 86 + 11).c_str());
    ESP_LOGI(tag, "  Passcode:           %s", std::string(data.begin() + 97, data.begin() + 97 + 5).c_str());
    ESP_LOGI(tag, "  User data:          %s", std::string(data.begin() + 102, data.begin() + 102 + 16).c_str());
    ESP_LOGI(tag, "  Setup passcode:     %s", std::string(data.begin() + 118, data.begin() + 118 + 16).c_str());
    // ESP_LOGI(TAG, "  UART1 Protocol: %d", data[184]);
    // ESP_LOGI(TAG, "  CAN Protocol: %d", data[185]);
    // ESP_LOGI(TAG, "  UART2 Protocol: %d", data[218]);
    // ESP_LOGI(TAG, "  RCV Time: %.1f h", (float) data[266] * 0.1f);
    // ESP_LOGI(TAG, "  RFV Time: %.1f h", (float) data[267] * 0.1f);
}