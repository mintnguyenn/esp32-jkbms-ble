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
    {
        if (event->connect.status != 0)
        {
            ESP_LOGE(tag, "CONNECTION FAILED; status=%d", event->connect.status);
            return 0;
        }
        ESP_LOGI(tag, "CONNECTION COMPLETED; conn_handle=%d", event->connect.conn_handle);

        // Initiate MTU exchange
        int rc = ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
        if (rc != 0)
        {
            ESP_LOGW(tag, "MTU exchange start failed; rc=%d", rc);
        }

        break;
    }

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
        // ESP_LOG_BUFFER_HEX(tag, data.data(), len);

        // Assemble the frame
        JkBmsBle::assemble(data.data(), len);

        break;
    }

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(tag, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                 event->mtu.conn_handle,
                 event->mtu.channel_id,
                 event->mtu.value);

        // Request Device info
        vTaskDelay(pdMS_TO_TICKS(500));
        JkBmsBle::write_register(COMMAND_DEVICE_INFO, 0x00000000, 0x00);
        vTaskDelay(pdMS_TO_TICKS(500));
        JkBmsBle::write_register(COMMAND_CELL_INFO, 0x00000000, 0x00);

        break;

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

    int rc = ble_gattc_write_no_rsp_flat(0, 0x12, frame, sizeof(frame));
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
    if (data == nullptr || length == 0)
        return;

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

        ESP_LOGI(tag, "Raw data frame (%d bytes) received", frame_buffer_.size());
        ESP_LOG_BUFFER_HEX(tag, frame_buffer_.data(), frame_buffer_.size());
        ESP_LOGI(tag, "\n");

        switch (frame_buffer_[4])
        {
        case 0x01:
            decode_device_settings_(frame_buffer_);
            break;

        case 0x02:
            decode_cell_info_(frame_buffer_);
            break;

        case 0x03:
            decode_device_info_(frame_buffer_);
            break;

        default:
            ESP_LOGW(tag, "Unknown command in response: 0x%02X", frame_buffer_[4]);
            break;
        }

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
    ESP_LOGI(tag, "  UART1 Protocol:     %d", data[184]);
    ESP_LOGI(tag, "  CAN Protocol:       %d", data[185]);
    ESP_LOGI(tag, "  UART2 Protocol:     %d", data[218]);
    ESP_LOGI(tag, "  RCV Time:           %.1f h", (float)data[266] * 0.1f);
    ESP_LOGI(tag, "  RFV Time:           %.1f h", (float)data[267] * 0.1f);
}

void JkBmsBle::decode_device_settings_(const std::vector<uint8_t> &data)
{
    auto jk_get_16bit = [&](size_t i) -> uint16_t
    { return (uint16_t(data[i + 1]) << 8) | (uint16_t(data[i + 0]) << 0); };
    auto jk_get_32bit = [&](size_t i) -> uint32_t
    {
        return (uint32_t(jk_get_16bit(i + 2)) << 16) | (uint32_t(jk_get_16bit(i + 0)) << 0);
    };

    ESP_LOGI(tag, "Device settings frame (%d bytes) received", data.size());
    // ESP_LOGVV(TAG, "  %s", format_hex_pretty(&data.front(), 160).c_str());
    // ESP_LOGVV(TAG, "  %s", format_hex_pretty(&data.front() + 160, data.size() - 160).c_str());
    ESP_LOGI(tag, "  Smart sleep voltage:                       %f", (float)jk_get_32bit(6) * 0.001f);
    ESP_LOGI(tag, "  Cell UVP:                                  %f V", (float)jk_get_32bit(10) * 0.001f);
    ESP_LOGI(tag, "  Cell UVPR:                                 %f V", (float)jk_get_32bit(14) * 0.001f);
    ESP_LOGI(tag, "  Cell OVP:                                  %f V", (float)jk_get_32bit(18) * 0.001f);
    ESP_LOGI(tag, "  Cell OVPR:                                 %f V", (float)jk_get_32bit(22) * 0.001f);
    ESP_LOGI(tag, "  Balance trigger voltage:                   %f V", (float)jk_get_32bit(26) * 0.001f);
    ESP_LOGI(tag, "  SOC 100%% voltage:                         %f V", (float)jk_get_32bit(30) * 0.001f);
    ESP_LOGI(tag, "  SOC 0%% voltage:                           %f V", (float)jk_get_32bit(34) * 0.001f);
    ESP_LOGI(tag, "  Voltage cell request charge voltage [RCV]: %f V", (float)jk_get_32bit(38) * 0.001f);
    ESP_LOGI(tag, "  Voltage cell request float voltage [RFV]:  %f V", (float)jk_get_32bit(42) * 0.001f);
    ESP_LOGI(tag, "  Power off voltage:                         %f V", (float)jk_get_32bit(46) * 0.001f);
    ESP_LOGI(tag, "  Max. charge current:                       %f A", (float)jk_get_32bit(50) * 0.001f);
    ESP_LOGI(tag, "  Charge OCP delay:                          %f s", (float)jk_get_32bit(54));
    ESP_LOGI(tag, "  Charge OCP recovery time:                  %f s", (float)jk_get_32bit(58));
    ESP_LOGI(tag, "  Max. discharge current:                    %f A", (float)jk_get_32bit(62) * 0.001f);
    ESP_LOGI(tag, "  Discharge OCP delay:                       %f s", (float)jk_get_32bit(66));
    ESP_LOGI(tag, "  Discharge OCP recovery time:               %f s", (float)jk_get_32bit(70));
    ESP_LOGI(tag, "  Short circuit protection recovery time:    %f s", (float)jk_get_32bit(74));
    ESP_LOGI(tag, "  Max. balance current:                      %f A", (float)jk_get_32bit(78) * 0.001f);
    ESP_LOGI(tag, "  Charge OTP:                                %f °C", (float)jk_get_32bit(82) * 0.1f);
    ESP_LOGI(tag, "  Charge OTP recovery:                       %f °C", (float)jk_get_32bit(86) * 0.1f);
    ESP_LOGI(tag, "  Discharge OTP:                             %f °C", (float)jk_get_32bit(90) * 0.1f);
    ESP_LOGI(tag, "  Discharge OTP recovery:                    %f °C", (float)jk_get_32bit(94) * 0.1f);
    ESP_LOGI(tag, "  Charge UTP:                                %f °C", (float)((int32_t)jk_get_32bit(98)) * 0.1f);
    ESP_LOGI(tag, "  Charge UTP recovery:                       %f °C", (float)((int32_t)jk_get_32bit(102)) * 0.1f);
    ESP_LOGI(tag, "  Mosfet OTP:                                %f °C", (float)((int32_t)jk_get_32bit(106)) * 0.1f);
    ESP_LOGI(tag, "  Mosfet OTP recovery:                       %f °C", (float)((int32_t)jk_get_32bit(110)) * 0.1f);
    ESP_LOGI(tag, "  Cell count:                                %f", (float)jk_get_32bit(114));
    ESP_LOGI(tag, "  Charge switch:     %s", (data[118] ? "ON" : "OFF"));
    ESP_LOGI(tag, "  Discharge switch:  %s", (data[122] ? "ON" : "OFF"));
    ESP_LOGI(tag, "  Balancer switch:   %s", (data[126] ? "ON" : "OFF"));
    ESP_LOGI(tag, "  Nominal battery capacity:                  %f Ah", (float)jk_get_32bit(130) * 0.001f);
    ESP_LOGI(tag, "  Short circuit protection delay:            %f us", (float)jk_get_32bit(134) * 1.0f);
    ESP_LOGI(tag, "  Start balance voltage:                     %f V", (float)jk_get_32bit(138) * 0.001f);

    for (uint8_t i = 0; i < 32; i++)
    {
        ESP_LOGI(tag, "  Con. wire resistance %d: %f Ohm", i + 1, (float)jk_get_32bit(i * 4 + 142) * 0.001f);
    }
    ESP_LOGI(tag, "  Device address:                            %d", data[270]);
    ESP_LOGI(tag, "  Precharge time:                            %d s", data[274]);
    ESP_LOGI(tag, "  Heating switch:     %s", check_bit_(data[282], 1) ? "ON" : "OFF");
    ESP_LOGI(tag, "  GPS Heartbeat:      %s", check_bit_(data[282], 4) ? "ON" : "OFF");
    ESP_LOGI(tag, "  Port switch:        %s", check_bit_(data[282], 8) ? "RS485" : "CAN");
    ESP_LOGI(tag, "  Special charger:    %s", check_bit_(data[282], 32) ? "ON" : "OFF");
    ESP_LOGI(tag, "  Smart sleep:                               %d h", data[286]);
    ESP_LOGI(tag, "  Data field enable control 0:               %d", data[287]);
}

void JkBmsBle::decode_cell_info_(const std::vector<uint8_t> &data)
{
    ESP_LOGI(tag, "Cell info frame (%d bytes) received", data.size());
    // TODO: Decode cell info
}