#include "ble_manager.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>

//--------------------------------------------------
// Helper Functions and Static Constants
//--------------------------------------------------

// Target device MAC address
static const uint8_t TARGET_DEVICE_ADDR[] = {0xB6, 0x05, 0x1F, 0x80, 0x47, 0xC8};

// Request command for JK-BMS
static const uint8_t COMMAND_CELL_INFO = 0x96;
static const uint8_t COMMAND_DEVICE_INFO = 0x97;

// Logging tag
static const char *m_tag = "Ble-Mgr";

/*
 * BLE callback for reset events.
 */
static void bleOnReset(int reason)
{
    ESP_LOGE(m_tag, "Resetting state; reason=%d", reason);
}

/*
 * BLE callback when the stack is synchronized.
 */
static void bleOnSync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    ESP_LOGI(m_tag, "BLE stack synchronized. Scanning can be executed now.");
}

/*
 * Simple CRC calculation for a block of data.
 */
uint8_t crc(const uint8_t data[], const uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++)
        crc += data[i];
    return crc;
}

//--------------------------------------------------
// BleManager Class Constructors/Destructor
//--------------------------------------------------

BleManager::BleManager() : m_dataProcessor_(nullptr) {}

BleManager::BleManager(IBmsDataProcessor *processor) : m_dataProcessor_(processor) {}

BleManager::~BleManager() {}

//--------------------------------------------------
// BleManager Class Implementation
//--------------------------------------------------

/*
 * Initialize the BLE stack.
 */
esp_err_t BleManager::initialize()
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
        ESP_LOGE(m_tag, "Failed to init NimBLE");
        return ESP_FAIL;
    }

    ble_hs_cfg.reset_cb = bleOnReset;
    ble_hs_cfg.sync_cb = bleOnSync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_svc_gap_device_name_set("esp32");
    nimble_port_freertos_init(BleManager::bleHostTask);
    return ESP_OK;
}

/*
 * Setter to assign the data processor used by BleManager.
 */
void BleManager::setDataProcessor(IBmsDataProcessor *processor)
{
    m_dataProcessor_ = processor;
}

/*
 * BLE scanning function. It starts scanning for devices.
 */
void BleManager::startScanning()
{
    uint8_t own_addr_type;
    if (ble_hs_id_infer_auto(0, &own_addr_type) != 0)
    {
        ESP_LOGE(m_tag, "Error determining address type");
        return;
    }

    struct ble_gap_disc_params disc_params = {
        .itvl = 0,
        .window = 0,
        .filter_policy = 0,
        .limited = 0,
        .passive = 1,
        .filter_duplicates = 1};

    int rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, BleManager::eventHandler, this);
    if (rc != 0)
    {
        ESP_LOGE(m_tag, "Failed to start scanning; rc=%d", rc);
    }
}

/*
 * Static event handler for BLE events.
 * Forwards the event to the instance's handleEvent() method.
 */
int BleManager::eventHandler(struct ble_gap_event *event, void *arg)
{
    BleManager *self = static_cast<BleManager *>(arg);
    if (self)
    {
        return self->handleEvent(event);
    }
    return 0;
}

/*
 * Processes BLE events.
 * Handles discovery, connection, notifications, and MTU updates.
 */
int BleManager::handleEvent(struct ble_gap_event *event)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_DISC:
    {
        ESP_LOGI(m_tag, "Device found: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->disc.addr.val[5], event->disc.addr.val[4], event->disc.addr.val[3],
                 event->disc.addr.val[2], event->disc.addr.val[1], event->disc.addr.val[0]);
        // Check if discovered device matches target address.
        if (memcmp(event->disc.addr.val, TARGET_DEVICE_ADDR, sizeof(TARGET_DEVICE_ADDR)) == 0)
        {
            if (ble_gap_disc_cancel() != 0)
            {
                ESP_LOGE(m_tag, "Failed to cancel scan");
                return 0;
            }

            uint8_t own_addr_type;
            if (ble_hs_id_infer_auto(0, &own_addr_type) != 0)
            {
                ESP_LOGE(m_tag, "Error determining address type");
                return 0;
            }
            int rc = ble_gap_connect(own_addr_type, &event->disc.addr, 30000, NULL, BleManager::eventHandler, this);
            if (rc != 0)
            {
                ESP_LOGE(m_tag, "Connection failed; rc=%d", rc);
            }
        }
        break;
    }

    case BLE_GAP_EVENT_CONNECT:
    {
        if (event->connect.status != 0)
        {
            ESP_LOGE(m_tag, "CONNECTION FAILED; status=%d", event->connect.status);
            return 0;
        }
        ESP_LOGI(m_tag, "CONNECTION COMPLETED; conn_handle=%d", event->connect.conn_handle);
        int rc = ble_gattc_exchange_mtu(event->connect.conn_handle, nullptr, nullptr);
        if (rc != 0)
        {
            ESP_LOGW(m_tag, "MTU exchange failed; rc=%d", rc);
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

        // Uncomment the next lines for debugging:
        // ESP_LOGI(m_tag, "Notification received; conn_handle=%d, attr_handle=%d, len=%d",
        //          event->notify_rx.conn_handle, event->notify_rx.attr_handle, len);
        // ESP_LOG_BUFFER_HEX("tag", data.data(), len);

        // Process the received data if a processor is available
        if (m_dataProcessor_ != nullptr)
        {
            m_dataProcessor_->processData(data, len);
        }

        break;
    }

    case BLE_GAP_EVENT_MTU:
    {
        ESP_LOGI(m_tag, "MTU update event; conn_handle=%d, cid=%d, mtu=%d",
                 event->mtu.conn_handle, event->mtu.channel_id, event->mtu.value);

        // After MTU update, the JK-BMS now can send a big data frame.
        // We can now send a command to the device to get the device info.
        write_register(COMMAND_DEVICE_INFO, 0x00000000, 0x00);
        vTaskDelay(pdMS_TO_TICKS(500));
        write_register(COMMAND_CELL_INFO, 0x00000000, 0x00);
        vTaskDelay(pdMS_TO_TICKS(500));
        break;
    }

    default:
        break;
    }
    return 0;
}

/*
 * Sends a write command to the device.
 * Constructs a frame with a header, command, and CRC, then writes it via GATT.
 */
bool BleManager::write_register(uint8_t address, uint32_t value, uint8_t length)
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
        ESP_LOGW(m_tag, "write_register failed; rc=%d", rc);
    }
    return rc == 0;
}

/*
 * BLE Host Task: this task runs the NimBLE host, processing BLE events continuously.
 */
void BleManager::bleHostTask(void *param)
{
    ESP_LOGI(m_tag, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}
