#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_littlefs.h"

#include "ble_manager.h"
#include "bms_data_decode.h"
#include "dashboard_server.h"
#include "wifi_manager.h"

// Global DashboardServer instance for use in WifiManager event handlers
DashboardServer dashboardServer;

/**
 * @brief Mount the LittleFS partition for web assets.
 * @return true if successful, false otherwise.
 */
bool mount_littlefs()
{
    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/web",
        .partition_label = "web",
        .partition = nullptr,
#ifdef CONFIG_LITTLEFS_SDMMC_SUPPORT
        .sdcard = nullptr,
#endif
        .format_if_mount_failed = false,
        .read_only = false,
        .dont_mount = false,
        .grow_on_mount = false,
    };

    esp_err_t err = esp_vfs_littlefs_register(&conf);
    if (err != ESP_OK)
    {
        ESP_LOGE("fs", "Failed to mount LittleFS (%s)", esp_err_to_name(err));
        return false;
    }

    FILE *f = fopen("/web/jk-bms_dashboard.html", "r");
    if (f)
    {
        ESP_LOGI("fs", "jk-bms_dashboard.html found.");
        fclose(f);
    }
    else
    {
        ESP_LOGE("fs", "jk-bms_dashboard.html missing.");
        return false;
    }

    return true;
}

/**
 * @brief Main application task.
 */
void my_task(void *pvParameters)
{
    // BmsDataDecode bmsParser;
    // BleManager bleManager(&bmsParser);
    // bleManager.initialize();

    while (true)
    {
        // keep the task alive
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void)
{
    // Init NVS (required before mounting FS or starting Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Mount LittleFS for web assets before starting Wi-Fi
    if (!mount_littlefs())
    {
        ESP_LOGE("main", "Filesystem mount failed. Exiting...");
        return;
    }

    // Initialize the Wi-Fi stack and event handlers
    WifiManager wifiManager;
    wifiManager.begin();

    // Start main application task
    xTaskCreatePinnedToCore(my_task, "esp32_jk-bms_ble_demonstration", 8192, nullptr, 1, nullptr, 1);
}