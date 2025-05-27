#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_littlefs.h"

#include "ble_manager.h"
#include "bms_data_decode.h"
#include "dashboard_server.h"

#define WIFI_SSID "Mitaelectric1"
#define WIFI_PASS "1234567890"

void connect_to_home_wifi()
{
    // Initialize NVS — required for WiFi
    ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize TCP/IP stack and default event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default WiFi STA interface
    esp_netif_create_default_wifi_sta();

    // Initialize WiFi with default config
    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    // Configure WiFi credentials
    wifi_config_t wifi_cfg = {};
    strncpy((char*)wifi_cfg.sta.ssid,     WIFI_SSID, sizeof(wifi_cfg.sta.ssid));
    strncpy((char*)wifi_cfg.sta.password, WIFI_PASS, sizeof(wifi_cfg.sta.password));

    // Set to station mode
    ESP_LOGI("wifi", "Setting WiFi mode to STA");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Apply configuration
    ESP_LOGI("wifi", "Configuring WiFi SSID:%s", WIFI_SSID);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));

    // Start WiFi
    ESP_LOGI("wifi", "Starting WiFi");
    ESP_ERROR_CHECK(esp_wifi_start());

    // Connect with stored credentials
    ESP_LOGI("wifi", "Connecting to WiFi...");
    ESP_ERROR_CHECK(esp_wifi_connect());
}

bool mount_littlefs()
{
    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/littlefs",
        .partition_label = "littlefs",
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

    FILE *f = fopen("/littlefs/jk-bms_dashboard.html", "r");
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

void my_task(void *pvParameters)
{
    // BmsDataDecode bmsParser;
    // BleManager bleManager(&bmsParser);
    // bleManager.initialize();

    DashboardServer dashboardServer;
    dashboardServer.start();

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

    // Connect to home Wi-Fi
    connect_to_home_wifi();

    // Mount LittleFS
    if (!mount_littlefs())
    {
        ESP_LOGE("main", "Filesystem mount failed. Exiting...");
        return;
    }

    xTaskCreatePinnedToCore(my_task, "esp32_jk-bms_ble_demonstration", 8192, nullptr, 1, nullptr, 1);
}