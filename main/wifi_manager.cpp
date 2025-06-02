#include "wifi_manager.h"
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_littlefs.h>
#include <esp_http_server.h>
#include <esp_err.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>

static const char *TAG = "WifiManager";
constexpr int WIFI_CONNECT_RETRIES = 5;
constexpr char DEFAULT_AP_SSID[] = "JK-BMS_AP";
constexpr int MAX_SSID_LENGTH = 32;
constexpr int MAX_PASS_LENGTH = 64;
constexpr char DEFAULT_AP_IP[] = "192.168.4.1";

//
static httpd_handle_t server = nullptr;

static esp_err_t root_get_handler(httpd_req_t *req) {
    const char *html = R"rawliteral(
        <html><body>
        <h3>Configure Wi-Fi</h3>
        <form method="POST" action="/set">
            SSID: <input name="ssid"><br>
            PASS: <input name="password"><br>
            <input type="submit" value="Save">
        </form>
        </body></html>
    )rawliteral";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t set_post_handler(httpd_req_t *req) {
    char content[128] = {};
    httpd_req_recv(req, content, sizeof(content) - 1);

    char ssid[64] = {}, password[64] = {};
    sscanf(content, "ssid=%63[^&]&password=%63s", ssid, password);  // crude parsing

    WifiManager wm;
    wm.saveCredentials(ssid, password);

    const char *resp = "Saved. Rebooting...";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

void start_config_web_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_start(&server, &config);

    httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &root);

    httpd_uri_t set = {
        .uri = "/set",
        .method = HTTP_POST,
        .handler = set_post_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &set);
}


WifiManager::WifiManager() = default;
WifiManager::~WifiManager() = default;

void WifiManager::begin()
{
    // Mount LittleFS
    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/config",
        .partition_label = "config",
        .partition = nullptr,
#ifdef CONFIG_LITTLEFS_SDMMC_SUPPORT
        .sdcard = nullptr,
#endif
        .format_if_mount_failed = true,
        .read_only = false,
        .dont_mount = false,
        .grow_on_mount = false,
    };
    esp_err_t err = esp_vfs_littlefs_register(&conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount LittleFS (%s)", esp_err_to_name(err));
    }

    // Initialize TCP/IP stack and event loop
    if (esp_netif_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_netif_init failed");
        return;
    }
    if (esp_event_loop_create_default() != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed");
        return;
    }
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    // Initialize Wi-Fi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&cfg) != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_init failed");
        return;
    }
    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_set_mode STA failed");
        return;
    }
    if (esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_set_storage failed");
        return;
    }
    if (esp_wifi_start() != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_start failed");
        return;
    }

    // Try to connect from stored credentials
    if (!connectWithStoredCredentials())
    {
        ESP_LOGI(TAG, "Starting AP mode for configuration");
        startApMode();
    }
}

bool WifiManager::connectWithStoredCredentials()
{
    if (!fileExists(configPath))
    {
        ESP_LOGI(TAG, "Config file not found");
        return false;
    }
    std::string ssid, pass;
    readCredentials(ssid, pass);
    return wifiConnectWithRetry();
}

bool WifiManager::saveCredentials(const std::string &ssid, const std::string &password)
{
    FILE *f = fopen(configPath.c_str(), "w");
    if (!f)
    {
        ESP_LOGE(TAG, "Failed to open config file for writing");
        return false;
    }
    int ret = fprintf(f, "{\"ssid\":\"%s\",\"password\":\"%s\"}", ssid.c_str(), password.c_str());
    fclose(f);
    if (ret < 0)
    {
        ESP_LOGE(TAG, "Failed to write credentials to file");
        return false;
    }
    ESP_LOGI(TAG, "Credentials saved: SSID:%s; Password: %s", ssid.c_str(), password.c_str());
    return true;
}

void WifiManager::resetCredentials()
{
    if (unlink(configPath.c_str()) == 0)
    {
        ESP_LOGI(TAG, "Config file deleted");
    }
    else
    {
        ESP_LOGW(TAG, "Config file not found or failed to delete");
    }
}

void WifiManager::startApMode()
{
    // Stop STA
    esp_wifi_stop();
    esp_wifi_set_mode(WIFI_MODE_AP);

    wifi_config_t ap_config = {};
    strncpy((char *)ap_config.ap.ssid, DEFAULT_AP_SSID, sizeof(ap_config.ap.ssid) - 1);
    ap_config.ap.ssid_len = 0;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_OPEN;
    esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    esp_wifi_start();
    ESP_LOGI(TAG, "AP Mode started: SSID=%s", ap_config.ap.ssid);
    // TODO: Start captive portal web server here

    start_config_web_server();
}

void WifiManager::stopApMode()
{
    esp_wifi_stop();
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    ESP_LOGI(TAG, "Stopped AP, back to STA mode");
}

bool WifiManager::isConnected() const
{
    wifi_ap_record_t ap_info;
    return esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK;
}

std::string WifiManager::getIp() const
{
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK)
    {
        char buf[16];
        esp_ip4addr_ntoa(&ip_info.ip, buf, sizeof(buf));
        return std::string(buf);
    }
    // Default AP IP
    return std::string(DEFAULT_AP_IP);
}

bool WifiManager::connectToWifi(const std::string &ssid, const std::string &password)
{
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, ssid.c_str(), sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password.c_str(), sizeof(wifi_config.sta.password));
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
        return false;
    }
    else
    {
        ESP_LOGI(TAG, "Connecting to SSID=%s successfully", ssid.c_str());
    }
    return true;
}

bool WifiManager::fileExists(const std::string &path) const
{
    struct stat st;
    return stat(path.c_str(), &st) == 0;
}

bool WifiManager::readCredentials(std::string &ssid, std::string &password) const
{
    FILE *f = fopen(configPath.c_str(), "r");
    if (!f)
    {
        ESP_LOGE(TAG, "Failed to open config file for reading");
        return false;
    }
    char buf[256] = {0};
    size_t len = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    if (len == 0)
    {
        ESP_LOGE(TAG, "Config file is empty");
        return false;
    }
    char ssid_buf[MAX_SSID_LENGTH] = {0};
    char pass_buf[MAX_PASS_LENGTH] = {0};
    int matched = sscanf(buf, "{\"ssid\":\"%31[^\"]\",\"password\":\"%63[^\"]\"}", ssid_buf, pass_buf);
    if (matched != 2)
    {
        ESP_LOGE(TAG, "Failed to parse credentials");
        return false;
    }
    ssid = std::string(ssid_buf);
    password = std::string(pass_buf);
    ESP_LOGI(TAG, "Read credentials SSID=%s", ssid.c_str());
    return true;
}

bool WifiManager::wifiConnectWithRetry(int maxRetries)
{
    std::string ssid, pass;
    readCredentials(ssid, pass);
    int retry = 0;
    while (retry < maxRetries)
    {
        if (connectToWifi(ssid, pass))
        {
            vTaskDelay(pdMS_TO_TICKS(5000)); // wait to connect
            if (isConnected())
            {
                ESP_LOGI(TAG, "Connected to SSID=%s", ssid.c_str());
                return true;
            }
        }
        retry++;
        ESP_LOGW(TAG, "Retrying Wi-Fi connection (%d/%d)", retry, maxRetries);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    ESP_LOGE(TAG, "Failed to connect after %d retries", maxRetries);
    return false;
}
