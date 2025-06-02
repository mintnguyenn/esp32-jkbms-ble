#include "dashboard_server.h"
#include "esp_log.h"
#include <cstdio>

static const char *TAG = "DashboardServer";
static const char *INDEX_PATH = "/web/jk-bms_dashboard.html";

DashboardServer::DashboardServer() = default;

DashboardServer::~DashboardServer()
{
    stop();
}

esp_err_t DashboardServer::start()
{
    if (server)
    {
        ESP_LOGI(TAG, "Server already running");
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    // Register “/” → dashboard.html
    httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = handleRoot,
        .user_ctx = nullptr};
    httpd_register_uri_handler(server, &root);

    // Register POST /api/start-scan
    httpd_uri_t connect = {
        .uri = "/api/connect",
        .method = HTTP_POST,
        .handler = handleConnect,
        .user_ctx = nullptr};
    httpd_register_uri_handler(server, &connect);

    // Register POST /api/stop-scan
    httpd_uri_t disconnect = {
        .uri = "/api/disconnect",
        .method = HTTP_POST,
        .handler = handleDisconnect,
        .user_ctx = nullptr};
    httpd_register_uri_handler(server, &disconnect);

    // Register GET /events for SSE
    httpd_uri_t events = {
        .uri = "/events",
        .method = HTTP_GET,
        .handler = handleEvents,
        .user_ctx = nullptr};
    httpd_register_uri_handler(server, &events);

    ESP_LOGI(TAG, "HTTP server started");
    return ESP_OK;
}

void DashboardServer::stop()
{
    if (server)
    {
        httpd_stop(server);
        server = nullptr;
        ESP_LOGI(TAG, "HTTP server stopped");
    }
}

esp_err_t DashboardServer::handleRoot(httpd_req_t *req)
{
    // Serve the main HTML page
    return serveStaticFile(req, INDEX_PATH, "text/html");
}

esp_err_t DashboardServer::handleConnect(httpd_req_t *req)
{
    // Read the request body
    int total_len = req->content_len;
    if (total_len <= 0)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing request body");
    }
    if (total_len > 1024)
    {
        // Arbitrary limit to prevent abuse
        return httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, "Request body too large");
    }

    // Allocate buffer for the request body
    char *buf = (char *)malloc(total_len + 1);
    if (!buf)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ESP32 is running out of memory");
    }
    int ret = httpd_req_recv(req, buf, total_len);
    if (ret <= 0)
    {
        free(buf);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read request body");
    }
    buf[ret] = '\0'; // Null-terminate the string

    // Parse JSON
    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    }
    cJSON *macItem = cJSON_GetObjectItem(root, "mac");
    if (!macItem || !cJSON_IsString(macItem))
    {
        cJSON_Delete(root);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid 'mac'");
    }
    std::string deviceMac = macItem->valuestring;
    cJSON_Delete(root);

    ESP_LOGI(TAG, "MAC address received from user: %s", deviceMac.c_str());

    // Attempt BLE connection

    // Build JSON response
    std::string deviceName = "test-device"; // Placeholder, replace with actual device name
    cJSON *res = cJSON_CreateObject();
    cJSON_AddStringToObject(res, "name", deviceName.c_str());
    cJSON_AddStringToObject(res, "mac", deviceMac.c_str());
    char *resp_str = cJSON_PrintUnformatted(res);
    cJSON_Delete(res);

    // Send JSON response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp_str);
    free(resp_str);

    return ESP_OK;
}

esp_err_t DashboardServer::handleDisconnect(httpd_req_t *req)
{
    // Disconnect device
    // TBD
    return ESP_OK;
}

esp_err_t DashboardServer::handleEvents(httpd_req_t *req)
{
    return ESP_OK;
}

esp_err_t DashboardServer::serveStaticFile(httpd_req_t *req, const char *path, const char *content_type)
{
    FILE *f = fopen(path, "r");
    if (!f)
    {
        ESP_LOGE(TAG, "Failed to open file: %s", path);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, content_type);

    char buffer[256];
    size_t read_bytes;
    while ((read_bytes = fread(buffer, 1, sizeof(buffer), f)) > 0)
    {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    // Signal end of chunked response
    httpd_resp_send_chunk(req, nullptr, 0);
    fclose(f);
    return ESP_OK;
}

esp_err_t DashboardServer::sendEvent(const std::string &message)
{
    // Stub: implement your queue + notification logic here
    return ESP_OK;
}
