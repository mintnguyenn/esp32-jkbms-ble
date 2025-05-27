#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include <string>

class DashboardServer
{
public:
    DashboardServer();
    ~DashboardServer();

    /**
     * Start the HTTP server and register all URI handlers.
     * Must be called after filesystem (LittleFS) is mounted.
     */
    esp_err_t start();

    /** Stop the HTTP server (if running). */
    void stop();

    /**
     * (Optional) Send a stringified JSON message to all connected SSE clients.
     * You can implement an internal queue so handleEvents() will pick it up.
     */
    esp_err_t sendEvent(const std::string &message);

private:
    httpd_handle_t server = nullptr;

    // URI handlers:
    static esp_err_t handleRoot(httpd_req_t *req);       // Serves dashboard.html
    static esp_err_t handleConnect(httpd_req_t *req);    // POST /api/start-scan
    static esp_err_t handleDisconnect(httpd_req_t *req); // POST /api/stop-scan
    static esp_err_t handleEvents(httpd_req_t *req);     // GET /events (SSE)

    /**
     * Utility: read a file from LittleFS and stream it back.
     * @param req           HTTP request handle
     * @param path          Full file path, e.g. "/littlefs/dashboard.html"
     * @param content_type  MIME type, e.g. "text/html"
     */
    static esp_err_t serveStaticFile(httpd_req_t *req,
                                     const char *path,
                                     const char *content_type);
};
