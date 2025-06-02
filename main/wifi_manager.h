#pragma once

#include <string>

#include <esp_event.h>

/**
 * @brief WifiManager handles Wi-Fi connection, configuration, and AP-mode setup for ESP32.
 *
 * This class manages Wi-Fi credentials using LittleFS, supports captive portal configuration,
 * and provides methods to switch between Station and Access Point modes.
 */
class WifiManager
{
public:
    /**
     * @brief Construct a new WifiManager object.
     */
    WifiManager();

    /**
     * @brief Destroy the WifiManager object.
     */
    ~WifiManager();

    /**
     * @brief Initialize LittleFS, set up ESP event handlers, and attempt Wi-Fi connection.
     *
     * If stored credentials are found, attempts to connect as a station.
     * If connection fails, starts AP mode for configuration.
     */
    void begin();

    /**
     * @brief Attempt to connect using credentials stored in wifi.json.
     * @return true if connection is successful, false otherwise.
     */
    bool connectWithStoredCredentials();

    /**
     * @brief Save user-provided SSID and password (from AP mode) to wifi.json.
     * @param ssid The Wi-Fi SSID.
     * @param password The Wi-Fi password.
     * @return true if credentials are saved successfully, false otherwise.
     */
    bool saveCredentials(const std::string &ssid, const std::string &password);

    /**
     * @brief Remove stored credentials (delete wifi.json).
     */
    void resetCredentials();

    /**
     * @brief Start Access Point mode with captive portal for Wifi configuration.
     */
    void startApMode();

    /**
     * @brief Stop Access Point mode and return to Station mode.
     */
    void stopApMode();

    /**
     * @brief Check if currently connected to a Wi-Fi network.
     * @return true if connected, false otherwise.
     */
    bool isConnected() const;

    /**
     * @brief Retrieve the current IP address as a string (e.g., "192.168.4.1").
     * @return The current IP address.
     */
    std::string getIp() const;

    /**
     * @brief Static event handler for when the station gets an IP address.
     * @param arg Pointer to user data (typically WifiManager instance).
     * @param event_base Event base.
     * @param event_id Event ID.
     * @param event_data Pointer to event-specific data.
     */
    static void onGotIp(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

    /**
     * @brief Static event handler for when the station is disconnected.
     * @param arg Pointer to user data (typically WifiManager instance).
     * @param event_base Event base.
     * @param event_id Event ID.
     * @param event_data Pointer to event-specific data.
     */
    static void onDisconnected(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

private:
    /**
     * @brief Connect to Wi-Fi using the provided SSID and password.
     * @param ssid The Wi-Fi SSID.
     * @param password The Wi-Fi password.
     * @return true if connection is initiated successfully, false otherwise.
     */
    bool connectToWifi(const std::string &ssid, const std::string &password);

    /**
     * @brief Check if a file exists at the given path.
     * @param path The file path.
     * @return true if the file exists, false otherwise.
     */
    bool fileExists(const std::string &path) const;

    /**
     * @brief Read SSID and password from wifi.json.
     * @param ssid Output parameter for the SSID.
     * @param password Output parameter for the password.
     * @return true if credentials are read successfully, false otherwise.
     */
    bool readCredentials(std::string &ssid, std::string &password) const;

    /**
     * @brief Attempt connection with retry logic.
     * @param maxRetries Maximum number of connection attempts (default: 5).
     * @return true if connection is successful, false otherwise.
     */
    bool wifiConnectWithRetry(int maxRetries = 5);

    /**
     * @brief Path in LittleFS where credentials are stored.
     */
    const std::string configPath = "/config/wifi.json";
};
