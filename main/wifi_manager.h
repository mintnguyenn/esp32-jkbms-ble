#pragma once

#include <string>

// WifiManager handles Wi-Fi connection, configuration, and AP-mode setup
class WifiManager {
public:
    WifiManager();
    ~WifiManager();

    // Initialize LittleFS, set up ESP event handlers, and attempt connection
    void begin();

    // Attempt to connect using credentials stored in wifi.json
    bool connectWithStoredCredentials();

    // Save user-provided SSID and password to LittleFS (wifi.json)
    bool saveCredentials(const std::string& ssid, const std::string& password);

    // Remove stored credentials (delete wifi.json)
    void resetCredentials();

    // Start Access Point mode with captive portal for configuration
    void startApMode();

    // Stop Access Point mode and return to Station mode
    void stopApMode();

    // Check if currently connected to a Wi-Fi network
    bool isConnected() const;

    // Retrieve the current IP address (e.g., "192.168.4.1")
    std::string getIp() const;

private:
    // Low-level routine to connect to Wi-Fi given SSID/password
    bool connectToWifi(const std::string& ssid, const std::string& password);

    // Check if a file exists at the given path
    bool fileExists(const std::string& path) const;

    // Read SSID and password from wifi.json
    bool readCredentials(std::string& ssid, std::string& password) const;

    // Attempt connection with retry logic (returns true on success)
    bool wifiConnectWithRetry(int maxRetries = 5);

    // Path in LittleFS where credentials are stored
    const std::string configPath = "/config/wifi.json";
};
