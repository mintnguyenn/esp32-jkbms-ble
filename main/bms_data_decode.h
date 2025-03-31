#ifndef BMS_DATA_PARSER_H
#define BMS_DATA_PARSER_H

#include "esp_log.h"
#include <vector>
#include <cstdint>
#include <string>
#include <cstring>
#include "ble_manager.h"  // To implement IBmsDataProcessor

class BmsDataDecode : public IBmsDataProcessor {
public:
    BmsDataDecode();
    ~BmsDataDecode();

    // IBmsDataProcessor override: process incoming raw BLE data.
    void processData(const std::vector<uint8_t> data, uint16_t length);

private:
    void assemble(const uint8_t *data, uint16_t length);
    void decodeFrame(const std::vector<uint8_t>& frame);
    void decodeDeviceInfo(const std::vector<uint8_t>& data);
    void decodeDeviceSettings(const std::vector<uint8_t>& data);
    void decodeCellInfo(const std::vector<uint8_t>& data);

    uint8_t calculateCRC(const uint8_t data[], uint16_t len);

    std::vector<uint8_t> m_frameBuffer;
    const char* m_tag = "BmsDataDecode";

    static const uint16_t MIN_RESPONSE_SIZE = 300;
    static const uint16_t MAX_RESPONSE_SIZE = 330;
};

#endif // BMS_DATA_PARSER_H
