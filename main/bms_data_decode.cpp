#include "bms_data_decode.h"

//--------------------------------------------------
// Utility Functions
//--------------------------------------------------
static bool check_bit(uint8_t mask, uint8_t flag) { return (mask & flag) == flag; }

//--------------------------------------------------
// BmsDataDecode Class Constructors/Destructor
//--------------------------------------------------

BmsDataDecode::BmsDataDecode() {}

BmsDataDecode::~BmsDataDecode() {}

//--------------------------------------------------
// BmsDataDecode Class Implementation
//--------------------------------------------------

/*
 * Calculate CRC for a given data block.
 */
uint8_t BmsDataDecode::calculateCRC(const uint8_t data[], uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++)
        crc += data[i];
    return crc;
}

/*
 * Process incoming data from the BMS.
 * This function is called by the BleManager class when new data is received.
 */
void BmsDataDecode::processData(const std::vector<uint8_t> data, uint16_t length)
{
    if (data.data() == nullptr || length == 0)
        return;
    assemble(data.data(), length);
}

/*
 * Assemble the incoming data into a complete frame.
 * This function handles the preamble, CRC check, and frame decoding.
 */
void BmsDataDecode::assemble(const uint8_t *data, uint16_t length)
{
    // If the accumulated frame is too long, clear it.
    if (m_frameBuffer.size() > MAX_RESPONSE_SIZE)
    {
        ESP_LOGW("BMS_DATA", "Frame dropped due to excessive length");
        m_frameBuffer.clear();
    }

    int offset = 0;
    // Skip "AT\r\n" junk at the beginning.
    if (length >= 4 && data[0] == 'A' && data[1] == 'T' && data[2] == '\r' && data[3] == '\n')
    {
        offset = 4;
        ESP_LOGW("BMS_DATA", "Skipping 'AT\\r\\n' junk");
    }

    // If a new preamble is detected, clear the current buffer.
    if ((length - offset) >= 4 &&
        data[offset] == 0x55 && data[offset + 1] == 0xAA &&
        data[offset + 2] == 0xEB && data[offset + 3] == 0x90)
    {
        m_frameBuffer.clear();
    }

    // Append new data.
    m_frameBuffer.insert(m_frameBuffer.end(), data + offset, data + length);

    // If enough data has been accumulated, perform a CRC check and decode the frame
    if (m_frameBuffer.size() >= MIN_RESPONSE_SIZE)
    {
        const uint8_t *raw = m_frameBuffer.data();
        const uint16_t frame_size = 300;
        uint8_t computed_crc = calculateCRC(raw, frame_size - 1);
        uint8_t remote_crc = raw[frame_size - 1];

        if (computed_crc != remote_crc)
        {
            ESP_LOGW("BMS_DATA", "CRC check failed: computed 0x%02X vs remote 0x%02X", computed_crc, remote_crc);
            m_frameBuffer.clear();
            return;
        }
        ESP_LOGI("BMS_DATA", "Raw data frame (%d bytes) received", (int)m_frameBuffer.size());
        decodeFrame(m_frameBuffer);
        m_frameBuffer.clear();
    }
}

/*
 * Decode the received frame based on the command type.
 * This function identifies the command type and calls the appropriate decoder function.
 */
void BmsDataDecode::decodeFrame(const std::vector<uint8_t> &frame)
{
    if (frame.size() < 5)
        return;
    // The 5th byte indicates the command type.
    switch (frame[4])
    {
    case 0x01:
        decodeDeviceSettings(frame);
        break;
    case 0x02:
        decodeCellInfo(frame);
        break;
    case 0x03:
        decodeDeviceInfo(frame);
        break;
    default:
        ESP_LOGW("BMS_DATA", "Unknown command in response: 0x%02X", frame[4]);
        break;
    }
}

/*
 * Decode the device information frame.
 * This function extracts and logs various device parameters from the received data.
 */
void BmsDataDecode::decodeDeviceInfo(const std::vector<uint8_t> &data)
{
    // Helpers to decode multi-byte values.
    auto get16 = [&](size_t i) -> uint16_t
    {
        return (uint16_t(data[i + 1]) << 8) | data[i];
    };
    auto get32 = [&](size_t i) -> uint32_t
    {
        return ((uint32_t)get16(i + 2) << 16) | get16(i);
    };

    ESP_LOGI("BMS_DATA", "Device info frame (%d bytes) received", (int)data.size());
    ESP_LOGI("BMS_DATA", "  Vendor ID:          %s", std::string(data.begin() + 6, data.begin() + 6 + 16).c_str());
    ESP_LOGI("BMS_DATA", "  Hardware version:   %s", std::string(data.begin() + 22, data.begin() + 22 + 8).c_str());
    ESP_LOGI("BMS_DATA", "  Software version:   %s", std::string(data.begin() + 30, data.begin() + 30 + 8).c_str());
    ESP_LOGI("BMS_DATA", "  Uptime:             %lus", (unsigned long)get32(38));
    ESP_LOGI("BMS_DATA", "  Power on count:     %lu", (unsigned long)get32(42));
    ESP_LOGI("BMS_DATA", "  Device name:        %s", std::string(data.begin() + 46, data.begin() + 46 + 16).c_str());
    ESP_LOGI("BMS_DATA", "  Device passcode:    %s", std::string(data.begin() + 62, data.begin() + 62 + 16).c_str());
    ESP_LOGI("BMS_DATA", "  Manufacturing date: %s", std::string(data.begin() + 78, data.begin() + 78 + 8).c_str());
    ESP_LOGI("BMS_DATA", "  Serial number:      %s", std::string(data.begin() + 86, data.begin() + 86 + 11).c_str());
    ESP_LOGI("BMS_DATA", "  Passcode:           %s", std::string(data.begin() + 97, data.begin() + 97 + 5).c_str());
    ESP_LOGI("BMS_DATA", "  User data:          %s", std::string(data.begin() + 102, data.begin() + 102 + 16).c_str());
    ESP_LOGI("BMS_DATA", "  Setup passcode:     %s", std::string(data.begin() + 118, data.begin() + 118 + 16).c_str());
    ESP_LOGI("BMS_DATA", "  UART1 Protocol:     %d", data[184]);
    ESP_LOGI("BMS_DATA", "  CAN Protocol:       %d", data[185]);
    ESP_LOGI("BMS_DATA", "  UART2 Protocol:     %d", data[218]);
    ESP_LOGI("BMS_DATA", "  RCV Time:           %.1f h", (float)data[266] * 0.1f);
    ESP_LOGI("BMS_DATA", "  RFV Time:           %.1f h", (float)data[267] * 0.1f);
}

/*
 * Decode the cell information frame.
 * This function extracts and logs various cell parameters from the received data.
 */
void BmsDataDecode::decodeDeviceSettings(const std::vector<uint8_t> &data)
{
    auto get16 = [&](size_t i) -> uint16_t
    {
        return (uint16_t(data[i + 1]) << 8) | data[i];
    };
    auto get32 = [&](size_t i) -> uint32_t
    {
        return ((uint32_t)get16(i + 2) << 16) | get16(i);
    };

    ESP_LOGI("BMS_DATA", "Device settings frame (%d bytes) received", (int)data.size());
    ESP_LOGI("BMS_DATA", "  Smart sleep voltage:                       %f", (float)get32(6) * 0.001f);
    ESP_LOGI("BMS_DATA", "  Cell UVP:                                  %f V", (float)get32(10) * 0.001f);
    ESP_LOGI("BMS_DATA", "  Cell UVPR:                                 %f V", (float)get32(14) * 0.001f);
    ESP_LOGI("BMS_DATA", "  Cell OVP:                                  %f V", (float)get32(18) * 0.001f);
    ESP_LOGI("BMS_DATA", "  Cell OVPR:                                 %f V", (float)get32(22) * 0.001f);
    ESP_LOGI("BMS_DATA", "  Balance trigger voltage:                   %f V", (float)get32(26) * 0.001f);
    ESP_LOGI("BMS_DATA", "  SOC 100%% voltage:                         %f V", (float)get32(30) * 0.001f);
    ESP_LOGI("BMS_DATA", "  SOC 0%% voltage:                           %f V", (float)get32(34) * 0.001f);
    ESP_LOGI("BMS_DATA", "  Voltage cell request charge voltage [RCV]: %f V", (float)get32(38) * 0.001f);
    ESP_LOGI("BMS_DATA", "  Voltage cell request float voltage [RFV]:  %f V", (float)get32(42) * 0.001f);
    ESP_LOGI("BMS_DATA", "  Power off voltage:                         %f V", (float)get32(46) * 0.001f);
    ESP_LOGI("BMS_DATA", "  Max. charge current:                       %f A", (float)get32(50) * 0.001f);
    ESP_LOGI("BMS_DATA", "  Charge OCP delay:                          %f s", (float)get32(54));
    ESP_LOGI("BMS_DATA", "  Charge OCP recovery time:                  %f s", (float)get32(58));
    ESP_LOGI("BMS_DATA", "  Max. discharge current:                    %f A", (float)get32(62) * 0.001f);
    ESP_LOGI("BMS_DATA", "  Discharge OCP delay:                       %f s", (float)get32(66));
    ESP_LOGI("BMS_DATA", "  Discharge OCP recovery time:               %f s", (float)get32(70));
    ESP_LOGI("BMS_DATA", "  Short circuit protection recovery time:    %f s", (float)get32(74));
    ESP_LOGI("BMS_DATA", "  Max. balance current:                      %f A", (float)get32(78) * 0.001f);
    ESP_LOGI("BMS_DATA", "  Charge OTP:                                %f °C", (float)get32(82) * 0.1f);
    ESP_LOGI("BMS_DATA", "  Charge OTP recovery:                       %f °C", (float)get32(86) * 0.1f);
    ESP_LOGI("BMS_DATA", "  Discharge OTP:                             %f °C", (float)get32(90) * 0.1f);
    ESP_LOGI("BMS_DATA", "  Discharge OTP recovery:                    %f °C", (float)get32(94) * 0.1f);
    ESP_LOGI("BMS_DATA", "  Charge UTP:                                %f °C", (float)((int32_t)get32(98)) * 0.1f);
    ESP_LOGI("BMS_DATA", "  Charge UTP recovery:                       %f °C", (float)((int32_t)get32(102)) * 0.1f);
    ESP_LOGI("BMS_DATA", "  Mosfet OTP:                                %f °C", (float)((int32_t)get32(106)) * 0.1f);
    ESP_LOGI("BMS_DATA", "  Mosfet OTP recovery:                       %f °C", (float)((int32_t)get32(110)) * 0.1f);
    ESP_LOGI("BMS_DATA", "  Cell count:                                %f", (float)get32(114));
    ESP_LOGI("BMS_DATA", "  Charge switch:     %s", (data[118] ? "ON" : "OFF"));
    ESP_LOGI("BMS_DATA", "  Discharge switch:  %s", (data[122] ? "ON" : "OFF"));
    ESP_LOGI("BMS_DATA", "  Balancer switch:   %s", (data[126] ? "ON" : "OFF"));
    ESP_LOGI("BMS_DATA", "  Nominal battery capacity:                  %f Ah", (float)get32(130) * 0.001f);
    ESP_LOGI("BMS_DATA", "  Short circuit protection delay:            %f us", (float)get32(134) * 1.0f);
    ESP_LOGI("BMS_DATA", "  Start balance voltage:                     %f V", (float)get32(138) * 0.001f);

    for (uint8_t i = 0; i < 32; i++)
    {
        ESP_LOGI("BMS_DATA", "  Con. wire resistance %d: %f Ohm", i + 1, (float)get32(i * 4 + 142) * 0.001f);
    }
    ESP_LOGI("BMS_DATA", "  Device address:                            %d", data[270]);
    ESP_LOGI("BMS_DATA", "  Precharge time:                            %d s", data[274]);
    ESP_LOGI("BMS_DATA", "  Heating switch:     %s", check_bit(data[282], 1) ? "ON" : "OFF");
    ESP_LOGI("BMS_DATA", "  GPS Heartbeat:      %s", check_bit(data[282], 4) ? "ON" : "OFF");
    ESP_LOGI("BMS_DATA", "  Port switch:        %s", check_bit(data[282], 8) ? "RS485" : "CAN");
    ESP_LOGI("BMS_DATA", "  Special charger:    %s", check_bit(data[282], 32) ? "ON" : "OFF");
    ESP_LOGI("BMS_DATA", "  Smart sleep:                               %d h", data[286]);
    ESP_LOGI("BMS_DATA", "  Data field enable control 0:               %d", data[287]);
}

/*
 * Decode the cell information frame.
 * This function extracts and logs various cell information from the received data.
 */
void BmsDataDecode::decodeCellInfo(const std::vector<uint8_t> &data)
{
    ESP_LOGI("BMS_DATA", "Cell info frame (%d bytes) received", (int)data.size());
    // TODO: Implement cell info decoding.
}
