#include "bms_data_decode.h"

//--------------------------------------------------
// Utility Functions and Variables
//--------------------------------------------------
static bool check_bit(uint8_t mask, uint8_t flag) { return (mask & flag) == flag; }

static const char *m_tag = "Jk-Bms";

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
    if (m_frameBuffer_.size() > MAX_RESPONSE_SIZE)
    {
        ESP_LOGW(m_tag, "Frame dropped due to excessive length");
        m_frameBuffer_.clear();
    }

    int offset = 0;
    // Skip "AT\r\n" junk at the beginning.
    if (length >= 4 && data[0] == 'A' && data[1] == 'T' && data[2] == '\r' && data[3] == '\n')
    {
        offset = 4;
        ESP_LOGW(m_tag, "Skipping 'AT\\r\\n' junk");
    }

    // If a new preamble is detected, clear the current buffer.
    if ((length - offset) >= 4 &&
        data[offset] == 0x55 && data[offset + 1] == 0xAA &&
        data[offset + 2] == 0xEB && data[offset + 3] == 0x90)
    {
        m_frameBuffer_.clear();
    }

    // Append new data.
    m_frameBuffer_.insert(m_frameBuffer_.end(), data + offset, data + length);

    // If enough data has been accumulated, perform a CRC check and decode the frame
    if (m_frameBuffer_.size() >= MIN_RESPONSE_SIZE)
    {
        const uint8_t *raw = m_frameBuffer_.data();
        const uint16_t frame_size = 300;
        uint8_t computed_crc = calculateCRC(raw, frame_size - 1);
        uint8_t remote_crc = raw[frame_size - 1];

        if (computed_crc != remote_crc)
        {
            ESP_LOGW(m_tag, "CRC check failed: computed 0x%02X vs remote 0x%02X", computed_crc, remote_crc);
            m_frameBuffer_.clear();
            return;
        }
        // ESP_LOGI(m_tag, "Raw data frame (%d bytes) received", (int)m_frameBuffer_.size());
        decodeFrame(m_frameBuffer_);
        m_frameBuffer_.clear();
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
        ESP_LOGW(m_tag, "Unknown command in response: 0x%02X", frame[4]);
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
    { return (uint16_t(data[i + 1]) << 8) | data[i]; };
    auto get32 = [&](size_t i) -> uint32_t
    { return ((uint32_t)get16(i + 2) << 16) | get16(i); };

    ESP_LOGI(m_tag, "Device info frame (%d bytes) received", (int)data.size());
    ESP_LOGI(m_tag, "  Vendor ID:          %s", std::string(data.begin() + 6, data.begin() + 6 + 16).c_str());
    ESP_LOGI(m_tag, "  Hardware version:   %s", std::string(data.begin() + 22, data.begin() + 22 + 8).c_str());
    ESP_LOGI(m_tag, "  Software version:   %s", std::string(data.begin() + 30, data.begin() + 30 + 8).c_str());
    ESP_LOGI(m_tag, "  Uptime:             %lus", (unsigned long)get32(38));
    ESP_LOGI(m_tag, "  Power on count:     %lu", (unsigned long)get32(42));
    ESP_LOGI(m_tag, "  Device name:        %s", std::string(data.begin() + 46, data.begin() + 46 + 16).c_str());
    ESP_LOGI(m_tag, "  Device passcode:    %s", std::string(data.begin() + 62, data.begin() + 62 + 16).c_str());
    ESP_LOGI(m_tag, "  Manufacturing date: %s", std::string(data.begin() + 78, data.begin() + 78 + 8).c_str());
    ESP_LOGI(m_tag, "  Serial number:      %s", std::string(data.begin() + 86, data.begin() + 86 + 11).c_str());
    ESP_LOGI(m_tag, "  Passcode:           %s", std::string(data.begin() + 97, data.begin() + 97 + 5).c_str());
    ESP_LOGI(m_tag, "  User data:          %s", std::string(data.begin() + 102, data.begin() + 102 + 16).c_str());
    ESP_LOGI(m_tag, "  Setup passcode:     %s", std::string(data.begin() + 118, data.begin() + 118 + 16).c_str());
    ESP_LOGI(m_tag, "  UART1 Protocol:     %d", data[184]);
    ESP_LOGI(m_tag, "  CAN Protocol:       %d", data[185]);
    ESP_LOGI(m_tag, "  UART2 Protocol:     %d", data[218]);
    ESP_LOGI(m_tag, "  RCV Time:           %.1f h", (float)data[266] * 0.1f);
    ESP_LOGI(m_tag, "  RFV Time:           %.1f h", (float)data[267] * 0.1f);
}

/*
 * Decode the cell information frame.
 * This function extracts and logs various cell parameters from the received data.
 */
void BmsDataDecode::decodeDeviceSettings(const std::vector<uint8_t> &data)
{
    // Helpers to decode multi-byte values.
    auto get16 = [&](size_t i) -> uint16_t
    { return (uint16_t(data[i + 1]) << 8) | data[i]; };
    auto get32 = [&](size_t i) -> uint32_t
    { return ((uint32_t)get16(i + 2) << 16) | get16(i); };

    ESP_LOGI(m_tag, "Device settings frame (%d bytes) received", (int)data.size());
    ESP_LOGI(m_tag, "  Smart sleep voltage:                       %f", (float)get32(6) * 0.001f);
    ESP_LOGI(m_tag, "  Cell UVP:                                  %f V", (float)get32(10) * 0.001f);
    ESP_LOGI(m_tag, "  Cell UVPR:                                 %f V", (float)get32(14) * 0.001f);
    ESP_LOGI(m_tag, "  Cell OVP:                                  %f V", (float)get32(18) * 0.001f);
    ESP_LOGI(m_tag, "  Cell OVPR:                                 %f V", (float)get32(22) * 0.001f);
    ESP_LOGI(m_tag, "  Balance trigger voltage:                   %f V", (float)get32(26) * 0.001f);
    ESP_LOGI(m_tag, "  SOC 100%% voltage:                         %f V", (float)get32(30) * 0.001f);
    ESP_LOGI(m_tag, "  SOC 0%% voltage:                           %f V", (float)get32(34) * 0.001f);
    ESP_LOGI(m_tag, "  Voltage cell request charge voltage [RCV]: %f V", (float)get32(38) * 0.001f);
    ESP_LOGI(m_tag, "  Voltage cell request float voltage [RFV]:  %f V", (float)get32(42) * 0.001f);
    ESP_LOGI(m_tag, "  Power off voltage:                         %f V", (float)get32(46) * 0.001f);
    ESP_LOGI(m_tag, "  Max. charge current:                       %f A", (float)get32(50) * 0.001f);
    ESP_LOGI(m_tag, "  Charge OCP delay:                          %f s", (float)get32(54));
    ESP_LOGI(m_tag, "  Charge OCP recovery time:                  %f s", (float)get32(58));
    ESP_LOGI(m_tag, "  Max. discharge current:                    %f A", (float)get32(62) * 0.001f);
    ESP_LOGI(m_tag, "  Discharge OCP delay:                       %f s", (float)get32(66));
    ESP_LOGI(m_tag, "  Discharge OCP recovery time:               %f s", (float)get32(70));
    ESP_LOGI(m_tag, "  Short circuit protection recovery time:    %f s", (float)get32(74));
    ESP_LOGI(m_tag, "  Max. balance current:                      %f A", (float)get32(78) * 0.001f);
    ESP_LOGI(m_tag, "  Charge OTP:                                %f °C", (float)get32(82) * 0.1f);
    ESP_LOGI(m_tag, "  Charge OTP recovery:                       %f °C", (float)get32(86) * 0.1f);
    ESP_LOGI(m_tag, "  Discharge OTP:                             %f °C", (float)get32(90) * 0.1f);
    ESP_LOGI(m_tag, "  Discharge OTP recovery:                    %f °C", (float)get32(94) * 0.1f);
    ESP_LOGI(m_tag, "  Charge UTP:                                %f °C", (float)((int32_t)get32(98)) * 0.1f);
    ESP_LOGI(m_tag, "  Charge UTP recovery:                       %f °C", (float)((int32_t)get32(102)) * 0.1f);
    ESP_LOGI(m_tag, "  Mosfet OTP:                                %f °C", (float)((int32_t)get32(106)) * 0.1f);
    ESP_LOGI(m_tag, "  Mosfet OTP recovery:                       %f °C", (float)((int32_t)get32(110)) * 0.1f);
    ESP_LOGI(m_tag, "  Cell count:                                %f", (float)get32(114));
    ESP_LOGI(m_tag, "  Charge switch:     %s", (data[118] ? "ON" : "OFF"));
    ESP_LOGI(m_tag, "  Discharge switch:  %s", (data[122] ? "ON" : "OFF"));
    ESP_LOGI(m_tag, "  Balancer switch:   %s", (data[126] ? "ON" : "OFF"));
    ESP_LOGI(m_tag, "  Nominal battery capacity:                  %f Ah", (float)get32(130) * 0.001f);
    ESP_LOGI(m_tag, "  Short circuit protection delay:            %f us", (float)get32(134) * 1.0f);
    ESP_LOGI(m_tag, "  Start balance voltage:                     %f V", (float)get32(138) * 0.001f);

    for (uint8_t i = 0; i < 32; i++)
    {
        ESP_LOGI(m_tag, "  Con. wire resistance %d: %f Ohm", i + 1, (float)get32(i * 4 + 142) * 0.001f);
    }
    ESP_LOGI(m_tag, "  Device address:                            %d", data[270]);
    ESP_LOGI(m_tag, "  Precharge time:                            %d s", data[274]);
    ESP_LOGI(m_tag, "  Heating switch:     %s", check_bit(data[282], 1) ? "ON" : "OFF");
    ESP_LOGI(m_tag, "  GPS Heartbeat:      %s", check_bit(data[282], 4) ? "ON" : "OFF");
    ESP_LOGI(m_tag, "  Port switch:        %s", check_bit(data[282], 8) ? "RS485" : "CAN");
    ESP_LOGI(m_tag, "  Special charger:    %s", check_bit(data[282], 32) ? "ON" : "OFF");
    ESP_LOGI(m_tag, "  Smart sleep:                               %d h", data[286]);
    ESP_LOGI(m_tag, "  Data field enable control 0:               %d", data[287]);
}

/*
 * Decode the cell information frame.
 * This function extracts and logs various cell information from the received data.
 */
void BmsDataDecode::decodeCellInfo(const std::vector<uint8_t> &data)
{
    // Throttle the output to avoid flooding the log.
    static uint32_t last_log_time = 0;
    uint32_t current_time = esp_log_timestamp();
    if (current_time - last_log_time < m_throttle_)
    {
        return;
    }
    last_log_time = current_time;

    // Helpers to decode multi-byte values.
    auto get16 = [&](size_t i) -> uint16_t
    { return (uint16_t(data[i + 1]) << 8) | data[i]; };
    auto get32 = [&](size_t i) -> uint32_t
    { return ((uint32_t)get16(i + 2) << 16) | get16(i); };

    uint8_t offset = 16; // Skip the header and command bytes
    uint8_t cells = 24 + (offset / 2);
    float min_cell_voltage = 100.0f;
    float max_cell_voltage = -100.0f;
    float average_cell_voltage = 0.0f;
    uint8_t min_voltage_cell = 0;
    uint8_t max_voltage_cell = 0;
    uint8_t cells_enabled = 0;
    ESP_LOGI(m_tag, "Cell info frame (%d bytes) received", (int)data.size());

    for (uint8_t i = 0; i < cells; i++)
    {
        float cell_voltage = get16(i * 2 + 6) * 0.001f;
        float cell_resistance = get16(i * 2 + 64 + offset) * 0.001f;

        cells_[i].cell_voltage_sensor = cell_voltage;
        cells_[i].cell_resistance_sensor = cell_resistance;

        if (cell_voltage > 0)
        {
            average_cell_voltage += cell_voltage;
            cells_enabled++;
        }
        if (cell_voltage > 0 && cell_voltage < min_cell_voltage)
        {
            min_cell_voltage = cell_voltage;
            min_voltage_cell = i + 1;
        }
        if (cell_voltage > max_cell_voltage)
        {
            max_cell_voltage = cell_voltage;
            max_voltage_cell = i + 1;
        }
    }
    average_cell_voltage /= cells_enabled;

    min_cell_voltage_sensor_ = min_cell_voltage;
    max_cell_voltage_sensor_ = max_cell_voltage;
    min_voltage_cell_sensor_ = (float)min_voltage_cell;
    max_voltage_cell_sensor_ = (float)max_voltage_cell;
    delta_cell_voltage_sensor_ = max_cell_voltage - min_cell_voltage;
    average_cell_voltage_sensor_ = average_cell_voltage;

    offset = offset * 2;

    power_tube_temperature_sensor_ = ((int16_t)get16(112 + offset)) * 0.1f;

    total_voltage_sensor_ = get32(118 + offset) * 0.001f;
    current_sensor_ = ((int32_t)get32(126 + offset)) * 0.001f;

    float power = total_voltage_sensor_ * current_sensor_;
    power_sensor_ = power;
    charging_power_sensor_ = std::max(0.0f, power);
    discharging_power_sensor_ = std::abs(std::min(0.0f, power));

    errors_bitmask_sensor_ = (float)((uint16_t(m_frameBuffer_[134 + offset]) << 8) | m_frameBuffer_[135 + offset]);

    balancing_current_sensor_ = ((int16_t)get16(138 + offset)) * 0.001f;
    balancing_sensor_ = m_frameBuffer_[140 + offset];
    balancing_binary_sensor_ = (m_frameBuffer_[140 + offset] != 0x00);

    state_of_charge_sensor_ = (float)m_frameBuffer_[141 + offset];
    capacity_remaining_sensor_ = get32(142 + offset) * 0.001f;
    total_battery_capacity_setting_sensor_ = get32(146 + offset) * 0.001f;
    charging_cycles_sensor_ = (float)get32(150 + offset);
    total_charging_cycle_capacity_sensor_ = get32(154 + offset) * 0.001f;

    total_runtime_sensor_ = (float)get32(162 + offset);
    charging_binary_sensor_ = (bool)m_frameBuffer_[166 + offset];
    discharging_binary_sensor_ = (bool)m_frameBuffer_[167 + offset];
    precharging_binary_sensor_ = (bool)m_frameBuffer_[168 + offset];
    heating_binary_sensor_ = (bool)m_frameBuffer_[183 + offset];
    heating_current_sensor_ = ((int16_t)get16(204 + offset)) * 0.001f;

    dry_contact_1_binary_sensor_ = check_bit(m_frameBuffer_[249 + offset], 2);
    dry_contact_2_binary_sensor_ = check_bit(m_frameBuffer_[249 + offset], 4);
    emergency_time_countdown_sensor_ = (float)get16(186 + offset);
    emergency_switch_ = (emergency_time_countdown_sensor_ > 0);

    // errors_text_sensor_ = this->error_bits_to_string_((uint16_t)errors_bitmask_sensor_);
    // total_runtime_formatted_text_sensor_ = format_total_runtime_((uint32_t)total_runtime_sensor_);

    printCellInfo();
}

void BmsDataDecode::printCellInfo()
{
    ESP_LOGI(m_tag, "JkBmsBle Config Dump");

    ESP_LOGI(m_tag, "Balancing: %d", balancing_binary_sensor_);
    ESP_LOGI(m_tag, "Precharging: %d", precharging_binary_sensor_);
    ESP_LOGI(m_tag, "Charging: %d", charging_binary_sensor_);
    ESP_LOGI(m_tag, "Discharging: %d", discharging_binary_sensor_);
    ESP_LOGI(m_tag, "Heating: %d", heating_binary_sensor_);
    ESP_LOGI(m_tag, "Emergency Switch: %d", emergency_switch_);
    ESP_LOGI(m_tag, "Dry Contact 1: %d", dry_contact_1_binary_sensor_);
    ESP_LOGI(m_tag, "Dry Contact 2: %d", dry_contact_2_binary_sensor_);

    ESP_LOGI(m_tag, "Min Cell Voltage: %.3f", min_cell_voltage_sensor_);
    ESP_LOGI(m_tag, "Max Cell Voltage: %.3f", max_cell_voltage_sensor_);
    ESP_LOGI(m_tag, "Delta Cell Voltage: %.3f", delta_cell_voltage_sensor_);
    ESP_LOGI(m_tag, "Avg Cell Voltage: %.3f", average_cell_voltage_sensor_);
    ESP_LOGI(m_tag, "Min Voltage Cell #: %.0f", min_voltage_cell_sensor_);
    ESP_LOGI(m_tag, "Max Voltage Cell #: %.0f", max_voltage_cell_sensor_);

    ESP_LOGI(m_tag, "Total Voltage: %.3f V", total_voltage_sensor_);
    ESP_LOGI(m_tag, "Current: %.3f A", current_sensor_);
    ESP_LOGI(m_tag, "Power: %.3f W", power_sensor_);
    ESP_LOGI(m_tag, "Charging Power: %.3f W", charging_power_sensor_);
    ESP_LOGI(m_tag, "Discharging Power: %.3f W", discharging_power_sensor_);

    ESP_LOGI(m_tag, "Power Tube Temperature: %.1f °C", power_tube_temperature_sensor_);
    ESP_LOGI(m_tag, "State of Charge: %.1f %%", state_of_charge_sensor_);
    ESP_LOGI(m_tag, "Capacity Remaining: %.3f Ah", capacity_remaining_sensor_);
    ESP_LOGI(m_tag, "Total Battery Capacity Setting: %.3f Ah", total_battery_capacity_setting_sensor_);
    ESP_LOGI(m_tag, "Charging Cycles: %.0f", charging_cycles_sensor_);
    ESP_LOGI(m_tag, "Total Charging Cycle Capacity: %.3f Ah", total_charging_cycle_capacity_sensor_);
    ESP_LOGI(m_tag, "Balancing Current: %.3f A", balancing_current_sensor_);

    ESP_LOGI(m_tag, "Heating Current: %.3f A", heating_current_sensor_);
    ESP_LOGI(m_tag, "Errors Bitmask: %.0f", errors_bitmask_sensor_);
    ESP_LOGI(m_tag, "Errors Text: %s", errors_text_sensor_.c_str());
    ESP_LOGI(m_tag, "Total Runtime: %.0f s", total_runtime_sensor_);
    ESP_LOGI(m_tag, "Total Runtime (Formatted): %s", total_runtime_formatted_text_sensor_.c_str());
    ESP_LOGI(m_tag, "Emergency Countdown: %.0f s", emergency_time_countdown_sensor_);

    for (int i = 0; i < 8; i++)
    {
        ESP_LOGI(m_tag, "Cell %2d - Voltage: %.3f V, Resistance: %.3f Ω",
                 i + 1, cells_[i].cell_voltage_sensor, cells_[i].cell_resistance_sensor);
    }
}
