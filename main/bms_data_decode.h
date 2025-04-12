#ifndef BMS_DATA_PARSER_H
#define BMS_DATA_PARSER_H

#include <string>
#include <vector>

#include "ble_manager.h" // To implement IBmsDataProcessor

class BmsDataDecode : public IBmsDataProcessor
{
public:
    BmsDataDecode();
    ~BmsDataDecode();

    // IBmsDataProcessor override: process incoming raw BLE data.
    void processData(const std::vector<uint8_t> data, uint16_t length);

private:
    void assemble(const uint8_t *data, uint16_t length);
    void decodeFrame(const std::vector<uint8_t> &frame);
    void decodeDeviceInfo(const std::vector<uint8_t> &data);
    void decodeDeviceSettings(const std::vector<uint8_t> &data);
    void decodeCellInfo(const std::vector<uint8_t> &data);
    void printCellInfo();

    uint8_t calculateCRC(const uint8_t data[], uint16_t len);

    std::vector<uint8_t> m_frameBuffer_;

    static const uint16_t MIN_RESPONSE_SIZE = 300;
    static const uint16_t MAX_RESPONSE_SIZE = 330;

    uint16_t m_throttle_ = 5000; // 5 seconds throttle for logging

    //
    struct Cell
    {
        float cell_voltage_sensor;
        float cell_resistance_sensor;
    } cells_[32];

    struct Temperature
    {
        float temperature_sensor_;
    } temperatures_[5];

    // Device Info Variables
    float cell_request_charge_voltage_time_number_;
    float cell_request_float_voltage_time_number_;

    // Settings Variables
    float smart_sleep_voltage_number_;
    float balance_trigger_voltage_number_;
    float cell_soc100_voltage_number_;
    float cell_soc0_voltage_number_;
    float cell_request_charge_voltage_number_;
    float cell_request_float_voltage_number_;
    float power_off_voltage_number_;
    float max_balance_current_number_;
    float max_charge_current_number_;
    float max_discharge_current_number_;
    float cell_voltage_overvoltage_protection_number_;
    float cell_voltage_overvoltage_recovery_number_;
    float cell_voltage_undervoltage_protection_number_;
    float cell_voltage_undervoltage_recovery_number_;
    float charge_overcurrent_protection_delay_number_;
    float charge_overcurrent_protection_recovery_time_number_;
    float discharge_overcurrent_protection_delay_number_;
    float discharge_overcurrent_protection_recovery_time_number_;
    float short_circuit_protection_delay_number_;
    float short_circuit_protection_recovery_time_number_;
    float charge_overtemperature_protection_number_;
    float charge_overtemperature_protection_recovery_number_;
    float discharge_overtemperature_protection_number_;
    float discharge_overtemperature_protection_recovery_number_;
    float charge_undertemperature_protection_number_;
    float charge_undertemperature_protection_recovery_number_;
    float power_tube_overtemperature_protection_number_;
    float power_tube_overtemperature_protection_recovery_number_;
    float cell_count_number_;
    float total_battery_capacity_number_;
    float balance_starting_voltage_number_;

    bool charging_switch_;
    bool discharging_switch_;
    bool balancer_switch_;
    bool heating_switch_;
    bool disable_temperature_sensors_switch_;
    bool display_always_on_switch_;
    bool smart_sleep_switch_;
    bool timed_stored_data_switch_;
    bool disable_pcl_module_switch_;
    bool charging_float_mode_switch_;

    // Cell Info Variables
    bool balancing_binary_sensor_;
    bool precharging_binary_sensor_;
    bool charging_binary_sensor_;
    bool discharging_binary_sensor_;
    bool heating_binary_sensor_;
    bool emergency_switch_;
    bool dry_contact_1_binary_sensor_;
    bool dry_contact_2_binary_sensor_;

    float balancing_sensor_;
    float min_cell_voltage_sensor_;
    float max_cell_voltage_sensor_;
    float min_voltage_cell_sensor_;
    float max_voltage_cell_sensor_;
    float delta_cell_voltage_sensor_;
    float average_cell_voltage_sensor_;
    float total_voltage_sensor_;
    float current_sensor_;
    float power_sensor_;
    float charging_power_sensor_;
    float discharging_power_sensor_;
    float power_tube_temperature_sensor_;
    float state_of_charge_sensor_;
    float capacity_remaining_sensor_;
    float total_battery_capacity_setting_sensor_;
    float charging_cycles_sensor_;
    float total_charging_cycle_capacity_sensor_;
    float total_runtime_sensor_;
    float balancing_current_sensor_;
    float errors_bitmask_sensor_;
    float emergency_time_countdown_sensor_;
    float heating_current_sensor_;

    std::string errors_text_sensor_;
    std::string total_runtime_formatted_text_sensor_;
};

#endif // BMS_DATA_PARSER_H
