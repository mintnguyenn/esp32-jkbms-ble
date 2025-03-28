#include "esp_log.h"
#include "nvs_flash.h"

/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gatt.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"

#include <vector>
#include <string>

extern "C"
{
#include "host/ble_gap.h"
}

class JkBmsBle
{
public:
  JkBmsBle();
  ~JkBmsBle();

  void ble_initialize();
  static void ble_start_scanning();
  static bool write_register(uint8_t address, uint32_t value, uint8_t length);
  static void assemble(const uint8_t *data, uint16_t length);
  static void decode_device_info_(const std::vector<uint8_t> &data);
  static void decode_device_settings_(const std::vector<uint8_t> &data);
  static void decode_cell_info_(const std::vector<uint8_t> &data);

  static float ieee_float_(uint32_t f) {
    static_assert(sizeof(float) == sizeof f, "`float` has a weird size.");
    float ret;
    memcpy(&ret, &f, sizeof(float));
    return ret;
  }

  static bool check_bit_(uint8_t mask, uint8_t flag) { return (mask & flag) == flag; }

private:
  static std::vector<uint8_t> frame_buffer_;
};

// BLE event handler (used in ble_gap_connect and ble_gap_disc)
extern "C" int ble_gap_event_handler(struct ble_gap_event *event, void *arg);

// Utility
uint8_t crc(const uint8_t data[], const uint16_t len);