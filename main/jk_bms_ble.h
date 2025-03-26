#include "esp_log.h"
#include "nvs_flash.h"

/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"

#include <vector>
#include <string>


class JkBmsBle {
  public:
  /* Default constructor */
  JkBmsBle();

  /* Default destructor */
  ~JkBmsBle();

  /* Initialize BLE */
  void init_ble();

  /* Write register */
  static bool write_register(uint8_t address, uint32_t value, uint8_t length);
};