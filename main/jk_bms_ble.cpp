#include "jk_bms_ble.h"

static int blecent_gap_event(struct ble_gap_event *event, void *arg)
{
  // struct ble_gap_conn_desc desc;
  struct ble_hs_adv_fields fields;
  int rc;

  switch (event->type) {
  case BLE_GAP_EVENT_DISC:
    rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
    if (rc != 0) return 0;

    // Print mac address
    ESP_LOGI(tag, "Device Address: %02x:%02x:%02x:%02x:%02x:%02x", event->disc.addr.val[5], event->disc.addr.val[4], event->disc.addr.val[3], event->disc.addr.val[2], event->disc.addr.val[1], event->disc.addr.val[0]);
    break;

  default:
    break;
  }

  return 0;
}

void ble_scan(){
  uint8_t own_addr_type;
  struct ble_gap_disc_params disc_params;
  int rc;

  /* Figure out address to use while advertising (no privacy for now) */
  rc = ble_hs_id_infer_auto(0, &own_addr_type);
  if (rc != 0) {
      MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
      return;
  }

  /* Tell the controller to filter duplicates; we don't want to process
   * repeated advertisements from the same device.
   */
  disc_params.filter_duplicates = 1;

  /**
   * Perform a passive scan.  I.e., don't send follow-up scan requests to
   * each advertiser.
   */
  disc_params.passive = 1;

  /* Use defaults for the rest of the parameters. */
  disc_params.itvl = 0;
  disc_params.window = 0;
  disc_params.filter_policy = 0;
  disc_params.limited = 0;

  rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                    blecent_gap_event, NULL);
  if (rc != 0) {
      MODLOG_DFLT(ERROR, "Error initiating GAP discovery procedure; rc=%d\n",
                  rc);
  }
}

static void blecent_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void blecent_on_sync(void)
{
    int rc;

    /* Make sure we have proper identity address set (public preferred) */
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

#if !CONFIG_EXAMPLE_INIT_DEINIT_LOOP
    /* Begin scanning for a peripheral to connect to. */
    ble_scan();
#endif
}

void blecent_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

JkBmsBle::JkBmsBle() {
  init_ble();
  // TODO Auto-generated constructor stub
}

JkBmsBle::~JkBmsBle() {
  // TODO Auto-generated destructor stub
}

void JkBmsBle::init_ble() {
  int rc;
    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if  (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to init nimble %d ", ret);
        return;
    }

    /* Configure the host. */
    ble_hs_cfg.reset_cb = blecent_on_reset;
    ble_hs_cfg.sync_cb = blecent_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("esp32");
    assert(rc == 0);

    nimble_port_freertos_init(blecent_host_task);
}

