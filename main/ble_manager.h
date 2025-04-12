#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <vector>

#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"

// Interface for processing incoming BMS data.
class IBmsDataProcessor {
public:
    virtual ~IBmsDataProcessor() {}
    virtual void processData(const std::vector<uint8_t> data, uint16_t length) = 0;
};

class BleManager {
public:
    BleManager();
    BleManager(IBmsDataProcessor *processor);

    ~BleManager();

    esp_err_t initialize();
    void startScanning();
    void setDataProcessor(IBmsDataProcessor *processor);

    // Static event handler called by the BLE stack.
    static int eventHandler(struct ble_gap_event *event, void *arg);

    // BLE host task for NimBLE.
    static void bleHostTask(void *param);

private:
    // Instance-level event handler that has access to member variables.
    int handleEvent(struct ble_gap_event *event);
    bool write_register(uint8_t address, uint32_t value, uint8_t length);

    IBmsDataProcessor *m_dataProcessor_;
};

#endif // BLE_MANAGER_H
