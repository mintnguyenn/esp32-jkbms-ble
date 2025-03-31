#include <stdio.h>

// #include "jk_bms_ble.h"
#include "ble_manager.h"
#include "bms_data_decode.h"

extern "C" void app_main(void)
{
    static BmsDataDecode bmsParser;
    static BleManager bleManager(&bmsParser);
    bleManager.initialize();

    // delay(1000);
    vTaskDelay(pdMS_TO_TICKS(1000));
    bleManager.startScanning();
}