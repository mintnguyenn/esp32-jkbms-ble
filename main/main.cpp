#include <stdio.h>

#include "ble_manager.h"
#include "bms_data_decode.h"

void my_task(void *pvParameters)
{
    BmsDataDecode bmsParser;
    BleManager bleManager(&bmsParser);

    bleManager.initialize();
    vTaskDelay(pdMS_TO_TICKS(1000));
    bleManager.startScanning();

    while (true)
    {
        // keep the task alive
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void)
{
    xTaskCreatePinnedToCore(my_task, "esp32_jk-bms_ble_demostration", 8192, nullptr, 1, nullptr, 1);
}