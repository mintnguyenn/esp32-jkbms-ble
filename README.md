# ESP32 JK-BMS BLE Monitor
A lightweight ESP32 firmware to connect with **JK-BMS** (Jikong Battery Management System) over **Bluetooth Low Energy (BLE)**, read live data such as cell voltages and temperatures, and publish them for further usage.

<br>

## ⚙️ Requirements
- ESP32 Dev Board
- ESP-IDF v5.x
- JK-BMS with BLE enabled (advertising `FFE0` service)
<br>

## 🚀 Getting Started

1. **Clone the repository**
   ```bash
   git clone https://github.com/mintnguyenn/ESP32-JK-BMS-BLE.git
   cd ESP32-JK-BMS-BLE
   ```

2. **Configure your environment (remember to enable Bluetooth config)**
   ```bash
   idf.py set-target esp32
   idf.py menuconfig
   ```

3. **Build and flash**
   ```bash
   idf.py build
   idf.py flash
   ```
<br>

## 🔧 How It Works
- The ESP32 starts by scanning for nearby BLE devices. It captures the MAC address of each advertising device and checks if it matches the known MAC address of the target JK-BMS.
- Once the target JK-BMS is detected, the ESP32 stops scanning and initiates a BLE connection to the target device.
- After a successful connection, the JK-BMS automatically initiates MTU negotiation. During this phase, the ESP32 must wait until the MTU exchange is completed to ensure reliable data transmission — especially since the JK-BMS sends large notification frames (up to 128 bytes).
- Once the MTU is successfully negotiated, the ESP32 sends write requests to retrieve data from the BMS. Specifically, the following commands are sent sequentially: `COMMAND_DEVICE_INFO (0x97)` and `COMMAND_CELL_INFO (0x96)`, written to the **FFE1 write handle** of the BMS. The write frame format follows the JK-BMS protocol.
- The JK-BMS responds with the requested data through BLE notification packets, typically on the **FFE1** characteristic.
- These notification packets are parsed by the `BmsDataDecode` class, which extracts meaningful metrics.
- The system continues to receive and decode updates from the BMS periodically, until either the ESP32 is reset, powered off, or manually disconnected.
<br>

## 📡 Notes

- Currently tested on JK-BMS V11.3, with configurations supporting 8–24 cells.
- Additional sensors or output targets (e.g., MQTT, TFT/LCD display) can be integrated easily.
<br>

## 🖥️ Output example
To be added

<br>

## 📌 TODO

- [ ]  Add write command support (e.g., balance control, charge toggle)  
- [ ]  Implement a Web UI or MQTT client for Home Assistant integration  
- [ ]  Optimize for low power consumption  
<br>

## 🧑‍💻 Author

**Minh Nguyen**  
Embedded Software Developer @ Bosch Global Software Technologies  
Casual Academic @ The University of Technology Sydney
