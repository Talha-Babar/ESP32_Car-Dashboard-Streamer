# ESP32 Vehicle Data Streaming with ELM327, Bluetooth, and ESP-NOW (Arduino Framework)

This project demonstrates using an ESP32 to collect and transmit vehicle data such as ethanol percentage, manifold pressure, and intake air temperature. The data is read via an ELM327 OBD-II scanner over Bluetooth, processed on the ESP32, and sent wirelessly to another ESP32 using ESP-NOW.

---

## Features

- **Bluetooth Communication**: Connects to an ELM327 OBD-II scanner to fetch vehicle data.
- **ESP-NOW Data Transmission**: Sends the data wirelessly to another ESP32 or compatible device.
- **Real-Time Vehicle Data Monitoring**:
  - Ethanol percentage.
  - Manifold pressure (converted to PSI).
  - Intake air temperature (converted to Fahrenheit).

---

## How It Works

1. **Bluetooth Communication**:

   - Connects to an ELM327 OBD-II scanner via Bluetooth.
   - Fetches vehicle data such as ethanol percentage, manifold pressure, and air temperature.

2. **Data Processing**:

   - Converts raw data from the scanner into human-readable values:
     - Temperature in Fahrenheit.
     - Pressure in PSI.

3. **ESP-NOW Transmission**:

   - Sends the processed data to another ESP32 using ESP-NOW for further processing or display.

4. **Real-Time Updates**:
   - Data is collected and transmitted in near real-time with configurable intervals.
