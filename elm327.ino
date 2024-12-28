
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "BluetoothSerial.h"
#include "ELMduino.h"
#include "esp_now.h"
#include "WiFi.h"

BluetoothSerial SerialBT;
ELM327 myELM327;

uint8_t broadcastAddress[] = {0x48, 0x27, 0xe2, 0xea, 0x81, 0x7c};

typedef struct
{
  float air;
  float pressure;
  uint8_t ethanol;
} sensor_data_t;

sensor_data_t myData;
esp_now_peer_info_t peerInfo = {};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print(F("\r\nLast Packet Send Status:\t"));
  Serial.println(F(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail"));
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  SerialBT.begin("ArduHUD", true);
  if (!SerialBT.connect("OBDII"))
  {
    Serial.println(F("Couldn't connect to OBD scanner - Phase 1"));
    while (1)
      ;
  }

  if (!myELM327.begin(SerialBT, false, 2000))
  {
    Serial.println(F("Couldn't connect to OBD scanner - Phase 2"));
    while (1)
      ;
  }
  Serial.println(F("Connected to ELM327"));

  if (esp_now_init() != ESP_OK)
  {
    Serial.println(F("Error initializing ESP-NOW"));
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println(F("Failed to add peer"));
    return;
  }

  xTaskCreatePinnedToCore(readSensorTask, "ReadSensorTask", 2048, NULL, 1, NULL, 0);
}

void loop()
{
  // Sending data via ESP-NOW
  esp_err_t result = esp_now_send(peerInfo.peer_addr, (uint8_t *)&myData, sizeof(myData));
  if (result != ESP_OK)
  {
    Serial.println("Error sending the data");
  }

  // Delay between sending data
  delay(1000); // Adjust as necessary
}

void readSensorTask(void *parameter)
{
  while (1)
  {

    uint8_t ethanol_temp = myELM327.ethonolPercent();

    if (myELM327.nb_rx_state == ELM_SUCCESS)
    {

      myData.ethanol = ethanol_temp;
      Serial.print(F("Ethanol Percent: "));
      Serial.print(myData.ethanol);
      Serial.println(F("%"));
    }
    else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
    {
      myELM327.printError();
    }
    delay(100);

    float air_temp = myELM327.intakeAirTemp();

    if (myELM327.nb_rx_state == ELM_SUCCESS)
    {
      // Convert air temperature from Celsius to Fahrenheit and save in struct
      myData.air = air_temp * (9 / 5 + 32);
      Serial.print("Air Temp (F): ");
      Serial.println(myData.air);
    }
    else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
    {
      myELM327.printError();
    }

    delay(100);
    float pressure_temp = myELM327.manifoldPressure();

    if (myELM327.nb_rx_state == ELM_SUCCESS)
    {
      // Convert manifold pressure from hPa to psi and save in struct
      myData.pressure = pressure_temp * 0.0145038;
      Serial.print("Pressure (psi): ");
      Serial.println(myData.pressure);
    }
    else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
    {
      myELM327.printError();
    }

    delay(100);

    vTaskDelay(pdMS_TO_TICKS(500)); // Adjust the delay as needed
  }
}

// #include "BluetoothSerial.h"
// #include "ELMduino.h"
// #include "esp_now.h"
// #include "WiFi.h"
//
// BluetoothSerial SerialBT;
// ELM327 myELM327;
//
// uint8_t broadcastAddress[] = {0x48, 0x27, 0xe2, 0xea, 0x81, 0x7c};
//
// typedef struct {
//   float air;
//   float pressure;
//   uint8_t ethanol;
// } sensor_data_t;
//
// sensor_data_t myData;
// esp_now_peer_info_t peerInfo = {};
//
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//   Serial.print(F("\r\nLast Packet Send Status:\t"));
//   Serial.println(F(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail"));
// }
//
// void setup() {
//   Serial.begin(115200);
//   WiFi.mode(WIFI_STA);
//
//   SerialBT.begin("ArduHUD", true);
//   if (!SerialBT.connect("OBDII")) {
//     Serial.println(F("Couldn't connect to OBD scanner - Phase 1"));
//     while (1);
//   }
//
//   if (!myELM327.begin(SerialBT, false, 2000)) {
//     Serial.println(F("Couldn't connect to OBD scanner - Phase 2"));
//     while (1);
//   }
//   Serial.println(F("Connected to ELM327"));
//
//   if (esp_now_init() != ESP_OK) {
//     Serial.println(F("Error initializing ESP-NOW"));
//     return;
//   }
//
//   esp_now_register_send_cb(OnDataSent);
//
//   memcpy(peerInfo.peer_addr, broadcastAddress, 6);
//   peerInfo.channel = 0;
//   peerInfo.encrypt = false;
//
//   if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//     Serial.println(F("Failed to add peer"));
//     return;
//   }
// }
//
// void loop() {
//
//
// myData.ethanol = myELM327.ethonolPercent();
//
// if (myELM327.nb_rx_state == ELM_SUCCESS) {
//   Serial.print(F("Ethanol Percent: "));
//   Serial.print(myData.ethanol);
//   Serial.println(F("%"));
// } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
//   myELM327.printError();
// }
// delay(100);
//
// myData.air = myELM327.intakeAirTemp();
//
// if (myELM327.nb_rx_state == ELM_SUCCESS) {
//   // Convert air temperature from Celsius to Fahrenheit and save in struct
//   myData.air = myData.air * 9 / 5 + 32;
//   Serial.print("Air Temp (F): ");
//   Serial.println(myData.air);
// } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
//   myELM327.printError();
// }
//
// delay(100);
// myData.pressure = myELM327.manifoldPressure();
//
// if (myELM327.nb_rx_state == ELM_SUCCESS) {
//   // Convert manifold pressure from hPa to psi and save in struct
//   myData.pressure = myData.pressure * 0.0145038;
//   Serial.print("Pressure (psi): ");
//   Serial.println(myData.pressure);
// } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
//   myELM327.printError();
// }
//
//
// delay(100);
//
//
//       esp_err_t result = esp_now_send(peerInfo.peer_addr, (uint8_t *)&myData, sizeof(myData));
//     memset(&myData, 0, sizeof(myData));
//     if (result != ESP_OK) {
//       Serial.println("Error sending the data");
// }
// }

#include "BluetoothSerial.h"
#include "ELMduino.h"
#include "esp_now.h"
#include "WiFi.h"

BluetoothSerial SerialBT;
ELM327 myELM327;

uint8_t broadcastAddress[] = {0x48, 0x27, 0xe2, 0xea, 0x81, 0x7c};

typedef struct
{
  float air;
  float pressure;
  uint8_t ethanol;
} sensor_data_t;

sensor_data_t myData;
esp_now_peer_info_t peerInfo = {};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print(F("\r\nLast Packet Send Status:\t"));
  Serial.println(F(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail"));
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  SerialBT.begin("ArduHUD", true);
  if (!SerialBT.connect("OBDII"))
  {
    Serial.println(F("Couldn't connect to OBD scanner - Phase 1"));
    while (1)
      ;
  }

  if (!myELM327.begin(SerialBT, false, 2000))
  {
    Serial.println(F("Couldn't connect to OBD scanner - Phase 2"));
    while (1)
      ;
  }
  Serial.println(F("Connected to ELM327"));

  if (esp_now_init() != ESP_OK)
  {
    Serial.println(F("Error initializing ESP-NOW"));
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println(F("Failed to add peer"));
    return;
  }
}

void loop()
{
  // Fetch ethanol percentage
  myData.ethanol = 0;
  myELM327.nb_rx_state = ELM_SUCCESS;
  while (myELM327.nb_rx_state != ELM_SUCCESS && myELM327.nb_rx_state != ELM_GETTING_MSG)
  {
    myELM327.nb_rx_state = myELM327.ethonolPercent();
    delay(100); // Delay for ELM327 to respond
  }

  if (myELM327.nb_rx_state == ELM_SUCCESS)
  {
    myData.ethanol = myELM327.getLastResult().toInt();
    Serial.print(F("Ethanol Percent: "));
    Serial.print(myData.ethanol);
    Serial.println(F("%"));
  }
  else
  {
    myELM327.printError();
  }

  delay(100);

  // Fetch manifold pressure
  myData.pressure = 0;
  myELM327.nb_rx_state = ELM_SUCCESS;
  while (myELM327.nb_rx_state != ELM_SUCCESS && myELM327.nb_rx_state != ELM_GETTING_MSG)
  {
    myELM327.nb_rx_state = myELM327.manifoldPressure();
    delay(100); // Delay for ELM327 to respond
  }

  if (myELM327.nb_rx_state == ELM_SUCCESS)
  {
    myData.pressure = myELM327.getLastResult().toFloat();
    // Convert manifold pressure from kPa to psi and save in struct
    myData.pressure = myData.pressure * 0.145038;
    Serial.print("Pressure (psi): ");
    Serial.println(myData.pressure);
  }
  else
  {
    myELM327.printError();
  }

  delay(100);

  // Fetch air temperature
  myData.air = 0;
  myELM327.nb_rx_state = ELM_SUCCESS;
  while (myELM327.nb_rx_state != ELM_SUCCESS && myELM327.nb_rx_state != ELM_GETTING_MSG)
  {
    myELM327.nb_rx_state = myELM327.intakeAirTemp();
    delay(100); // Delay for ELM327 to respond
  }

  if (myELM327.nb_rx_state == ELM_SUCCESS)
  {
    myData.air = myELM327.getLastResult().toFloat();
    // Convert air temperature from Celsius to Fahrenheit and save in struct
    myData.air = myData.air * 9 / 5 + 32;
    Serial.print("Air Temp (F): ");
    Serial.println(myData.air);
  }
  else
  {
    myELM327.printError();
  }

  delay(100);

  // Sending data via ESP-NOW
  esp_err_t result = esp_now_send(peerInfo.peer_addr, (uint8_t *)&myData, sizeof(myData));
  if (result != ESP_OK)
  {
    Serial.println("Error sending the data");
  }

  delay(1000); // A longer delay between consecutive transmissions
}
