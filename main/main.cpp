#include <Arduino.h>

#include "NimBLEDevice.h"
#include "NimBLELog.h"
#include "esp_pm.h"

#include <stdio.h>

#include "ADS1120.h"

// #include "FreeRTOS.h"
#include <freertos/stream_buffer.h>

// extern "C" {void app_main(void);}

// static NimBLEServer* pServer;


BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

StreamBufferHandle_t xStreamBuffer = NULL;

#define SERVICE_UUID "e331016b-6618-4f8f-8997-1a2c7c9e5fa3"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

ADS1120 adc;

// Core0 is for BLE
// Core1 is for everything else. Setup & loop run on core 1.
// ISR is handled on the core that sets it.

// TODO change to core 1 and setup ISR
// TaskHandle_t Core0Task;
// void codeForCore0Task(void *parameter) {
//   while (true) {
//     Serial.print("Task 0 loop on core ");
//     Serial.println(xPortGetCoreID());
//     delay(1000);
//   }
// }

class MyServerCallbacks : public BLEServerCallbacks {
  // void onConnect(BLEServer *pServer) {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
    deviceConnected = true;
    BLEDevice::startAdvertising();
    Serial.print("On connect callback on core ");
    Serial.println(xPortGetCoreID());
  };

  // void onDisconnect(BLEServer *pServer) {
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
    deviceConnected = false;
    Serial.print("On disco callback on core ");
    Serial.println(xPortGetCoreID());
  }
};

void setup_ble() {
  // Create the BLE Device
  BLEDevice::init("1234567890");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  uint32_t prop =   NIMBLE_PROPERTY::READ   |
                    NIMBLE_PROPERTY::WRITE  |
                    NIMBLE_PROPERTY::NOTIFY |
                    NIMBLE_PROPERTY::INDICATE;
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, prop);

  // Create a BLE Descriptor
  // Automatically created by nimble. Needed for bluedroid
  // pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  // This has been removed in nimble
  // pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

// When we flag a piece of code with the IRAM_ATTR attribute, the compiled code
// is placed in the ESP32’s Internal RAM (IRAM). Otherwise the code is kept in
// Flash. And Flash on ESP32 is much slower than internal RAM.
void IRAM_ATTR isr_adc_drdy() {
  uint32_t adcValue = adc.readADC();
  xStreamBufferSendFromISR(xStreamBuffer, &adcValue, sizeof(adcValue), 0);
}

void setup_adc() {
  uint8_t PIN_NUM_CLK = 12;
  uint8_t PIN_NUM_MISO = 11;
  uint8_t PIN_NUM_MOSI = 13;
  uint8_t PIN_DRDY = 10;
  uint8_t PIN_CS = 2; // TODO remove

  adc.begin(PIN_NUM_CLK, PIN_NUM_MISO, PIN_NUM_MOSI, PIN_CS, PIN_DRDY);

  adc.setGain(128);
  adc.setMultiplexer(0x00); // AIN0 AIN1
  adc.setPGAbypass(0);
  
  adc.setDataRate(0b110); // 2000 SPS
  // adc.setDataRate(0b00); // 40 SPS
  adc.setOpMode(2); // turbo
  adc.setConversionMode(1); // continuous

  // adc.setVoltageRef(0); // internal
  // adc.setVoltageRef(1); // refp0 refn0
  adc.setVoltageRef(3); //analog voltage supply

  adc.setDRDYmode(0); // no drdy mode

  uint8_t contents = adc.readRegister(0);
  Serial.print("register 0 contents ");
  Serial.println(contents);

  contents = adc.readRegister(1);
  Serial.print("register 1 contents ");
  Serial.println(contents);

  contents = adc.readRegister(2);
  Serial.print("register 2 contents ");
  Serial.println(contents);

  contents = adc.readRegister(3);
  Serial.print("register 3 contents ");
  Serial.println(contents);

  xStreamBuffer = xStreamBufferCreate(2000, 1);
  assert( xStreamBuffer != NULL );

  // TODO figure out if this function call is needed
  // The digitalPinToInterrupt() function takes a pin as an argument, and
  // returns the same pin if it can be used as an interrupt.
  //  Setup ISR to handle the falling edge of drdy
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY), isr_adc_drdy, FALLING);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting!");

    // // esp_pm_config_esp32_t pm_config = {
    // esp_pm_config_t pm_config = {
    //     .max_freq_mhz = 80,
    //     .min_freq_mhz = 10,
    //     .light_sleep_enable = false,
    // };
    // esp_pm_configure(&pm_config);
    // ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    // Set up Core 0 task handler
    // xTaskCreatePinnedToCore(codeForCore0Task, "Core 0 task", 10000, NULL, 1,
    //                         &Core0Task, 0);

    delay(500);
    setup_ble();
    delay(500);
    setup_adc();  // TODO force run this on core 1 to make sure ISR is on core 1
    delay(500);

    Serial.println("Started!");
    // Setup ADC
    // Setup handling the interupt from the
}

void loop() {
  // notify changed value
  // Serial.print("loop ");
  // Serial.print(deviceConnected);
  // Serial.print(oldDeviceConnected);
  // Serial.println("");
  vTaskDelay(1); // Needed otherwise watchdog timer
  if (deviceConnected) {

    uint8_t batch[251];
    size_t bytesRead = 0;
    bytesRead = xStreamBufferReceive(xStreamBuffer, batch, 251, 0);

    if (bytesRead > 0) {
        pCharacteristic->setValue(batch, bytesRead);
        pCharacteristic->notify();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // bluetooth stack will go into congestion, if too many packets
    // are sent, in 6 hours test i was able to go as low as 3ms
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    vTaskDelay(pdMS_TO_TICKS(500));  // give the bluetooth stack the chance to get things ready
    Serial.println("start advertising");

    pServer->startAdvertising();  // restart advertising
    Serial.print("disconnecting loop callback on core ");
    Serial.println(xPortGetCoreID());
    oldDeviceConnected = false;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    Serial.println("When does this happen?");
    oldDeviceConnected = true;
  }

}

