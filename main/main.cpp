// #include <Arduino.h>
// #include <BLE2902.h>
// #include <BLEDevice.h>
// #include <BLEServer.h>
// #include <BLEUtils.h>

// #include "ADS1120.h"

// // #include "FreeRTOS.h"
// #include <freertos/stream_buffer.h>

// BLEServer *pServer = NULL;
// BLECharacteristic *pCharacteristic = NULL;
// bool deviceConnected = false;
// bool oldDeviceConnected = false;

// StreamBufferHandle_t xStreamBuffer = NULL;

// #define SERVICE_UUID "e331016b-6618-4f8f-8997-1a2c7c9e5fa3"
// #define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ADS1120 adc;

// // Core0 is for BLE
// // Core1 is for everything else. Setup & loop run on core 1.
// // ISR is handled on the core that sets it.

// // TODO change to core 1 and setup ISR
// TaskHandle_t Core0Task;
// void codeForCore0Task(void *parameter) {
//   while (true) {
//     Serial.print("Task 0 loop on core ");
//     Serial.println(xPortGetCoreID());
//     delay(1000);
//   }
// }

// class MyServerCallbacks : public BLEServerCallbacks {
//   void onConnect(BLEServer *pServer) {
//     deviceConnected = true;
//     BLEDevice::startAdvertising();
//     Serial.print("On connect callback on core ");
//     Serial.println(xPortGetCoreID());
//   };

//   void onDisconnect(BLEServer *pServer) {
//     deviceConnected = false;
//     Serial.print("On disco callback on core ");
//     Serial.println(xPortGetCoreID());
//   }
// };

// void setup_ble() {
//   // Create the BLE Device
//   BLEDevice::init("Larry ESP32");

//   // Create the BLE Server
//   pServer = BLEDevice::createServer();
//   pServer->setCallbacks(new MyServerCallbacks());

//   // Create the BLE Service
//   BLEService *pService = pServer->createService(SERVICE_UUID);

//   // Create a BLE Characteristic
//   pCharacteristic = pService->createCharacteristic(
//       CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ |
//                                BLECharacteristic::PROPERTY_WRITE |
//                                BLECharacteristic::PROPERTY_NOTIFY |
//                                BLECharacteristic::PROPERTY_INDICATE);

//   // Create a BLE Descriptor
//   pCharacteristic->addDescriptor(new BLE2902());

//   // Start the service
//   pService->start();

//   // Start advertising
//   BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
//   pAdvertising->addServiceUUID(SERVICE_UUID);
//   pAdvertising->setScanResponse(false);
//   pAdvertising->setMinPreferred(
//       0x0);  // set value to 0x00 to not advertise this parameter
//   BLEDevice::startAdvertising();
//   Serial.println("Waiting a client connection to notify...");
// }

// // When we flag a piece of code with the IRAM_ATTR attribute, the compiled code
// // is placed in the ESP32’s Internal RAM (IRAM). Otherwise the code is kept in
// // Flash. And Flash on ESP32 is much slower than internal RAM.
// void IRAM_ATTR isr_adc_drdy() {
//   uint32_t adcValue = adc.readADC();
//   xStreamBufferSendFromISR(xStreamBuffer, &adcValue, sizeof(adcValue), 0);
// }

// void setup_adc() {
//   uint8_t PIN_NUM_CLK = 12;
//   uint8_t PIN_NUM_MISO = 11;
//   uint8_t PIN_NUM_MOSI = 13;
//   uint8_t PIN_DRDY = 10;
//   uint8_t PIN_CS = 2; // TODO remove

//   adc.begin(PIN_NUM_CLK, PIN_NUM_MISO, PIN_NUM_MOSI, PIN_CS, PIN_DRDY);

//   adc.setGain(128);
//   adc.setMultiplexer(0x00); // AIN0 AIN1
//   adc.setPGAbypass(0);
  
//   adc.setDataRate(0b110); // 2000 SPS
//   // adc.setDataRate(0b00); // 40 SPS
//   adc.setOpMode(2); // turbo
//   adc.setConversionMode(1); // continuous

//   // adc.setVoltageRef(0); // internal
//   // adc.setVoltageRef(1); // refp0 refn0
//   adc.setVoltageRef(3); //analog voltage supply

//   adc.setDRDYmode(0); // no drdy mode

//   uint8_t contents = adc.readRegister(0);
//   Serial.print("register 0 contents ");
//   Serial.println(contents);

//   contents = adc.readRegister(1);
//   Serial.print("register 1 contents ");
//   Serial.println(contents);

//   contents = adc.readRegister(2);
//   Serial.print("register 2 contents ");
//   Serial.println(contents);

//   contents = adc.readRegister(3);
//   Serial.print("register 3 contents ");
//   Serial.println(contents);

//   xStreamBuffer = xStreamBufferCreate(2000, 1);
//   assert( xStreamBuffer != NULL );

//   // TODO figure out if this function call is needed
//   // The digitalPinToInterrupt() function takes a pin as an argument, and
//   // returns the same pin if it can be used as an interrupt.
//   //  Setup ISR to handle the falling edge of drdy
//   attachInterrupt(digitalPinToInterrupt(PIN_DRDY), isr_adc_drdy, FALLING);
// }

// void setup() {
//   Serial.begin(115200);
//   Serial.println("Starting!");

//   // Set up Core 0 task handler
//   // xTaskCreatePinnedToCore(codeForCore0Task, "Core 0 task", 10000, NULL, 1,
//   //                         &Core0Task, 0);

//   delay(500);
//   setup_ble();
//   delay(500);
//   setup_adc();  // TODO force run this on core 1 to make sure ISR is on core 1
//   delay(500);

//   Serial.println("Started!");
//   // Setup ADC
//   // Setup handling the interupt from the
// }

// void loop() {
//   // notify changed value
//   // Serial.print("loop ");
//   // Serial.print(deviceConnected);
//   // Serial.print(oldDeviceConnected);
//   // Serial.println("");

//   if (deviceConnected) {

//     uint8_t batch[251];
//     size_t bytesRead = 0;
//     bytesRead = xStreamBufferReceive(xStreamBuffer, batch, 251, 0);

//     if (bytesRead > 0) {
//         pCharacteristic->setValue(batch, bytesRead);
//         pCharacteristic->notify();
//         delay(10);
//     }
//     // bluetooth stack will go into congestion, if too many packets
//     // are sent, in 6 hours test i was able to go as low as 3ms
//   }
//   // disconnecting
//   if (!deviceConnected && oldDeviceConnected) {
//     delay(500);  // give the bluetooth stack the chance to get things ready
//     Serial.println("start advertising");

//     pServer->startAdvertising();  // restart advertising
//     Serial.print("disconnecting loop callback on core ");
//     Serial.println(xPortGetCoreID());
//     oldDeviceConnected = false;
//   }
//   // connecting
//   if (deviceConnected && !oldDeviceConnected) {
//     // do stuff here on connecting
//     oldDeviceConnected = true;
//   }
// }

#include "NimBLEDevice.h"
#include "NimBLELog.h"
#include "esp_pm.h"

#include <stdio.h>

extern "C" {void app_main(void);}

static NimBLEServer* pServer;

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
        printf("Client address: %s\n", connInfo.getAddress().toString().c_str());

        /** We can use the connection handle here to ask for different connection parameters.
         *  Args: connection handle, min connection interval, max connection interval
         *  latency, supervision timeout.
         *  Units; Min/Max Intervals: 1.25 millisecond increments.
         *  Latency: number of intervals allowed to skip.
         *  Timeout: 10 millisecond increments, try for 3x interval time for best results.
         */
        pServer->updateConnParams(connInfo.getConnHandle(), 24, 4*48, 0, 18);
    };

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
        printf("Client disconnected - start advertising\n");
        NimBLEDevice::startAdvertising();
    };

    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) {
        printf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
        pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 60);
    };

/********************* Security handled here **********************
****** Note: these are the same return values as defaults ********/
    uint32_t onPassKeyDisplay(){
        printf("Server Passkey Display\n");
        /** This should return a random 6 digit number for security
         *  or make your own static passkey as done here.
         */
        return 123456;
    };

    void onConfirmPIN(NimBLEConnInfo& connInfo, uint32_t pass_key){
        printf("The passkey YES/NO number: %" PRIu32 "\n", pass_key);
        /** Inject false if passkeys don't match. */
        NimBLEDevice::injectConfirmPIN(connInfo, true);
    };

    void onAuthenticationComplete(NimBLEConnInfo& connInfo){
        /** Check that encryption was successful, if not we disconnect the client */
        if(!connInfo.isEncrypted()) {
            NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
            printf("Encrypt connection failed - disconnecting client\n");
            return;
        }
        printf("Starting BLE work!");
    };
};

/** Handler class for characteristic actions */
class CharacteristicCallbacks: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
        printf("%s : onRead(), value: %s\n",
               pCharacteristic->getUUID().toString().c_str(),
               pCharacteristic->getValue().c_str());
    }

    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
        printf("%s : onWrite(), value: %s\n",
               pCharacteristic->getUUID().toString().c_str(),
               pCharacteristic->getValue().c_str());
    }

    /** Called before notification or indication is sent,
     *  the value can be changed here before sending if desired.
     */
    void onNotify(NimBLECharacteristic* pCharacteristic) {
        printf("Sending notification to clients\n");
    }

    /**
     *  The value returned in code is the NimBLE host return code.
     */
    void onStatus(NimBLECharacteristic* pCharacteristic, int code) {
        printf("Notification/Indication return code: %d, %s\n",
               code, NimBLEUtils::returnCodeToString(code));
    }

    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) {
        std::string str = "Client ID: ";
        str += connInfo.getConnHandle();
        str += " Address: ";
        str += connInfo.getAddress().toString();
        if(subValue == 0) {
            str += " Unsubscribed to ";
        }else if(subValue == 1) {
            str += " Subscribed to notfications for ";
        } else if(subValue == 2) {
            str += " Subscribed to indications for ";
        } else if(subValue == 3) {
            str += " Subscribed to notifications and indications for ";
        }
        str += std::string(pCharacteristic->getUUID());

        printf("%s\n", str.c_str());
    }
};

/** Handler class for descriptor actions */
class DescriptorCallbacks : public NimBLEDescriptorCallbacks {
    void onWrite(NimBLEDescriptor* pDescriptor, NimBLEConnInfo& connInfo) {
        std::string dscVal = pDescriptor->getValue();
        printf("Descriptor witten value: %s\n", dscVal.c_str());
    };

    void onRead(NimBLEDescriptor* pDescriptor, NimBLEConnInfo& connInfo) {
        printf("%s Descriptor read\n", pDescriptor->getUUID().toString().c_str());
    };;
};


/** Define callback instances globally to use for multiple Charateristics \ Descriptors */
static DescriptorCallbacks dscCallbacks;
static CharacteristicCallbacks chrCallbacks;

void notifyTask(void * parameter){
    for(;;) {
        if(pServer->getConnectedCount()) {
            NimBLEService* pSvc = pServer->getServiceByUUID("BAAD");
            if(pSvc) {
                NimBLECharacteristic* pChr = pSvc->getCharacteristic("F00D");
                if(pChr) {
                    pChr->notify();
                }
            }
        }
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main(void) {


    esp_pm_config_esp32_t pm_config = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 10,
        .light_sleep_enable = true,
    };
    esp_pm_configure(&pm_config);
    // ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    printf("Starting NimBLE Server\n");

    /** sets device name */
    NimBLEDevice::init("NimBLE");

    /** Set the IO capabilities of the device, each option will trigger a different pairing method.
     *  BLE_HS_IO_DISPLAY_ONLY    - Passkey pairing
     *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
     *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
     */
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); // use passkey
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

    /** 2 different ways to set security - both calls achieve the same result.
     *  no bonding, no man in the middle protection, secure connections.
     *
     *  These are the default values, only shown here for demonstration.
     */
    //NimBLEDevice::setSecurityAuth(false, false, true);
    NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pDeadService = pServer->createService("DEAD");
    NimBLECharacteristic* pBeefCharacteristic = pDeadService->createCharacteristic(
                                               "BEEF",
                                               NIMBLE_PROPERTY::READ |
                                               NIMBLE_PROPERTY::WRITE |
                               /** Require a secure connection for read and write access */
                                               NIMBLE_PROPERTY::READ_ENC |  // only allow reading if paired / encrypted
                                               NIMBLE_PROPERTY::WRITE_ENC   // only allow writing if paired / encrypted
                                              );

    pBeefCharacteristic->setValue("Burger");
    pBeefCharacteristic->setCallbacks(&chrCallbacks);

    /** 2902 and 2904 descriptors are a special case, when createDescriptor is called with
     *  either of those uuid's it will create the associated class with the correct properties
     *  and sizes. However we must cast the returned reference to the correct type as the method
     *  only returns a pointer to the base NimBLEDescriptor class.
     */
    NimBLE2904* pBeef2904 = (NimBLE2904*)pBeefCharacteristic->createDescriptor("2904");
    pBeef2904->setFormat(NimBLE2904::FORMAT_UTF8);
    pBeef2904->setCallbacks(&dscCallbacks);


    NimBLEService* pBaadService = pServer->createService("BAAD");
    NimBLECharacteristic* pFoodCharacteristic = pBaadService->createCharacteristic(
                                               "F00D",
                                               NIMBLE_PROPERTY::READ |
                                               NIMBLE_PROPERTY::WRITE |
                                               NIMBLE_PROPERTY::NOTIFY
                                              );

    pFoodCharacteristic->setValue("Fries");
    pFoodCharacteristic->setCallbacks(&chrCallbacks);

    /** Custom descriptor: Arguments are UUID, Properties, max length in bytes of the value */
    NimBLEDescriptor* pC01Ddsc = pFoodCharacteristic->createDescriptor(
                                               "C01D",
                                               NIMBLE_PROPERTY::READ |
                                               NIMBLE_PROPERTY::WRITE|
                                               NIMBLE_PROPERTY::WRITE_ENC, // only allow writing if paired / encrypted
                                               20
                                              );
    pC01Ddsc->setValue("Send it back!");
    pC01Ddsc->setCallbacks(&dscCallbacks);

    /** Start the services when finished creating all Characteristics and Descriptors */
    pDeadService->start();
    pBaadService->start();

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    /** Add the services to the advertisment data **/
    pAdvertising->addServiceUUID(pDeadService->getUUID());
    pAdvertising->addServiceUUID(pBaadService->getUUID());
    /** If your device is battery powered you may consider setting scan response
     *  to false as it will extend battery life at the expense of less data sent.
     */
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    printf("Advertising Started\n");

    xTaskCreate(notifyTask, "notifyTask", 5000, NULL, 1, NULL);
}