#include "Arduino.h"

extern "C" void app_main()
{
    initArduino();

    // Arduino-like setup()
    Serial.begin(115200);
    while(!Serial){
        ; // wait for serial port to connect
    }
    int pin_sleep_profile = 2;
    pinMode(pin_sleep_profile, OUTPUT);
    digitalWrite(pin_sleep_profile, HIGH);

    // Arduino-like loop()
    while(true){
        Serial.println("loop");
        // esp_sleep_enable_timer_wakeup(1000000); // 1 second
        esp_sleep_enable_timer_wakeup(1); // 1 milli second
        digitalWrite(pin_sleep_profile, LOW);
        esp_light_sleep_start();
        digitalWrite(pin_sleep_profile, HIGH);

        // sleep(1);
    }

    // WARNING: if program reaches end of function app_main() the MCU will restart.
}