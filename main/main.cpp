#include <esp_arduino_version.h>
#include <esp_mac.h>
#include <esp_pm.h>

#include "adc_proc.h"
#include "ble_proc.h"

#include "debug_pin.h"
#include <HardwareSerial.h>

#ifndef GIT_REVISION
#define GIT_REVISION "?"
#endif

// Core0 is for BLE
// Core1 is for everything else. Setup & loop run on core 1.
// ISR is handled on the core that sets it.
constexpr uint32_t CORE_BLE = 0;
constexpr uint32_t CORE_APP = 1;

extern "C" void app_main(void) {
	initArduino();

	Serial.begin(115200);
	Serial.println("Starting Arduino version:");
	Serial.println(ESP_ARDUINO_VERSION_STR);
	Serial.print("Git revision: ");
	Serial.println(GIT_REVISION);
	Serial.print("Running on Core: ");
	Serial.println(xPortGetCoreID());
	// Serial.print("Config PM SLP IRAM OPT (put lightsleep into ram):");
	// Serial.println(CONFIG_PM_SLP_IRAM_OPT);

	Serial.println("MAC address:");
	uint64_t _chipmacid = 0LL;
	esp_efuse_mac_get_default((uint8_t *)(&_chipmacid));
	Serial.printf("0x%" PRIx64 "\n", _chipmacid);

	setupBle(CORE_BLE);
	setupAdc(CORE_APP);

	const esp_pm_config_t pmConfig = {
	    .max_freq_mhz       = 80,
	    .min_freq_mhz       = 10,
	    .light_sleep_enable = false,
	};
	esp_err_t err = esp_pm_configure(&pmConfig);
	Serial.print("pm err ");
	Serial.println(err);
	// ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
	Serial.println("Started!");
}
