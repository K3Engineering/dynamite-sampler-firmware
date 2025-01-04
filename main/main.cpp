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
constexpr uint32_t CORE_BLE = CONFIG_BT_NIMBLE_PINNED_TO_CORE;
constexpr uint32_t CORE_APP = CONFIG_ARDUINO_RUNNING_CORE;
static_assert(CORE_BLE != CORE_APP);

#include "esp_partition.h"

static void setupCalibration() {
// awaiting final spec
#pragma pack(push, 1)
	struct NvsDataLoadcellCalibration {
		uint32_t calibration0;
		uint32_t calibration1;
		uint32_t calibration2;
	};
#pragma pack(pop)

	constexpr esp_partition_type_t    CUSTOM_PARTITION_CALIBRATION = esp_partition_type_t(0x40);
	constexpr esp_partition_subtype_t CUSTOM_SUBTYPE_CALIBRATION   = esp_partition_subtype_t(6);

	NvsDataLoadcellCalibration data;
	if (const esp_partition_t *ptr = esp_partition_find_first(
	        CUSTOM_PARTITION_CALIBRATION, CUSTOM_SUBTYPE_CALIBRATION, "loadcell_calib")) {
		if (ESP_OK == esp_partition_read_raw(ptr, 0, &data, sizeof(data))) {
			Serial.print("ADC calibration data ");
			Serial.println(data.calibration0);
		}
	}
}

extern "C" void app_main(void) {
	initArduino();

	Serial.begin(115200);
	Serial.println("Starting Arduino version:");
	Serial.println(ESP_ARDUINO_VERSION_STR);
	Serial.print("Git revision: ");
	Serial.println(GIT_REVISION);
	Serial.print("Running on Core: ");
	Serial.println(xPortGetCoreID());

#ifndef CONFIG_MOCK_ADC
	Serial.println("Mock adc flag not set");
#else
	Serial.print("Mock ADC flag: ");
	Serial.println(CONFIG_MOCK_ADC);
#endif

	// Serial.print("Config PM SLP IRAM OPT (put lightsleep into ram):");
	// Serial.println(CONFIG_PM_SLP_IRAM_OPT);

	Serial.println("MAC address:");
	uint64_t _chipmacid = 0LL;
	esp_efuse_mac_get_default((uint8_t *)(&_chipmacid));
	Serial.printf("0x%" PRIx64 "\n", _chipmacid);

	setupBle(CORE_BLE);
	setupAdc(CORE_APP);
	setupCalibration();

	constexpr esp_pm_config_t pmConfig = {
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
