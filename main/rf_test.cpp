#include "rf_test.h"

#include <esp_log.h>

#if CONFIG_DYNAMITE_RF_TX_TEST

#if !CONFIG_BT_NIMBLE_DTM_MODE_TEST
#error "The RF TX test requires CONFIG_BT_NIMBLE_DTM_MODE_TEST=y"
#endif

#include <NimBLEDevice.h>

#include <host/ble_gap.h>

constexpr char TAG[] = "RFTEST";

// Fixed BLE channel to hammer. Carrier frequency: 2402 + 2 * channel MHz.
constexpr uint8_t txChannel = CONFIG_DYNAMITE_RF_TX_TEST_CHANNEL;
static_assert(txChannel <= 39);

// Number of payload bytes in each test packet.
constexpr uint8_t testDataLen = 0x25; // 37 bytes, legacy max

// Test packet payload type, see BT Core Spec Vol 4, Part E, 7.8.28.
#if CONFIG_DYNAMITE_RF_TX_TEST_PAYLOAD_PRBS9
constexpr uint8_t payloadType = 0x00;
#elif CONFIG_DYNAMITE_RF_TX_TEST_PAYLOAD_F0
constexpr uint8_t payloadType = 0x01;
#elif CONFIG_DYNAMITE_RF_TX_TEST_PAYLOAD_AA
constexpr uint8_t payloadType = 0x02;
#elif CONFIG_DYNAMITE_RF_TX_TEST_PAYLOAD_PRBS15
constexpr uint8_t payloadType = 0x03;
#elif CONFIG_DYNAMITE_RF_TX_TEST_PAYLOAD_FF
constexpr uint8_t payloadType = 0x04;
#elif CONFIG_DYNAMITE_RF_TX_TEST_PAYLOAD_00
constexpr uint8_t payloadType = 0x05;
#elif CONFIG_DYNAMITE_RF_TX_TEST_PAYLOAD_0F
constexpr uint8_t payloadType = 0x06;
#elif CONFIG_DYNAMITE_RF_TX_TEST_PAYLOAD_55
constexpr uint8_t payloadType = 0x07;
#endif

// PHY: 0x01 = 1M, 0x02 = 2M, 0x03 = Coded S=8, 0x04 = Coded S=2.
#if CONFIG_DYNAMITE_RF_TX_TEST_PHY_1M
constexpr uint8_t txPhy = 0x01;
#elif CONFIG_DYNAMITE_RF_TX_TEST_PHY_2M
constexpr uint8_t txPhy = 0x02;
#elif CONFIG_DYNAMITE_RF_TX_TEST_PHY_CODED_S8
constexpr uint8_t txPhy = 0x03;
#elif CONFIG_DYNAMITE_RF_TX_TEST_PHY_CODED_S2
constexpr uint8_t txPhy = 0x04;
#endif

static const char *dtmEvtName(uint8_t evt) {
	switch (evt) {
	case BLE_GAP_DTM_TX_START_EVT: return "TX_START";
	case BLE_GAP_DTM_RX_START_EVT: return "RX_START";
	case BLE_GAP_DTM_END_EVT: return "END";
	default: return "?";
	}
}

// The controller reports the result of the DTM commands asynchronously.
static int onDtmGapEvent(ble_gap_event *event, void *) {
	if (event->type == BLE_GAP_EVENT_TEST_UPDATE) {
		ESP_LOGI(TAG, "DTM %s, status 0x%02x", dtmEvtName(event->dtm_state.update_evt),
		         event->dtm_state.status);
	}
	return 0;
}

#endif // CONFIG_DYNAMITE_RF_TX_TEST

bool rfTestStartIfEnabled() {
#if CONFIG_DYNAMITE_RF_TX_TEST
	NimBLEDevice::setCustomGapHandler(onDtmGapEvent);
	// Max out the TX power for RF measurements.
	if (!NimBLEDevice::setPower(9)) {
		ESP_LOGE(TAG, "Set TX power failed");
	}
	// HCI LE Enhanced Transmitter Test: transmit continuously on one channel.
	if (int rc = ble_gap_dtm_enh_tx_start(txChannel, testDataLen, payloadType, txPhy)) {
		ESP_LOGE(TAG, "DTM TX test start failed, rc=%d", rc);
		return false;
	}
	ESP_LOGI(TAG, "DTM TX test: ch %u (%u MHz), %uB payload type %u, phy %u", txChannel,
	         2402 + 2 * txChannel, testDataLen, payloadType, txPhy);
	return true;
#else
	return false;
#endif
}
