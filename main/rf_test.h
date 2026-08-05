#ifndef _RF_TEST_H
#define _RF_TEST_H

// BLE Direct Test Mode (DTM) transmitter test, for RF debugging.
// When enabled in menuconfig (Dynamite Sampler hardware configuration ->
// "Continuous BLE TX test on a fixed channel"), the device does not advertise.
// Instead the BLE controller transmits test packets continuously on a single
// fixed channel (no frequency hopping) at the configured TX power.
//
// Returns true when the RF test is active and normal BLE operation
// (advertising) should not be started.
bool rfTestStartIfEnabled();

#endif // _RF_TEST_H
