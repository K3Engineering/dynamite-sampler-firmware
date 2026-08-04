#ifndef _HW_ID_H
#define _HW_ID_H

#include <esp_mac.h>
#include <stdio.h>

// Hardware ID derived from the eFuse MAC address. out is 12 hex chars + NUL.
inline void hwIdStr(char out[13]) {
	uint8_t mac[8]; // size - see esp_efuse_mac_get_default() docs.
	esp_efuse_mac_get_default(mac);
	snprintf(out, 13, "%02x%02x%02x%02x%02x%02x", mac[5], mac[4], mac[3], mac[2], mac[1],
	         mac[0]);
}

#endif // _HW_ID_H
