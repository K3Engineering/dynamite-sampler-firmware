#ifndef _USER_KVS_H
#define _USER_KVS_H

#include <stddef.h>

bool initUserKeyValStorage();
bool readDeviceByIdx(const char *cmd, char *reply, size_t replySz);
bool writeDeviceKeyVal(const char *cmd);
bool readDeviceKey(const char *cmd, char *reply, size_t replySz);
bool deleteDeviceKey(const char *cmd);

#endif // _USER_KVS_H
