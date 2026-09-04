#ifndef _USER_KVS_H
#define _USER_KVS_H

#include <stddef.h>

#include <esp_err.h>

bool initUserKeyValStorage();
bool processKvsCommand(const char *rq, size_t rqLen, char *reply, size_t replySz);
esp_err_t kvsReadFactoryString(const char *key, char *out, size_t *outLen);

#endif // _USER_KVS_H
