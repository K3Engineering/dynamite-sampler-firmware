#ifndef _USER_KVS_H
#define _USER_KVS_H

#include <stddef.h>

#include <esp_err.h>

enum class KvsResult { Ok, Rejected, Error };

bool initUserKeyValStorage();
KvsResult processKvsCommand(const char *rq, size_t rqLen, char *reply, size_t replySz);
esp_err_t kvsReadBoardModel(char *out, size_t outLen);
esp_err_t kvsReadDeviceName(char *out, size_t outLen);

#endif // _USER_KVS_H
