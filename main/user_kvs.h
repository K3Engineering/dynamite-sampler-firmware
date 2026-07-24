#ifndef _USER_KVS_H
#define _USER_KVS_H

#include <stddef.h>

bool initUserKeyValStorage();
bool processKvsCommand(const char *rq, size_t rqLen, char *reply, size_t replySz);

#endif // _USER_KVS_H
