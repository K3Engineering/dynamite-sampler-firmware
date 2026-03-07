#ifndef _RUNTIME_STATS_H
#define _RUNTIME_STATS_H

// To enable profiling of tasks,
// use config menu or add this options to the SDKCONFIG file
// CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS = y
// Also you need log trace level >= INFO to see the output

void setupStats(int core);

#endif // _RUNTIME_STATS_H
