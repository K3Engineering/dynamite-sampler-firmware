#ifndef _DEBUG_PIN_H
#define _DEBUG_PIN_H

#include <soc/gpio_num.h>

// There are 2 pin on the v2.0.1 board that can be used for debugging.
constexpr gpio_num_t PIN_DEBUG_TOP = GPIO_NUM_47;
constexpr gpio_num_t PIN_DEBUG_BOT = GPIO_NUM_21;

#endif // _DEBUG_PIN_H
