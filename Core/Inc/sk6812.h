#ifndef _LED_DRIVER_SK6812
#define _LED_DRIVER_SK6812

#include <cstdint>

namespace thermoregulator {
void led_set_RGB(uint8_t red, uint8_t green, uint8_t blue);
void led_render();
}

#endif  // _LED_DRIVER_SK6812
