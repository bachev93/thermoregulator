#ifndef _LED_DRIVER_SK6812
#define _LED_DRIVER_SK6812

#include <cstdint>

void led_set_RGB(uint8_t r, uint8_t g, uint8_t b);
void led_render();

#endif  // _LED_DRIVER_SK6812
