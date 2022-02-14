#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "stm32f0xx_hal_gpio.h"

namespace thermoregulator {
enum class OperatingModeType {LOW, MIDDLE, HIGH};
struct OperatingModeParams {
  OperatingModeType mode;
  int16_t low_threshold;
  int16_t high_threshold;
};

struct Pin {
  GPIO_TypeDef* port;
  uint16_t pin;
};

struct Color {
  explicit Color(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness = 20) :
  r(red / brightness),
  g(green / brightness),
  b(blue / brightness) {}
  uint8_t r, g, b;
};

namespace constants {
// ADC resolution, VCC / 2^12
const float adc_res = 3.3 / (1 << 12);

// ADC pin voltage max value, in volts
const float v_adc = 2.1f;
// ADC pin voltage level for powering off the device, in volts
const float v_adc_low_level = 1.5f;

// battery charging level and I2C connection check time, in sec
const uint8_t check_time = 10;

// working time, in sec
// const int working_time = 1200;
const uint16_t working_time = 30;

// idle time, in sec
// const int idle_time = 300;
const uint16_t idle_time = 15;

// blink show status time, in sec
const uint8_t status_time = 2;

// I2C response time. in sec
// const uint8_t wait_for_tmp117 = 120;
const uint8_t wait_for_tmp117 = 15;

const OperatingModeParams low_mode = {OperatingModeType::LOW, 38, 40};
const OperatingModeParams middle_mode = {OperatingModeType::MIDDLE, 39, 41};
const OperatingModeParams high_mode = {OperatingModeType::HIGH, 40, 42};

const Pin mode_led1 = {GPIOA, GPIO_PIN_7};
const Pin mode_led2 = {GPIOA, GPIO_PIN_6};
const Pin mode_led3 = {GPIOA, GPIO_PIN_5};
const Pin btn = {GPIOA, GPIO_PIN_0};
const Pin charge_state_pin1 = {GPIOA, GPIO_PIN_2};
const Pin charge_state_pin2 = {GPIOA, GPIO_PIN_3};

const auto i2c_timeout_ms = 1000u;

const Color red(255, 0, 0);
const Color green(0, 255, 0);
const Color blue(0, 0, 255);
const Color orange(255, 153, 51);
const Color yellow(255, 255, 0);
// const Color off = {0, 0, 0};
}
}

#endif // CONSTANTS_H
