#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "stm32f0xx_hal_gpio.h"

namespace thermoregulator {
enum class OperatingModeType {LOW, MIDDLE, HIGH, MODE_COUNT};
struct OperatingModeParams {
  OperatingModeType mode;
  int16_t low_threshold;
  int16_t high_threshold;
};

struct Pin {
  GPIO_TypeDef* port;
  uint16_t pin;
};

namespace constants {
// battery voltage max value, in volts
const float vbat = 3.3f;

// battery voltage level for powering off the device, in volts
const float vbat_low_level = 1.5f;

// battery charging level check time, in sec
const int battery_check_time = 10;

// working time, in sec
// const int working_time = 1200;
const int working_time = 30;

// idle time, in sec
// const int idle_time = 300;
const int idle_time = 15;

// blink show status time, in sec
const int status_time = 5;

const OperatingModeParams low_mode = {OperatingModeType::LOW, 38, 40};
const OperatingModeParams middle_mode = {OperatingModeType::MIDDLE, 40, 45};
const OperatingModeParams high_mode = {OperatingModeType::HIGH, 45, 50};

const Pin mode_led1 = {GPIOA, GPIO_PIN_7};
const Pin mode_led2 = {GPIOA, GPIO_PIN_6};
const Pin mode_led3 = {GPIOA, GPIO_PIN_5};
const Pin btn = {GPIOA, GPIO_PIN_0};
const Pin charge_state_pin1 = {GPIOA, GPIO_PIN_2};
const Pin charge_state_pin2 = {GPIOA, GPIO_PIN_3};
}
}

#endif // CONSTANTS_H
