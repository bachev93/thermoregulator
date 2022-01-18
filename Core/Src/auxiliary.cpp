#include <iterator>
#include <numeric>

#include "auxiliary.h"
// #include "sk6812.h"

namespace thermoregulator {
OperatingMode::OperatingMode(I2C_HandleTypeDef* hi2c) :
  params_(constants::low_mode),
  sensor1_(hi2c, ADDR::FIRST),
  sensor2_(hi2c, ADDR::SECOND) {
    sensor1_.begin();
    sensor1_.setAlertFunctionMode(true);
    sensor1_.setLowLimit(params_.low_threshold);
    sensor1_.setHighLimit(params_.high_threshold);

    sensor2_.begin();
    sensor2_.setAlertFunctionMode(true);
    sensor2_.setLowLimit(params_.low_threshold);
    sensor2_.setHighLimit(params_.high_threshold);
  }

void OperatingMode::change_mode() {
  switch (params_.mode) {
  case OperatingModeType::LOW:
    params_ = constants::middle_mode; break;
  case OperatingModeType::MIDDLE:
    params_ = constants::high_mode; break;
  case OperatingModeType::HIGH:
    params_ = constants::low_mode; break;
  default:
    // printf("unknown operating mode type\r\n");
    break;
  }

  sensor1_.setLowLimit(params_.low_threshold);
  sensor1_.setHighLimit(params_.high_threshold);

  sensor2_.setLowLimit(params_.low_threshold);
  sensor2_.setHighLimit(params_.high_threshold);
}

void OperatingMode::blink_leds() const {
  namespace c = thermoregulator::constants;
  HAL_GPIO_WritePin(c::mode_led1.port, c::mode_led1.pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(c::mode_led2.port, c::mode_led2.pin,
                    GPIO_PinState(params_.mode > OperatingModeType::LOW));
  HAL_GPIO_WritePin(c::mode_led3.port, c::mode_led3.pin,
                    GPIO_PinState(params_.mode > OperatingModeType::MIDDLE));
}

void OperatingMode::reset_leds() const {
  // printf("reset status leds\r\n");
  HAL_GPIO_WritePin(constants::mode_led1.port, constants::mode_led1.pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(constants::mode_led2.port, constants::mode_led2.pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(constants::mode_led3.port, constants::mode_led3.pin, GPIO_PIN_RESET);
}

OperatingModeParams OperatingMode::current_mode() const {
  return params_;
}

void OperatingMode::enable_heating() {
  sensor1_.setLowLimit(params_.low_threshold);
  sensor1_.setHighLimit(params_.high_threshold);

  sensor2_.setLowLimit(params_.low_threshold);
  sensor2_.setHighLimit(params_.high_threshold);
}

void OperatingMode::disable_heating() {
  sensor1_.setLowLimit(0);
  sensor1_.setHighLimit(0);

  sensor2_.setLowLimit(0);
  sensor2_.setHighLimit(0);
}

ButtonPressType check_button_press(GPIO_TypeDef* port, uint16_t pin, uint32_t time_ms_short, uint32_t time_ms_long) {
  auto result = ButtonPressType::NO_PRESS;
  auto curr_time = HAL_GetTick();
  auto diff_time = time_ms_long - time_ms_short;

  if(HAL_GPIO_ReadPin(port, pin)) {
    result = ButtonPressType::SHORT_PRESS;
    // debounce time
    HAL_Delay(time_ms_short);

    while(HAL_GPIO_ReadPin(port, pin)) {
      if(HAL_GetTick() - curr_time >= diff_time) {
        result = ButtonPressType::LONG_PRESS;
        break;
      }
    }
  }
  // short delay to counter debounce on release
  HAL_Delay(100);
  return result;
}

DeviceStatus device_status() {
  bool state1 = HAL_GPIO_ReadPin(constants::charge_state_pin1.port, constants::charge_state_pin1.pin);
  bool state2 = HAL_GPIO_ReadPin(constants::charge_state_pin2.port, constants::charge_state_pin2.pin);

  DeviceStatus res;
  if(state1 && state2) {
    res = DeviceStatus::DEVICE_WORKING;
  } else if(!state1 && state2) {
    res = DeviceStatus::DEVICE_CHARGING;
  } else if(state1 && !state2) {
    res = DeviceStatus::DEVICE_CHARGED;
  } else {
    res = DeviceStatus::UNKNOWN;
  }

  return res;
}

void change_addr_led_behaviour(DeviceStatus dev_state, Color color) {
  switch (dev_state) {
  case DeviceStatus::DEVICE_WORKING:
    set_addr_led_color(color);
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    // printf("device is working, address LED color depends on battery charging level\r\n");
    // TODO: change color by battery level. now the color is red;
    // led_render();
    // led_set_all_RGB(255, 0, 0);
    break;
  case DeviceStatus::DEVICE_CHARGING:
    set_addr_led_color(Color::Blue);
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    // printf("device is charging, PWM blue address LED\r\n");
    // led_render();
    // led_set_all_RGB(0, 0, 255);
    break;
  case DeviceStatus::DEVICE_CHARGED:
    set_addr_led_color(Color::Blue);
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    // printf("device is charged, blue address LED\r\n");
    // led_render();
    // led_set_all_RGB(0, 0, 255);
    break;
  default:
    set_addr_led_color(color);
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    // printf("unknown charging status\r\n");
    break;
  }
}

float get_battery_voltage(ADC_HandleTypeDef* hadc) {
  // TODO: sample_size should become compile-time constant
  static const auto samples_size = 10ul;
  uint32_t samples[samples_size];

  HAL_ADC_Start(hadc);
  for(size_t i = 0ul; i < samples_size; ++i) {
    HAL_ADC_PollForConversion(hadc, 1);
    samples[i] = HAL_ADC_GetValue(hadc);
  }
  HAL_ADC_Stop(hadc);

  auto average = std::accumulate(std::begin(samples), std::end(samples), 0u) / samples_size;
  // TODO: change 4095 to constant 12 bit integer max val
  return constants::vbat / 4095 * average;
}

void poweroff() {
  // printf("powering off\r\n");

  // GPIO_InitTypeDef GPIO_InitStruct = {0};
  // GPIO_InitStruct.Pin = constants::btn.pin;
  // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  // HAL_GPIO_Init(constants::btn.port, &GPIO_InitStruct);
}

Color get_color_by_battery_level(float bat_level) {
  // TODO: add here some maths to calculate color
  return Color::Green;
}

void set_addr_led_color(Color) {
  // TODO: For Max
}
}
