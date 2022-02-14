#include "auxiliary.h"
#include "sk6812.h"

namespace thermoregulator {
OperatingMode::OperatingMode(I2C_HandleTypeDef* hi2c) :
  params_(constants::low_mode),
  sensor1_(hi2c, ADDR::FIRST),
  sensor2_(hi2c, ADDR::SECOND) {}

OperatingMode::operator bool() const {
  return sensor1_.check() && sensor2_.check();
}

void OperatingMode::change_mode(OperatingModeType mode_type) {
  switch (mode_type) {
    case OperatingModeType::LOW:
      params_ = constants::low_mode;
      break;
    case OperatingModeType::MIDDLE:
      params_ = constants::middle_mode;
      break;
    case OperatingModeType::HIGH:
      params_ = constants::high_mode;
      break;
    default:
      params_ = constants::low_mode;
      break;
  }

  sensor1_.setLowLimit(params_.low_threshold);
  sensor1_.setHighLimit(params_.high_threshold);
  sensor2_.setLowLimit(params_.low_threshold);
  sensor2_.setHighLimit(params_.high_threshold);

  blink_leds();
}

void OperatingMode::blink_leds() const {
  namespace c = thermoregulator::constants;
  HAL_GPIO_WritePin(c::mode_led1.port, c::mode_led1.pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(c::mode_led2.port, c::mode_led2.pin, GPIO_PinState(params_.mode > OperatingModeType::LOW));
  HAL_GPIO_WritePin(c::mode_led3.port, c::mode_led3.pin, GPIO_PinState(params_.mode > OperatingModeType::MIDDLE));
}

void OperatingMode::set_alert_function_mode() {
  sensor1_.enableAlertFunctionMode();
  sensor2_.enableAlertFunctionMode();
}

// void OperatingMode::reset_leds() const {
//   // printf("reset status leds\r\n");
//   HAL_GPIO_WritePin(constants::mode_led1.port, constants::mode_led1.pin, GPIO_PIN_RESET);
//   HAL_GPIO_WritePin(constants::mode_led2.port, constants::mode_led2.pin, GPIO_PIN_RESET);
//   HAL_GPIO_WritePin(constants::mode_led3.port, constants::mode_led3.pin, GPIO_PIN_RESET);
// }

void OperatingMode::enable_heating() {
  sensor1_.setLowLimit(params_.low_threshold);
  sensor1_.setHighLimit(params_.high_threshold);
  sensor2_.setLowLimit(params_.low_threshold);
  sensor2_.setHighLimit(params_.high_threshold);
}

void OperatingMode::disable_heating() {
  sensor1_.setLowLimit(0.);
  sensor1_.setHighLimit(0.);
  sensor2_.setLowLimit(0.);
  sensor2_.setHighLimit(0.);
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

void change_addr_led_behaviour(DeviceStatus dev_state) {
  switch (dev_state) {
  case DeviceStatus::DEVICE_WORKING:
    // в режиме работы будет повторный вызов изменения состояния по напряжению
    break;
  case DeviceStatus::DEVICE_CHARGING:
    // PWM blue color
    for(int i = 0; i < blue.b; ++i) {
      led_set_RGB(0, 0, i);
      led_render();
      HAL_Delay(10);
    }
    for(int i = blue.b; i >= 0; --i) {
      led_set_RGB(0, 0, i);
      led_render();
      HAL_Delay(10);
    }
    break;
  case DeviceStatus::DEVICE_CHARGED:
    set_addr_led_color(blue);
    break;
  case DeviceStatus::UNKNOWN:
    // PWM red color
    for(int i = 0; i < red.r; ++i) {
      led_set_RGB(i, 0, 0);
      led_render();
      HAL_Delay(10);
    }
    for(int i = red.r; i >= 0; --i) {
      led_set_RGB(i, 0, 0);
      led_render();
      HAL_Delay(10);
    }
    break;
  default:
    set_addr_led_color(red);
    break;
  }
}

void change_addr_led_behaviour(float voltage) {
  set_addr_led_color(volt2color(voltage));
  // set_addr_led_color(yellow);
}

void reset_addr_led() {
  set_addr_led_color(off);
}

float get_battery_voltage(ADC_HandleTypeDef* hadc) {
  static const auto samples_size = 10ul;
  uint32_t accumulator = 0u;

  HAL_ADC_Start(hadc);
  for(size_t i = 0ul; i < samples_size; ++i) {
    HAL_ADC_PollForConversion(hadc, 1);
    accumulator += HAL_ADC_GetValue(hadc);
  }
  HAL_ADC_Stop(hadc);

  auto average = accumulator / samples_size;
  return constants::adc_res * average;
}

void poweroff() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = constants::btn.pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(constants::btn.port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(constants::btn.port, constants::btn.pin, GPIO_PIN_SET);
}

Color volt2color(float bat_level) {
  static const auto range = constants::v_adc - constants::v_adc_low_level;
  static const auto delta = range / 4.f;

  if (bat_level >= constants::v_adc - delta) {
    return green;
  } else if (bat_level >= constants::v_adc - 2 * delta) {
    return yellow;
  } else if (bat_level >= constants::v_adc - 3 * delta) {
    return orange;
  } else {
    return red;
  }
}

void set_addr_led_color(Color c) {
  led_set_RGB(c.r, c.g, c.b);
  led_render();
}
}
