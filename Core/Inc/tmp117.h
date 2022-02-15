#ifndef TMP117_H
#define TMP117_H

#include "stm32f0xx.h"
#include "stm32f0xx_hal_i2c.h"

namespace thermoregulator {
enum class ADDR : uint8_t { _GND = 0b1001000 << 1, _V = 0b1001001 << 1, FIRST = _GND, SECOND = _V };

static const auto DEVICE_ID_VALUE = 0x0117; // Value found in the device ID register on reset (page 24 Table 3 of datasheet)
static const auto TMP117_RESOLUTION = .0078125f;    // Resolution of the device, found on (page 1 of datasheet)

// Address of the registers. This can be found on page 23 of the datasheet
enum class TMP117_Register : uint8_t {
  TMP117_TEMP_RESULT = 0X00,
  TMP117_CONFIGURATION = 0x01,
  TMP117_T_HIGH_LIMIT = 0X02,
  TMP117_T_LOW_LIMIT = 0X03,
  TMP117_DEVICE_ID = 0X0F
};

class tmp117 {
public:
  explicit tmp117(I2C_HandleTypeDef* hi2c, ADDR addr);
  bool check() const;
  void setLowLimit(float lowLimit);
  void setHighLimit(float highLimit);
  void enableAlertFunctionMode();

private:
  I2C_HandleTypeDef* hi2c_;
  ADDR addr_;

  int16_t readRegister(TMP117_Register reg) const;
  HAL_StatusTypeDef writeRegister(TMP117_Register reg, uint16_t data);
};
}

#endif // TMP117_H
