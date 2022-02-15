#include "tmp117.h"
#include "constants.h"

namespace thermoregulator {
/**
 * @brief конструктор
 * 
 * @param hi2c - указатель на хендлер I2C
 * @param addr - адрес устройства 
 */
tmp117::tmp117(I2C_HandleTypeDef* hi2c, ADDR addr) :
  hi2c_(hi2c),
  addr_(addr) {}

/**
 * @brief 
 * check connection and device id
 */
bool tmp117::check() const {
  auto deviceID = readRegister(TMP117_Register::TMP117_DEVICE_ID);
  return DEVICE_ID_VALUE == deviceID;
}

/**
 * @brief 
 * 	This function sets the alert function mode to either "alert" or
	"therm" mode. This can be found on page 25 of the datasheet.
 */
void tmp117::enableAlertFunctionMode() {
  auto alert_func_mode = readRegister(TMP117_Register::TMP117_CONFIGURATION);
  alert_func_mode |= 1 << 4;  //set register bit to be 1

  writeRegister(TMP117_Register::TMP117_CONFIGURATION, alert_func_mode);
}

/**
 * @brief 
 * 	This function allows the user to set the low limit register to whatever
	specified value, as long as in the range for the temperature sensor. This
	function can be used as a threshold for Therm mode and or Alert mode.
	The values are signed integers since they can be negative.
 * @param lowLimit 
 */
void tmp117::setLowLimit(float lowLimit) {
  int16_t final_limit = lowLimit / TMP117_RESOLUTION;
  writeRegister(TMP117_Register::TMP117_T_LOW_LIMIT, final_limit);
}

/**
 * @brief 
 * 	This function allows the user to set the high limit register to whatever
	specified value, as long as in the range for the temperature sensor. This
	function can be used as a threshold for Therm mode and or Alert mode
	The values are signed integers since they can be negative.
 * @param highLimit 
 */
void tmp117::setHighLimit(float highLimit) {
  int16_t final_limit = highLimit / TMP117_RESOLUTION;
  writeRegister(TMP117_Register::TMP117_T_HIGH_LIMIT, final_limit);
}

/**
 * @brief 
 * Wire data to a TMP117 register
 * @param reg
 * @param data
 */
HAL_StatusTypeDef tmp117::writeRegister(TMP117_Register reg, uint16_t data) {
  uint8_t buf[3] = {static_cast<uint8_t>(reg),
                    static_cast<uint8_t>(data >> static_cast<uint8_t>(8)),
                    static_cast<uint8_t>(data & 0xff)};
  return HAL_I2C_Master_Transmit(hi2c_, static_cast<uint16_t>(addr_), buf, 3, constants::i2c_timeout_ms);
}

/**
 * @brief
  This function reads the register bytes from the sensor when called upon.
  This reads 2 bytes of information from the 16-bit registers.
  * @param reg - регистр, из которого надо прочитать
  * @return uint16_t
  */
int16_t tmp117::readRegister(TMP117_Register reg) const {
  auto state = HAL_I2C_Master_Transmit(hi2c_, static_cast<uint16_t>(addr_), reinterpret_cast<uint8_t *>(&reg), 1, constants::i2c_timeout_ms);
  if (state == HAL_OK) {
    uint8_t buf[2] = {0};
    state = HAL_I2C_Master_Receive(hi2c_, static_cast<uint16_t>(addr_), buf, 2, constants::i2c_timeout_ms);
    if (state == HAL_OK) {
      int16_t res = (buf[0] << 8) | buf[1];
      // int16_t res = (buf[1] << 8) | buf[0];
      return res;
    } else {
      return -1;
    }
  }
  return -1;
}

}
