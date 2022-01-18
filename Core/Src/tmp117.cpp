#include "tmp117.h"

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
bool tmp117::begin() {
  auto deviceID = readRegister(static_cast<uint8_t>(TMP117_Register::TMP117_DEVICE_ID));
  if(deviceID != DEVICE_ID_VALUE) {
    return false;
  }
  return true;
}

/**
 * @brief 
  This function reads the register bytes from the sensor when called upon.
  This reads 2 bytes of information from the 16-bit registers. 
  * @param reg - регистр, из которого надо прочитать
  * @return uint16_t 
  */
uint16_t tmp117::readRegister(uint8_t reg) {
  auto state = HAL_I2C_Master_Transmit(hi2c_, static_cast<uint16_t>(addr_), &reg, 1, HAL_MAX_DELAY);
  if(state == HAL_OK) {
    uint8_t buf[2] = {0};
    state = HAL_I2C_Master_Receive(hi2c_, static_cast<uint16_t>(addr_), buf, 2, HAL_MAX_DELAY);
    if(state == HAL_OK) {
      int16_t res = (buf[0] << 8) | buf[1];
      // int16_t res = (buf[1] << 8) | buf[0];
      return res;
    } else {
      return -1;
    }
  }
  return -1;
}

/**
 * @brief 
 * This function reads the temperature reading from the sensor
and returns the value in degrees celsius.

NOTE: The data type of digitalTemp is a signed integer, meaning that the 
value of the binary number being read will be negative if the MSB is 1,
and positive if the bit is 0. 
  * @return float 
  */
float tmp117::readTempC() {
  int16_t digital_tempC = readRegister(static_cast<uint8_t>(TMP117_Register::TMP117_TEMP_RESULT));
  return digital_tempC * TMP117_RESOLUTION;
}

/**
 * @brief Get the Temperature Offset object
 * This function reads the temperature offset. This reads from the register
value 0x07 (TMP117_TEMP_OFFSET). This can be found on page 23 of the 
datasheet. 
  * 
  * @return float 
  */
float tmp117::getTemperatureOffset() {
  int16_t offset = readRegister(static_cast<uint8_t>(TMP117_Register::TMP117_TEMP_OFFSET));
  return offset * TMP117_RESOLUTION;
}

/**
 * @brief 
 * 	This function reads the low limit register that is set by the user.
	The values are signed integers since they can be negative.
 * @return float 
 */
float tmp117::getLowLimit() {
  int16_t low_limit = 0;
  low_limit = readRegister(static_cast<uint8_t>(TMP117_Register::TMP117_T_LOW_LIMIT));
  return low_limit * TMP117_RESOLUTION;
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
  writeRegister(static_cast<uint8_t>(TMP117_Register::TMP117_T_LOW_LIMIT), final_limit);
}

/**
 * @brief 
 * 	This function reads the high limit register that is set by the user.
	The values are signed integers since they can be negative.
 * @return float 
 */
float tmp117::getHighLimit(){
  int16_t high_limit = 0;
  high_limit = readRegister(static_cast<uint8_t>(TMP117_Register::TMP117_T_HIGH_LIMIT));
  return high_limit * TMP117_RESOLUTION;
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
  writeRegister(static_cast<uint8_t>(TMP117_Register::TMP117_T_HIGH_LIMIT), final_limit);
}

/**
 * @brief 
 * 	This function reads configuration register. Use this function if you need to read
	certain flags before they are cleared. This can be found on page 25 of the
	datasheet.
 * @return uint16_t 
 */
uint16_t tmp117::getConfigurationRegister() {
  return readRegister(static_cast<uint8_t>(TMP117_Register::TMP117_CONFIGURATION));
}

/**
 * @brief 
 * 	This function checks to see if there is data ready to be sent
	from the TMP117. This can be found in Page 25 Table 6 of the 
	data sheet.
 * @return true 
 * @return false 
 */
bool tmp117::dataReady() {
  auto response = readRegister(static_cast<uint8_t>(TMP117_Register::TMP117_CONFIGURATION));

  // If statement to see if the 13th bit of the register is 1 or not
  if(response & 1 << 13) {
    return true;
  }
  return false;
}

/**
 * @brief 
 * 	This function sets the alert function mode to either "alert" or
	"therm" mode. This can be found on page 25 of the datasheet.
 * @param setAlertMode 
 */
void tmp117::setAlertFunctionMode(bool setAlertMode) {
  auto alert_func_mode = readRegister(static_cast<uint8_t>(TMP117_Register::TMP117_CONFIGURATION));

  if(setAlertMode) {  // 1: Therm mode
    alert_func_mode |= 1 << 4;  //set register bit to be 1
  }else { // 0: alert mode
    alert_func_mode &= ~(1 << 4); //set register bit to be 0
  }

  writeRegister(static_cast<uint8_t>(TMP117_Register::TMP117_CONFIGURATION), alert_func_mode);
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool tmp117::getAlertFunctionMode() {
  auto response = readRegister(static_cast<uint8_t>(TMP117_Register::TMP117_CONFIGURATION));
  if(response & 1 << 4) {
    return true;  // therm mode
  }
  return false; // alert mode
}

/**
 * @brief 
 * Wire data to a TMP117 register
 * @param reg 
 * @param data 
 */
void tmp117::writeRegister(uint8_t reg, uint16_t data) {
  uint8_t buf[3] = {reg, data >> 8, data};
  auto state = HAL_I2C_Master_Transmit(hi2c_, static_cast<uint16_t>(addr_), buf, 3, HAL_MAX_DELAY);
}
}
