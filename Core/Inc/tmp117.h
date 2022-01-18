#ifndef TMP117_H
#define TMP117_H

#include "stm32f0xx.h"
#include "stm32f0xx_hal_i2c.h"

namespace thermoregulator {
enum class ADDR : uint8_t { _GND = 0b1001000 << 1, _V = 0b1001001 << 1, FIRST = _GND, SECOND = _V };

static const auto DEVICE_ID_VALUE = 0x0117; // Value found in the device ID register on reset (page 24 Table 3 of datasheet)
static const auto TMP117_RESOLUTION = .0078125f;    // Resolution of the device, found on (page 1 of datasheet)
static const auto CONTINUOUS_CONVERSION_MODE = 0b00;    // Continuous Conversion Mode
static const auto ONE_SHOT_MODE = 0b11; // One Shot Conversion Mode
static const auto SHUTDOWN_MODE = 0b01; // Shutdown Conversion Mode

// Address of the registers. This can be found on page 23 of the datasheet
enum class TMP117_Register : uint8_t {
  TMP117_TEMP_RESULT = 0X00,
  TMP117_CONFIGURATION = 0x01,
  TMP117_T_HIGH_LIMIT = 0X02,
  TMP117_T_LOW_LIMIT = 0X03,
  TMP117_EEPROM_UL = 0X04,
  TMP117_EEPROM1 = 0X05,
  TMP117_EEPROM2 = 0X06,
  TMP117_TEMP_OFFSET = 0X07,
  TMP117_EEPROM3 = 0X08,
  TMP117_DEVICE_ID = 0X0F
};

// Configuration register found on page 25 Figure 26 and Table 6
union ConfigurationReg {
  struct ConfigurationFields {
	uint8_t EMPTY : 1;			// Empty bit in register
	uint8_t TMP_SOFT_RESET : 1; // Software reset bit
	uint8_t DR_ALERT : 1;		// ALERT pin select bit
	uint8_t POL : 1;			// ALERT pin polarity bit
	uint8_t T_NA : 1;			// Therm/alert mode select
	uint8_t AVG : 2;			// Conversion averaging modes
	uint8_t CONV : 3;			// Conversion cycle bit
	uint8_t MOD : 2;			// Set conversion mode
	uint8_t EEPROM_BUSY : 1;	// EEPROM busy flag
	uint8_t DATA_READY : 1;		// Data ready flag
	uint8_t LOW_ALERT : 1;		// Low Alert flag
	uint8_t HIGH_ALERT : 1;		// High Alert flag
    };
  uint16_t CONFIGURATION_COMBINED;
};

// Device ID Register used for checking if the device ID is the same as declared
// This register is found on Page 30 of the datasheet in Table 15 and Figure 34
union DeviceIDReg {
  struct DeviceIDFields {
    uint16_t DID : 12; // Indicates the device ID
    uint8_t REV : 4;   // Indicates the revision number
  };
  uint16_t DEVICE_ID_COMBINED;
};

class tmp117 {
public:
  explicit tmp117(I2C_HandleTypeDef* hi2c, ADDR addr);
  bool begin();
  float readTempC();
  float getTemperatureOffset();
  void setLowLimit(float lowLimit);
  float getLowLimit();
  float getHighLimit();
  void setHighLimit(float highLimit);
  uint16_t getConfigurationRegister();
  bool dataReady();
  void setAlertFunctionMode(bool setAlertMode);
  bool getAlertFunctionMode();

private:
  I2C_HandleTypeDef* hi2c_;
  ADDR addr_;

  uint16_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint16_t data);
};
}

#endif // TMP117_H
