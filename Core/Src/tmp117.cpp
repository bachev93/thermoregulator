#include "tmp117.h"

namespace thermoregulator {
tmp117::tmp117(I2C_HandleTypeDef* hi2c, ADDR addr, int16_t ll, int16_t hl, int16_t conf)
    : hi2c_(hi2c)
    , addr_(addr) {
    message m { RG::CONF, conf };
    HAL_I2C_Master_Transmit(hi2c_, static_cast<uint16_t>(addr_), reinterpret_cast<uint8_t*>(&m), sizeof(message), HAL_MAX_DELAY);
    set_low_limit(ll);
    set_high_limit(hl);
}

void tmp117::set_low_limit(int16_t v) {
    message m { RG::LLIM, t2lsb_v(v) };
    HAL_I2C_Master_Transmit(hi2c_, static_cast<uint16_t>(addr_), reinterpret_cast<uint8_t*>(&m), sizeof(message), HAL_MAX_DELAY);
}

void tmp117::set_high_limit(int16_t v) {
    message m { RG::HLIM, t2lsb_v(v) };
    HAL_I2C_Master_Transmit(hi2c_, static_cast<uint16_t>(addr_), reinterpret_cast<uint8_t*>(&m), sizeof(message), HAL_MAX_DELAY);
}

float tmp117::get_temperature() const {
    message m { RG::TEMP, 0 };
    HAL_I2C_Master_Transmit(hi2c_, static_cast<uint16_t>(addr_), reinterpret_cast<uint8_t*>(&m), 1, HAL_MAX_DELAY);
    HAL_Delay(1);
    HAL_I2C_Master_Receive(hi2c_, static_cast<uint16_t>(addr_) , reinterpret_cast<uint8_t*>(&m.word), sizeof(m.word), HAL_MAX_DELAY);
    return lsb_v2t(m.word);
}

int16_t t2lsb_v(int16_t v) {
    return int16_t(v / lsb);
}

float lsb_v2t(int16_t v) {
    return v * lsb;
}
}
