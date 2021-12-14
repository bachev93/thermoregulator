#ifndef TMP117_H
#define TMP117_H

#include <cstdint>

#include "stm32f0xx.h"
#include "stm32f0xx_hal_i2c.h"

namespace thermoregulator {
enum class RG : uint8_t { TEMP, CONF, HLIM, LLIM };
enum class ADDR : uint8_t { _GND = 0b1001000 << 1, _V = 0b1001001 << 1, FIRST = _GND, SECOND = _V };

static const auto lsb = .0078125;

/**
 @brief: буфер обмена по шине I2C
 @detailed: 3 байта
 */
#pragma pack(push, 1)
struct message {
    RG reg_;
    int16_t word;
};
#pragma pack(pop)
static_assert(sizeof(message) == 3 * sizeof(uint8_t), "");


/**
 * @brief: Функция перевода
 * @param: Значение температуры в градусах Цельсия
 * @return: Значение температуры в условных единицах цены младшего бита
 */
int16_t t2lsb_v(int16_t v);

/**
 * @brief: Функция перевода
 * @param: Значение температуры в условных единицах цены младшего бита
 * @return: Значение температуры в градусах Цельсия
 */
float lsb_v2t(int16_t v);

struct tmp117 {    
    /**
     @brief: конструктор
     @param: hi2c - указатель на хендлер I2C
     @param: ll - low limit, градусы Цельсия
     @param: hl - high limit, градусы Цельсия
     @param: conf - конфигурация датчика (по-умолчанию RESET)
    */
    explicit tmp117(I2C_HandleTypeDef* hi2c, ADDR addr, int16_t ll, int16_t hl, int16_t conf = 0x02);
    float get_temperature() const;

    void set_low_limit(int16_t);
    void set_high_limit(int16_t);
private:
    I2C_HandleTypeDef* hi2c_;
    ADDR addr_;
};
}

#endif // TMP117_H
