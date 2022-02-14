#include "stm32f0xx_hal.h"
#include "sk6812.h"

extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim3_ch4_up;

#define PWM_HI (28)
#define PWM_LO (14)

// LED parameters
// SK6812 MINI, 3 colors
#define NUM_BPP (3)
#define LEDS_COUNT (2)
// 8 bytes by one color. we have NUM_BPP colors and LEDS_COUNT leds
#define DATA_LEN (NUM_BPP * 8 * LEDS_COUNT)
// reset time. Must be >= 80 us
#define RESET_LEN (72)

namespace thermoregulator {
// LED color buffer
uint8_t rgb_arr[NUM_BPP] = {0};

// LED write buffer
#define WR_BUF_LEN (DATA_LEN + RESET_LEN)
uint8_t wr_buf[WR_BUF_LEN] = {0};

// Set a single color (RGB) to index
void led_set_RGB(uint8_t red, uint8_t green, uint8_t blue) {
  rgb_arr[0] = green;
  rgb_arr[1] = red;
  rgb_arr[2] = blue;
}

// Shuttle the data to the LEDs!
void led_render() {
  if(hdma_tim3_ch4_up.State != HAL_DMA_STATE_READY) {
    // Ongoing transfer, cancel!
    for(uint8_t i = 0; i < WR_BUF_LEN; ++i) wr_buf[i] = 0;
    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
    return;
  }

  for(uint8_t i = 0; i < 8; ++i) {
    // first LED
    wr_buf[i     ] = PWM_LO << (((rgb_arr[0] << i) & 0x80) > 0);
    wr_buf[i +  8] = PWM_LO << (((rgb_arr[1] << i) & 0x80) > 0);
    wr_buf[i + 16] = PWM_LO << (((rgb_arr[2] << i) & 0x80) > 0);
    // second LED, same color
    wr_buf[i + 24] = PWM_LO << (((rgb_arr[0] << i) & 0x80) > 0);
    wr_buf[i + 32] = PWM_LO << (((rgb_arr[1] << i) & 0x80) > 0);
    wr_buf[i + 40] = PWM_LO << (((rgb_arr[2] << i) & 0x80) > 0);
  }

  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t *)wr_buf, WR_BUF_LEN);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  for(uint8_t i = 0; i < WR_BUF_LEN; ++i) wr_buf[i] = 0;
  HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
}
}
