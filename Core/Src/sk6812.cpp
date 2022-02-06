#include "stm32f0xx_hal.h"
#include "sk6812.h"

extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim3_ch4_up;

#define PWM_HI (28)
#define PWM_LO (14)

// LED parameters
#define NUM_BPP (3) // SK6812 MINI
#define NUM_PIXELS (1)
#define NUM_BYTES (NUM_BPP * NUM_PIXELS)
#define DATA_LEN (NUM_BPP * 8)
#define RESET_LEN (72)

// LED color buffer
uint8_t rgb_arr[NUM_BYTES] = {0};

// LED write buffer
#define WR_BUF_LEN (DATA_LEN + RESET_LEN)
uint8_t wr_buf[WR_BUF_LEN] = {0};

// Set a single color (RGB) to index
void led_set_RGB(uint8_t r, uint8_t g, uint8_t b) {
  rgb_arr[0] = g; // g;
  rgb_arr[1] = r;
  rgb_arr[2] = b; // b;
}

// Shuttle the data to the LEDs!
void led_render() {
  if(hdma_tim3_ch4_up.State != HAL_DMA_STATE_READY) {
    // Ongoing transfer, cancel!
    for(uint8_t i = 0; i < WR_BUF_LEN; ++i) wr_buf[i] = 0;
    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
    return;
  }

  for(uint_fast8_t i = 0; i < 8; ++i) {
    wr_buf[i     ] = PWM_LO << (((rgb_arr[0] << i) & 0x80) > 0);
    wr_buf[i +  8] = PWM_LO << (((rgb_arr[1] << i) & 0x80) > 0);
    wr_buf[i + 16] = PWM_LO << (((rgb_arr[2] << i) & 0x80) > 0);
  }

  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t *)wr_buf, WR_BUF_LEN);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  for(uint8_t i = 0; i < WR_BUF_LEN; ++i) wr_buf[i] = 0;
  HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);

}
