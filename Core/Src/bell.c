#include "stm32f1xx_hal.h"
#include "bell.h"

static TIM_HandleTypeDef *htimBell;
static uint32_t pwmChannel;
static uint8_t bellCNT = 0;
static uint8_t forverFlag = 0;

HAL_StatusTypeDef BELL_Init(TIM_HandleTypeDef *htim, uint32_t channel) {
    htimBell = htim;
    pwmChannel = channel;
    return HAL_TIM_PWM_Stop_IT(htimBell, pwmChannel);
}

BELL_StateTypeDef BELL_GetState(void) {
    if (forverFlag) return BELL_FORVER;
    return (TIM_CHANNEL_STATE_GET(htimBell, pwmChannel) == HAL_TIM_CHANNEL_STATE_BUSY) ? BELL_LIMITED : BELL_OFF;
}

HAL_StatusTypeDef BELL_Start(uint8_t cnt) {
    if (cnt <= 0) return HAL_ERROR;
    if (TIM_CHANNEL_STATE_GET(htimBell, pwmChannel) == HAL_TIM_CHANNEL_STATE_BUSY) return HAL_BUSY;
    bellCNT = cnt;
    HAL_TIM_PWM_Start_IT(htimBell, pwmChannel);
    return HAL_OK;
}

HAL_StatusTypeDef BELL_StartForver(void) {
    forverFlag = 1;
    if (TIM_CHANNEL_STATE_GET(htimBell, pwmChannel) == HAL_TIM_CHANNEL_STATE_BUSY) return HAL_OK;
    return HAL_TIM_PWM_Start_IT(htimBell, pwmChannel);
}

HAL_StatusTypeDef BELL_Stop(void) {
    forverFlag = 0;
    return HAL_TIM_PWM_Stop_IT(htimBell, pwmChannel);
}

void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef * htim) {
    if (htim == htimBell) {
        if (forverFlag) return;
        --bellCNT;
        if (bellCNT <= 0) HAL_TIM_PWM_Stop_IT(htimBell, pwmChannel);
    }
}