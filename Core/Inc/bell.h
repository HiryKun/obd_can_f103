#ifndef __BELL_H
#define __BELL_H

/**
 * @brief 警铃状态结构体
 * 
 */
typedef enum {
    BELL_OFF = 0,   //铃声关闭
    BELL_LIMITED,   //铃声有限次开启
    BELL_FORVER     //铃声保持开启
} BELL_StateTypeDef;

/**
 * @brief 警铃初始化函数
 * 
 *  此函数仅重置标志位，不初始化定时器外设
 * 
 * @param htim 产生PWM的定时器句柄
 * @param channel PWM通道，数值为 TIM_CHANNEL_x
 * @return HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT() 返回值
 */
HAL_StatusTypeDef BELL_Init(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief 获取警铃状态
 * 
 * @return BELL_StateTypeDef 
 * @retval BELL_OFF 铃声关闭
 * @retval BELL_LIMITED 铃声有限次开启
 * @retval BELL_FORVER 铃声保持开启
 */
BELL_StateTypeDef BELL_GetState(void);

/**
 * @brief 有限次响起警铃
 * 
 * @param cnt 响铃次数 0 ~ 255
 * @return HAL_StatusTypeDef 
 * @retval HAL_OK 成功
 * @retval HAL_BUSY 已在响铃
 * @retval HAL_ERROR 参数错误
 */
HAL_StatusTypeDef BELL_Start(uint8_t cnt);

/**
 * @brief 保持警铃响起
 * 
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef BELL_StartForver(void);

/**
 * @brief 停止响铃
 * 
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef BELL_Stop(void);

#endif