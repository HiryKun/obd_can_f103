/**
 * @file obd.h
 * @author HiryKun (1951086367@qq.com)
 * @brief 适用于CAN的OBD通讯驱动库
 * @version 0.1
 * @date 2025-04-01
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __OBD_H
#define __OBD_H

/* 发送/接收缓存大小 */
#define OBD_BUFFER_SIZE         32
/* OBD的CAN数据帧长度 */
#define OBD_CAN_DLC             8

/* OBD请求CAN ID */
#define OBD_REQ_FUNC_ADDR       0x7DF
#define OBD_RESP_ECU1_ADDR      0x7E8
#define OBD_RESP_ECU2_ADDR      0x7E9
#define OBD_RESP_ECU3_ADDR      0x7EA
#define OBD_RESP_ECU4_ADDR      0x7EB
#define OBD_RESP_ECU5_ADDR      0x7EC
#define OBD_RESP_ECU6_ADDR      0x7ED
#define OBD_RESP_ECU7_ADDR      0x7EE
#define OBD_RESP_ECU8_ADDR      0x7EF

/* OBD服务ID，目前只需要01 */
#define OBD_SERVICE_RX_01       0x41
#define OBD_SERVICE_TX_01       0x01
/* OBD PIDs */
#define OBD_PID_ENGINE_RPM      0x0C
#define OBD_PID_VEHICLE_SPEED   0x0D
/* 占位符 */
#define OBD_TX_PLACEHOLDER      0x00,0x00,0x00,0x00,0x00,0x00

/**
 * @brief 缓存数据类型，使用结构体存储的循环队列
 * 
 */
typedef struct {
    uint8_t front;
    uint8_t rear;
    uint8_t data[OBD_BUFFER_SIZE][OBD_CAN_DLC];
} OBD_BufferTypeDef;

/**
 * @brief 车辆状态结构体
 * 
 */
typedef struct {
    uint32_t    canStatus;      //CAN状态
    uint16_t    engineRPM;      //引擎转速
    uint8_t     speed;          //车辆速度
} OBD_VehicleStatusTypeDef;

/**
 * @brief 缓存状态枚举类型
 * 
 */
typedef enum {
    BUFFER_EMPTY    = 0x00,     //缓存为空（空队列）
    BUFFER_FULL,                //缓存已满（满队列）
    BUFFER_BUSY                 //缓存有数据（队列不空但未满）
} OBD_BufferStateTypeDef;


/**
 * @brief OBD初始化函数
 * 
 * 设置CAN滤波器，启动CAN，并激活接收中断
 * 
 * @param hcan CAN外设句柄
 */
HAL_StatusTypeDef OBD_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief 查询发送缓存状态
 * 
 * @return OBD_BufferStateTypeDef 缓存状态
 */
OBD_BufferStateTypeDef OBD_GetTxBufferState(void);

/**
 * @brief 查询接收缓存状态
 * 
 * @return OBD_BufferStateTypeDef 缓存状态
 */
OBD_BufferStateTypeDef OBD_GetRxBufferState(void);

/**
 * @brief OBD发送缓存处理函数
 * 
 * 从发送缓存中取出数据，发送到CAN总线上
 * 
 * 一次只发送一帧数据，应被频繁调用
 * 
 * 但要避免循环长期阻塞，应时常检查 CAN_Error_Code
 * 
 * @return HAL_StatusTypeDef 是否成功把数据放入发送邮箱
 * @retval HAL_OK 成功写入发送邮箱
 * @retval HAL_BUSY 发送邮箱还有数据未发送，或上次发送后还未接收到回复报文
 * @retval HAL_ERROR 缓存为空，无需发送
 */
HAL_StatusTypeDef OBD_TxBufferProcess(void);

/**
 * @brief OBD接收缓存处理函数
 * 
 * 从接收缓存中取出数据，解析并存储到车辆状态数组中
 * 
 * 一次只解析一帧数据，应被频繁调用
 * 
 * @return HAL_StatusTypeDef 解析结果
 * @retval HAL_OK 成功解析
 * @retval HAL_BUSY 未知PID报文，无法解析
 * @retval HAL_ERROR 缓存为空，无需解析
 */
HAL_StatusTypeDef OBD_RxBufferProcess(void);

/**
 * @brief 获取车辆状态
 * 
 * @param pTarget 存放车辆状态的结构体指针
 * @retval HAL_OK 成功获取车辆状态
 * @retval HAL_ERROR 目标结构体指针为空
 */
HAL_StatusTypeDef OBD_GetVehicleStatus(OBD_VehicleStatusTypeDef * pTarget);

/**
 * @brief 发送数据，添加进发送缓存
 * 
 * @param data 要发送的数据指针，数据长度应该为8Byte
 * @return HAL_StatusTypeDef 是否成功添加进发送缓存
 * @retval HAL_OK 添加成功
 * @retval HAL_ERROR 缓存已满，添加失败
 */
HAL_StatusTypeDef OBD_TxMessage(uint8_t *pData);

#endif