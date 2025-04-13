/**
 * @file obd.c
 * @author HiryKun (1951086367@qq.com)
 * @brief 适用于CAN的OBD通讯驱动库
 * @version 0.1
 * @date 2025-04-01
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "stm32f1xx_hal.h"
#include "obd.h"
#include <string.h>

static OBD_VehicleStatusTypeDef vehicleStatus;
static CAN_HandleTypeDef *hcanOBD;
static uint8_t allowTx = 1;
static uint32_t lastMailbox = CAN_TX_MAILBOX0;
static OBD_BufferTypeDef txBuffer, rxBuffer;
static CAN_TxHeaderTypeDef txHeader = {
    .StdId  = OBD_REQ_FUNC_ADDR,
    .ExtId  = 0,
    .RTR    = CAN_ID_STD,
    .RTR    = CAN_RTR_DATA,
    .DLC    = OBD_CAN_DLC,
   .TransmitGlobalTime = DISABLE
};

static OBD_BufferStateTypeDef GetBufferState(OBD_BufferTypeDef *pBuffer, uint8_t queueSize) {
    if (pBuffer->front == pBuffer->rear) return BUFFER_EMPTY;
    if ((pBuffer->rear + 1) % queueSize == pBuffer->front) return BUFFER_FULL;
    return BUFFER_BUSY;
}

static HAL_StatusTypeDef Dequeue(OBD_BufferTypeDef *pBuffer, uint8_t queueSize) {
    if (GetBufferState(pBuffer, queueSize) == BUFFER_EMPTY) return HAL_ERROR;
    ++pBuffer->front;
    if (pBuffer->front >= queueSize) pBuffer->front = 0;
    return HAL_OK;
}

static HAL_StatusTypeDef Enqueue(OBD_BufferTypeDef *pBuffer, uint8_t *pData, uint8_t queueSize) {
    if (GetBufferState(pBuffer, queueSize) == BUFFER_FULL) return HAL_ERROR;
    memcpy(pBuffer->data[pBuffer->rear], pData, OBD_CAN_DLC);
    ++pBuffer->rear;
    if (pBuffer->rear >= queueSize) pBuffer->rear = 0;
    return HAL_OK;
}

HAL_StatusTypeDef OBD_Init(CAN_HandleTypeDef *hcan) {
    hcanOBD = hcan;
    /* CAN滤波器，只接收ECU1（0x7E8）的报文 */
    CAN_FilterTypeDef filter = {
        .FilterIdHigh           = OBD_RESP_ECU1_ADDR << 5,
        .FilterIdLow            = 0xFFFF,
        .FilterMaskIdHigh       = 0xFFFF,
        .FilterMaskIdLow        = 0xFFFF,
        .FilterFIFOAssignment   = CAN_FILTER_FIFO0,
        .FilterBank             = 0,
        .FilterMode             = CAN_FILTERMODE_IDLIST,
        .FilterScale            = CAN_FILTERSCALE_16BIT,
        .FilterActivation       = CAN_FILTER_ENABLE,
        .SlaveStartFilterBank   = 0
    };
    HAL_CAN_ConfigFilter(hcanOBD, &filter);
    HAL_CAN_ActivateNotification(hcanOBD, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(hcanOBD, CAN_IT_ERROR_WARNING);
    HAL_CAN_ActivateNotification(hcanOBD, CAN_IT_BUSOFF);
    HAL_CAN_ActivateNotification(hcanOBD, CAN_IT_LAST_ERROR_CODE);
    HAL_CAN_ActivateNotification(hcanOBD, CAN_IT_ERROR);
    return HAL_CAN_Start(hcanOBD);
}

inline OBD_BufferStateTypeDef OBD_GetTxBufferState(void) {
    return GetBufferState(&txBuffer, OBD_BUFFER_SIZE);
}

inline OBD_BufferStateTypeDef OBD_GetRxBufferState(void) {
    return GetBufferState(&rxBuffer, OBD_BUFFER_SIZE);
}

HAL_StatusTypeDef OBD_TxBufferProcess(void) {
    if (GetBufferState(&rxBuffer, OBD_BUFFER_SIZE) == BUFFER_EMPTY) return HAL_ERROR;
    /*
    * 对于OBD协议，成功发送一个CAN报文后，车辆ECU必有一报文回复
    * 所以在发送报文后，需要等待接受一次报文，才能继续发送下一个报文
    * 否则可能导致无意义发送，无法获取车辆信息
    */
    if (!allowTx || HAL_CAN_GetTxMailboxesFreeLevel(hcanOBD) < 3) return HAL_BUSY;
    HAL_CAN_AddTxMessage(hcanOBD, &txHeader, txBuffer.data[txBuffer.front], &lastMailbox);
    allowTx = 0;
    Dequeue(&txBuffer, OBD_BUFFER_SIZE);
    return HAL_OK;
}

HAL_StatusTypeDef OBD_RxBufferProcess(void) {
    vehicleStatus.canStatus = HAL_CAN_GetError(hcanOBD);
    if (GetBufferState(&rxBuffer, OBD_BUFFER_SIZE) == BUFFER_EMPTY) return HAL_ERROR;
    uint8_t data[OBD_CAN_DLC];
    memcpy(data, rxBuffer.data[rxBuffer.front], OBD_CAN_DLC);
    Dequeue(&rxBuffer, OBD_BUFFER_SIZE);
    if (data[1] != OBD_SERVICE_RX_01) return HAL_BUSY;
    switch (data[2]) {
    case OBD_PID_ENGINE_RPM:
        vehicleStatus.engineRPM = (data[3] << 8) | data[4];
        break;
    case OBD_PID_VEHICLE_SPEED:
        vehicleStatus.speed = data[3];
    default:
        return HAL_BUSY;
        break;
    }
    return HAL_OK;
}

HAL_StatusTypeDef OBD_TxMessage(uint8_t *pData) {
    return Enqueue(&txBuffer, pData, OBD_BUFFER_SIZE);
}

HAL_StatusTypeDef OBD_GetVehicleStatus(OBD_VehicleStatusTypeDef * pTarget) {
    if (pTarget == NULL) return HAL_ERROR;
    memcpy(pTarget, &vehicleStatus, sizeof(vehicleStatus));
    return HAL_OK;
}

/*接收中断处理函数*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (GetBufferState(&rxBuffer, OBD_BUFFER_SIZE) == BUFFER_EMPTY) return;  //接收缓存已满，不读取邮箱，根据CAN控制器配置溢出
    allowTx = 1;
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[OBD_CAN_DLC];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data);
    Enqueue(&rxBuffer, data, OBD_BUFFER_SIZE);
}