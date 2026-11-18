#include "can.h"

#include <string.h>

#include "board.h"

static CAN_HandleTypeDef hcan1;
static can_frame_t latest_frame;
static volatile uint32_t latest_sequence;

bool can_init(void)
{
    __HAL_RCC_CAN1_CLK_ENABLE();

    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 1;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        return false;
    }

    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK ||
        HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return false;
    }

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    return HAL_CAN_ActivateNotification(&hcan1,
                                         CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK;
}

bool can_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox = 0;
    uint8_t payload[8] = {0};

    if (len > sizeof(payload) || (len > 0U && data == 0))
    {
        return false;
    }
    if (len > 0U)
    {
        memcpy(payload, data, len);
    }

    header.ExtId = id;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = len;
    header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(&hcan1, &header, payload, &mailbox) == HAL_OK;
}

bool can_read_latest(can_frame_t *frame, uint32_t *sequence)
{
    if (frame == 0 || sequence == 0 || latest_sequence == 0U)
    {
        return false;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *frame = latest_frame;
    *sequence = latest_sequence;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return true;
}

void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1)
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U)
    {
        CAN_RxHeaderTypeDef header = {0};
        can_frame_t frame = {0};
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header,
                                 frame.data) != HAL_OK)
        {
            return;
        }
        if (header.IDE != CAN_ID_EXT || header.RTR != CAN_RTR_DATA)
        {
            continue;
        }

        frame.id = header.ExtId;
        frame.len = (uint8_t)header.DLC;
        latest_frame = frame;
        latest_sequence++;
    }
}
