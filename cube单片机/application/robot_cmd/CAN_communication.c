/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       CAN_communication.c/h
  * @brief      CAN通信部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-27-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================
板间通信时stdid的内容如下
data_type + (data_id << 4) + target_id

bit 0-3: target_id
bit 4-7: data_id
bit 8-11: data_type

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "CAN_communication.h"

#include "bsp_can.h"
#include "can_typedef.h"
#include "string.h"
#include "user_lib.h"

// static CanCtrlData_s CAN_CTRL_DATA = {
//     .tx_header.IDE = CAN_ID_STD,
//     .tx_header.RTR = CAN_RTR_DATA,
//     .tx_header.DLC = 8,
// };

/*-------------------- Private functions --------------------*/
// 板间通信

/**
 * @brief          发送数据
 * @param[in]      hcan CAN句柄
 * @param[in]      std_id 数据包ID
 * @param[in]      data 包含8个字节的数据的指针
 * @retval         none
 */
// static void SendData(uint8_t can, uint16_t std_id, uint8_t * data)
// {
//     if (can == 1)
//         CAN_CTRL_DATA.hcan = &hcan1;
//     else if (can == 2)
//         CAN_CTRL_DATA.hcan = &hcan2;
//     else
//         return;

//     CAN_CTRL_DATA.tx_header.StdId = std_id;

//     memcpy(CAN_CTRL_DATA.tx_data, data, 8);

//     CAN_SendTxMessage(&CAN_CTRL_DATA);
// }

/*-------------------- Public functions --------------------*/

/*------------------------------ End of File ------------------------------*/
