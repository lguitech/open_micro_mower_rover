/*********************************************************************
 *
 *  This file is part of the [OPEN_MICRO_MOWER_ROVER] project.
 *  Licensed under the MIT License for non-commercial purposes.
 *  Author: Brook Li
 *  Email: lguitech@126.com
 *
 *  For more details, refer to the LICENSE file or contact [lguitech@126.com].
 *
 *  Commercial use requires a separate license.
 *
 *  This software is provided "as is", without warranty of any kind.
 *
 *********************************************************************/

#include "era_types.h"
#include "uart.h"
#include "databuffer.h"

extern uint8_t diff_recv_buffer[DIFFDATA_RECV_LEN];
extern uint8_t gnss_recv_buffer[DIFFDATA_RECV_LEN];
extern uint8_t pose_recv_buffer[POSEDATA_RECV_LEN];

extern DiffDataBuffer diffDataBuffer;
extern GnssDataBuffer gnssDataBuffer;
extern PoseDataBuffer poseDataBuffer;
extern NaviDataBuffer naviDataBuffer;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;



extern int imu_frame_count;
extern int gga_frame_count;
extern int ksxt_frame_count;
static void printInfo(char* info) 
{
    char strCount[32];
    sprintf(strCount, "%d, %d, %d ", imu_frame_count, ksxt_frame_count, gga_frame_count);
	char strResult[128];
	strcpy(strResult, strCount);
	strcat(strResult, info);

    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)strResult, strlen(strResult));
}

int uart4_free_to_send = 1;
int uart1_free_to_send = 1;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart == &huart1) {
        uart1_free_to_send = 1;
    }
    else if (huart == &huart4) {
        uart4_free_to_send = 1;
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
        uint8_t* pRecvData;
        int len = era_get_recv_buffer((uint8_t*)&diffDataBuffer, &pRecvData);
        memcpy(pRecvData, diff_recv_buffer, DIFFDATA_RECV_LEN/2);
        era_rx_complete((uint8_t*)&diffDataBuffer);
    }
    else if (huart == &huart2) {
        uint8_t* pRecvData;
        int len = era_get_recv_buffer((uint8_t*)&poseDataBuffer, &pRecvData);
        memcpy(pRecvData, pose_recv_buffer, POSEDATA_RECV_LEN/2);
        era_rx_complete((uint8_t*)&poseDataBuffer);
    }
    else if (huart == &huart4) {
        uint8_t* pRecvData;
        int len = era_get_recv_buffer((uint8_t*)&gnssDataBuffer, &pRecvData);
        memcpy(pRecvData, gnss_recv_buffer, GNSSDATA_RECV_LEN/2);
        era_rx_complete((uint8_t*)&gnssDataBuffer);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
        uint8_t* pRecvData;
        int len = era_get_recv_buffer((uint8_t*)&diffDataBuffer, &pRecvData);
        memcpy(pRecvData, diff_recv_buffer + DIFFDATA_RECV_LEN/2, DIFFDATA_RECV_LEN/2);
        era_rx_complete((uint8_t*)&diffDataBuffer);
    }
    else if (huart == &huart2) {
        uint8_t* pRecvData;
        int len = era_get_recv_buffer((uint8_t*)&poseDataBuffer, &pRecvData);
        memcpy(pRecvData, pose_recv_buffer + POSEDATA_RECV_LEN/2, POSEDATA_RECV_LEN/2);
        era_rx_complete((uint8_t*)&poseDataBuffer);
    }
    else if (huart == &huart4) {
        uint8_t* pRecvData;
        int len = era_get_recv_buffer((uint8_t*)&gnssDataBuffer, &pRecvData);
        memcpy(pRecvData, gnss_recv_buffer + GNSSDATA_RECV_LEN/2, GNSSDATA_RECV_LEN/2);
        era_rx_complete((uint8_t*)&gnssDataBuffer);
    }
}

void init_data_buffer()
{
    era_init_buffer(DIFFDATA_ITEM_NUM, DIFFDATA_ITEM_LEN, (uint8_t*)&diffDataBuffer);
    era_init_buffer(GNSSDATA_ITEM_NUM, GNSSDATA_ITEM_LEN, (uint8_t*)&gnssDataBuffer);
    era_init_buffer(POSEDATA_ITEM_NUM, POSEDATA_ITEM_LEN, (uint8_t*)&poseDataBuffer);
    era_init_buffer(NAVIDATA_ITEM_NUM, NAVIDATA_ITEM_LEN, (uint8_t*)&naviDataBuffer);
}

void uart3_start_recv()
{
    HAL_UART_Receive_DMA(&huart3, diff_recv_buffer, DIFFDATA_RECV_LEN);
}

void uart2_start_recv()
{
    HAL_UART_Receive_DMA(&huart2, pose_recv_buffer, POSEDATA_RECV_LEN);
}

void uart4_start_recv()
{
    HAL_UART_Receive_DMA(&huart4, gnss_recv_buffer, GNSSDATA_RECV_LEN);
}

void uart1_check_send()
{
    if (uart1_free_to_send) {
        uint8_t* pSendData;
        int len = era_get_send_buffer((uint8_t*)&naviDataBuffer, &pSendData);
		if (len != 0) {
			uart1_free_to_send = 0;
			HAL_UART_Transmit_DMA(&huart1, pSendData, len);
		}
    }
}

void uart4_check_send()
{
    if (uart4_free_to_send) {
        uint8_t* pSendData;
        int len = era_get_send_buffer((uint8_t*)&diffDataBuffer, &pSendData);
		if (len != 0) {
			uart4_free_to_send = 0;
			HAL_UART_Transmit_DMA(&huart4, pSendData, len);
		}
    }
}