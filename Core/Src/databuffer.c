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

#include "databuffer.h"

uint8_t diff_recv_buffer[DIFFDATA_RECV_LEN];
uint8_t gnss_recv_buffer[GNSSDATA_RECV_LEN];
uint8_t pose_recv_buffer[POSEDATA_RECV_LEN];

DiffDataBuffer diffDataBuffer;
GnssDataBuffer gnssDataBuffer;
PoseDataBuffer poseDataBuffer;
NaviDataBuffer naviDataBuffer;

void era_init_buffer(int item_num, int item_len, uint8_t* pData)
{
    int size = 4 * sizeof(int) + (item_num * item_len);
    memset(pData, 0 ,size);
    int* pItemNum = (int*)pData;
    int* pItemLen = (int*)(pData + sizeof(int));
    *pItemNum = item_num;
    *pItemLen = item_len;
}

int era_get_recv_buffer(uint8_t* pBuffer, uint8_t** ppData)
{
    int item_num = *((int*)pBuffer);
    int item_len = *((int*)(pBuffer + sizeof(int)));
    int* pHead = (int*)(pBuffer + 2 * sizeof(int));
    int* pTail = (int*)(pBuffer + 3 * sizeof(int));
    uint8_t* pData = pBuffer + 4 * sizeof(int);

    *ppData = pData + (*pTail) * item_len;
	return item_len;
}

void era_rx_complete(uint8_t* pBuffer)
{
    int item_num = *((int*)pBuffer);
    //int item_len = *((int*)(pBuffer + sizeof(int)));
    int* pHead = (int*)(pBuffer + 2 * sizeof(int));
    int* pTail = (int*)(pBuffer + 3 * sizeof(int));
    //uint8_t* pData = pBuffer + 4 * sizeof(int);

    *pTail = (*pTail + 1) % item_num;
    if (*pTail == *pHead) {
        *pHead = (*pHead + 1) % item_num;
    }

}
int era_get_send_buffer(uint8_t* pBuffer, uint8_t** ppData)
{
    int item_num = *((int*)pBuffer);
    int item_len = *((int*)(pBuffer + sizeof(int)));
    int* pHead = (int*)(pBuffer + 2 * sizeof(int));
    int* pTail = (int*)(pBuffer + 3 * sizeof(int));
    uint8_t* pData = pBuffer + 4 * sizeof(int);

    if (*pHead == *pTail) {
        return 0;
    }
    *ppData = pData + *pHead * item_len;
    *pHead = (*pHead + 1) % item_num;
	return item_len;
}
