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

#include "proc_uart2.h"
#include "databuffer.h"
#include "util.h"

extern NaviDataBuffer naviDataBuffer;
extern PoseDataBuffer poseDataBuffer;

static int pos_tail_imu = 0; 
#define PARSE_BUFFER_LEN_IMU 54
static uint8_t parse_buffer_imu[PARSE_BUFFER_LEN_IMU];

int imu_frame_count = 0;
static void deal_one_frame()
{
    uint8_t* pData;
    int len = era_get_recv_buffer((uint8_t*)&naviDataBuffer, &pData);
    pData[0] = 0xAA;
    pData[1] = 0x55;
    pData[2] = 3;
    memcpy(pData + 4, parse_buffer_imu + 6, 44);
    pData[3] = era_calc_checksum(pData + 4, 44);
    era_rx_complete((uint8_t*)&naviDataBuffer);
    //printInfo("deal one frame imu");
    imu_frame_count ++;
}

static int find_begin_tag_imu(unsigned char* pData, int len)
{
	int index_begin = 0;
	for (; index_begin < len-1; index_begin ++) {
		if (pData[index_begin] == 0xAA && pData[index_begin + 1] == 0x55) {
			break;
		}
	}
	if (index_begin < len -1) {
		return index_begin;
	}
	else {
		return -1;
	}
}

static void parse_imu_data(uint8_t* pData, int len)
{
	int pos_offset = 0;
	while(pos_offset < len) {
		if (pos_tail_imu == 0) {
			int index_begin = find_begin_tag_imu(pData + pos_offset, len - pos_offset);
			if (index_begin == -1) {
				return; //head not found 
			}
			pos_offset += index_begin;
		}
		

		int copy_num = era_min(PARSE_BUFFER_LEN_IMU - pos_tail_imu, len - pos_offset);
		memcpy(parse_buffer_imu + pos_tail_imu, pData + pos_offset, copy_num);
		pos_tail_imu += copy_num;
		if (pos_tail_imu == PARSE_BUFFER_LEN_IMU) {
			deal_one_frame();
			pos_tail_imu = 0;
		}
		pos_offset += copy_num;
	}
}

void proc_data_uart2()
{
    uint8_t* pData = NULL;
    int len = era_get_send_buffer((uint8_t*)&poseDataBuffer, &pData);
    if (len == 0) {
        return;
    }
    parse_imu_data(pData, len);
}
