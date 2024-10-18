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

#ifndef __ERA_TYPE_H__
#define __ERA_TYPE_H__

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f4xx_hal.h"

//gnss raw data
typedef struct _EraRawGnss  //size = 44
{
    uint32_t timestamp;

    uint8_t loc_index; 
    uint8_t heading_index;
    uint8_t front_sat;
    uint8_t rear_sat;
    
    double lon;
    double lat;

    float speed; // m/s
    float height;
    float heading;  //antenna heading
    float pitch;    //antenna pitch 
    uint32_t reserve;
} EraRawGnss;

typedef struct _EraRawGGA  //size = 44
{
    uint32_t timestamp; 

    uint8_t loc_index;
    uint8_t sat_num;
    uint16_t hdop;//Horizontal Dilution of Precision

    double lon;
    double lat;

    float ant_height;//
    float ellip_height;// ellipsoidal height"
    uint16_t age;
    uint16_t base_index;
    uint32_t reserve1;
    uint32_t reserve2;
} EraRawGGA;

#define DIFFDATA_RECV_LEN 1024

#define DIFFDATA_ITEM_NUM 4
#define DIFFDATA_ITEM_LEN 512
typedef struct _DiffDataBuffer {
    int item_num;
    int item_len;
    int pos_head;
    int pos_tail;
    uint8_t buffer[DIFFDATA_ITEM_NUM * DIFFDATA_ITEM_LEN];
} DiffDataBuffer;

#define POSEDATA_RECV_LEN 128

#define POSEDATA_ITEM_NUM 20
#define POSEDATA_ITEM_LEN 64
typedef struct _PoseDataBuffer {
    int item_num;
    int item_len;
    int pos_head;
    int pos_tail;
    uint8_t buffer[POSEDATA_ITEM_NUM * POSEDATA_ITEM_LEN];
} PoseDataBuffer;

#define GNSSDATA_RECV_LEN 128

#define GNSSDATA_ITEM_NUM 8
#define GNSSDATA_ITEM_LEN 64
typedef struct _GnssDataBuffer {
    int item_num;
    int item_len;
    int pos_head;
    int pos_tail;
    uint8_t buffer[GNSSDATA_ITEM_NUM * GNSSDATA_ITEM_LEN];
} GnssDataBuffer;

#define NAVIDATA_ITEM_NUM 20
#define NAVIDATA_ITEM_LEN 48
typedef struct _NaviDataBuffer {
    int item_num;
    int item_len;
    int pos_head;
    int pos_tail;
    uint8_t buffer[NAVIDATA_ITEM_NUM * NAVIDATA_ITEM_LEN];
} NaviDataBuffer;


#endif
