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

#include "proc_uart4.h"
#include "databuffer.h"
#include "util.h"

extern GnssDataBuffer gnssDataBuffer;
extern NaviDataBuffer naviDataBuffer;

#define PARSE_BUFFER_LEN_GNSS 512
static uint8_t parse_buffer_gnss[PARSE_BUFFER_LEN_GNSS];
static int pos_tail_gnss = 0; 

static EraRawGnss raw_gnss;
static EraRawGGA raw_gga;

int ksxt_frame_count = 0;
int gga_frame_count = 0;

static void deal_one_frame_ksxt(int len)
{
    int item = 1;
    int index_prev = 6;
    int ms;
    for (int i=6; i<len; i++) {
        if (parse_buffer_gnss[i] == ',' || parse_buffer_gnss[i] == '\n') {
            parse_buffer_gnss[i] = 0;
            switch (item) {
            case 1:
                //20230329074910.90
                if (i == index_prev) {
                    raw_gnss.timestamp = 0;
                }
                else {
                    ms = atoi((char*)parse_buffer_gnss + i - 2);

                    parse_buffer_gnss[i - 3] = 0; 

                    while(parse_buffer_gnss[index_prev] == ' ' || parse_buffer_gnss[index_prev] == '\t') {
                        index_prev ++;
                    }  
                    if (i - index_prev < 9) {
                        raw_gnss.timestamp = 0;
                    }             
                    else {
                        raw_gnss.timestamp = atoi((char*)parse_buffer_gnss + i - 9) * 1000 + ms * 10;
                    }
                }
                break;
            case 2:
                //lon
                raw_gnss.lon = era_atof((char*)parse_buffer_gnss, index_prev, i);
                break;
            case 3:
                //lat
                raw_gnss.lat = era_atof((char*)parse_buffer_gnss, index_prev, i);
                break;
            case 4:
                //height
                raw_gnss.height = (float)era_atof((char*)parse_buffer_gnss, index_prev, i);
                break;
            case 5:
                //heading
                raw_gnss.heading = (float)era_atof((char*)parse_buffer_gnss, index_prev, i);
                break;             
            case 6:
                //pitch
                raw_gnss.pitch = (float)era_atof((char*)parse_buffer_gnss, index_prev, i);
                break;  
            case 8:
                //speed
                raw_gnss.speed = (float)(era_atof((char*)parse_buffer_gnss, index_prev, i) / 3.6);
                break;  
            case 10:
                //loc_index
                raw_gnss.loc_index = era_atoi((char*)parse_buffer_gnss, index_prev, i);
                break;                      
            case 11:
                //heading_index
                raw_gnss.heading_index = era_atoi((char*)parse_buffer_gnss, index_prev, i);
                break;      
            case 12:
                //front sat
                raw_gnss.front_sat = era_atoi((char*)parse_buffer_gnss, index_prev, i);
                break;      
            case 13:
                //rear sat
                raw_gnss.rear_sat = era_atoi((char*)parse_buffer_gnss, index_prev, i);
                break;                      
            default:
                break;
            }
            index_prev = i + 1;
            item ++;
        }
    }
}

static void deal_one_frame_gga(int len)
{
    //$GNGGA,064454.00,4008.76242256,N,11624.68850149,E,1,09,4.4,43.4136,M,-8.1052,M,,*5F
    //$GNGGA,,,,,,0,,,,,,,,*78
    int item = 1;
    int index_prev = 7;
    int ms;
    for (int i=7; i<len; i++) {
        if (parse_buffer_gnss[i] == ',' || parse_buffer_gnss[i] == '*') {
            parse_buffer_gnss[i] = 0;
            switch (item) {
            case 1:
                if (index_prev == i) {
                    raw_gga.timestamp = 0;
                }
                else {
                    //064454.00
                    ms = atoi((char*)parse_buffer_gnss + i - 2);
                    parse_buffer_gnss[i - 3] = 0; 
                    raw_gga.timestamp = atoi((char*)parse_buffer_gnss + i - 9) * 1000 + ms * 10;
                }
                break;
            case 2:
                //lat
                raw_gga.lat = era_atof((char*)parse_buffer_gnss, index_prev, i);
                break;
            case 4:
                //lon
                raw_gga.lon = era_atof((char*)parse_buffer_gnss, index_prev, i);
                break;
            case 6:
                //loc_index
                raw_gga.loc_index = era_atoi((char*)parse_buffer_gnss, index_prev, i);
                break;             
            case 7:
                //sat_num
                raw_gga.sat_num = era_atoi((char*)parse_buffer_gnss, index_prev, i);
                break;  
            case 8:
                //hdop
                raw_gga.hdop = (uint16_t)(era_atof((char*)parse_buffer_gnss, index_prev, i) * 10);
                break;  
            case 9:
                //ant_height
                raw_gga.ant_height = era_atof((char*)parse_buffer_gnss, index_prev, i);
                break;                      
            case 11:
                //ellip_height
                raw_gga.ellip_height = era_atof((char*)parse_buffer_gnss, index_prev, i);
                break;      
            case 13:
                //age
                if(index_prev == i) {
                    raw_gga.age = 0xffff;
                }
                else {
                    raw_gga.age = era_atoi((char*)parse_buffer_gnss, index_prev, i);
                }
                break;      
            case 14:
                //base_index
                if(index_prev == i) {
                    raw_gga.base_index = 0xffff;
                }
                else {
                    raw_gga.base_index = era_atoi((char*)parse_buffer_gnss, index_prev, i);
                }
                break;      
            default:
                break;
            }
            index_prev = i + 1;
            item ++;
        }
    }
}

static void deal_one_frame(int len)
{
	//parse         
	//$KSXT,20230329074910.90,116.41137724,40.14605145,21.8135,0.00,0.00,19.27,0.026,,1,0,0,11,,,,0.009,0.025,0.075,,*12
	//$KSXT,             0.00,0.00000000  ,0.00000000 ,0.0000 ,0.00,0.00,0.00 ,0.000,,0,0,0, 0,,,,0.000,0.000,0.000,,*26
	if (strncmp((char*)parse_buffer_gnss, "$KSXT", 5) == 0)
	{
		deal_one_frame_ksxt(len);

        uint8_t* pData;
        int len = era_get_recv_buffer((uint8_t*)&naviDataBuffer, &pData);

        pData[0] = 0xAA;
        pData[1] = 0x55;
        pData[2] = 1;
        int offset = 4;
        memcpy(pData + offset, (uint8_t*)&raw_gnss.timestamp, 4); offset += 4;
        pData[offset++] = raw_gnss.loc_index;
        pData[offset++] = raw_gnss.heading_index;
        pData[offset++] = raw_gnss.front_sat;
        pData[offset++] = raw_gnss.rear_sat;

        memcpy(pData + offset, (uint8_t*)&raw_gnss.lon, 8); offset += 8;
        memcpy(pData + offset, (uint8_t*)&raw_gnss.lat, 8); offset += 8;
        memcpy(pData + offset, (uint8_t*)&raw_gnss.speed, 4); offset += 4;
        memcpy(pData + offset, (uint8_t*)&raw_gnss.height, 4); offset += 4;
        memcpy(pData + offset, (uint8_t*)&raw_gnss.heading, 4); offset += 4;
        memcpy(pData + offset, (uint8_t*)&raw_gnss.pitch, 4); offset += 4;
        memcpy(pData + offset, (uint8_t*)&raw_gnss.reserve, 4); //offset += 4;
    
        pData[3] = era_calc_checksum(pData + 4, 44);
        
        era_rx_complete((uint8_t*)&naviDataBuffer);
        ksxt_frame_count ++;
	}
	else if ((strncmp((char*)parse_buffer_gnss, "$GPGGA", 6) == 0) || 
		(strncmp((char*)parse_buffer_gnss, "$GNGGA", 6) == 0)) 
	{
		deal_one_frame_gga(len);

        uint8_t* pData;
        int len = era_get_recv_buffer((uint8_t*)&naviDataBuffer, &pData);

        pData[0] = 0xAA;
        pData[1] = 0x55;
        pData[2] = 2;
        int offset = 4;
        memcpy(pData + offset, (uint8_t*)&raw_gga.timestamp, 4); offset += 4;
        pData[offset++] = raw_gga.loc_index;
        pData[offset++] = raw_gga.sat_num;
        memcpy(pData + offset, (uint16_t*)&raw_gga.hdop, 2); offset += 2;
        memcpy(pData + offset, (uint8_t*)&raw_gga.lon, 8); offset += 8;
        memcpy(pData + offset, (uint8_t*)&raw_gga.lat, 8); offset += 8;
        memcpy(pData + offset, (uint8_t*)&raw_gga.ant_height, 4); offset += 4;
        memcpy(pData + offset, (uint8_t*)&raw_gga.ellip_height, 4); offset += 4;
        memcpy(pData + offset, (uint8_t*)&raw_gga.age, 2); offset += 2;
        memcpy(pData + offset, (uint8_t*)&raw_gga.base_index, 2); //offset += 2;

        pData[3] = era_calc_checksum(pData + 4, 44);
        era_rx_complete((uint8_t*)&naviDataBuffer);
        gga_frame_count ++;
    }
}

static int find_begin_tag_gnss(uint8_t* pData, int len)
{
    int index_begin = 0;
    for (; index_begin < len; index_begin ++) {
        if (pData[index_begin] == '$') {
            break;
        }
    }
	if (index_begin < len) {
		return index_begin;
	}
	else {
		return -1;
	}    
}

static void parse_gnss_data(uint8_t* pData, int len)
{
    int pos_offset = 0;
	while(pos_offset < len) {
		if (pos_tail_gnss == 0) {  
			int index_begin = find_begin_tag_gnss(pData + pos_offset, len - pos_offset);
			if (index_begin == -1) {
				return; //head not found 
			}
            pos_offset += index_begin;
		}

        while(pos_offset < len) {
            parse_buffer_gnss[pos_tail_gnss++] = pData[pos_offset];
            if (pData[pos_offset++] == '\n') {
                deal_one_frame(pos_tail_gnss); 
                pos_tail_gnss = 0;
                break;
            }
        }
        
	}
}

void proc_data_uart4()
{
    uint8_t* pData = NULL;
    int len = era_get_send_buffer((uint8_t*)&gnssDataBuffer, &pData);
    if (len == 0) {
        return;
    }
    parse_gnss_data(pData, len);
}
