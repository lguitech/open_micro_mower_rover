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

#ifndef __DATABUFFER_H__
#define __DATABUFFER_H__

#include "era_types.h"

void era_init_buffer(int item_num, int item_len, uint8_t* pData);
int era_get_recv_buffer(uint8_t* pBuffer, uint8_t** ppData);
void era_rx_complete(uint8_t* pBuffer);
int era_get_send_buffer(uint8_t* pBuffer, uint8_t** ppData);

#endif
