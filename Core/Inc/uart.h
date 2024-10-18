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

#ifndef __UART_H__
#define __UART_H__

#include "era_types.h"
void init_data_buffer();
void uart2_start_recv();
void uart3_start_recv();
void uart4_start_recv();
void uart1_check_send();
void uart4_check_send();
#endif
