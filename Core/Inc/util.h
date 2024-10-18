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

#ifndef __UTIL_H__
#define __UTIL_H__

#include "era_types.h"

int era_min(int a, int b);
uint8_t era_calc_checksum(uint8_t* sentence, int len);
double era_atof(char* strInput, int start_inex, int end_index);
int era_atoi(char* strInput, int start_inex, int end_index);

#endif