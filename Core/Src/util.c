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

#include "util.h"


int era_min(int a, int b) 
{
	if (a <= b) {
		return a;
	}
	else {
		return b;
	}
}


uint8_t era_calc_checksum(uint8_t* sentence, int len)
{
	unsigned char checksum = 0;
	for (int i = 0; i < len; i++) {
		checksum ^= sentence[i];
	}
	return checksum;
}

double era_atof(char* strInput, int start_inex, int end_index)
{
    if (start_inex == end_index) {
        return 0.0;
    }
    else {
        return atof((char*)strInput + start_inex);
    }
}

int era_atoi(char* strInput, int start_inex, int end_index)
{
    if (start_inex == end_index) {
        return 0;
    }
    else {
        return atoi((char*)strInput + start_inex);
    }
}
