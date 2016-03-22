#ifndef __DATA_TRANSFER_H
#define __DATA_TRANSFER_H
#include "sys.h"

typedef struct int16_rcget{
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
				int16_t AUX5;
				int16_t AUX6;}
T_RC_Data;


void Data_Receive_Anl(void);
void lock();
void un_lock();

#endif
