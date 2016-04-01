#ifndef __PID_H
#define __PID_H
#include "sys.h"
typedef struct PID
	{
		
		double SetPoint; //  Desired Value
		double Kp; // Proportional Const
		double Ki; // Integral Const
		double Kd; //  Derivative Const
		
		double Integ;
		double Deriv;
		double Error;
		double PreError;
		double Output; // 
		
} PID;
void PID_Deal(void);
void PID_Parameter_Init(void);

#endif

