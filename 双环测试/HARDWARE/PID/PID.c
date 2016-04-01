#include "pid.h"
#include "timer.h"
#include "usart.h"
#include "suanfa.h"
#include "motor.h"
#include "math.h"
 
#define dTime 0.005
  
extern float M_deal1,M_deal2,M_deal3,M_deal4;
extern EularAngle EA;
extern Acce acc;
extern Gyro gyro;
extern int Power;			//油门
float integrator;
float ROLL, PITCH, YAW;
u8 FLY=1;	
EularAngle EA_command={0};
PID Pitch_out,Roll_out,Pitch_inner,Roll_inner;

	
//32位整型数限幅
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high) 
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


//浮点数限幅
float constrain_float(float amt, float low, float high) 
{
	if (isnan(amt))
	{
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/********增量式PID******************/
static void PID_Position(PID *PIDx,float target,float measure,double dT_Inerval)
{
	PIDx->Error = target - measure;
	PIDx->Deriv = (PIDx->Error - PIDx->PreError)/dT_Inerval;
	PIDx->PreError = PIDx->Error;
	
	if(fabs(PIDx->Integ)<Power)   // 比油门大时不积分
	{
		PIDx->Integ =	PIDx->Integ + PIDx->Error * dT_Inerval;
		
		PIDx->Integ = constrain_float(PIDx->Integ,-300,300);  //积分限幅
	}
	else PIDx->Integ = 0;
	
	PIDx->Output = PIDx->Kp * PIDx->Error + PIDx->Ki * PIDx->Integ + PIDx->Kd * PIDx->Deriv;
}




void PID_Parameter_Init()
{
	Pitch_out.Kp = 1;//3.5
	Pitch_out.Ki = 0;
	Pitch_out.Kd = 0;
			
	Pitch_inner.Kp = 0.7;//0.7
	Pitch_inner.Ki = 1;//0.5
	Pitch_inner.Kd = 0.3;//0.03

	Roll_out.Kp =	Pitch_out.Kp ;
	Roll_out.Ki =	Pitch_out.Ki ;
	Roll_out.Kd =	Pitch_out.Kd ;
	Roll_inner.Kp =	Pitch_inner.Kp;//
	Roll_inner.Ki =	Pitch_inner.Ki;
	Roll_inner.Kd =	Pitch_inner.Kd;
}
void  reset_I(void)
{
	integrator = 0;
}
 
void PID_Deal( void )
{
//out_loop
	PID_Position(&Roll_out,EA_command.Roll,EA.Roll,dTime);
	PID_Position(&Pitch_out,EA_command.Pitch,EA.Pitch,dTime);

//inner_loop
	PID_Position(&Roll_inner,Roll_out.Output,gyro.x,dTime);
	PID_Position(&Pitch_inner,Pitch_out.Output,gyro.y,dTime);

	ROLL = Roll_inner.Output;
	PITCH = Pitch_inner.Output;
	YAW = 0;
//PID输出转为电机控制量
	if(FLY)
	{
		MM_Drive(ROLL, PITCH, YAW);
	}else
	{											   
		MM_Set(0);
	}	
}

	

	




 	


