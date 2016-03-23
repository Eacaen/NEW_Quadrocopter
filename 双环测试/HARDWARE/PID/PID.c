#include "pid.h"
#include "timer.h"
#include "usart.h"
#include "suanfa.h"
#include "motor.h"
#include "math.h"

#define FLYANGLE_MAX 30
#define imax         1000
#define PID_INNER_LOOP_TIME		5000	//单位为us
// #define PID_OUTER_LOOP_TIME		5000	//单位为us

u8 FLY=1;
EularAngle EA_command={0};
int32_t RateError[3];
int yawRate = 50;
//上一次的误差输入
	int32_t last_error;
	
//PID参数
float kP_out= 0;
float kP = 0;
float kI = 0;
float kD = 0;
	
extern EularAngle EA;
extern Acce acc;
extern Gyro gyro;
extern int Power;			//油门
float integrator; 

extern float M_deal1,M_deal2,M_deal3,M_deal4;

enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
 
};



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


void  reset_I(void)
{
	integrator = 0;
}

int32_t  get_out_p(int32_t error)
{
	    return  (float)error * kP_out ;

		
//     return constrain_float((float)error * kP_out,30,-30);
}

int32_t  get_p(int32_t error)
{
    return (float)error * kP;
}

int32_t get_i(int32_t error, float dt)
{
	integrator += ((float)error * kI) * dt;
	integrator = constrain_float(integrator, -imax, +imax);		
	return integrator;
}



int32_t get_d(int32_t error, float dt)
{
	float derivative = (error - last_error) / dt;
	last_error = error;
	return kD * derivative ;
}


int32_t get_pid(int32_t error, float dt)
{
    return get_p(error) + get_i(error, dt) + get_d(error, dt);
}

//飞行器姿态外环控制
void ANO_FlyControl_Attitude_Outter_Loop(void)
{
	int32_t	errorAngle[2];
	
	//计算角度误差值
	errorAngle[ROLL]  = constrain_int32((EA_command.Roll * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - EA.Roll; 
	errorAngle[PITCH] = constrain_int32((EA_command.Pitch* 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - EA.Pitch; 
	
	//获取角速度
// 	Gyro = mpu6050.Get_Gyro() / 4;
	
	//得到外环PID输出
	RateError[ROLL] = get_out_p(errorAngle[ROLL]) - gyro.x;
	
// 	RateError[PITCH] = get_out_p(errorAngle[PITCH]) - gyro.y;
	RateError[PITCH] = 0;
// 	RateError[YAW] = ((int32_t)(yawRate) * EA_command.Yaw)  - gyro.z;		
	
// 	printf("%d  %.1f\r\n",RateError[ROLL],gyro.x);
}


//飞行器姿态内环控制
void ANO_FlyControl_Attitude_Inner_Loop(void)
{
	int32_t PIDTerm[3];
	u8 i=0;
	for(i=0; i<3;i++)
	{
		//当油门低于检查值时积分清零
		if (Power-10 <= 0)	
			 reset_I();
		
		//得到内环PID输出
		PIDTerm[i] = get_pid(RateError[i], PID_INNER_LOOP_TIME*1e-6);
	}
	
// 	PIDTerm[YAW] = -constrain_int32(PIDTerm[YAW], -100 - fabs(EA_command.Yaw), +100 + fabs(EA_command.Yaw));	
		
	PIDTerm[YAW] = 0;
	
	//PID输出转为电机控制量
	if(FLY)
	{
		
		MM_Drive(PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
	}else
	{											   
		MM_Set(0);
	}
}	


void PID_Deal( void )
{
	ANO_FlyControl_Attitude_Outter_Loop();
	ANO_FlyControl_Attitude_Inner_Loop();
	
	
}


 	


