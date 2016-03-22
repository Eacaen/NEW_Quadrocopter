#include "delay.h"
#include "mpu6050.h"
#include "usart.h"
#include "math.h"
#include "suanfa.h"
#include "timer.h"
#include "pid.h"
#include "motor.h"
#include "led.h"
#include "hmc.h"
#include "pwm.h"
#include "math.h"
/* 
The code from AHRS copy by Eacaen

*/
extern Acce acc;
extern Gyro gyro;
extern short Hmc_X,Hmc_Y,Hmc_Z;
extern EularAngle EA;
extern EularAngle EA_Origin; 
extern float cord[3];	
extern int Power;										//油门
extern float kP;
extern float kI;
extern float kD;

struct K_Filter K_F;
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;  // scaled integral error

// #define halfT   GET_NOWTIME() * 0.5f   //采样间隔的一半
#define Kp 2.0f // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f// integral gain governs rate of convergence of gyroscope biases
#define hu_du 0.01744

float halfT;
float integralFBx,integralFBy,integralFBz;
float qa0, qa1, qa2, qa3;
float integralFBhand,handdiff;
float halftime ;
float  now; // TIM2 采样时间
float acc_vector = 0;//  加速度合力M/S^2

//Fast inverse square-root
/**************************????********************************************
函数原型：float invSqrt(float x) 
功能   ：快速算1/sqrt
*******************************************************************************/
float invSqrt(float x) 
{
		float halfx = 0.5f * x;
		float y = x;
		long i = *(long*)&y;
		i = 0x5f3759df - (i>>1);
		y = *(float*)&i;
		y = y * (1.5f - (halfx * y * y));
		return y;
}



/**************************实现函数********************************************
函数原型；IMU_AHRSupdate（）
功能   ：	更新AHRS四元数
*******************************************************************************/
int k,i;
void IMU_AHRSupdate(struct Acce acc,struct Gyro gyro,float mx, float my, float mz) 
{
// 		volatile float norm;
// 		volatile float hx, hy, hz, bx, bz;
// 		volatile float vx, vy, vz, wx, wy, wz;
// 		volatile float ex, ey, ez;

	  float norm;
		float hx, hy, hz, bx, bz;
		float vx, vy, vz, wx, wy, wz;
		float ex, ey, ez;	
	  float gx,gy, gz,ax,ay,az;
		float temp0,temp1,temp2,temp3;
		float temp;
	
		
		
		float q0q0 = q0*q0;
		float q0q1 = q0*q1;
		float q0q2 = q0*q2;
		float q0q3 = q0*q3;
		float q1q1 = q1*q1;
		float q1q2 = q1*q2;
		float q1q3 = q1*q3;
		float q2q2 = q2*q2;
		float q2q3 = q2*q3;
		float q3q3 = q3*q3; 
			
		ax=acc.x;
		ay=acc.y;
		az=acc.z;
	
		gx=gyro.x*hu_du;
		gy=gyro.y*hu_du;
		gz=gyro.z*hu_du;
 
		norm = invSqrt(ax*ax + ay*ay + az*az); 
		ax = ax * norm;
		ay = ay * norm;
		az = az * norm;

		norm = invSqrt(mx*mx + my*my + mz*mz); 
		mx = mx * norm;
		my = my * norm;
		mz = mz * norm;

		// compute reference direction of flux
		hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
		hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
		hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2); 
		bx = sqrtf((hx*hx) + (hy*hy));
		bz = hz; 

		// estimated direction of gravity and flux (v and w)
		vx = 2*(q1q3 - q0q2);
		vy = 2*(q0q1 + q2q3);
		vz = q0q0 - q1q1 - q2q2 + q3q3;
		
		wx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
		wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
		wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2);


		// error is sum of cross product between reference direction of fields and direction measured by sensors
		ex = (ay*vz - az*vy) + (my*wz - mz*wy);
		ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
		ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

 
		now = GET_NOWTIME();
		halfT = now * 0.5;
// 		printf("%f\r\n",now);
		
		if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
		{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;
		ezInt = ezInt + ez * Ki * halfT;

		// adjusted gyroscope measurements
		gx = gx + (Kp*ex + exInt);
		gy = gy + (Kp*ey + eyInt);
		gz = gz + (Kp*ez + ezInt);
		}
		// integrate quaternion rate and normalise
		temp0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
		temp1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
		temp2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
		temp3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

		// normalise quaternion
		norm = invSqrt(temp0*temp0 + temp1*temp1 + temp2*temp2 + temp3*temp3);
		q0 = temp0 * norm;
		q1 = temp1 * norm;
		q2 = temp2 * norm;
		q3 = temp3 * norm;
// 		printf("** %.1f %.1f %.1f %.1f \r\n",q0,q1,q2,q3);
}





//计算四元数

void  MPUpDate(struct Acce acc,struct Gyro gyro)
{
        float norm;
        float vx, vy, vz;
        float ex, ey, ez; 
				float ax, ay, az;
				float gx, gy, gz;		
				ax=acc.x;
				ay=acc.y;
				az=acc.z;
	
				gx=gyro.x*hu_du;
				gy=gyro.y*hu_du;
				gz=gyro.z*hu_du;

// normalise the measurements
        norm = invSqrt(ax*ax + ay*ay + az*az);
				if(norm == 0){ printf(" ERROR  ");}	
        ax = ax * norm;
        ay = ay * norm;
        az = az * norm;      
	
// estimated direction of gravity    q1=1
        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
// error is sum of cross product between reference direction of field and direction measured by sensor
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);
// integral error scaled integral gain
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;

// adjusted gyroscope measurements
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;

// integrate quaternion rate and normalise
		now = GET_NOWTIME();
// 		printf("%lf\r\n",now);
		halfT = now * 0.5;
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

// normalise quaternion
        norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 * norm;
        q1 = q1 * norm;
        q2 = q2 * norm;
        q3 = q3 * norm;
				
}

// 四元数-->欧拉角

void EularAngle_calculate(EularAngle *ea)
{
		
		float t11,t12,t13;
		float	t21,t22,t23;
		float	t31,t32,t33;
	
		t11 = q0*q0+q1*q1-q2*q2-q3*q3;
		t12=2.0*(q1*q2+q0*q3);
		t13=2.0*(q1*q3-q0*q2);
// 		t21=2.0*(q1*q2-q0*q3);
// 		t22=q0*q0-q1*q1+q2*q2-q3*q3;
		t23=2.0*(q2*q3+q0*q1);
// 		t31=2.0*(q1*q3+q0*q2);
// 		t32=2.0*(q2*q3-q0*q1);
		t33=q0*q0-q1*q1-q2*q2+q3*q3;
 
		(*ea).Roll = atan2(t23,t33)*57.3;
		(*ea).Pitch = -asin(t13)*57.3;
	
// 		(*ea).Yaw = atan2(t12,t11)*57.3;
	
		(*ea).Yaw = Correct_yaw();
// 		if ((*ea).Yaw < 0)
// 		{
// 					(*ea).Yaw += (360);
// 		}
// // 		printf("%.2lf  %.2lf  %.2lf\r\n",ea.Roll,ea.Pitch,ea.Yaw);
}



void suanfa_GetOrigin()
{
	static int  x=0;
	static double roll=0,pitch=0,yaw=0;
	LED3 =!LED3;
	for(x=0;x<100;x++)
	{
		LED3 =!LED3;
		Data_Pare();
// 		IMU_AHRSupdate(acc, gyro, Hmc_X,Hmc_Y,Hmc_Z);
		MPUpDate( acc, gyro);
		EularAngle_calculate(&EA);
		delay_ms(10);
// 					printf("JBJB--- %.2lf  %.2lf  %.2lf\r\n",EA.Roll,EA.Pitch,EA.Yaw);	

	}	
	for(x=0;x<100;x++)
	{
		delay_ms(10);
		Data_Pare();
// 		IMU_AHRSupdate(acc, gyro, Hmc_X,Hmc_Y,Hmc_Z);
  	MPUpDate( acc, gyro);
		EularAngle_calculate(&EA);
		
	 	roll  = roll  + EA.Roll;
		pitch = pitch + EA.Pitch;
		yaw   = yaw   + EA.Yaw;
		delay_ms(10);
			printf("hehe--- %.2lf  %.2lf  %.2lf\r\n",EA.Roll,EA.Pitch,EA.Yaw);	
	}
	
	EA_Origin.Roll  = roll  / 100.0;
  EA_Origin.Pitch = pitch / 100.0;
  EA_Origin.Yaw   = yaw   / 100.0;
	roll=0,pitch=0,yaw=0;
	
	LED3 =!LED3;
}
void suanfa( )
{
	Data_Pare();
// 	IMU_AHRSupdate(acc, gyro, Hmc_X,Hmc_Y,Hmc_Z);
	MPUpDate( acc, gyro);
	EularAngle_calculate(&EA);	
	
	EA.Roll  -= EA_Origin.Roll ;
	EA.Pitch -= EA_Origin.Pitch;
	EA.Yaw   -= EA_Origin.Yaw  ;
 
// 	if(fabs(EA.Roll>60) || (fabs(EA.Pitch)>60) )
// 		{
// 				Power = 0; kP = 0;kI = 0;
// 		}
}

 




