#include "motor.h"
#include "timer.h"
#include "usart.h"
#include "pwm.h"
#include "led.h"
/*********************************************************
The code
Thanks for  @	匿名
All rights Reserve
*********************************************************/

int Power= 0;			//油门

//MOTOR Drive
float M_deal1,M_deal2,M_deal3,M_deal4;
void MM_Drive(int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw)
{
  pidTermRoll = - pidTermRoll;
	pidTermPitch = -pidTermPitch;
//四轴X型
	M_deal1 = Power + pidTermRoll - pidTermPitch + pidTermYaw; //前左
	M_deal2 = Power + pidTermRoll + pidTermPitch - pidTermYaw; //前右
	M_deal3 = Power - pidTermRoll + pidTermPitch + pidTermYaw; //后右
	M_deal4 = Power - pidTermRoll - pidTermPitch - pidTermYaw; //后左
	

	if(M_deal1<=0)M_deal1=0;
	if(M_deal1>=999)M_deal1=999;
	if(M_deal2<=0)M_deal2=0;
	if(M_deal2>=999)M_deal2=999;
	if(M_deal3<=0)M_deal3=0;
	if(M_deal3>=999)M_deal3=999;
	if(M_deal4<=0)M_deal4=0;
	if(M_deal4>=999)M_deal4=999;	  
//M1
	TIM_SetCompare4(TIM2,M_deal4 );//LED0 =!LED0;
 
//M2
	TIM_SetCompare2(TIM2,M_deal1 );//LED1 =!LED1;
 
//M3
	TIM_SetCompare1(TIM2,M_deal2 );//LED2 =!LED2;
 
//M4
	TIM_SetCompare3(TIM2,M_deal3   );//LED3 =!LED3;
 
// 	printf("%.1f  %.1f  %.1f  %.1f\r\n",M_deal4,M_deal1,M_deal2,M_deal3);

}
