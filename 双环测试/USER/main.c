#include "allinclude.h"


float cord[3];

extern EularAngle EA;
extern EularAngle EA_Origin;
extern int Power;	//油门
extern float KP;
extern float TD;
extern u8 NRF_flag;
extern u8 tmp_buf[30];
extern u8 LOCK,UN_LOCK;
extern float Target_x,Target_y,Target_yaw;		//目标值 

extern float M_deal1,M_deal2,M_deal3,M_deal4;


 int main(void)
 {		
	u16 sta , i=0;
	int t = 200;
	delay_init();	    	 //延时函数初始化	
	LED_Init();
	LED0 = 0;delay_ms(200); LED0 = 1;
	LED1 = 0;delay_ms(200); LED1 = 1;
	LED2 = 0;delay_ms(200); LED2 = 1;
	LED3 = 0;delay_ms(200); LED3 = 1;
	 
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(9600);	 	//串口初始化为9600
	 
	EXTIX_Init( );
 	NRF24L01_Init();    	//初始化NRF24L01 
	
	NRF24L01_RX_Mode();		
 	while(NRF24L01_Check() == 1)//检查NRF24L01是否在位.	
	{
			LED3=!LED3;				printf("NRF ERROR!!\r\n");

			delay_ms(100 );
	}
	printf("NRF okk!!\r\n");
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
	NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器
	
	
	TIM2_PWM_Init(999,9);  //PWM OUT
	TIM3_Int_Init(0xffff,71);  //做计时器用

	IIC_Init();	 
	InitMPU6050();
	Init_HMC5883();
	suanfa_GetOrigin(); //初始欧拉角 
	

	
	TIM4_Int_Init(49,7199);	  		//PID调速中断 放在最后初始化，防止打断角度校准 200hz 0.005s
	LOCK = 0;
	UN_LOCK = 1;
	
	while(1){
		
		
		if(LOCK)
		{
			
			lock();
			LED0 = 0;delay_ms(500);LED0 = 1;
			Power = 0;
			Target_x = 0;
			Target_y = 0;
				
		}
		if(UN_LOCK)
		{
			suanfa();
				PID_Deal( );
			
				printf("%.1f  %.1f  %.1f  %.1f  ",M_deal4,M_deal1,M_deal2,M_deal3);

			printf("%.2lf  %.2lf  %.2lf\r\n",EA.Roll,EA.Pitch,EA.Yaw);	
// 			TIM_Cmd(TIM4, ENABLE);
			Data_Receive_Anl();
		}
		
// 		printf("--p\r\n");
			 
			
					}
}
void EXTI9_5_IRQHandler(void)
{	
	if(EXTI_GetITStatus(EXTI_Line5)!=RESET)		//PR 1 发生中断请求
	{		
			EXTI_ClearITPendingBit(EXTI_Line5);  //清除c5上的中断标志位
			
			TIM_Cmd(TIM4, DISABLE);
			NRF_flag= 1;
// 			printf("%d\r\n",NRF_flag);
			Nrf_Stop;
	}
	LED2 =!LED2;
}


void TIM4_IRQHandler(void)   //TIM4中断
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
// 			Data_Receive_Anl();
			PID_Deal( );
// 			printf("p\r\n");
		}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 

}
