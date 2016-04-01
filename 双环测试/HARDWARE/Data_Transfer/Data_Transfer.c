#include "data_transfer.h"
#include "allinclude.h"

u8 tmp_buf[30]={0x00};
T_RC_Data Rc_D;
u8 NRF_flag = 0;
u8 LOCK=1,UN_LOCK = 1;
u8 first_unlock = 0;

extern int Power;										//油门
extern PID Pitch_out,Roll_out,Pitch_inner,Roll_inner;
float Target_x,Target_y,Target_yaw;		//目标值 
extern EularAngle EA_command;

void msg_check_2();

int Limit_Power(int num,double min,double max)
{
	if(num<=min){num=min;return num;}
	else
	if(num>max){ num=max;return num;}
	else
		return num;

}

void lock()
{
	u8 i=0;
	if(LOCK)
		{
// 			printf("locking\r\n");
		if(NRF_flag == 1)
					{
							if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,则显示出来.
							{
								for(i=0;i<9;i++)
								{
									if(tmp_buf[i] != 0xff)break;
									UN_LOCK = 0;
								}
								if(i>=9)
								{
									UN_LOCK = 1;//解锁成功
									LOCK = 0;   //不需要继续解锁
// 									TIM_Cmd(TIM4, ENABLE);
									first_unlock =1;
									i=0;
								}
							NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
							}
							
		
				NRF_flag = 0;
					
					Nrf_Star;
					LED2 =!LED2;	
				}
		}
	}
void un_lock()
{ 
	u8 i=0;
	if(UN_LOCK)
		{
// 			printf("moving\r\n");
		if(NRF_flag == 1)
					{
							if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,则显示出来.
							{
								
// 								while(tmp_buf[i]!= '\0')
// 								{
// 									printf("tmp_buf[%d]  %x\r\n",i,tmp_buf[i]);
// 									i++;
// 								}
								i=0;
							NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
							}
							
		// 			if( tmp_buf[0] != 0xff && tmp_buf[1] != 0xff );
						
					if( tmp_buf[0] == 0xff && tmp_buf[1] == 0xff ) 	
					{
						if(first_unlock)
						{
							Rc_D.THROTTLE  = 0;
							Rc_D.PITCH		 = 0;
							Rc_D.ROLL	 		 = 0;
							first_unlock = 0;
						}
						else
						{
							Rc_D.THROTTLE 		= (vs16)(tmp_buf[4]<<8)|(tmp_buf[5]);
							Rc_D.PITCH			  = (vs16)(tmp_buf[6]<<8)|(tmp_buf[7]);
							Rc_D.ROLL	 		    = (vs16)(tmp_buf[8]<<8)|(tmp_buf[9]);
					
					
// 					printf("--      %d %d %d\r\n",Rc_D.THROTTLE,Rc_D.PITCH,Rc_D.ROLL);
// 					if(Rc_D.THROTTLE != 2083)Power = Rc_D.THROTTLE / 5 -150;
						
					if(Rc_D.THROTTLE >3000)Power =Power + 100;
					if(Rc_D.THROTTLE <1000)Power =Power - 100;		
						Power = Limit_Power(Power,0,999);
					
					if(Rc_D.ROLL == 2073 || fabs(Rc_D.ROLL - 2073)>20)
					{
						Target_y = ( Rc_D.ROLL - 2073 )/300;
						
						EA_command.Roll = Target_y;
					}
					else Target_y = 0;
					
					if(Rc_D.PITCH == 2116 || fabs(Rc_D.PITCH - 2116)>20)
					{
						Target_x = -( Rc_D.PITCH - 2116 )/300;
						
						EA_command.Pitch = Target_x;
					}
					else Target_x = 0;
										
					
					NRF_flag = 0;
// 					printf("%d\r\n",NRF_flag);

					TIM_Cmd(TIM4, ENABLE);
					Nrf_Star;
					LED2 =!LED2;	
				}
				EA_command.Yaw = 0;
			}
		if( tmp_buf[0] != 0xff && tmp_buf[1] != 0xff ) 	
		{
			msg_check_2();
			NRF_flag = 0;
			TIM_Cmd(TIM4, ENABLE);
			Nrf_Star;
			LED2 =!LED2;
			
		}
		}
	
	}
}
void Data_Receive_Anl()
{
	static u8 i =0 ;
	if(LOCK)lock();				
	if(UN_LOCK)un_lock();				
// 		
// 					printf("					       %d %d %d\r\n",Rc_D.THROTTLE,Rc_D.PITCH,Rc_D.ROLL);
// 					printf("%d  %.2lf  %.2lf\r\n",Power,Target_x,Target_y);
// 	printf("%d %.1f %.1f %.1f %.1f\r\n",Power,kP_out,kP,kI,kD);			

}


void msg_check_2()
{
	
	char a[20][40];
	int i=0,t=0,u=0,num=0;
// 	if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,则显示出来.
// 		{
// // 				for(i=0;i<30;i++)
// // 						{
// // 							printf("tmp_buf[%d]==---->%d\r\n",i,tmp_buf[i]-48);		
// // 						}		
// 					NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
// 		}
// 		
// 		i=0;
		
	while(tmp_buf[t] != NULL)
			{		
					if(tmp_buf[t] != 32)
					{
						a[u][num]=tmp_buf[t];
						num++;
						t++;
					}
					while(tmp_buf[t] == 32)
					{
						t++;
						if(tmp_buf[t] != 32)
						{
							a[u][num]='\0';
							u++;
							num=0;
							break;
						}

					}
					
			}
	Power  = atof(a[0]);
	Pitch_out.Kp = atof(a[1]);//3.5
	Pitch_out.Ki = atof(a[2]);
	Pitch_out.Kd = atof(a[3]);
			
	Pitch_inner.Kp = atof(a[4]);//0.7
	Pitch_inner.Ki = atof(a[5]);//0.5
	Pitch_inner.Kd = atof(a[6]);//0.03

	Roll_out.Kp =	Pitch_out.Kp ;
	Roll_out.Ki =	Pitch_out.Ki ;
	Roll_out.Kd =	Pitch_out.Kd ;
	Roll_inner.Kp =	Pitch_inner.Kp;//
	Roll_inner.Ki =	Pitch_inner.Ki;
	Roll_inner.Kd =	Pitch_inner.Kd;

			for(i=0;i<u+1;i++)
		{
			for(num = 0;num< 20;num++)a[i][num] = NULL;
		}		
	i=0;
	t=0;
	u=0;
	num=0;
}
