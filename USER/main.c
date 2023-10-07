#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
 
/************************************************
 ALIENTEK精英STM32开发板实验9
 PWM输出实验  
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/

void read_from_tof(void);
void write_tof1(void);
void get_tof_graph(void);			//while	
void get_tof_graph2(void);		//if
void test1(void);
u16 angle_to_pwmdata(float angle);

 int main(void)
 {		
 //	u16 led0pwmval=0;
//	u8 dir=1;	
	u8 Res = 0;
	u16 i,j;
	 u8 t;
	 u16 len;
	 

	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init1(115200);	 //串口1初始化为115200
	USART2_Init(115200);//串口2初始化为115200
//	i2c_init();						//I2C初始化
 	TIM3_PWM_Init(20000,72);	 //不分频。PWM周期=20ms
	/*
	0度对应500
	90度对应1500
	180对应2500
	*/
	
	
	printf("%d",angle_to_pwmdata(0));
	printf("%d",angle_to_pwmdata(90));
	printf("%d",angle_to_pwmdata(180));
	
//	TIM_SetCompare3(TIM3,angle_to_pwmdata(150));
//	delay_ms(5000);
	 
	 while(1){
		 
		 delay_ms(3000);
		printf("begin\r\n");
		get_tof_graph();
		printf("over\r\n");
	}
}


void read_from_tof(void)
{
	u16 i;
	u8 data[8] = {0x01,0x03,0x00,0x10,0x00,0x01,0x85,0xcf};			//01 03 00 10 00 01 85 CF
		for(i=0;i<8;i++)
		{
			USART_SendData(USART2, data[i]);//向串口2发送数据
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//等待发送结束
		}
}


void write_tof1(void)				//设置1号机的测距模式为高精度
{
	u16 i;
	u8 data[8] = {0x01,0x06,0x00,0x04,0x00,0x00,0xc8,0x0b};			//01 06 00 04 00 00 C8 0B
		for(i=0;i<8;i++)
		{
			USART_SendData(USART2, data[i]);//向串口2发送数据
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//等待发送结束
		}
}
	

void get_tof_graph(void)
{
	u16 i,j,data,len;
	u16 x = 500;
	u16 y = 500;
	
	float x_angle = 100;
	float	y_angle = 70;
	
	float x_judge = 0;
	float	y_judge = 0;
	
	float x_step = 0.4;
	float y_step = 0.2;
	
	float x_side = 160;
	float y_side = 110;
	
//	printf("%d %d \r\n" ,x_angle + x_judge ,x_side);
	
			while( x_angle < x_side)		//若超出边界，则跳出循环
		 {
			 x = angle_to_pwmdata(x_angle + x_judge);
			 TIM_SetCompare3(TIM3,x);
			 
			 x_angle += x_step;								//每次转动x_step角度	
			 y_angle = 70;
			 
			 TIM_SetCompare4(TIM3,angle_to_pwmdata(y_angle));
			 delay_ms(300);
			 
					 while(y_angle < y_side){
						 
								y = angle_to_pwmdata(y_angle + y_judge);
								TIM_SetCompare4(TIM3,y);
						 
								y_angle += y_step;								//每次转动x_step角度
								 
								data = 0;
								 
								read_from_tof();
								 
								len=USART_RX_STA2&0x00ff;//得到此次接收到的数据长度
								if(USART_RX_BUF2[0] == 1 &&USART_RX_BUF2[1] == 3 && USART_RX_BUF2[2] == 2 &&len >= 7)
								{
									data = USART_RX_BUF2[3]<<8;
									data |= USART_RX_BUF2[4];
									
								}
								USART_RX_STA2=0;
								
								delay_ms(300);
								 
								printf("%d %d %d \r\n" ,y ,x ,data);
						}
		
			}
		
}

void get_tof_graph2(void)
{
	u16 i,j,data,len,x,y;
	
		float x_angle = 90;
	float	y_angle = 0;
	
	
			for(i = 1500;i>600;i-=10)
		 {
			 x = - i/10 + 150;
			 TIM_SetCompare3(TIM3,i);
			 
			 TIM_SetCompare4(TIM3,600);
			 delay_ms(300);
			 
					 for(j = 600;j<2400 ;j+=10){
								TIM_SetCompare4(TIM3,j);
								 
								data = 0;
								 
								read_from_tof();
								 
								len=USART_RX_STA2&0x3fff;//得到此次接收到的数据长度
								if(USART_RX_BUF2[0] == 1 &&USART_RX_BUF2[1] == 3 && USART_RX_BUF2[2] == 2 &&len >= 7)
								{
									data = USART_RX_BUF2[3]<<8;
									data |= USART_RX_BUF2[4];
									
								}
								USART_RX_STA2=0;
								delay_ms(250);
								 
								printf("%.3f %.3f %d\r\n",y_angle,x_angle,data);
						}
		
			}
		
}



void test1(void)
{
					
	u16 i,x,j,len,data1;
				i-=10;
				x = - i/10 + 150;
				TIM_SetCompare3(TIM3,i);
				
				delay_ms(100);
				
				for(j = 2400;j>600 ;j-=20){
				TIM_SetCompare4(TIM3,j);
//				 y[j/10-50] = j;
				 data1 = 0;
				 
				read_from_tof();
				len=USART_RX_STA2&0x3fff;//得到此次接收到的数据长度
				if(USART_RX_BUF2[0] == 1 &&USART_RX_BUF2[1] == 3 && USART_RX_BUF2[2] == 2 &&len >= 5)
				{
					data1 = USART_RX_BUF2[3]<<8;
					data1 |= USART_RX_BUF2[4];
					
				}
				USART_RX_STA2=0;
				delay_ms(100);
				 
				printf("%d %d %d\r\n",(j-600)/10,x,data1);
				}
				 
			
}
 

u16 angle_to_pwmdata(float angle)
{
	float data;
	int pwmdata;
	u16 pwmdata1;
	data = (angle/180.0) * 2000 + 500;
//	printf("%.3f\r\n",data);
	pwmdata = (int)data;
//	printf("%d\r\n",pwmdata);
	pwmdata1 = (u16)pwmdata;
//	printf("%d\r\n",pwmdata1);
	return pwmdata1;

	
	
}
