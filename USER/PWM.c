#include "PWM.h"
 
void TIM4_PWM_Init(u16 arr,u16 psc)			//初始化 定时器4_PWM
{
    //宏定义 配置
	GPIO_InitTypeDef TIM4_PWM_Pin;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM4_OCInitStructure;
	
    //时钟配置
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//使能定时器4时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   //使能GPIO外设功能模块时钟
	
	//配置PWM输出IO口
	TIM4_PWM_Pin.GPIO_Mode = GPIO_Mode_AF_PP;			    //功能模式：复用推挽输出
	TIM4_PWM_Pin.GPIO_Pin = GPIO_Pin_7;                     //IO管脚号：7
	TIM4_PWM_Pin.GPIO_Speed = GPIO_Speed_50MHz;             //传输速度：50M
	GPIO_Init(GPIOB,&TIM4_PWM_Pin);
	
	//初始化TIM4
	TIM_TimeBaseStructure.TIM_Period = arr - 1; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc - 1; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	//初始化TIM4_CH2  PWM模式	 
	TIM4_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM4_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM4_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OC2Init(TIM4, &TIM4_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC2
 
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR2上的预装载寄存器
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4
}