#include "PWM.h"
 
void TIM4_PWM_Init(u16 arr,u16 psc)			//��ʼ�� ��ʱ��4_PWM
{
    //�궨�� ����
	GPIO_InitTypeDef TIM4_PWM_Pin;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM4_OCInitStructure;
	
    //ʱ������
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//ʹ�ܶ�ʱ��4ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   //ʹ��GPIO���蹦��ģ��ʱ��
	
	//����PWM���IO��
	TIM4_PWM_Pin.GPIO_Mode = GPIO_Mode_AF_PP;			    //����ģʽ�������������
	TIM4_PWM_Pin.GPIO_Pin = GPIO_Pin_7;                     //IO�ܽźţ�7
	TIM4_PWM_Pin.GPIO_Speed = GPIO_Speed_50MHz;             //�����ٶȣ�50M
	GPIO_Init(GPIOB,&TIM4_PWM_Pin);
	
	//��ʼ��TIM4
	TIM_TimeBaseStructure.TIM_Period = arr - 1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc - 1; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	//��ʼ��TIM4_CH2  PWMģʽ	 
	TIM4_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM4_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM4_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�
	TIM_OC2Init(TIM4, &TIM4_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC2
 
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
	
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
}