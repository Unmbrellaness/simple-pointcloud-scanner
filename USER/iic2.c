#include "stm32f10x.h"
#include "sys.h" 
#include "delay.h"
#include "iic2.h"



void i2c_init(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2 , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 		//B10,B11
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;  //复用开漏输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
		
	
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 50000;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	
	
	I2C_Init(I2C2, &I2C_InitStructure);
	
	I2C_Cmd(I2C2, ENABLE); 
}


void i2c_write_reg(u8 deviceAddr, u8 registerAddr, u8 value)
{
    // Send start condition
    I2C_GenerateSTART(I2C2, ENABLE);

    // Wait for start condition to be sent
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));		//EV5

    // Send device address with write bit
    I2C_Send7bitAddress(I2C2, deviceAddr, I2C_Direction_Transmitter);

    // Wait for device address to be sent
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));		//EV5

    // Send register address
    I2C_SendData(I2C2, registerAddr);

    // Wait for register address to be sent
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING));

    // Send value
    I2C_SendData(I2C2, value);

    // Wait for value to be sent
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // Send stop condition
    I2C_GenerateSTOP(I2C2, ENABLE);
}

u8 I2C2_ReadRegister(u8 deviceAddr, u8 registerAddr)
{
    u8 value;

    // Send start condition
    I2C_GenerateSTART(I2C2, ENABLE);

    // Wait for start condition to be sent
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

    // Send device address with write bit
    I2C_Send7bitAddress(I2C2, deviceAddr, I2C_Direction_Transmitter);

    // Wait for device address to be sent
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    // Send register address
    I2C_SendData(I2C2, registerAddr);

    // Wait for register address to be sent
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING));

    // Send repeated start condition
    I2C_GenerateSTART(I2C2, ENABLE);

    // Wait for repeated start condition to be sent
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

    // Send device address with read bit
    I2C_Send7bitAddress(I2C2, deviceAddr, I2C_Direction_Receiver);

    // Wait for device address to be sent
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    // Disable ACK
    I2C_AcknowledgeConfig(I2C2, DISABLE);

    // Send stop condition
    I2C_GenerateSTOP(I2C2, ENABLE);

    // Wait for value to be received
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));

    // Read value
    value = I2C_ReceiveData(I2C2);
		
		I2C_AcknowledgeConfig(I2C2, ENABLE);

    return value;
}



