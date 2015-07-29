/*
 * myCan.cpp
 *
 *  Created on: Jul 10, 2014
 *      Author: pgamov
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_can.h"
#include "misc.h"
#include "mycan.h"

#include "bsp.hpp"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

void __transmitCAN_FRAME(uint16_t StdId);
void __transmitCAN_MSG(uint16_t StdId, uint8_t *data);
bool __transmitCAN_MSG(uint16_t StdId, uint8_t *data,uint8_t len);

static void __CAN_gpio_init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);
}

static void __CAN_NVIC_Config(void){
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void __MY_CAN_Config(void){
	CAN_InitTypeDef       CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	RCC_ClocksTypeDef 	  RCC_Clocks;

	__CAN_gpio_init();
	__CAN_NVIC_Config();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);

	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	RCC_GetClocksFreq(&RCC_Clocks);

//	CAN cell init
	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=DISABLE;
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=ENABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;

//	42MHz divided by my Baudrate of 125KHz is 336 clock cycles
//	336 clk = CAN_Prescaler * (1 + CAN_BS1 + CAN_BS2) CAN_SJW???
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;    // SJW (1 bis 4 möglich)
	CAN_InitStructure.CAN_BS1=CAN_BS1_4tq;   // Samplepoint 72%
	CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;    // Samplepoint 72%


	CAN_InitStructure.CAN_Prescaler = RCC_Clocks.PCLK1_Frequency/1000000;

	(CAN_Init(CAN1, &CAN_InitStructure));

	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;

	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);


//	CAN FIFO0 message pending interrupt enable
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN1, CAN_IT_FMP1, ENABLE);                           //Nachrichten im FIFO "1"
	CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);

}
volatile bool recv = false;
volatile uint8_t recvId = 0;
extern "C" void USB_LP_CAN1_RX0_IRQHandler(void){

	CanRxMsg RxMessage;
	RxMessage.StdId = 0x00;
	RxMessage.ExtId = 0x00;
	RxMessage.IDE = 0;
	RxMessage.DLC = 0;
	RxMessage.FMI = 0;
	RxMessage.Data[0] = 0x00;
	RxMessage.Data[1] = 0x00;

	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
//	if( (RxMessage.StdId != 0x011) && (RxMessage.StdId != 0x101) && (RxMessage.StdId != 0x102) &&
//			(RxMessage.StdId != 0x40) && (RxMessage.StdId != 0x41) ){
//		printf("0x%.3X ={",RxMessage.StdId);
//		for(uint8_t i=0; i < RxMessage.DLC ; i++)
//			printf("0x%.X,",RxMessage.Data[i]);
//		printf("}\n");
//	}
	if(RxMessage.StdId == 0x011){
		recv = true;
		recvId = RxMessage.Data[RxMessage.DLC-1];
	}
}

void catTest(void){

	__MY_CAN_Config();

	msg_ID0x21_t espMSG;
	msg_ID0x23_t doorMSG;
	msg_ID0x40_t statusMSG;
	msg_ID0x41_t odometerMSG;
	msg_ID0x80_t fuelMSG;
	memset(&espMSG,0,8);
	memset(&doorMSG,0,8);
	memset(&statusMSG,0,8);
	memset(&odometerMSG,0,8);
	memset(&fuelMSG,0,8);

	espMSG.DRIFT_SNOW = 0;
	espMSG.DRIFT_SNOW_SOUND = 0;
	espMSG.ESP_OFF = 0;


	doorMSG.BRIGHTNESS = 0x0F;
	doorMSG.DOOR_FRONT_LEFT = 1;
	doorMSG.DOOR_FRONT_RIGHT = 1;
	doorMSG.DOOR_REAR_LEFT = 0;
	doorMSG.DOOR_REAR_RIGHT = 0;
	doorMSG.DOOR_TRUNK = 0;
	doorMSG.ECO = 1;
	doorMSG.ENGUINE_TEMPERATURE = 120;
	doorMSG.FOG_LAMP_FRONT = 0;
	doorMSG.FOG_LAMP_REAR = 0;
	doorMSG.IGNITION_KEY_IN_ACC = 0;
	doorMSG.IGNITION_KEY_PRESENT = 1;

	fuelMSG.FUEL_LEVEL = 20;

	statusMSG.ENGINE_RUNNING_STATUS = 1;
	statusMSG.IGNITION_STATUS = 1;

	odometerMSG.ODOMETER_READINGS = 200;


	uint8_t data[8];
	uint8_t IntegrUnit[8];
	uint8_t setter=0;
	memset(&data,0x00,8);
	memset(&IntegrUnit,setter,8);

	volatile	uint32_t i = 0;
	volatile	uint32_t j = 0;
	uint32_t pidMAX=0x150;
	uint32_t pidMIN=0x000;
	uint16_t pid;
	uint8_t *ptr = (uint8_t*)&doorMSG;

	while(1){

		BSP.powerChecker();

//		for(uint16_t pid=pidMIN; pid < pidMAX; pid++){
//			memset(&IntegrUnit,setter,8);
//
//			if((pid != 0x21) && (pid != 0x23) && (pid != 0x40) && (pid != 0x41) && (pid != 0x80))
//				__transmitCAN_MSG(pid,IntegrUnit);
//		}
//		if(recv == true){

//			memset(&IntegrUnit,setter,8);
//			recv = false;
//
//			for(pid=pidMIN; pid < pidMAX; pid++){
//				if((pid != 0x21) && (pid != 0x23) && (pid != 0x40) && (pid != 0x41) && (pid != 0x80))
//					__transmitCAN_MSG(pid,IntegrUnit,8);
//			}
	}
}
void __transmitCAN_FRAME(uint16_t StdId){
	CanTxMsg TxMessage;
	uint8_t TransmitMailbox = 0;
	uint16_t i = 0;

	TxMessage.StdId = StdId;
	TxMessage.RTR = CAN_RTR_Remote;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.DLC = 0;
	memset(TxMessage.Data,0,8);
	TxMessage.ExtId = 0;

	TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
	while((CAN_TransmitStatus(CAN1, TransmitMailbox)  !=  CANTXOK) && (i  !=  0xFFFF)){
		i++;
	}
//	if(CAN_TransmitStatus(CAN1, TransmitMailbox)  !=  CANTXOK)
//		printf("Error in transmition. id = 0x%.3X \n",StdId);
//	else
//		printf("Sucess!!. id = 0x%.3X \n",StdId);
}
void __transmitCAN_MSG(uint16_t StdId, uint8_t* data){
	__transmitCAN_MSG(StdId,data,8);
}
bool __transmitCAN_MSG(uint16_t StdId, uint8_t* data, uint8_t len ){
	CanTxMsg TxMessage;
	uint8_t TransmitMailbox = 0;
	uint16_t i = 0;

	TxMessage.StdId = StdId;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.DLC = len;

	memcpy(TxMessage.Data,data,TxMessage.DLC);

	TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
	while((CAN_TransmitStatus(CAN1, TransmitMailbox)  !=  CANTXOK) && (i  !=  0xFFFF)){
		i++;
	}
	if(CAN_TransmitStatus(CAN1, TransmitMailbox)  !=  CANTXOK)
		return false;
	else
		return true;
}

