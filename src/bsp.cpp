/*
 * bsp.cpp
 *
 *  Created on: Jul 10, 2014
 *      Author: pgamov
 */

#include "bsp.hpp"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_adc.h"
#include "misc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_tim.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static void __gpioInit(void);
static void __gpioCANPOWERInit(void);
static void __gpioCANTransiverInit(void);
static void __gpioCANInit(void);
static void __gpioADCInit(void);
static void __gpioIRQInit(void);
static void __gpioSpeedoInit(void);
static void __gpioPWMInit(void);
static void __timerInit(void);



static void __nvicInit(void);
static void __extiInit(void);
static void __adcInit(void);

BoardSupportPack &BSP = BoardSupportPack::getInstance();
volatile uint64_t BoardSupportPack::SysTickDelay;
volatile uint16_t BoardSupportPack::pwmCaptureTout;
volatile uint64_t BoardSupportPack::transmitCounter;

extern "C" void SysTick_Handler(void){
	BoardSupportPack::getInstance().SysTick_Handler();
}




void BoardSupportPack::SystemInit(void){
	extern void __MY_CAN_Config(void);
	ignitionState = false;
	memset(&espMSG,0,8);
	memset(&doorMSG,0,8);
	memset(&statusMSG,0,8);
	memset(&odometerMSG,0,8);
	memset(&fuelMSG,0,8);
	doorMSG.BRIGHTNESS = BRIGHTNESS_6;
	doorMSG.IGNITION_KEY_PRESENT = 1;
	pwmCaptureTout = ticksPerSecond/50;

	statusMSG.ENGINE_RUNNING_STATUS = 1;
	statusMSG.IGNITION_STATUS = 1;
	RCC_ClocksTypeDef RCC_ClockFreq;
	RCC_GetClocksFreq(&RCC_ClockFreq);
	SysTick_Config(RCC_ClockFreq.HCLK_Frequency / BSP.ticksPerSecond);
	RCC_ClockSecuritySystemCmd(ENABLE);
	__gpioInit();
	__nvicInit();
	__extiInit();
	__adcInit();
	__MY_CAN_Config();
	__timerInit();
}
BoardSupportPack::BoardSupportPack(){
	SystemInit();
	setCAN_Regulator(true);
}
BoardSupportPack::~BoardSupportPack(){

}

void BoardSupportPack::SysTick_Handler(){
	if (SysTickDelay)
		SysTickDelay--;
	if(transmitCounter++%(ticksPerSecond/10))
		toSend = true;
	if(pwmCaptureTout)
		pwmCaptureTout--;
	else{
		if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_3))
			doorMSG.BRIGHTNESS = BRIGHTNESS_6;
	}
}

void BoardSupportPack::delay_ms(uint64_t delay){
	SysTickDelay = delay/(1000/ticksPerSecond);
	while(SysTickDelay);
}
void BoardSupportPack::delay_s(uint64_t delay){
	SysTickDelay = delay*ticksPerSecond;
	while(SysTickDelay);
}


void BoardSupportPack::setCAN_Regulator(const bool state){
	GPIO_WriteBit(GPIOC,GPIO_Pin_13, (BitAction)state);
}

void BoardSupportPack::goToStop(void){
	setCAN_Regulator(false);
	SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;
	PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI);
	SystemInit();
	setCAN_Regulator(true);
//	delay_s(3);
	CAN_WakeUp(CAN1);
	extern void __transmitCAN_FRAME(uint16_t StdId);
	__transmitCAN_FRAME(0x100);

}
void BoardSupportPack::transmitData(void){
	fuelMSG.FUEL_LEVEL = getFuelGuage();
	doorMSG.ENGUINE_TEMPERATURE = getEnguineTemperature();
	getDoorOpen();
	espMSG.ESP_OFF = getOilLevelLow();
	espMSG.DRIFT_SNOW = getNoChargeCurrent();
	extern void __transmitCAN_MSG(uint16_t StdId, uint8_t* data);

	uint8_t data[8];
	memset(data,0,8);

	__transmitCAN_MSG(0x23,(uint8_t*)&doorMSG);
	if(getIgnitionState()){
//		__transmitCAN_MSG(0x20,(uint8_t*)&data); // info from BUI about errors
//		__transmitCAN_MSG(0x22,(uint8_t*)&data); // if this message arrives, car is Off (0x20 should be sent)
		__transmitCAN_MSG(0x21,(uint8_t*)&espMSG);
		__transmitCAN_MSG(0x40,(uint8_t*)&statusMSG);
//		__transmitCAN_MSG(0x41,(uint8_t*)&odometerMSG);
		__transmitCAN_MSG(0x80,(uint8_t*)&fuelMSG);
	}
}


bool BoardSupportPack::getIgnitionState(void){
	ignitionState = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8);
	return ignitionState;
}
void BoardSupportPack::setDoorOpen(uint8_t door){
    doorOpen = door;
}
uint8_t BoardSupportPack::getDoorOpen(void){
	if(!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_10)){
		doorOpen = 0x0F;
		doorMSG.DOOR_FRONT_LEFT = 1;
		doorMSG.DOOR_FRONT_RIGHT = 1;
		doorMSG.DOOR_REAR_LEFT= 1;
		doorMSG.DOOR_REAR_RIGHT = 1;
	}else{
		doorOpen = 0x00;
		doorMSG.DOOR_FRONT_LEFT = 0;
		doorMSG.DOOR_FRONT_RIGHT = 0;
		doorMSG.DOOR_REAR_LEFT= 0;
		doorMSG.DOOR_REAR_RIGHT = 0;
	}
	return doorOpen;
}
void BoardSupportPack::setOilLevelLow(bool state){
	oilLowLevel = state;
}
bool BoardSupportPack::getOilLevelLow(void){
	if(!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11))
		oilLowLevel = true;
	else
		oilLowLevel = false;
	return oilLowLevel;
}
void BoardSupportPack::setNoChargeCurrent(bool state){
	noChargeCurrent = state;
}
bool BoardSupportPack::getNoChargeCurrent(void){
	if(!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12))
		noChargeCurrent = true;
	else
		noChargeCurrent = false;
	return noChargeCurrent;
}

void BoardSupportPack::powerChecker(void){
	if(getIgnitionState() == false && getDoorOpen() == 0)
		goToStop();
	if(toSend){
		toSend=false;
		transmitData();
		delay_ms(150);
	}
}

void BoardSupportPack::setBrightness(uint8_t dutyCycle){
	pwmCaptureTout = ticksPerSecond/200;

	uint8_t dist15 = abs(dutyCycle-15);
	uint8_t dist35 = abs((float)dutyCycle-35);
	uint8_t dist80 = abs((float)dutyCycle-80);
	if(dist80 < dist35)
		doorMSG.BRIGHTNESS = BRIGHTNESS_1;
	else if(dist35 < dist15)
		doorMSG.BRIGHTNESS = BRIGHTNESS_3;
	else
		doorMSG.BRIGHTNESS = BRIGHTNESS_4;
}

static void __gpioInit(void){
	__gpioCANPOWERInit();
	__gpioCANTransiverInit();
	__gpioCANInit();
	__gpioADCInit();
	__gpioIRQInit();
	__gpioPWMInit();
	__gpioSpeedoInit();
}

static void __gpioCANPOWERInit(void){
	GPIO_InitTypeDef  GPIO_InitStructure;

	BKP_TamperPinCmd(DISABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
static void __gpioCANTransiverInit(void){
	GPIO_InitTypeDef  GPIO_InitStructure;

	BKP_TamperPinCmd(DISABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6);
}
static void __gpioCANInit(void){
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB,ENABLE);

	// TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);
}
static void __gpioADCInit(void){
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
static void __gpioIRQInit(void){
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
static void __gpioSpeedoInit(void){
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
static void __gpioPWMInit(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef gpioInitStuct;

	gpioInitStuct.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStuct.GPIO_Pin = GPIO_Pin_15;
	gpioInitStuct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&gpioInitStuct);
	gpioInitStuct.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOB,&gpioInitStuct);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);
}
static void __nvicInit(void){
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}
static void __extiInit(void){
	EXTI_InitTypeDef EXTI_InitSructrure;

	EXTI_InitSructrure.EXTI_Line = EXTI_Line8 | EXTI_Line9 | EXTI_Line10 | EXTI_Line11 | EXTI_Line12;
	EXTI_InitSructrure.EXTI_LineCmd = ENABLE;
	EXTI_InitSructrure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitSructrure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitSructrure);

	EXTI_Init(&EXTI_InitSructrure);
}



static void __adcInit(){
	ADC_InitTypeDef ADCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	ADCInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADCInitStructure.ADC_ScanConvMode = ENABLE;
	ADCInitStructure.ADC_ContinuousConvMode = ENABLE;
	ADCInitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADCInitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADCInitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADCInitStructure);
	ADC_Init(ADC2, &ADCInitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 1, ADC_SampleTime_239Cycles5);
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);

	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC2);
	while (ADC_GetResetCalibrationStatus(ADC2));
	ADC_StartCalibration(ADC2);
	while (ADC_GetCalibrationStatus(ADC2));
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);
}
static void __timerInit(void){

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef timer_base;
	TIM_TimeBaseStructInit(&timer_base);
	timer_base.TIM_Prescaler = 0xF;
	TIM_TimeBaseInit(TIM2, &timer_base);

	TIM_ICInitTypeDef timer_ic;
	timer_ic.TIM_Channel = TIM_Channel_1;
	timer_ic.TIM_ICPolarity = TIM_ICPolarity_Rising;
	timer_ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
	timer_ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	timer_ic.TIM_ICFilter = 0;
	TIM_PWMIConfig(TIM2, &timer_ic);

	TIM_SelectInputTrigger(TIM2, TIM_TS_TI1F_ED);
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

	TIM_Cmd(TIM2, ENABLE);

	NVIC_EnableIRQ(TIM2_IRQn);
}
extern "C" void TIM2_IRQHandler(void){
	uint16_t period_capture, duty_cycle_capture;
	static volatile uint8_t capture_is_first = 1;
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET){
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		NVIC_DisableIRQ(TIM2_IRQn);
		period_capture = TIM_GetCapture1(TIM2);
		duty_cycle_capture = TIM_GetCapture2(TIM2);
		uint8_t res = duty_cycle_capture*100/(period_capture+duty_cycle_capture);

		NVIC_EnableIRQ(TIM2_IRQn);

		if (!capture_is_first){
			BSP.setBrightness(res);
		}
		capture_is_first = 0;

		if (TIM_GetFlagStatus(TIM3, TIM_FLAG_CC1OF) != RESET){
			TIM_ClearFlag(TIM3, TIM_FLAG_CC1OF);
		}
	}
}
extern "C" void EXTI9_5_IRQHandler(void){

	bool ignition = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8);
	bool speedometer = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_9);

	// ignition
	if(EXTI_GetFlagStatus(EXTI_Line8)){
		BSP.setIgnitionState(ignition);
		EXTI_ClearFlag(EXTI_Line8);
	}

	// speedometer data
	if(EXTI_GetFlagStatus(EXTI_Line9)){
		static uint8_t lowCount = 0;
		static uint8_t hiCount = 0;
		if(speedometer)
			hiCount++;
		else
			lowCount++;
		if(hiCount >= 2){
			GPIO_SetBits(GPIOB,GPIO_Pin_12);
			hiCount = 0;
		}
		if(lowCount >= 2){
			GPIO_ResetBits(GPIOB,GPIO_Pin_12);
			lowCount = 0;
		}
		EXTI_ClearFlag(EXTI_Line9);
	}
}

extern "C" void EXTI15_10_IRQHandler(void){
	if(EXTI_GetFlagStatus(EXTI_Line10)){
		EXTI_ClearFlag(EXTI_Line10);
	}
	if(EXTI_GetFlagStatus(EXTI_Line11)){
		EXTI_ClearFlag(EXTI_Line11);
	}
	if(EXTI_GetFlagStatus(EXTI_Line12)){
		EXTI_ClearFlag(EXTI_Line12);
	}
}

uint8_t BoardSupportPack::getEnguineTemperature(void){
	int16_t thisRes = ADC_GetConversionValue(ADC1)>>4;

	if(thisRes > 175)
		thisRes = 365 - thisRes*1.38;
	else
		thisRes = 214 - thisRes*0.52;

	//	 thisRes = (thisRes>>3)*0.846;

	if(thisRes > 255 || thisRes < 0)
		thisRes = 254;
	return thisRes;
}
uint8_t BoardSupportPack::getFuelGuage(void){
	int16_t thisRes = ADC_GetConversionValue(ADC2)>>4;

	if(thisRes < 44 )
		thisRes =44;
	if(thisRes < 155)
		thisRes =  thisRes*1.8-78;
	else
		thisRes = 255;
	if(thisRes > 255 || thisRes < 0)
		thisRes = 255;
	return thisRes;
}

