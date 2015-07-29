/*
 * bspSTM32.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: pgamov
 */


#include "bspSTM32.hpp"
#include "stm32f10x.h"


using namespace BoardSupportPackageSpace;


void BoardSupportPackageSTM32::systemStop(void){
	SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;
#ifdef DEBUG
	DBGMCU_Config(DBGMCU_STOP, ENABLE);
#endif
	PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI);
	SystemInit();
}
void BoardSupportPackageSTM32::systemSleep(void){
#ifdef DEBUG
	DBGMCU_Config(DBGMCU_SLEEP, ENABLE);
#endif
	__WFI();
}
void BoardSupportPackageSTM32::systemStandby(void){
#ifdef DEBUG
	DBGMCU_Config(DBGMCU_STANDBY, ENABLE);
#endif
	PWR_EnterSTANDBYMode();
}

const uint32_t& BoardSupportPackageSTM32::getSystemFrequency() const{
	return RCC_Clocks.SYSCLK_Frequency;
}

const frequencySource& BoardSupportPackageSTM32::getClockSource() const{
	return (frequencySource)RCC_GetSYSCLKSource();
}
const uint32_t& BoardSupportPackageSTM32::getHCLKFrequency() const{
	return RCC_Clocks.HCLK_Frequency;
}
const uint32_t& BoardSupportPackageSTM32::getPCLK1Frequency() const{
	return RCC_Clocks.PCLK1_Frequency;
}
const uint32_t& BoardSupportPackageSTM32::getPCLK2Frequency() const{
	return RCC_Clocks.PCLK2_Frequency;
}
const uint32_t& BoardSupportPackageSTM32::getADCCLKFrequency() const{
	return RCC_Clocks.ADCCLK_Frequency;
}

void BoardSupportPackageSTM32::getClockFrequences(){
	RCC_GetClocksFreq(&RCC_Clocks);
}
BoardSupportPackageSTM32::~BoardSupportPackageSTM32(){

}
