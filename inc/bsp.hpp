/*
 * bsp.hpp
 *
 *  Created on: 22 ����. 2014 �.
 *      Author: shapa
 */

#ifndef BSP_HPP_
#define BSP_HPP_

#include <stdint.h>
#include "mycan.h"

class BoardSupportPack{

public:
        static BoardSupportPack& getInstance(){
                static BoardSupportPack theSingleInstance;
                return theSingleInstance;
        }
        void delay_ms(uint64_t);
        void delay_s(uint64_t);
        void SysTick_Handler(void);

        void SystemInit(void);

        void setCAN_Regulator(const bool state);
        void goToStop(void);

        uint8_t getEnguineTemperature(void);
        uint8_t getFuelGuage(void);

        void powerChecker(void);

        void setIgnitionState(bool state){ ignitionState = state;}
        bool getIgnitionState(void);

        void setDoorOpen(uint8_t door);
        uint8_t getDoorOpen(void);
        void setOilLevelLow(bool state);
        bool getOilLevelLow(void);
        void setNoChargeCurrent(bool state);
        bool getNoChargeCurrent(void);

        void setBrightness(uint8_t);

public:
		const static uint32_t ticksPerSecond = 1000;

private:
        BoardSupportPack();
        BoardSupportPack(const BoardSupportPack& root);
        BoardSupportPack& operator=(const BoardSupportPack&);
        ~BoardSupportPack();
        void transmitData(void);


private:
        volatile static uint64_t SysTickDelay;
        volatile static uint16_t pwmCaptureTout;
        volatile static uint64_t transmitCounter;
        uint8_t doorOpen;
        bool ignitionState;
        bool oilLowLevel;
        bool noChargeCurrent;
        bool toSend;

    	msg_ID0x21_t espMSG;
    	msg_ID0x23_t doorMSG;
    	msg_ID0x40_t statusMSG;
    	msg_ID0x41_t odometerMSG;
    	msg_ID0x80_t fuelMSG;
};
extern BoardSupportPack &BSP;

#endif //BSP_HPP_
