/*
 * bspBase.hpp
 *
 *  Created on: Jul 14, 2014
 *      Author: pgamov
 */

#ifndef BSPBASE_HPP_
#define BSPBASE_HPP_

#include <stdint.h>

namespace BoardSupportPackageSpace{

	class BoardSupportPackageBase{

	public:

		void delay_ms(uint64_t val){
			SysTickDelay = val*ticksPerSecond/1000;
			while(SysTickDelay);
		}
		void delay_s(uint64_t val){
			SysTickDelay = val*ticksPerSecond;
			while(SysTickDelay);
		}
		virtual void SysTick_Handler(void){
			decrementSysTick();
			if(upTimeUs++ > 1000){
				upTime++;
				upTimeUs = 0;
			}
		}

		virtual void systemStop(void)=0;
		virtual void systemSleep(void)=0;
		virtual void systemStandby(void)=0;

		virtual const uint32_t& getSystemFrequency()const=0;
		const uint32_t& getTicksPerSecond() const{return ticksPerSecond;}

		inline void decrementSysTick(){
			if(SysTickDelay && SysTickDelay--){}
		}

		const uint64_t&  getUpTime() const{return upTime;}
		const uint16_t& getUpTimeUs() const{return upTimeUs;}


	protected:
		BoardSupportPackageBase() : ticksPerSecond(1000), SysTickDelay(0), upTime(0), upTimeUs(0){getClockFrequences();};
		BoardSupportPackageBase(const uint32_t ticks) : ticksPerSecond(ticks), SysTickDelay(0), upTime(0), upTimeUs(0){getClockFrequences();};
		BoardSupportPackageBase(const BoardSupportPackageBase& root);
		BoardSupportPackageBase& operator=(const BoardSupportPackageBase&);
	public:
		virtual ~BoardSupportPackageBase(){};


	private:
		const uint32_t ticksPerSecond;
		volatile uint64_t SysTickDelay;
		uint64_t upTime;
		uint16_t upTimeUs;

	private:
		virtual void getClockFrequences(){};
	};
}


#endif /* BSPBASE_HPP_ */
