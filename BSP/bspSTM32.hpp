/*
 * bspSTM32.hpp
 *
 *  Created on: Jul 14, 2014
 *      Author: pgamov
 */

#ifndef BSPSTM32_HPP_
#define BSPSTM32_HPP_

#include "bspBase.hpp"
#include "stm32f10x_rcc.h"

namespace BoardSupportPackageSpace{

enum frequencySource{
	source_HSI = 0x00,
	source_HSE = 0x04,
	source_PLL = 0x08,
};

	class BoardSupportPackageSTM32 : public BoardSupportPackageBase{

	public:

		virtual void systemStop(void);
		virtual void systemSleep(void);
		virtual void systemStandby(void);

		virtual const uint32_t& getSystemFrequency()const;

		const frequencySource& getClockSource() const;
		const uint32_t& getHCLKFrequency() const;
		const uint32_t& getPCLK1Frequency() const;
		const uint32_t& getPCLK2Frequency() const;
		const uint32_t& getADCCLKFrequency() const;


	public:
		static BoardSupportPackageSTM32& getInstance(){
			static BoardSupportPackageSTM32 theSingleInstance;
			return theSingleInstance;
		}
		virtual ~BoardSupportPackageSTM32();
	private:
		BoardSupportPackageSTM32(){};


	private:
		RCC_ClocksTypeDef RCC_Clocks;
		virtual void getClockFrequences();
	};

}


#endif /* BSPSTM32_HPP_ */
