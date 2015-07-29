/*
 * TJA105X.hpp
 *
 *  Created on: Jul 14, 2014
 *      Author: pgamov
 */

#ifndef TJA105X_HPP_
#define TJA105X_HPP_

#include <stdint.h>

class TJA105x {
	enum TranscieverStatus{
		OK,
		POWER_FAILURE
	};
	enum TranscieverStates{
		SLEEP,
		GOTO_SLEEP,
		VccSTANDBY,
		NORMAL
	};


public:
	TJA105x() : opertaionTout(1000){};
	TJA105x(const uint32_t& tout) : opertaionTout(tout){};
	~TJA105x();

	bool setState(const TranscieverStates& state);
	void sendWakeUpSequence();

private:
	bool coldStart();
	virtual void wakeUpHandler()=0;
	bool setStateSleep();
	bool setStateGoToSleep();
	bool setStateVccStandby();
	bool setStateNormal();

private:
	virtual void setPin_EN(const bool& state)=0;
	virtual void setPin_STB(const bool& state)=0;
	virtual void setPin_WAKE(const bool& state)=0;
	virtual const bool& getPin_EN()const=0;
	virtual const bool& getPin_STB()const=0;
	virtual const bool& getPin_WAKE()const=0;

	virtual const bool& getPin_ERR()const=0;

private:
	const uint32_t opertaionTout;
};



#endif /* TJA105X_HPP_ */
