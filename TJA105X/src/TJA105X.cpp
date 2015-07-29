/*
 * TJA105X.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: pgamov
 */


#include "TJA105X.hpp"


bool TJA105x::coldStart(){
	setPin_STB(false);
	setPin_EN(false);
	if(getPin_ERR() == true){
		setPin_STB(true);
		wakeUpHandler();
	}
	setPin_EN(true);
	return true;
}
void TJA105x::sendWakeUpSequence(){
//	todo: send 0x019 frame - wakeup
}

bool TJA105x::setStateSleep(){
	setStateGoToSleep();
//TODO: enable wakeUpInterrrupt
	return true;
}
bool TJA105x::setStateGoToSleep(){
	setPin_STB(false);
	setPin_EN(true);
	return true;
}
bool TJA105x::setStateVccStandby(){
	setPin_STB(true);
	setPin_EN(false);
	return true;
}
bool TJA105x::setStateNormal(){
	setPin_STB(true);
	setPin_EN(true);
	return true;
}
