/*
 * message.cpp
 *
 *  Created on: Jul 11, 2014
 *      Author: pgamov
 */

#include "message.hpp"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

namespace subaruCAN{

	const char* Message::toString(){
		return messageString;
	}

	void Message::setMessage(const char *msgString){
		messageString = new char[strlen(msgString)+8];
		sprintf((char*)messageString,"[0x%.3X] %s", messageId, messageString);
	}

	Message::~Message(){
		delete messageString;
	}

};

