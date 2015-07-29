/*
 * canMessage.cpp
 *
 *  Created on: Jul 11, 2014
 *      Author: pgamov
 */


#include "canMessage.hpp"
#include "assert.h"



extern bool __transmitCAN_MSG(uint16_t StdId, uint8_t *data,uint8_t len);

namespace subaruCAN{
	bool CanMessage::sendMessage(){
		assert(messageId != (uint32_t)-1);
		assert(lenth <= 8);
		return __transmitCAN_MSG(messageId, data, lenth);
	}


	CanMessage::~CanMessage(){
	}
}
