/*
 * canMessage.hpp
 *
 *  Created on: Jul 11, 2014
 *      Author: pgamov
 */

#ifndef CANMESSAGE_HPP_
#define CANMESSAGE_HPP_

#include "message.hpp"
#include "string.h"

namespace subaruCAN{

	class CanMessage : public Message{

		public:
			virtual bool sendMessage();
			const uint32_t& getLength()const{return lenth;}
		public:
			CanMessage() : lenth(0){};
			CanMessage(const uint32_t msgId, const uint32_t msgLenth) : Message(msgId), lenth(msgLenth){};
			CanMessage(const uint32_t msgId, const uint32_t msgLenth, const char *msgString) : Message(msgId, msgString), lenth(msgLenth){};
			CanMessage(const char *msgString, const uint32_t msgLenth) : Message(msgString), lenth(msgLenth){};
			CanMessage(const CanMessage& msg) : Message(msg), lenth(msg.lenth){};
			virtual ~CanMessage();

		protected:
			uint8_t *data;
			const uint32_t lenth;
	};
}

#endif /* CANMESSAGE_HPP_ */
