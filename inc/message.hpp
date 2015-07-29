/*
 * message.hpp
 *
 *  Created on: Jul 11, 2014
 *      Author: pgamov
 */

#ifndef MESSAGE_HPP_
#define MESSAGE_HPP_


#include <stdint.h>

namespace subaruCAN{

	class Message{
		public:
			virtual bool sendMessage()=0;
			virtual const char *toString();

			const uint32_t& getId() const{return messageId;}

		public:
			Message(): messageId(-1), messageString("\0"){};
			Message(const uint32_t msgId): messageId(msgId), messageString("\0"){};
			Message(const uint32_t msgId, const char *msgString) : messageId(msgId){	setMessage(msgString);};
			Message(const char *msgString) : messageId(-1){ setMessage(msgString);};
			Message(const Message& msg): messageId(msg.messageId), messageString(msg.messageString){};
			virtual ~Message();

		public:
			const uint32_t messageId;
			const char *messageString;
		private:
			virtual void setMessage(const char *msgString);
	};
}


#endif /* MESSAGE_HPP_ */
