/*
 * subaruCANMessaes.hpp
 *
 *  Created on: Jul 11, 2014
 *      Author: pgamov
 */

#ifndef SUBARUCANMESSAES_HPP_
#define SUBARUCANMESSAES_HPP_

#include "canMessage.hpp"

namespace subaruCAN{

	class MessageESP : public CanMessage{
	private:
		const static uint32_t MSG_ID = 0x21;
		const static uint32_t MSG_LENGTH = 8;

	public:
		void setSnowDriftingOn(bool val){DRIFT_SNOW = val;}
		bool getSnowDriftingOn(){return DRIFT_SNOW;}

		void setSnowDriftingSoundOn(bool val){DRIFT_SNOW_SOUND = val;}
		bool getSnowDriftingSoundOn(){return DRIFT_SNOW_SOUND;}

		void setESPon(bool val){ESP_OFF  = !val;}
		bool getESPon(){return !ESP_OFF;}


	public:
		MessageESP() : CanMessage(MSG_ID, MSG_LENGTH, "Drift, Esp, etc."){
			data = dataRaw;
		};
		~MessageESP(){};
	private:
		union{
			uint8_t dataRaw[MSG_LENGTH];
			struct{
				uint8_t DRIFT_SNOW:1;
				uint8_t DRIFT_SNOW_SOUND:1;
				uint8_t ESP_OFF:1;
				uint8_t z_1:5;

				uint8_t byte_1;
				uint8_t byte_2;
				uint8_t byte_3;
				uint8_t byte_4;
				uint8_t byte_5;
				uint8_t byte_6;
				uint8_t byte_7;
			};
		};
	};




	class MessageStatusInfo : public CanMessage{
	private:
		const static uint32_t MSG_ID = 0x23;
		const static uint32_t MSG_LENGTH = 8;
		enum BrightnessLevel{
			BRIGHTNESS_1 = 0,
			BRIGHTNESS_2 = 0x02,
			BRIGHTNESS_3 = 0x04,
			BRIGHTNESS_4 = 0x08,
			BRIGHTNESS_5 = 0x0B,
			BRIGHTNESS_6 = 0x0F
		};

	public:
		void setECOOn(bool val){ECO = val;}
		bool getECOOn(){return ECO;}

		void setFogLampFrontOn(bool val){FOG_LAMP_FRONT = val;}
		bool getFogLampFrontOn(){return FOG_LAMP_FRONT;}

		void setFogLampRearOn(bool val){FOG_LAMP_REAR = val;}
		bool getFogLampRearOn(){return FOG_LAMP_REAR;}


		void setDoorFrontLeftOpen(bool val){DOOR_FRONT_LEFT  = val;}
		bool getDoorFrontLeftOpen(){return DOOR_FRONT_LEFT;}

		void setDoorFrontRigthOpen(bool val){DOOR_FRONT_RIGHT  = val;}
		bool getDoorFrontRigthOpen(){return DOOR_FRONT_RIGHT;}

		void setDoorRearLeftOpen(bool val){DOOR_REAR_LEFT  = val;}
		bool getDoorRearLeftOpen(){return DOOR_REAR_LEFT;}

		void setDoorRearRightOpen(bool val){DOOR_FRONT_RIGHT  = val;}
		bool getDoorRearRightOpen(){return DOOR_FRONT_RIGHT;}

		void setDoorTrunk(bool val){DOOR_TRUNK  = val;}
		bool getDoorTrunk(){return DOOR_TRUNK;}


		void setIgnitionKeyPresent(bool val){IGNITION_KEY_PRESENT  = val;}
		bool getIgnitionKeyPresent(){return IGNITION_KEY_PRESENT;}

		void setIgnitionKeyInACC(bool val){IGNITION_KEY_IN_ACC  = val;}
		bool getIgnitionKeyInACC(){return IGNITION_KEY_IN_ACC;}


		void setBrightness(BrightnessLevel val){BRIGHTNESS  = val;}
		BrightnessLevel getBrightness(){return BRIGHTNESS;}

		void setEnguineTemperature(uint8_t val){ENGUINE_TEMPERATURE  = val;}
		uint8_t getEnguineTemperature(){return ENGUINE_TEMPERATURE;}

	public:
		MessageStatusInfo() : CanMessage(MSG_ID, MSG_LENGTH, "Information: doors, key, FogLamps, brightness, temperature"){
			data = dataRaw;
		};
		~MessageStatusInfo(){};
		union{
			uint8_t dataRaw[MSG_LENGTH];
			struct{
				uint8_t byte_0;
			//	b1
				uint8_t z_1:2;
				uint8_t ECO:1;
				uint8_t z_2:2;
				uint8_t FOG_LAMP_FRONT:1;
				uint8_t FOG_LAMP_REAR:1;

				uint8_t byte_2;
			//	b3
				uint8_t DOOR_FRONT_RIGHT:1;
				uint8_t DOOR_FRONT_LEFT:1;
				uint8_t DOOR_REAR_RIGHT:1;
				uint8_t DOOR_REAR_LEFT:1;
				uint8_t DOOR_TRUNK:1;
				uint8_t z_3:1;
				uint8_t IGNITION_KEY_PRESENT:1;
				uint8_t IGNITION_KEY_IN_ACC:1;
			//	b4
				uint8_t z_4:4;
				BrightnessLevel BRIGHTNESS:4;
			//	b5,6
				uint8_t z_5;
				uint8_t z_6;
			//	b7
				uint8_t ENGUINE_TEMPERATURE; // x - 40
			};
		};
	};



	class MessageIGNITION : public CanMessage{
	private:
		const static uint32_t MSG_ID = 0x40;
		const static uint32_t MSG_LENGTH = 8;

	public:
		void setIgnitionStatus(bool val){IGNITION_STATUS = val;}
		bool getIgnitionStatus(){return IGNITION_STATUS;}

		void setEnguineRunningStatus(bool val){ENGINE_RUNNING_STATUS = val;}
		bool getEnguineRunningStatus(){return ENGINE_RUNNING_STATUS;}

	public:
		MessageIGNITION() : CanMessage(MSG_ID, MSG_LENGTH, "Ignition status, enguine running status"){
			data = dataRaw;
		};
		~MessageIGNITION(){};
		union{
			uint8_t dataRaw[MSG_LENGTH];
			struct{
				uint8_t byte_0:7;
				uint8_t IGNITION_STATUS:1;
				uint8_t byte_1;
				uint8_t byte_2;
				uint8_t byte_3;
				uint8_t byte_4;
				uint8_t byte_5;
				uint8_t byte_6;
				uint8_t z7_1:3;
				uint8_t ENGINE_RUNNING_STATUS:1;
				uint8_t z7_2:4;
			};
		};
	};



	class MessageFUEL : public CanMessage{
	private:
		const static uint32_t MSG_ID = 0x80;
		const static uint32_t MSG_LENGTH = 8;

	public:
		void setFuelLevel(uint8_t val){FUEL_LEVEL = val;}
		uint8_t getFuelLevel(){return FUEL_LEVEL;}

	public:
		MessageFUEL() : CanMessage(MSG_ID, MSG_LENGTH, "Fuel level info"){
			data = dataRaw;
		};
		~MessageFUEL(){};
		union{
			uint8_t dataRaw[MSG_LENGTH];
			struct{
				uint8_t FUEL_LEVEL; // 255 - x

				uint8_t byte_1;
				uint8_t byte_2;
				uint8_t byte_3;
				uint8_t byte_4;
				uint8_t byte_5;
				uint8_t byte_6;
				uint8_t byte_7;

			};
		};
	};

}




#endif /* SUBARUCANMESSAES_HPP_ */
