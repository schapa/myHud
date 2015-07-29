/*
 * mycan.h
 *
 *  Created on: 15 ����. 2014 �.
 *      Author: shapa
 */

#ifndef MYCAN_H_
#define MYCAN_H_

#include <stdint.h>


typedef enum{
	BRIGHTNESS_1 = 0,
	BRIGHTNESS_2 = 0x02,
	BRIGHTNESS_3 = 0x04,
	BRIGHTNESS_4 = 0x08,
	BRIGHTNESS_5 = 0x0B,
	BRIGHTNESS_6 = 0x0F
}BrightnessLevel_e;

typedef struct{
	uint8_t DRIFT_SNOW:1;
	uint8_t DRIFT_SNOW_SOUND:1;
	uint8_t ESP_OFF:1;
	uint8_t z_1:5;

	uint8_t byte_1;
	uint8_t byte_2;
	uint8_t byte_3;
	uint8_t byte_4;
	uint8_t byte_5;
	uint8_t byte_6; // some codes displayed on disp in idle.
	uint8_t byte_7;

}msg_ID0x21_t;

typedef struct{
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
	uint8_t BRIGHTNESS:4;

//	b5,6
	uint8_t z_5;
	uint8_t z_6;

//	b7
	uint8_t ENGUINE_TEMPERATURE; // x - 40

}msg_ID0x23_t;

typedef struct{
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
}msg_ID0x40_t;

typedef struct{
	uint8_t byte_0;
	uint8_t byte_1;
	uint8_t byte_2;
	uint8_t byte_3;

	uint32_t ODOMETER_READINGS:24;  //1bit/km
	uint8_t byte_7;
}msg_ID0x41_t;

typedef struct{
	uint8_t FUEL_LEVEL; // 255 - x

	uint8_t byte_1;
	uint8_t byte_2;
	uint8_t byte_3;
	uint8_t byte_4;
	uint8_t byte_5;
	uint8_t byte_6;
	uint8_t byte_7;

}msg_ID0x80_t;



void catTest(void);


#endif /* MYCAN_H_ */
