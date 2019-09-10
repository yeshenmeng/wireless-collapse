#ifndef __IOT_OPERATE_H__
#define __IOT_OPERATE_H__
#include "main.h"
#include "iotobject.h"
#include "inclinometer.h"


#define LONG_ADDR_ID						1
#define SHORT_ADDR_ID						2
#define SAMPLE_MODE_ID						3
#define SAMPLE_INTERVAL_ID					4
#define TIME_STAMP_ID						5
#define BATTERY_LEVEL_ID					6
#define TEMPERATURE_ID						7
#define DATA_X_ANGLE_ID 					8
#define DATA_Y_ANGLE_ID 					9
#define DATA_X_THRESHOLD_ID 				10
#define DATA_Y_THRESHOLD_ID 				11

typedef struct {
	uint8_t gas_gauge_flag;
	uint8_t gas_gauge;

	iot_object_t *sensor;
	inclinometer_obj_t *inclinometer_obj;

	void (*operate)(void);
} iot_dev_t;

void iot_operate(void);
iot_dev_t * iot_init(iot_object_t *sensor, inclinometer_obj_t *inclinometer_obj);
void iot_write_long_addr(uint8_t* value);
void iot_write_short_addr(uint8_t* value);

#endif
