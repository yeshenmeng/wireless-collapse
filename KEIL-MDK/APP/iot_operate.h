#ifndef __IOT_OPERATE_H__
#define __IOT_OPERATE_H__
#include "main.h"
#include "sys_param.h"
#include "iotobject.h"
#include "collapse.h"
#include "lora_transmission.h"


#define LONG_ADDR_ID					1
#define SHORT_ADDR_ID					2
#define SAMPLE_MODE_ID					3
#define SAMPLE_INTERVAL_ID				4
#define TIME_STAMP_ID					5
#define ACCEL_SLOPE_THRESHOLD_ID 		6
#define CONSECUTIVE_DATA_POINTS_ID		7
#define BATTERY_LEVEL_ID				8
#define TEMPERATURE_ID					9
#define DATA_X_ACCEL_ID 				10
#define DATA_Y_ACCEL_ID 				11
#define DATA_Z_ACCEL_ID 				12
#if (IOT_PROTOCOL_WITH_ANGLE == 1)
#define DATA_X_ANGLE_ID 				13
#define DATA_Y_ANGLE_ID 				14
#define DATA_Z_ANGLE_ID 				15
#endif

typedef struct {
	uint8_t gas_gauge_flag;
	uint8_t gas_gauge;

	iot_object_t *sensor;
	collapse_obj_t *collapse_obj;
	lora_obj_t* lora_obj;

	void (*operate)(void);
} iot_dev_t;

void iot_operate(void);
iot_dev_t * iot_init(iot_object_t *sensor, collapse_obj_t *collapse_obj, lora_obj_t* lora_obj);
void iot_write_long_addr(uint8_t* value);
void iot_write_short_addr(uint8_t* value);

#endif
