#include "iot_operate.h"
#include "sys_param.h"
#include "calendar.h"
#include "ble_init.h"
#include "host_net_swap.h"
#include "lora_transmission.h"
#include "wireless_comm_services.h"


static iot_dev_t IoT_dev;

/* 向IoT对象BUF中写入长地址 */
void iot_write_long_addr(uint8_t* value)
{
	IoT_dev.sensor->writePropFromBuf(LONG_ADDR_ID, value);
	IoT_dev.sensor->resetPropChangeFlag(LONG_ADDR_ID);
}

/* 向IoT对象BUF中写入短地址 */
void iot_write_short_addr(uint8_t* value)
{
	IoT_dev.sensor->writePropFromBuf(SHORT_ADDR_ID, value);
	IoT_dev.sensor->resetPropChangeFlag(SHORT_ADDR_ID);
}

/* 向IoT对象BUF中写入采样模式 */
static void iot_write_sample_mode(uint8_t value)
{
	IoT_dev.sensor->writePropFromBuf(SAMPLE_MODE_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(SAMPLE_MODE_ID);
}

/* 向IoT对象BUF中写入采样间隔 */
static void iot_write_sample_interval(uint32_t value)
{
	IoT_dev.sensor->writePropFromBuf(SAMPLE_INTERVAL_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(SAMPLE_INTERVAL_ID);
}

/* 向IoT对象BUF中写入时间戳 */
static void iot_write_time_stamp(uint32_t value)
{
	IoT_dev.sensor->writePropFromBuf(TIME_STAMP_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(TIME_STAMP_ID);
}

/* 向IoT对象BUF中写入电池电量 */
static void iot_write_battery_level(uint8_t value)
{
	IoT_dev.sensor->writePropFromBuf(BATTERY_LEVEL_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(BATTERY_LEVEL_ID);
}

/* 向IoT对象BUF中写入温度数据 */
static void iot_write_temp(float value)
{
	IoT_dev.sensor->writePropFromBuf(TEMPERATURE_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(TEMPERATURE_ID);
}

/* 向IoT对象BUF中写入倾角X轴数据 */
static void iot_write_x_angle(float value)
{
	IoT_dev.sensor->writePropFromBuf(DATA_X_ANGLE_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(DATA_X_ANGLE_ID);
}

/* 向IoT对象BUF中写入倾角Y轴数据 */
static void iot_write_y_angle(float value)
{
	IoT_dev.sensor->writePropFromBuf(DATA_Y_ANGLE_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(DATA_Y_ANGLE_ID);
}

/* 向IoT对象BUF中写入倾角X轴阈值数据 */
static void iot_write_x_angle_threshold(float value)
{
	IoT_dev.sensor->writePropFromBuf(DATA_X_THRESHOLD_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(DATA_X_THRESHOLD_ID);
}

/* 向IoT对象BUF中写入倾角Y轴阈值数据 */
static void iot_write_y_angle_threshold(float value)
{
	IoT_dev.sensor->writePropFromBuf(DATA_Y_THRESHOLD_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(DATA_Y_THRESHOLD_ID);
}

/* 保存IoT协议设置的长地址 */
static void iot_set_long_addr(uint8_t* value)
{
	sys_param_t* param = sys_param_get_handle();
	sys_param_set((uint8_t*)&param->dev_long_addr, value, sizeof(param->dev_long_addr));
}

/* 保存IoT协议设置的短地址 */
static void iot_set_short_addr(uint8_t* value)
{
	sys_param_t* param = sys_param_get_handle();
	sys_param_set((uint8_t*)&param->dev_short_addr, value, sizeof(param->dev_short_addr));
}

/* 保存IoT协议设置的采样模式 */
static void iot_set_sample_mode(uint8_t value)
{
	sys_param_t* param = sys_param_get_handle();
	sys_param_set((uint8_t*)&param->iot_mode, (uint8_t*)&value, sizeof(value));
}

/* 保存IoT协议设置的采样间隔 */
static void iot_set_sample_interval(uint32_t value)
{
	sys_param_t* param = sys_param_get_handle();
	sys_param_set((uint8_t*)&param->iot_sample_interval, (uint8_t*)&value, sizeof(value));
}

/* 保存IoT协议设置的时间戳 */
static void iot_set_time_stamp(uint32_t value)
{
	calendar_mod_t* calendar = calendar_get_handle();
	calendar->set_time_stamp(value);
}

/* 保存IoT协议设置的倾角X轴阈值 */
static void iot_set_x_angle_threshold(float value)
{
	sys_param_t* param = sys_param_get_handle();
	sys_param_set((uint8_t*)&param->iot_x_angle_threshold, (uint8_t*)&value, sizeof(value));
}

/* 保存IoT协议设置的倾角Y轴阈值 */
static void iot_set_y_angle_threshold(float value)
{
	sys_param_t* param = sys_param_get_handle();
	sys_param_set((uint8_t*)&param->iot_y_angle_threshold, (uint8_t*)&value, sizeof(value));
}

/* IoT协议处理与IoT对象数据处理 */
void iot_operate(void)
{
	/* 处理IoT协议设置的长地址同时更新数据到蓝牙协议中 */
	if(IoT_dev.sensor->isPropChanged(LONG_ADDR_ID))
	{
		uint8_t value[8];
		IoT_dev.sensor->readPropToBuf(LONG_ADDR_ID, (uint8_t *)&value);
		IoT_dev.sensor->resetPropChangeFlag(LONG_ADDR_ID);
		sys_param_t* param = sys_param_get_handle();
		for(int i=0; i<sizeof(value); i++)
		{
			if(value[i] != param->dev_long_addr[i])
			{
				lora_reset();
				break;
			}
		}
		iot_set_long_addr(value);
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_long_addr_update(value);
	}
	
	/* 处理IoT协议设置的短地址同时更新数据到蓝牙协议中 */
	if(IoT_dev.sensor->isPropChanged(SHORT_ADDR_ID))
	{
		uint8_t value[2];
		IoT_dev.sensor->readPropToBuf(SHORT_ADDR_ID, (uint8_t *)&value);
		IoT_dev.sensor->resetPropChangeFlag(SHORT_ADDR_ID);
		wireless_comm_services_t* wirelessCommSvc = Wireless_CommSvcGetHandle();
		if(wirelessCommSvc->_conn_short_addr_update_flag == 0) 
		{
			sys_param_t* param = sys_param_get_handle();
			for(int i=0; i<sizeof(value); i++)
			{
				if(value[i] != param->dev_short_addr[i])
				{
					lora_reset();
					break;
				}
			}
		}
		wirelessCommSvc->_conn_short_addr_update_flag= 0;
		iot_set_short_addr(value);
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_short_addr_update(value);
	}
	
	/* 处理IoT协议设置的采样模式同时更新数据到蓝牙协议中 */
	if(IoT_dev.sensor->isPropChanged(SAMPLE_MODE_ID))
	{
		uint8_t value;
		IoT_dev.sensor->readPropToBuf(SAMPLE_MODE_ID, (uint8_t *)&value);
		IoT_dev.sensor->resetPropChangeFlag(SAMPLE_INTERVAL_ID);
		iot_set_sample_mode(value);
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_sample_mode_update(value);
	}
	
	/* 处理IoT协议设置的采样间隔同时更新数据到蓝牙协议中 */
	if(IoT_dev.sensor->isPropChanged(SAMPLE_INTERVAL_ID))
	{
		uint32_t value;
		IoT_dev.sensor->readPropToBuf(SAMPLE_INTERVAL_ID, (uint8_t *)&value);
		IoT_dev.sensor->resetPropChangeFlag(SAMPLE_INTERVAL_ID);
#if COMM_TRANSMISSION_FORMAT == 1
		swap_reverse((uint8_t*)&value, sizeof(value));
#endif
		iot_set_sample_interval(value);
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_sample_interval_update(value);
	}
	
	/* 处理IoT协议设置的时间戳同时更新数据到蓝牙协议中 */
	if(IoT_dev.sensor->isPropChanged(TIME_STAMP_ID))
	{
		uint32_t value;
		IoT_dev.sensor->readPropToBuf(TIME_STAMP_ID, (uint8_t *)&value);
		IoT_dev.sensor->resetPropChangeFlag(TIME_STAMP_ID);
#if COMM_TRANSMISSION_FORMAT == 1
		swap_reverse((uint8_t*)&value, sizeof(value));
#endif
		iot_set_time_stamp(value);
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_time_stamp_update(value);
	}
	
	/* 处理IoT协议设置的倾角X轴阈值同时更新数据到蓝牙协议中 */
	if(IoT_dev.sensor->isPropChanged(DATA_X_THRESHOLD_ID))
	{
		float value;
		IoT_dev.sensor->readPropToBuf(DATA_X_THRESHOLD_ID, (uint8_t *)&value);
		IoT_dev.sensor->resetPropChangeFlag(DATA_X_THRESHOLD_ID);
#if COMM_TRANSMISSION_FORMAT == 1
		swap_reverse((uint8_t*)&value, sizeof(value));
#endif
		iot_set_x_angle_threshold(value);
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_x_angle_threshold_update(value);
	}
	
	/* 处理IoT协议设置的倾角Y轴阈值同时更新数据到蓝牙协议中 */
	if(IoT_dev.sensor->isPropChanged(DATA_Y_THRESHOLD_ID))
	{
		float value;
		IoT_dev.sensor->readPropToBuf(DATA_Y_THRESHOLD_ID, (uint8_t *)&value);
		IoT_dev.sensor->resetPropChangeFlag(DATA_Y_THRESHOLD_ID);
#if COMM_TRANSMISSION_FORMAT == 1
		swap_reverse((uint8_t*)&value, sizeof(value));
#endif
		iot_set_y_angle_threshold(value);
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_y_angle_threshold_update(value);
	}

	/* 处理设备电池属性数据同时更新数据到蓝牙协议中 */
	if(IoT_dev.gas_gauge_flag == 1)
	{
		IoT_dev.gas_gauge_flag = 0;
		iot_write_battery_level(IoT_dev.gas_gauge);
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_battery_update(IoT_dev.gas_gauge);
	}	
	
	/* 处理设备温度、X轴角度、Y轴角度属性数据同时更新数据到蓝牙协议中 */
	if(IoT_dev.inclinometer_obj->data.update_flag == 1)
	{
		IoT_dev.inclinometer_obj->data.update_flag = 0;
		iot_write_temp(IoT_dev.inclinometer_obj->data.temp);
		iot_write_x_angle(IoT_dev.inclinometer_obj->data.x_angle);
		iot_write_y_angle(IoT_dev.inclinometer_obj->data.y_angle);
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_temperature_update(IoT_dev.inclinometer_obj->data.temp);
		ble_char_update_handle->dev_x_angle_update(IoT_dev.inclinometer_obj->data.x_angle);
		ble_char_update_handle->dev_y_angle_update(IoT_dev.inclinometer_obj->data.y_angle);
	}
}

/* IoT协议初始化 */
iot_dev_t * iot_init(iot_object_t *sensor, inclinometer_obj_t *inclinometer_obj)
{
	sys_param_t* param = sys_param_get_handle();
	
	IoT_dev.gas_gauge_flag = 0;
	IoT_dev.gas_gauge = 100;

	IoT_dev.sensor = sensor;
	IoT_dev.inclinometer_obj = inclinometer_obj;
	
	/* IoT对象属性数据初始化 */
	iot_write_long_addr(param->dev_long_addr);
	iot_write_short_addr(param->dev_short_addr);
	iot_write_sample_mode(param->iot_mode);
	iot_write_sample_interval(param->iot_sample_interval);	
	iot_write_time_stamp(0);
	iot_write_battery_level(IoT_dev.gas_gauge);
	iot_write_temp(25);
	iot_write_x_angle(0);
	iot_write_y_angle(0);
	iot_write_x_angle_threshold(param->iot_x_angle_threshold);
	iot_write_y_angle_threshold(param->iot_y_angle_threshold);	
	
	IoT_dev.operate = iot_operate;
	
	return &IoT_dev;
}



