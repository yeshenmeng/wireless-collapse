#include "iot_operate.h"
#include "calendar.h"
#include "ble_init.h"
#include "host_net_swap.h"
#include "lora_transmission.h"
#include "wireless_comm_services.h"


static iot_dev_t IoT_dev;

/* ��IoT����BUF��д�볤��ַ */
void iot_write_long_addr(uint8_t* value)
{
	IoT_dev.sensor->writePropFromBuf(LONG_ADDR_ID, value);
	IoT_dev.sensor->resetPropChangeFlag(LONG_ADDR_ID);
}

/* ��IoT����BUF��д��̵�ַ */
void iot_write_short_addr(uint8_t* value)
{
	IoT_dev.sensor->writePropFromBuf(SHORT_ADDR_ID, value);
	IoT_dev.sensor->resetPropChangeFlag(SHORT_ADDR_ID);
}

/* ��IoT����BUF��д�����ģʽ */
static void iot_write_sample_mode(uint8_t value)
{
	IoT_dev.sensor->writePropFromBuf(SAMPLE_MODE_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(SAMPLE_MODE_ID);
}

/* ��IoT����BUF��д�������� */
static void iot_write_sample_interval(uint32_t value)
{
	IoT_dev.sensor->writePropFromBuf(SAMPLE_INTERVAL_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(SAMPLE_INTERVAL_ID);
}

/* ��IoT����BUF��д��ʱ��� */
static void iot_write_time_stamp(uint32_t value)
{
	IoT_dev.sensor->writePropFromBuf(TIME_STAMP_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(TIME_STAMP_ID);
}

/* ��IoT����BUF��д����ٶȱ仯��б����ֵ���� */
static void iot_write_accel_slope_threshold(uint16_t value)
{
	IoT_dev.sensor->writePropFromBuf(ACCEL_SLOPE_THRESHOLD_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(ACCEL_SLOPE_THRESHOLD_ID);
}

/* ��IoT����BUF��д���������ݵ����� */
static void iot_write_consecutive_data_points(uint16_t value)
{
	IoT_dev.sensor->writePropFromBuf(CONSECUTIVE_DATA_POINTS_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(CONSECUTIVE_DATA_POINTS_ID);
}

/* ��IoT����BUF��д���ص��� */
static void iot_write_battery_level(uint8_t value)
{
	IoT_dev.sensor->writePropFromBuf(BATTERY_LEVEL_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(BATTERY_LEVEL_ID);
}

/* ��IoT����BUF��д���¶����� */
static void iot_write_temp(float value)
{
	IoT_dev.sensor->writePropFromBuf(TEMPERATURE_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(TEMPERATURE_ID);
}

/* ��IoT����BUF��д��X����ٶ����� */
static void iot_write_x_accel(float value)
{
	IoT_dev.sensor->writePropFromBuf(DATA_X_ACCEL_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(DATA_X_ACCEL_ID);
}

/* ��IoT����BUF��д��Y����ٶ����� */
static void iot_write_y_accel(float value)
{
	IoT_dev.sensor->writePropFromBuf(DATA_Y_ACCEL_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(DATA_Y_ACCEL_ID);
}

/* ��IoT����BUF��д��Z����ٶ����� */
static void iot_write_z_accel(float value)
{
	IoT_dev.sensor->writePropFromBuf(DATA_Z_ACCEL_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(DATA_Z_ACCEL_ID);
}

#if (IOT_PROTOCOL_WITH_ANGLE == 1)
/* ��IoT����BUF��д��X��Ƕ����� */
static void iot_write_x_angle(float value)
{
	IoT_dev.sensor->writePropFromBuf(DATA_X_ANGLE_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(DATA_X_ANGLE_ID);
}

/* ��IoT����BUF��д��Y��Ƕ����� */
static void iot_write_y_angle(float value)
{
	IoT_dev.sensor->writePropFromBuf(DATA_Y_ANGLE_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(DATA_Y_ANGLE_ID);
}

/* ��IoT����BUF��д��Z��Ƕ����� */
static void iot_write_z_angle(float value)
{
	IoT_dev.sensor->writePropFromBuf(DATA_Z_ANGLE_ID, (uint8_t *)&value);
	IoT_dev.sensor->resetPropChangeFlag(DATA_Z_ANGLE_ID);
}
#endif

/* ����IoTЭ�����õĳ���ַ */
static void iot_set_long_addr(uint8_t* value)
{
	sys_param_t* param = sys_param_get_handle();
	for(int i=0; i<sizeof(value); i++)
	{
		if(value[i] != param->dev_long_addr[i])
		{
			lora_reset();
			break;
		}
	}
	sys_param_set((uint8_t*)&param->dev_long_addr, value, sizeof(param->dev_long_addr));
}

/* ����IoTЭ�����õĶ̵�ַ */
static void iot_set_short_addr(uint8_t* value)
{
	sys_param_t* param = sys_param_get_handle();
	wireless_comm_services_t* wirelessCommSvc = Wireless_CommSvcGetHandle();
	if(wirelessCommSvc->_conn_short_addr_update_flag == 0) //����ʱ����Э���еĶ̵�ֱַ�����ò���Ҫ�뱾�ض̵�ַ�Ƚ�
	{
		for(int i=0; i<sizeof(value); i++)
		{
			if(value[i] != param->dev_short_addr[i]) //IOTЭ�������еĶ̵�ַ��Ҫ�Ƚ�
			{
				lora_reset();
				break;
			}
		}
	}
	wirelessCommSvc->_conn_short_addr_update_flag= 0;
	sys_param_set((uint8_t*)&param->dev_short_addr, value, sizeof(param->dev_short_addr));
}

/* ����IoTЭ�����õĲ���ģʽ */
static void iot_set_sample_mode(uint8_t value)
{
	sys_param_t* param = sys_param_get_handle();
	if(value != param->iot_mode)
	{
		sys_param_set((uint8_t*)&param->iot_mode, (uint8_t*)&value, sizeof(value));
		IoT_dev.collapse_obj->iot_set_mode();
	}
}

/* ����IoTЭ�����õĲ������ */
static void iot_set_sample_interval(uint32_t value)
{
	if(value != 0)
	{
		sys_param_t* param = sys_param_get_handle();
		sys_param_set((uint8_t*)&param->iot_period, (uint8_t*)&value, sizeof(value));
		if(param->iot_mode == PERIOD_MODE)
		{
			sys_param_set((uint8_t*)&param->iot_sample_period, (uint8_t*)&value, sizeof(value));
		}
		else if(param->iot_mode == TRIGGER_MODE)
		{
			sys_param_set((uint8_t*)&param->iot_trigger_period, (uint8_t*)&value, sizeof(value));
		}
		IoT_dev.collapse_obj->iot_set_period();
	}
}

/* ����IoTЭ�����õ�ʱ��� */
static void iot_set_time_stamp(uint32_t value)
{
	calendar_mod_t* calendar = calendar_get_handle();
	calendar->set_time_stamp(value);
}

/* ����IoTЭ�����õļ��ٶȱ仯��б����ֵ */
static void iot_set_accel_slope_threshold(uint16_t value)
{
	(value > 1000) ? (value = 1000) : (1);
	sys_param_t* param = sys_param_get_handle();
	sys_param_set((uint8_t*)&param->iot_accel_slope_threshold, (uint8_t*)&value, sizeof(value));
	IoT_dev.collapse_obj->iot_set_accel_slope_threshold();
}

/* ����IoTЭ�����õ��������ݵ� */
static void iot_set_consecutive_data_points(uint16_t value)
{
	(value > 65200) ? (value = 65200) : (1);
	sys_param_t* param = sys_param_get_handle();
	sys_param_set((uint8_t*)&param->iot_consecutive_data_points, (uint8_t*)&value, sizeof(value));
	IoT_dev.collapse_obj->iot_set_consecutive_data_points();
}

/* IoTЭ�鴦����IoT�������ݴ��� */
void iot_operate(void)
{
	/* ����IoTЭ�����õĳ���ַͬʱ�������ݵ�����Э���� */
	if(IoT_dev.sensor->isPropChanged(LONG_ADDR_ID))
	{
		uint8_t value[8];
		IoT_dev.sensor->readPropToBuf(LONG_ADDR_ID, (uint8_t *)&value);
		IoT_dev.sensor->resetPropChangeFlag(LONG_ADDR_ID);
		iot_set_long_addr(value);
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_long_addr_update(value);
	}
	
	/* ����IoTЭ�����õĶ̵�ַͬʱ�������ݵ�����Э���� */
	if(IoT_dev.sensor->isPropChanged(SHORT_ADDR_ID))
	{
		uint8_t value[2];
		IoT_dev.sensor->readPropToBuf(SHORT_ADDR_ID, (uint8_t *)&value);
		IoT_dev.sensor->resetPropChangeFlag(SHORT_ADDR_ID);
		iot_set_short_addr(value);
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_short_addr_update(value);
	}
	
	/* ����IoTЭ�����õ�ʱ���ͬʱ�������ݵ�����Э���� */
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
	
	/* ����IoTЭ�����õļ��ٶȱ仯��б����ֵͬʱ�������ݵ�����Э���� */
	if(IoT_dev.sensor->isPropChanged(ACCEL_SLOPE_THRESHOLD_ID))
	{
		uint16_t value;
		IoT_dev.sensor->readPropToBuf(ACCEL_SLOPE_THRESHOLD_ID, (uint8_t *)&value);
		IoT_dev.sensor->resetPropChangeFlag(ACCEL_SLOPE_THRESHOLD_ID);
#if COMM_TRANSMISSION_FORMAT == 1
		swap_reverse((uint8_t*)&value, sizeof(value));
#endif
		iot_set_accel_slope_threshold(value);
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_accel_slope_threshold_update(value);
	}
	
	/* ����IoTЭ�����õ��������ݵ�ͬʱ�������ݵ�����Э���� */
	if(IoT_dev.sensor->isPropChanged(CONSECUTIVE_DATA_POINTS_ID))
	{
		uint16_t value;
		IoT_dev.sensor->readPropToBuf(CONSECUTIVE_DATA_POINTS_ID, (uint8_t *)&value);
		IoT_dev.sensor->resetPropChangeFlag(CONSECUTIVE_DATA_POINTS_ID);
#if COMM_TRANSMISSION_FORMAT == 1
		swap_reverse((uint8_t*)&value, sizeof(value));
#endif
		iot_set_consecutive_data_points(value);
		IoT_dev.collapse_obj->iot_set_consecutive_data_points();
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_consecutive_data_points_update(value);
	}
	
	/* ����IoTЭ�����õĲ���ģʽͬʱ�������ݵ�����Э���� */
	if(IoT_dev.sensor->isPropChanged(SAMPLE_MODE_ID))
	{
		uint8_t value;
		IoT_dev.sensor->readPropToBuf(SAMPLE_MODE_ID, (uint8_t *)&value);
		IoT_dev.sensor->resetPropChangeFlag(SAMPLE_MODE_ID);
		iot_set_sample_mode(value);
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_sample_mode_update(value);
	}
	
	/* ����IoTЭ�����õĲ������ͬʱ�������ݵ�����Э���� */
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

	/* �����豸�����������ͬʱ�������ݵ�����Э���� */
	if(IoT_dev.gas_gauge_flag == 1)
	{
		IoT_dev.gas_gauge_flag = 0;
		iot_write_battery_level(IoT_dev.gas_gauge);
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_battery_update(IoT_dev.gas_gauge);
	}	
	
	/* �����豸�¶ȡ�������������ͬʱ�������ݵ�����Э���� */
	if(IoT_dev.collapse_obj->update_flag == 1)
	{
		IoT_dev.collapse_obj->update_flag = 0;
		iot_write_temp(IoT_dev.collapse_obj->data.temp_c);
		iot_write_x_accel(IoT_dev.collapse_obj->data.accel.x_accel);
		iot_write_y_accel(IoT_dev.collapse_obj->data.accel.y_accel);
		iot_write_z_accel(IoT_dev.collapse_obj->data.accel.z_accel);

#if (IOT_PROTOCOL_WITH_ANGLE == 1)
		iot_write_x_angle(IoT_dev.collapse_obj->data.angle.x_angle);
		iot_write_y_angle(IoT_dev.collapse_obj->data.angle.y_angle);
		iot_write_z_angle(IoT_dev.collapse_obj->data.angle.z_angle);
#endif
		
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_temperature_update(IoT_dev.collapse_obj->data.temp_c);
		ble_char_update_handle->dev_x_accel_update(IoT_dev.collapse_obj->data.accel.x_accel);
		ble_char_update_handle->dev_y_accel_update(IoT_dev.collapse_obj->data.accel.y_accel);
		ble_char_update_handle->dev_z_accel_update(IoT_dev.collapse_obj->data.accel.z_accel);
		ble_char_update_handle->dev_x_angle_update(IoT_dev.collapse_obj->data.angle.x_angle);
		ble_char_update_handle->dev_y_angle_update(IoT_dev.collapse_obj->data.angle.y_angle);
		ble_char_update_handle->dev_z_angle_update(IoT_dev.collapse_obj->data.angle.z_angle);
	}
	
	static int8_t lora_rssi = -127;
	if(lora_rssi != IoT_dev.lora_obj->get_rssi())
	{
		lora_rssi = IoT_dev.lora_obj->get_rssi();
		ble_char_update_t* ble_char_update_handle = ble_char_update_handle_get();
		ble_char_update_handle->dev_lora_rssi_update(lora_rssi);
	}
	
//	LORA�ź�ǿ��
}

/* IoTЭ���ʼ�� */
iot_dev_t * iot_init(iot_object_t *sensor, collapse_obj_t *collapse_obj, lora_obj_t* lora_obj)
{
	sys_param_t* param = sys_param_get_handle();
	
	IoT_dev.gas_gauge_flag = 0;
	IoT_dev.gas_gauge = 100;

	IoT_dev.sensor = sensor;
	IoT_dev.collapse_obj = collapse_obj;
	IoT_dev.lora_obj = lora_obj;
	
	/* IoT�����������ݳ�ʼ�� */
	iot_write_long_addr(param->dev_long_addr);
	iot_write_short_addr(param->dev_short_addr);
	iot_write_sample_mode(param->iot_mode);
	iot_write_sample_interval(param->iot_period);	
	iot_write_time_stamp(0);
	iot_write_accel_slope_threshold(param->iot_accel_slope_threshold);
	iot_write_consecutive_data_points(param->iot_consecutive_data_points);
	iot_write_battery_level(IoT_dev.gas_gauge);
	iot_write_temp(25);
	iot_write_x_accel(0);
	iot_write_y_accel(0);
	iot_write_z_accel(0);

#if (IOT_PROTOCOL_WITH_ANGLE == 1)
	iot_write_x_angle(0);
	iot_write_y_angle(0);
	iot_write_z_angle(0);
#endif
	
	IoT_dev.operate = iot_operate;
	
	return &IoT_dev;
}



