/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_sdh.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_init.h"
#include "low_power_manage.h"
#include "sw_timer_rtc.h"
#include "bluetooth_low_power.h"
#include "rng_lpm.h"
#include "lora_transmission.h"
#include "sys_proc.h"
#include "inclinometer.h"
#include "iotobject.h"
#include "wireless_comm_services.h"
#include "signal_detect.h"
#include "sw_signal_detect.h"
#include "flash.h"
#include "iot_operate.h"
#include "sys_param.h"
#include "calendar.h"
#include "uart_svc.h"
#include "light.h"
#include "ble_char_handler.h"
#include "sw_bat_soc.h"
#include "collapse.h"

#define DEAD_BEEF	0xDEADBEEF	/**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

lpm_obj_t* m_lpm;
ble_obj_t *m_ble;
lora_obj_t* m_lora;
slp_obj_t* m_slp;
inclinometer_obj_t* m_inclinometer;
collapse_obj_t* m_collapse;
sw_signal_detect_obj_t*  m_sw_signal_detect;
wireless_comm_services_t* wireless_comm_svc;
uint32_t sys_run_cnt = 0;

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/* ��ʼ�������ʱ�� */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/* rtcʱ������ */
static void lfclk_cfg(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

/**
 * @brief  ��ʼ��������Э�����Գ���
 * @param  sensor: ָ��iot_object_t�ṹ�ľ��
 * @retval None
 */
void iot_set_prop(iot_object_t *sensor) {
	//init addr, long addr, short addr are fixed. No need to set
	sensor->setPropCount(12);
	sensor->setPropLen(SAMPLE_MODE_ID, 1);
	sensor->setPropLen(SAMPLE_INTERVAL_ID, 4);
	sensor->setPropLen(TIME_STAMP_ID, 4);
	sensor->setPropLen(BATTERY_LEVEL_ID, 1);
	sensor->setPropLen(TEMPERATURE_ID, 4);
	sensor->setPropLen(DATA_X_ANGLE_ID, 4);
	sensor->setPropLen(DATA_Y_ANGLE_ID, 4);
	sensor->setPropLen(DATA_X_THRESHOLD_ID, 4);
	sensor->setPropLen(DATA_Y_THRESHOLD_ID, 4);
}

/* ����͹���ʱ�Ĵ��� */
void lpm_enter_handler(void)
{
	if(nrf_sdh_is_enabled() == true)
	{
		sd_power_dcdc_mode_set(0);
	}
	else
	{
		NRF_POWER->DCDCEN = 0; //�͹��Ĺر�DCDC
	}

	if(m_ble->state == BLE_STA_IDLE)
	{
		LIGHT_OFF();
	}
}

/* �˳��͹���ʱ�Ĵ��� */
void lpm_exit_handler(void)
{
	if(nrf_sdh_is_enabled() == true)
	{
		sd_power_dcdc_mode_set(1);
	}
	else
	{
		NRF_POWER->DCDCEN = 1; //�˳��͹��Ĵ�DCDC
	}
}

/**@brief Function for application main entry.
 */
int main(void)
{
	/**********************************Ӳ���豸��ʼ��**********************************/
	NRF_POWER->DCDCEN = 1; 									//��DCDC
	lfclk_cfg(); 											//RTCʱ��Դ����
	light_init(); 											//�豸ָʾ�Ƴ�ʼ��
	timers_init(); 											//��ʱ����ʼ������RTC1
	fs_flash_init();						 				//flash��ʼ��
	
	/**********************************����ģ���ʼ��**********************************/
	sys_param_init(); 										//ϵͳ������ʼ��
	iot_object_t *sensor = createSensorHandler(); 			//�����������ʼ��
	iot_set_prop(sensor);
	sensor->init();
	wireless_comm_svc = createWirelessCommServiceHandler(); //����ͨ�ŷ����ʼ��
	wireless_comm_svc->setSensorHandler(sensor);
#if (SIGNAL_DET_HW == 1)
	hw_signal_detect_init(); 								//Ӳ���źż�⹦�ܳ�ʼ��
#endif
	swt_init(); 											//�����ʱ����ʼ��
	calendar_init(); 										//�������ܳ�ʼ����ʼ��
	rng_lpm_init(); 										//�͹����������������ʼ��
#if (BAT_SOC_DET_SW == 1)
	sw_bat_soc_init();										//�����ص�������ʼ��
#endif

	/**********************************BLE��ʼ��**********************************/
	ble_softdev_init();

	/**********************************Ӧ�ó�ʼ��**********************************/
	m_lpm = lpm_init(); 									//�͹��Ĺ����ʼ��
	m_ble = ble_init(m_lpm); 								//BLE�����ʼ��	
#if (SIGNAL_DET_SW == 1)
	m_sw_signal_detect = sw_signal_detect_task_init(m_lpm); //����źż�������ʼ��
#endif
	m_lora = lora_task_init(m_lpm); 						//LORA�����ʼ��
	m_slp = slp_task_init(m_lpm); 							//ϵͳ�͹��������ʼ��
	m_collapse = collapse_init(m_lpm);						//�����������ʼ��
//	iot_init(sensor, m_inclinometer); 						//������Э�����Գ�ʼ��
	
	/**********************************ϵͳ��������**********************************/
	sys_task_t task;
	task.ble_task = m_ble;
	task.slp_task = m_slp;
	task.lora_task = m_lora;
	task.signal_detect_task = m_sw_signal_detect;
	task.collapse_task = m_collapse;
	sys_task_init(&task); 									//ϵͳ�����ʼ��
	sys_startup(); 											//ϵͳ����
	LIGHT_OFF();
	
	nrf_delay_ms(300);
	LIGHT_OFF();
	nrf_delay_ms(300);
//	uart_init();

	while(1)
	{
		/* ϵͳ������� */
		sys_task_schd();
		
		/* �͹��Ĺ����������� */
		m_lpm->task_operate(lpm_enter_handler, lpm_exit_handler);
		
		/* BLE�������� */
		m_ble->task_operate();
		
		/* LORA�������� */
		m_lora->task_operate();
		
		/* ϵͳ�͹����������� */
		m_slp->task_operate();
		
		/* �������������� */
		m_collapse->task_operate();
		
		/* ����źż���������� */
#if (SIGNAL_DET_SW == 1)
		m_sw_signal_detect->task_operate();
#endif
		
		/* ������Э�鴦�� */
//		iot_operate();
		
		/* ϵͳ���ݴ洢 */
		sys_save_param_to_flash();
		
		/* ����������ݴ洢 */
#if (BAT_SOC_DET_SW == 1)
		sw_bat_soc_param_to_flash();
#endif
		
		/* ��������Э�鴦�� */
		ble_char_req_handler();

		sys_run_cnt++;		
//		uart_run();
	}
}


/**
 * @}
 */
