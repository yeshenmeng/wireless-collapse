#include "lora_transmission.h"
#include "sys_param.h"
#include "wireless_comm_services.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"
#include "sw_timer_rtc.h"
#include "rng_lpm.h"
#include "light.h"
#include "string.h"
#include "math.h"
#include "sx1262.h"
#include "host_net_swap.h"
#include "sw_bat_soc.h"
#include "stdlib.h"


typedef enum {
	LORA_OFFLINE,
	LORA_CONNECT,
}lora_conn_status_t;

static uint8_t lora_mtu = 200;
static uint8_t lora_tx_count = 0;
static uint8_t lora_tx_buf[255] = {0};
static lora_state_t lora_state;
static lora_obj_t lora_obj;
static lora_conn_status_t lora_conn_status = LORA_OFFLINE;
static lora_conn_status_t lora_pre_conn_status = LORA_OFFLINE;

/* 等待网关回复最大数据长度
   4：协议报头长度
   1：协议数据段长度
   2：协议测点短地址长度
   1：协议属性个数长度
   7：协议最大属性MASK
   8：协议属性值长地址
   2：协议属性值短地址
   1：协议属性值模式
   4：协议属性值采样间隔
   4：协议属性值时间戳
   4：协议属性值X轴阈值
   4：协议属性值Y轴阈值
   2：协议CRC校验
 */
static uint8_t lora_max_wait_reply_len = 4 + 1 + 2 + 1 + 7 + 8 + 2 + 1 + 4 + 4 + 4 + 4 + 2;

/* SPI驱动程序实例ID,ID和外设编号对应，0:SPI0  1:SPI1 2:SPI2 */
#define SPI_INSTANCE  0 
/* 定义名称为spi的SPI驱动程序实例 */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE); 
/* SPI传输完成标志 */
static volatile bool spi_xfer_done; 
static radio_drv_funcs_t wireless_drv;

/* LORA任务停止处理 */
__weak void lora_task_stop_handler(void* param){};
void lora_tx_cplt_cb(uint8_t* pData, uint16_t size);

static void gpiote_in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	if(pin == LORA_IRQ_PIN)
	{
		if(action == NRF_GPIOTE_POLARITY_LOTOHI)
		{
			radio_state_t rf_st;
			wireless_drv.radio_GetStatus(&rf_st);
			if(rf_st == RX_RUNING_ST)
			{
				lora_tx_cplt_cb(NULL, NULL); //接收LORA数据
			}
		}
	}
}

static void lora_ext_int_cfg(void)
{
	nrfx_gpiote_in_config_t nrfx_gpiote_in_config;
	
	if (!nrf_drv_gpiote_is_init())
	{
			uint32_t err_code;
			err_code = nrf_drv_gpiote_init();
			APP_ERROR_CHECK(err_code);
	}
	
	nrfx_gpiote_in_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
	nrfx_gpiote_in_config.is_watcher = false;
	nrfx_gpiote_in_config.pull = NRF_GPIO_PIN_NOPULL;
	nrfx_gpiote_in_config.hi_accuracy = true;
	nrf_drv_gpiote_in_init(LORA_IRQ_PIN, &nrfx_gpiote_in_config, &gpiote_in_pin_handler);
	nrf_drv_gpiote_in_event_enable(LORA_IRQ_PIN, true);
}

/* SPI事件处理函数 */
static void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
	spi_xfer_done = true; //设置SPI传输完成  
}

static void lora_spi_cfg(void)
{
	nrf_gpio_cfg_output(LORA_SPI_CS_PIN);
	LORA_SPI_CS_DISABLE();
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.ss_pin   = NRF_DRV_SPI_PIN_NOT_USED;
	spi_config.miso_pin = LORA_SPI_MISO_PIN;
	spi_config.mosi_pin = LORA_SPI_MOSI_PIN;
	spi_config.sck_pin  = LORA_SPI_SCK_PIN;
	APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}

static void lora_spi_cfg_default(void)
{
	nrf_drv_spi_uninit(&spi);
	nrf_gpio_cfg_default(LORA_SPI_MISO_PIN);
	nrf_gpio_cfg_default(LORA_SPI_MOSI_PIN);
	nrf_gpio_cfg_default(LORA_SPI_SCK_PIN);
	nrf_gpio_cfg_default(LORA_SPI_CS_PIN);
}

void lora_spi_transmit_receive(uint8_t* tx_buffer, uint8_t tx_length, uint8_t* rx_buffer, uint8_t rx_length)
{
	spi_xfer_done = false;
	LORA_SPI_CS_ENABLE();
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_buffer, tx_length, rx_buffer, rx_length));
	while(!spi_xfer_done);
}

static void lora_radio_init(void)
{
	wireless_drv = radio_sx1262_lora_init();
	wireless_drv.radio_reset();
	wireless_drv.radio_init();
}

static void lora_cfg(void)
{
	nrf_gpio_cfg(LORA_POWER_PIN,
				 NRF_GPIO_PIN_DIR_OUTPUT,
				 NRF_GPIO_PIN_INPUT_DISCONNECT,
				 NRF_GPIO_PIN_NOPULL,
				 NRF_GPIO_PIN_S0S1,
				 NRF_GPIO_PIN_NOSENSE);

	nrf_gpio_cfg(LORA_TRANSMIT_PIN,
				 NRF_GPIO_PIN_DIR_OUTPUT,
				 NRF_GPIO_PIN_INPUT_DISCONNECT,
				 NRF_GPIO_PIN_NOPULL,
				 NRF_GPIO_PIN_S0S1,
				 NRF_GPIO_PIN_NOSENSE);
	
	nrf_gpio_cfg(LORA_RECEIVE_PIN,
				 NRF_GPIO_PIN_DIR_OUTPUT,
				 NRF_GPIO_PIN_INPUT_DISCONNECT,
				 NRF_GPIO_PIN_NOPULL,
				 NRF_GPIO_PIN_S0S1,
				 NRF_GPIO_PIN_NOSENSE);
	
	nrf_gpio_cfg(LORA_RESET_PIN,
				 NRF_GPIO_PIN_DIR_OUTPUT,
				 NRF_GPIO_PIN_INPUT_DISCONNECT,
				 NRF_GPIO_PIN_NOPULL,
				 NRF_GPIO_PIN_S0S1,
				 NRF_GPIO_PIN_NOSENSE);

	nrf_gpio_cfg(LORA_BUSY_PIN,
				 NRF_GPIO_PIN_DIR_INPUT,
				 NRF_GPIO_PIN_INPUT_CONNECT,
				 NRF_GPIO_PIN_NOPULL,
				 NRF_GPIO_PIN_S0S1,
				 NRF_GPIO_PIN_NOSENSE);
							
	LORA_POWER_ENABLE();
	LORA_TRANSMIT_ENABLE();
	LORA_RECEIVE_DISABLE();
	LORA_RESET_DISABLE();
	lora_ext_int_cfg();
	lora_spi_cfg();
	lora_radio_init();
	nrf_delay_us(100);
}
	
static void lora_cfg_default(void)
{
	lora_spi_cfg_default();
	nrf_gpio_cfg_default(LORA_POWER_PIN);
	nrf_gpio_cfg_default(LORA_TRANSMIT_PIN);
	nrf_gpio_cfg_default(LORA_RECEIVE_PIN);
	nrf_gpio_cfg_default(LORA_RESET_PIN);
	nrf_gpio_cfg_default(LORA_BUSY_PIN);
	nrf_gpio_cfg_default(LORA_IRQ_PIN);
}	

static void lora_task_start(void)
{
	if(lora_obj.state == LORA_IDLE)
	{
		lora_obj.lpm_obj->task_set_stat(LORA_TASK_ID, LPM_TASK_STA_RUN);
		lora_state = LORA_ACTIVE;
		lora_pre_conn_status = lora_conn_status;
	}
}

static void lora_task_stop(void)
{
	if(lora_obj.state == LORA_ACTIVE)
	{
		lora_obj.param.tx_fail_times = 0;
		lora_state = LORA_STOP;
	}
}

static void lora_set_tx_data(uint8_t* p_data, uint8_t size)
{
	for(int i=0; i<sizeof(lora_tx_buf); i++)
	{
		lora_tx_buf[i] = *(p_data+i);
	}
}

static void lora_idle_lpm(void)
{
	if(lora_obj.param.is_idle_enter_lp == 0)
	{
		return;
	}
	
	lora_obj.lpm_obj->task_set_stat(LORA_TASK_ID, LPM_TASK_STA_LP);  /* 设置任务低功耗状态 */
}

void swt_lora_task_time_slice_cb(void)
{
	if(lora_state!=LORA_TIMEOUT && LORA_TIMEOUT!=LORA_STOP)
	{
		lora_state = LORA_TIMEOUT;
	}
}

void swt_lora_tx_timeout_cb(void)
{
	if(lora_state!=LORA_TIMEOUT && LORA_TIMEOUT!=LORA_STOP)
	{
#if (BAT_SOC_DET_SW == 1)
		sw_bat_soc_mod_t* sw_bat_soc_mod = sw_bat_soc_get_handle();
		sw_bat_soc_mod->lora_tx_fail_power_add();
#endif
		lora_state = LORA_TX_FAIL;
	}
}

void swt_lora_idle_cb(void)
{
	if(lora_state!=LORA_TIMEOUT && LORA_TIMEOUT!=LORA_STOP)
	{
		lora_state = LORA_ACTIVE;
	}
}

void lora_tx_cplt_cb(uint8_t* pData, uint16_t size)
{
	if(lora_obj.state == LORA_ACTIVE)
	{
		if(lora_state!=LORA_TIMEOUT && LORA_TIMEOUT!=LORA_STOP)
		{
			lora_state = LORA_TX_SUCCESS;
		}
	}
}

static uint32_t lora_random_delay(void)
{
	static uint8_t max_flag = 0;
	static uint32_t delayTime = 0;
	rng_lpm_mod_t* rng_lpm_mod = rng_lpm_get_handle();
	uint32_t randomNum;
	
	if(lora_obj.param.tx_fail_times == 1)
	{
		max_flag = 0;
	}
	
	rng_lpm_mod->ramdom_upper = lora_obj.param.random_delay_upper;
	rng_lpm_mod->ramdom_lower = lora_obj.param.random_delay_lower;
	rng_lpm_mod->generate_random_number(&randomNum, 1);
	
	if(max_flag == 0)
	{
		delayTime = pow(2, lora_obj.param.tx_fail_times-1) * lora_obj.param.delay_base_time + randomNum;
	}
	
	if(max_flag == 0 && lora_obj.param.tx_max_delay_time < delayTime)
	{
		max_flag = 1;
	}
	
	if(max_flag == 1)
	{
		delayTime = lora_obj.param.tx_max_delay_time + randomNum;
	}
	
	return delayTime;
}

static void lora_load_conn_cmd_buf(void)
{
	lora_tx_count = 0;
	
	/* 命令头 */
	uint32_t cmdHeader = CMD_CONNECT;
#if (COMM_TRANSMISSION_FORMAT == 1)
	cmdHeader = swap_htonl(cmdHeader);
#endif
	memcpy(&lora_tx_buf[lora_tx_count], (uint8_t*)&cmdHeader, sizeof(cmdHeader));
	lora_tx_count += sizeof(cmdHeader);
	
	/* 数据段长度 */
	lora_tx_buf[lora_tx_count] = 8;
	lora_tx_count += 1;
	
	/* 数据段测点长地址 */
	wireless_comm_services_t* wirelessCommSvc = Wireless_CommSvcGetHandle();
	wirelessCommSvc->_sensor->readPropToBuf(1, &lora_tx_buf[lora_tx_count]);
	lora_tx_count += 8;
	
	/* CRC16 */
	uint16_t crc16 = wirelessCommSvc->modbusRtuCRC(lora_tx_buf, lora_tx_count);
#if (CRC_TRANSMISSION_FORMAT == 1)
	crc16 = swap_htons(crc16);
#endif
	memcpy(&lora_tx_buf[lora_tx_count], (uint8_t*)&crc16, sizeof(crc16));
	lora_tx_count += sizeof(crc16);
}

static void lora_load_publish_cmd_buf(void)
{
	lora_tx_count = 0;
	
	/* 命令头 */
	uint32_t cmdHeader = CMD_PUBLISH;
#if (COMM_TRANSMISSION_FORMAT == 1)
	cmdHeader = swap_htonl(cmdHeader);
#endif
	memcpy(&lora_tx_buf[lora_tx_count], (uint8_t*)&cmdHeader, sizeof(cmdHeader));
	lora_tx_count += sizeof(cmdHeader);
	
	/* 数据段长度 */
	lora_tx_buf[lora_tx_count] = 20;
	lora_tx_count += 1;
	
	/* 数据段测点短地址 */
	wireless_comm_services_t* wirelessCommSvc = Wireless_CommSvcGetHandle();
	wirelessCommSvc->_sensor->readPropToBuf(2, &lora_tx_buf[lora_tx_count]);
	lora_tx_count += 2;	
	
	/* 数据段属性个数 */
	lora_tx_buf[lora_tx_count] = 4;
	lora_tx_count += 1;
	
	/* 数据段属性ID */
	lora_tx_buf[lora_tx_count] = 6;
	lora_tx_count += 1;
	lora_tx_buf[lora_tx_count] = 7;
	lora_tx_count += 1;
	lora_tx_buf[lora_tx_count] = 8;
	lora_tx_count += 1;
	lora_tx_buf[lora_tx_count] = 9;
	lora_tx_count += 1;
	
	/* 数据段属性数据 */
	wirelessCommSvc->_sensor->readPropToBuf(6, &lora_tx_buf[lora_tx_count]);
	lora_tx_count += 1;
	wirelessCommSvc->_sensor->readPropToBuf(7, &lora_tx_buf[lora_tx_count]);
#if (COMM_TRANSMISSION_FORMAT == 1)
	swap_reverse((uint8_t*)&lora_tx_buf[lora_tx_count], 4);
#endif
	lora_tx_count += 4;
	wirelessCommSvc->_sensor->readPropToBuf(8, &lora_tx_buf[lora_tx_count]);
#if (COMM_TRANSMISSION_FORMAT == 1)
	swap_reverse((uint8_t*)&lora_tx_buf[lora_tx_count], 4);
#endif
	lora_tx_count += 4;
	wirelessCommSvc->_sensor->readPropToBuf(9, &lora_tx_buf[lora_tx_count]);
#if (COMM_TRANSMISSION_FORMAT == 1)
	swap_reverse((uint8_t*)&lora_tx_buf[lora_tx_count], 4);
#endif
	lora_tx_count += 4;

	/* CRC16 */
	uint16_t crc16 = wirelessCommSvc->modbusRtuCRC(lora_tx_buf, lora_tx_count);
#if (CRC_TRANSMISSION_FORMAT == 1)
	crc16 = swap_htons(crc16);
#endif
	memcpy(&lora_tx_buf[lora_tx_count], (uint8_t*)&crc16, sizeof(crc16));
	lora_tx_count += sizeof(crc16);
}

static FlagStatus lora_cad_detect(void)
{
	uint8_t p_state;
	wireless_drv.radio_cadmode(&p_state);
	if(p_state == SX1262_CAD_NONE)
	{
		p_state = SET;
	}
	else
	{
		p_state = RESET;
	}
	
	return (FlagStatus)p_state;
}

static void lora_task_operate(void)
{
	uint8_t LoraOutState;
	swt_mod_t* timer = swt_get_handle();
	
	switch((uint8_t)lora_state)
	{
		case LORA_ACTIVE:
		{
			lora_state = LORA_IDLE;
			lora_obj.lpm_obj->task_set_stat(LORA_TASK_ID, LPM_TASK_STA_RUN);
			
			if(lora_obj.state == LORA_IDLE)
			{
				if(lora_conn_status == LORA_OFFLINE)
				{
					lora_load_conn_cmd_buf();
				}
				else if(lora_conn_status == LORA_CONNECT)
				{
					lora_load_publish_cmd_buf();
				}
				
				lora_obj.state = LORA_ACTIVE;
				timer->lora_task_time_slice->start(lora_obj.param.task_time_slice); /* 启动LORA时间片定时器 */
			}
				
			lora_cfg();
			if(lora_cad_detect() == SET)
			{
				LIGHT_1_ON();
				if(wireless_drv.radio_TXData(lora_tx_buf, lora_tx_count) == LORA_RET_CODE_ERR)
				{
					lora_state = LORA_TX_FAIL;
				}
				else
				{
					uint32_t rx_air_time = wireless_drv.radio_time_on_air_get(lora_max_wait_reply_len);
					timer->lora_tx_timeout->start(rx_air_time+lora_obj.param.rx_timeout_base); /* 启动LORA数据接收超时定时器 */
				}
				LORA_TRANSMIT_DISABLE();
				LORA_RECEIVE_ENABLE();
				wireless_drv.radio_Rxmode();
				LIGHT_1_OFF();
			}
			else
			{
				lora_state = LORA_TX_FAIL;
			}
			break;
		}

		case LORA_TX_SUCCESS:
		{
			lora_state = LORA_IDLE;
			timer->lora_tx_timeout->stop();
		
			/* 接收LORA数据 */
			uint8_t rx_size = 0;
			uint8_t *rx_data = (uint8_t*)malloc(255);
			wireless_drv.radio_dio1_irq_func(rx_data, &rx_size);
			rx_size = 4 + 1 + *(rx_data + 4) + 2; //根据接收的数据长度获取总的数据长度
			wireless_comm_services_t* wirelessCommSvc = Wireless_CommSvcGetHandle();
			wirelessCommSvc->wirelessRxCpltCallBack(rx_data, rx_size); //解析数据
			free((void*)rx_data);
			rx_data = NULL;
			uint8_t ret = wirelessCommSvc->parseMasterMsg();
			lora_cfg_default();
		
			/* 数据解析成功 */
			if(ret == 1)
			{
				lora_obj.param.tx_fail_times = 0;
				if(lora_conn_status == LORA_OFFLINE)
				{
					timer->lora_task_time_slice->stop();
					lora_conn_status = LORA_CONNECT; //连接LORA网关成功
					LoraOutState = LORA_OUT_STATE_LINK;
					lora_obj.state = LORA_IDLE;
					lora_obj.lpm_obj->task_set_stat(LORA_TASK_ID, LPM_TASK_STA_STOP);
					lora_task_stop_handler((void *)&LoraOutState); //LORA任务停止处理
				}
				else
				{
					lora_state = LORA_TIMEOUT; //数据推送成功
				}
			}
			/* 数据解析失败 */
			else
			{
				lora_state = LORA_TX_FAIL; 
			}
			
#if (BAT_SOC_DET_SW == 1)
			sw_bat_soc_mod_t* sw_bat_soc_mod = sw_bat_soc_get_handle();
			sw_bat_soc_mod->lora_tx_success_power_add();
#endif
			break;
		}
			
		case LORA_TX_FAIL:
			lora_state = LORA_IDLE;
			lora_obj.param.tx_fail_times++;
			lora_cfg_default();
		
			if(lora_obj.param.tx_fail_times >= lora_obj.param.tx_max_fail_times)
			{
				lora_state = LORA_TIMEOUT; /* 超过发送失败次数 */
			}
			else
			{
				timer->lora_idle->start(lora_random_delay()); /* 计算随机延时时间,启动LORA空闲定时器 */
				lora_idle_lpm(); /* 进入LORA低功耗,等待唤醒 */
			}
			break;
		
		/* 时间片定时器超时事件 */
		case LORA_TIMEOUT:
			if(lora_pre_conn_status == LORA_OFFLINE)
			{
				LoraOutState = LORA_OUT_STATE_OFFLINE;
			}
			else if(lora_pre_conn_status==LORA_CONNECT && lora_obj.param.tx_fail_times!=0)
			{
				lora_conn_status = LORA_OFFLINE;
				LoraOutState = LORA_OUT_STATE_DISCON;
			}
			else if(lora_pre_conn_status==LORA_CONNECT && lora_obj.param.tx_fail_times==0)
			{
				LoraOutState = LORA_OUT_STATE_CONNECT;
			}
		/* 主动停止LORA任务 */
		case LORA_STOP:
			lora_state = LORA_IDLE;
			lora_obj.param.tx_fail_times = 0;
			timer->lora_task_time_slice->stop(); /* 停止所有LORA定时器 */
			timer->lora_tx_timeout->stop();
			timer->lora_idle->stop();
			lora_cfg_default();
			lora_obj.state = LORA_IDLE;
			lora_obj.lpm_obj->task_set_stat(LORA_TASK_ID, LPM_TASK_STA_STOP);
			lora_task_stop_handler((void *)&LoraOutState); /* LORA任务停止处理 */
			break;
	}
}

void lora_reset(void)
{
	lora_state = LORA_IDLE;
	lora_obj.state = LORA_IDLE;
	lora_conn_status = LORA_OFFLINE;
	lora_pre_conn_status = LORA_OFFLINE;
	lora_obj.param.tx_fail_times = 0;
	
	swt_mod_t* timer = swt_get_handle();
	timer->lora_idle->stop();
	timer->lora_task_time_slice->stop();
	timer->lora_tx_timeout->stop();
	
	lora_cfg_default();
	LIGHT_1_OFF();
	
	lora_obj.lpm_obj->task_set_stat(LORA_TASK_ID, LPM_TASK_STA_STOP);
	uint8_t LoraOutState = LORA_OUT_STATE_DISCON;
	lora_task_stop_handler((void *)&LoraOutState); /* LORA任务停止处理 */
}

lora_obj_t* lora_task_init(lpm_obj_t* lpm_obj)
{
	lora_state = LORA_IDLE;
	lora_obj.state = LORA_IDLE;
	
	lora_obj.param.delay_base_time = LORA_DELAY_BASE_TIME;
	lora_obj.param.random_delay_upper = LORA_RANDOME_DELAY_UPPER;
	lora_obj.param.random_delay_lower = LORA_RANDOME_DELAY_LOWER;
	lora_obj.param.tx_max_delay_time = LORA_TX_MAX_DELAY_TIME;
	lora_obj.param.task_time_slice = SWT_LORA_TIME_SLICE_TIME;
	lora_obj.param.rx_timeout_base = LORA_RX_TIMEOUT_BASE;
	lora_obj.param.tx_fail_times = 0;
	lora_obj.param.tx_max_fail_times = LORA_TX_MAX_FIAL_TIMES;
	lora_obj.param.is_idle_enter_lp = ENABLE;
	
	lora_obj.lpm_obj = lpm_obj;
	
	lora_obj.task_start = lora_task_start;
	lora_obj.task_stop = lora_task_stop;
	lora_obj.set_tx_data = lora_set_tx_data;
	lora_obj.task_operate = lora_task_operate;
	
	lora_obj.lpm_obj->task_reg(LORA_TASK_ID);
	
	return &lora_obj;
}













