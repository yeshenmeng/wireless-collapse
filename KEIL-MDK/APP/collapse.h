#ifndef __COLLAPSE_H__
#define __COLLAPSE_H__
#include "main.h"
#include "low_power_manage.h"


#define COLLAPSE_MODE						TRIGGER_MODE	//TRIGGER_MODE：触发模式，PERIOD_MODE：周期模式
#define COLLAPSE_TRIGGER_PERIOD				2000			//触发模式下持续触发时的采样周期
#define COLLAPSE_SAMPLE_PERIOD				5000			//采样模式下的数据采样周期
#define COLLAPSE_FIFO_IN_PERIOD				500				//持续触发时向FIFO写入数据的周期

#define BMA456_USE_FIFO						1 				//0：禁用FIFO功能，1：使用FIFO功能
#define BMA456_WTM_SIZE						1000			//设置FIFO水印大小，实际设置值比此值小20字节

/* 软件模拟SPI引脚设置 */
#define COLLAPSE_SW_SPI_SCK_PIN				15
#define COLLAPSE_SW_SPI_MOSI_PIN			16
#define COLLAPSE_SW_SPI_MISO_PIN			17
#define COLLAPSE_SW_SPI_CS_PIN				18

/* 软件模拟SPI端口设置 */
#define COLLAPSE_SW_SPI_SCK_PORT			P0
#define COLLAPSE_SW_SPI_MOSI_PORT			P0
#define COLLAPSE_SW_SPI_MISO_PORT			P0
#define COLLAPSE_SW_SPI_CS_PORT				P0

#define COLLAPSE_SW_SPI_SCK_SET()			nrf_gpio_pin_set(COLLAPSE_SW_SPI_SCK_PIN)
#define COLLAPSE_SW_SPI_SCK_CLR()			nrf_gpio_pin_clear(COLLAPSE_SW_SPI_SCK_PIN)
#define COLLAPSE_SW_SPI_MOSI_SET()			nrf_gpio_pin_set(COLLAPSE_SW_SPI_MOSI_PIN)
#define COLLAPSE_SW_SPI_MOSI_CLR()			nrf_gpio_pin_clear(COLLAPSE_SW_SPI_MOSI_PIN)
#define COLLAPSE_SW_SPI_CS_ENABLE()			nrf_gpio_pin_clear(COLLAPSE_SW_SPI_CS_PIN)
#define COLLAPSE_SW_SPI_CS_DISABLE()		nrf_gpio_pin_set(COLLAPSE_SW_SPI_CS_PIN)
#define COLLAPSE_SW_SPI_MISO_READ() 		nrf_gpio_pin_read(COLLAPSE_SW_SPI_MISO_PIN)

#define COLLAPSE_INT1_PIN					6 //加速度计中断信号1引脚
#define COLLAPSE_INT2_PIN					5 //加速度计中断信号2引脚

typedef enum {
	COLLAPSE_IDLE,
	COLLAPSE_ACTIVE,
	COLLAPSE_DATA,
	COLLAPSE_STOP,
}collapse_state_t;

typedef enum {
	PERIOD_MODE = 0,
	TRIGGER_MODE,
}sens_mode_t;

typedef __packed struct {
	float x_accel;
	float y_accel;
	float z_accel;
}sens_accel_t;

typedef __packed struct {
	float x_angle;
	float y_angle;
	float z_angle;
}sens_angle_t;

typedef __packed struct {
	float temp_c;
	sens_accel_t accel;
	sens_angle_t angle;
}sens_data_t;

typedef struct {
	uint8_t update_flag;
	sens_mode_t mode;
	uint32_t period;
	sens_data_t data;
	
	collapse_state_t state;
	
	lpm_obj_t* lpm_obj;
	
	void (*task_start)(void);
	void (*task_stop)(void);
	void (*task_operate)(void);
}collapse_obj_t;

collapse_obj_t* collapse_init(lpm_obj_t* lpm_obj);

#endif


