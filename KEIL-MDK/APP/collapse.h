#ifndef __COLLAPSE_H__
#define __COLLAPSE_H__
#include "main.h"
#include "low_power_manage.h"


#define COLLAPSE_MODE						TRIGGER_MODE	//TRIGGER_MODE������ģʽ��PERIOD_MODE������ģʽ
#define COLLAPSE_TRIGGER_PERIOD				2000			//����ģʽ�³�������ʱ�Ĳ�������
#define COLLAPSE_SAMPLE_PERIOD				5000			//����ģʽ�µ����ݲ�������
#define COLLAPSE_FIFO_IN_PERIOD				500				//��������ʱ��FIFOд�����ݵ�����

#define BMA456_USE_FIFO						1 				//0������FIFO���ܣ�1��ʹ��FIFO����
#define BMA456_WTM_SIZE						1000			//����FIFOˮӡ��С��ʵ������ֵ�ȴ�ֵС20�ֽ�

/* ����ģ��SPI�������� */
#define COLLAPSE_SW_SPI_SCK_PIN				15
#define COLLAPSE_SW_SPI_MOSI_PIN			16
#define COLLAPSE_SW_SPI_MISO_PIN			17
#define COLLAPSE_SW_SPI_CS_PIN				18

/* ����ģ��SPI�˿����� */
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

#define COLLAPSE_INT1_PIN					6 //���ٶȼ��ж��ź�1����
#define COLLAPSE_INT2_PIN					5 //���ٶȼ��ж��ź�2����

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

