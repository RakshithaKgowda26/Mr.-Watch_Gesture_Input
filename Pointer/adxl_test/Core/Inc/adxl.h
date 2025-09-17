/*
 * adxl.h
 *
 *  Created on: Nov 13, 2022
 *      Author: 91944
 */

#ifndef ADXL_H_
#define ADXL_H_

#define ADXL_CS_PORT	GPIOA
#define ADXL_CS_PIN		GPIO_PIN_4

#define ADXL_CS_EN()		HAL_GPIO_WritePin (ADXL_CS_PORT, ADXL_CS_PIN, GPIO_PIN_RESET)
#define ADXL_CS_DIS()		HAL_GPIO_WritePin (ADXL_CS_PORT, ADXL_CS_PIN, GPIO_PIN_SET)

#define ADXL_TEST_VAL	0x55

#define ADXL_WRITE_COMMAND	(0 << 7)
#define ADXL_READ_COMMAND	(1 << 7)
#define ADXL_MB				(1 << 6)	//multi byte read or write

#define ADXL_WRCMD_LEN		2
#define ADXL_RDCMD_LEN		1
#define ADXL_SPI_TIMO_VAL	100

extern SPI_HandleTypeDef hspi1;

// Register Addresses
#define ADXL_DEVID 					0x0
#define ADXL_THRESH_TAP				0x1D
#define ADXL_OFFX					0x1E
#define ADXL_OFFY					0x1F
#define ADXL_OFFZ					0x20
#define ADXL_DUR					0x21
#define ADXL_LATENT					0x22
#define ADXL_WINDOW					0x23
#define ADXL_THRESH_ACT				0x24
#define ADXL_THRESH_INACT			0x25
#define ADXL_TIME_INAT				0x26
#define ADXL_ACT_INACT_CTL			0x27
#define ADXL_THRESH_FF 				0x28
#define ADXL_TIME_FF				0x29
#define ADXL_TAP_AXES				0x2A
#define ADXL_TAP_STATUS				0x2B
#define ADXL_BW_RATE				0x2C
#define ADXL_POWER_CTL 				0x2D
#define ADXL_INT_ENABLE				0x2E
#define ADXL_INT_MAP				0x2F
#define ADXL_INT_SOURCE				0x30
#define ADXL_DATA_FORMAT 			0x31
#define ADXL_DATA0					0x32	//6 regs 0x32-0x37
#define ADXL_FIFO_CTL 				0x38

#define DATA_READY					0x80

#define RANGE_2g					0
#define RANGE_4g					1
#define RANGE_8g					2
#define RANGE_16g					3

#define WAKEUP_8HZ					0
#define WAKEUP_4HZ					1
#define WAKEUP_2HZ					2
#define WAKEUP_1HZ					3
#define LINK_BIT					0x20
#define MEASURE						0x08

#define ADXL_VERSION_NUM			0xe5


int adxl345_init (void);
void adxl_spi_read (uint8_t regaddr, uint8_t numregs, uint8_t *regdata);
void adxl_spi_write (uint8_t regaddr, uint8_t regval);
void adxl_get_accl_val(uint8_t *databuf, uint8_t entries);
uint8_t poll_adxl_event(void);
void adxl_read_regs(void);
void adxl_calibrate(void);
void adxl_start(void);
void adxl_stop(void);

extern volatile uint8_t adxl_event;
extern uint8_t adxl_xyzval[6];



#endif /* ADXL_H_ */
