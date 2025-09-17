/*
 * adxl.c
 *
 *  Created on: Nov 13, 2022
 *      Author: 91944
 */

#include "main.h"
#include "adxl.h"
#include "stdio.h"
#include "string.h"

uint8_t adxl_version;
uint8_t adxl_xyzval[6];
uint8_t adxl_data[6];
volatile uint8_t adxl_event;
uint8_t stsregval;
uint8_t fifostsval;

short calib_value[3];


extern uint8_t uart_data[128];
extern UART_HandleTypeDef huart2;
volatile uint32_t adxl_count;
uint32_t adxl_ovrn;
uint32_t adxl_ovrn_iter;
uint32_t activity_count;
volatile uint32_t calib_complete;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	adxl_event = 1;
	//HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	adxl_count++;
	if (adxl_count == 100)
		calib_complete = 1;

}

uint8_t poll_adxl_event(void)
{
	uint8_t val;

	adxl_spi_read(ADXL_INT_SOURCE, 1, &stsregval);
	if (stsregval & DATA_READY)
	{
		adxl_count++;
		if (stsregval & 3)
		{
			adxl_ovrn++;
			adxl_ovrn_iter = adxl_count;
		}
		if (stsregval & 0x10)
			activity_count++;
		return 1;
	}
	return 0;
}

void adxl_spi_write (uint8_t regaddr, uint8_t regval)
{
	//write multiple bytes to device
	//first byte has reg addr, mb bit and write command bit
	adxl_data[0] = (regaddr | ADXL_WRITE_COMMAND);
	adxl_data[1] = regval;
	ADXL_CS_EN();
	HAL_SPI_Transmit (&hspi1, adxl_data, ADXL_WRCMD_LEN, ADXL_SPI_TIMO_VAL);
	ADXL_CS_DIS();
}

void adxl_spi_read (uint8_t regaddr, uint8_t numregs, uint8_t *regdata)
{
	regaddr |= (ADXL_READ_COMMAND | ADXL_MB);
	ADXL_CS_EN();
	//read cmd with regaddr
	HAL_SPI_Transmit (&hspi1, &regaddr, ADXL_RDCMD_LEN, ADXL_SPI_TIMO_VAL);
	//read data from reg
	HAL_SPI_Receive (&hspi1, regdata, numregs, ADXL_SPI_TIMO_VAL);
	ADXL_CS_DIS();
}


int adxl345_init (void)
{
	uint8_t val;

	//clear all bits
	adxl_spi_write (ADXL_POWER_CTL, 0x00);
	adxl_spi_write (ADXL_INT_ENABLE, 0);
	adxl_spi_write (ADXL_FIFO_CTL, 2);	//just to turn off water mask interrupt
	//read interrupt source
	while(1)
	{
		adxl_spi_read(ADXL_INT_SOURCE, 1, &stsregval);
		//if data ready read anddiscard data
		if (stsregval & DATA_READY)
			adxl_get_accl_val(adxl_xyzval, 1);
		if (stsregval == 0)
			break;
	}
	adxl_spi_write (ADXL_DATA_FORMAT, RANGE_4g);  //+-4g, 10 bit resolution, 4 wire spi
	adxl_spi_read(ADXL_DEVID, 1, &adxl_version);
	if (adxl_version != ADXL_VERSION_NUM)
		return -1;
	//test rw with ADXL_THRESH_TAP register
	adxl_spi_write(ADXL_THRESH_TAP, ADXL_TEST_VAL);
	adxl_spi_read(ADXL_THRESH_TAP, 1, &val);
	if (val != ADXL_TEST_VAL)
		return -1;
	adxl_spi_write(ADXL_THRESH_TAP, 0);
	adxl_spi_write(ADXL_BW_RATE, 8);

	//program thresholds
	adxl_spi_write (ADXL_THRESH_ACT, 2);
	adxl_spi_write (ADXL_THRESH_INACT, 2);
	adxl_spi_write (ADXL_TIME_INAT, 10);
	adxl_spi_write (ADXL_ACT_INACT_CTL, 0x77);

	//all interrupts to int1 line
	adxl_spi_write (ADXL_INT_MAP, 0);
	adxl_spi_write (ADXL_POWER_CTL, (LINK_BIT | WAKEUP_8HZ));
	//adxl_read_regs();
	return 0;
}

void adxl_start(void)
{
	uint8_t val;
	adxl_spi_read (ADXL_POWER_CTL, 1, &val);
	val |= MEASURE;
	adxl_spi_write (ADXL_POWER_CTL,val);
}

void adxl_stop(void)
{
	uint8_t val;
	adxl_spi_read (ADXL_POWER_CTL, 1, &val);
	val &= ~MEASURE;
	adxl_spi_write (ADXL_POWER_CTL,val);
}

void adxl_get_accl_val(uint8_t *databuf, uint8_t entries)
{
	adxl_spi_read(ADXL_DATA0, (6 * entries), databuf);
}


void adxl_read_regs(void)
{
	uint32_t ii;
	uint8_t regval;
	uint8_t len;

	for (ii = 0x1D; ii <= 0x39; ii++)
	{
		adxl_spi_read(ii, 1, &regval);
		len = sprintf(uart_data, "%02x: %02x\r\n", ii, regval);
		HAL_UART_Transmit(&huart2, uart_data, len, 100);
		//HAL_Delay(100);
	}
}


#define CALIB_SAMPLES	100
void adxl_calibrate(void)
{
	uint16_t count;
	uint32_t val;

	calib_value[0] = 0;
	calib_value[1] = 0;
	calib_value[2] = 0;
	count = 0;
	adxl_start();
	while(1)
	{
		if (poll_adxl_event())
		{
			count++;
			adxl_get_accl_val(adxl_xyzval, 1);
			val = (adxl_xyzval[0] + (adxl_xyzval[1] * 256));
			calib_value[0] += val;
			val = (adxl_xyzval[2] + (adxl_xyzval[3] * 256));
			calib_value[1] += val;
			val = (adxl_xyzval[4] + (adxl_xyzval[5] * 256));
			calib_value[2] += val;
			if (count == CALIB_SAMPLES)
			{
				adxl_stop();
				calib_value[0] /= CALIB_SAMPLES;
				calib_value[1] /= CALIB_SAMPLES;
				calib_value[2] /= CALIB_SAMPLES;
				break;
			}
		}
	}
}
