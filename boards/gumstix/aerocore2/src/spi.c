/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file aerocore2_spi.c
 *
 * Board-specific SPI functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <unistd.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include <up_arch.h>
#include <chip.h>
#include <stm32.h>
#include "board_config.h"

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ************************************************************************************/

__EXPORT void stm32_spiinitialize(void)
{
#ifdef CONFIG_STM32_SPI3
	px4_arch_configgpio(GPIO_SPI_CS_GYRO);
	px4_arch_configgpio(GPIO_SPI_CS_ACCEL_MAG);
	px4_arch_configgpio(GPIO_SPI_CS_BARO);

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	px4_arch_gpiowrite(GPIO_SPI_CS_GYRO, 1);
	px4_arch_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
	px4_arch_gpiowrite(GPIO_SPI_CS_BARO, 1);

	px4_arch_configgpio(GPIO_EXTI_GYRO_DRDY);
	px4_arch_configgpio(GPIO_EXTI_MAG_DRDY);
	px4_arch_configgpio(GPIO_EXTI_ACCEL_DRDY);
#endif

#ifdef CONFIG_STM32_SPI4
	px4_arch_configgpio(GPIO_SPI_CS_FRAM);
	px4_arch_gpiowrite(GPIO_SPI_CS_FRAM, 1);
#endif

}

#ifdef CONFIG_STM32_SPI1
__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* there is only one device broken-out so select it */
	px4_arch_gpiowrite(GPIO_SPI1_NSS, !selected);
}

__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_STM32_SPI2
__EXPORT void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* there is only one device broken-out so select it */
	px4_arch_gpiowrite(GPIO_SPI2_NSS, !selected);
}

__EXPORT uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_STM32_SPI3
__EXPORT void stm32_spi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	switch (devid) {
	case PX4_SPIDEV_GYRO:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI_CS_GYRO, !selected);
		px4_arch_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_BARO, 1);
		break;

	case PX4_SPIDEV_ACCEL_MAG:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI_CS_GYRO, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, !selected);
		px4_arch_gpiowrite(GPIO_SPI_CS_BARO, 1);
		break;

	case PX4_SPIDEV_BARO:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI_CS_GYRO, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_BARO, !selected);
		break;

	default:
		break;
	}
}
#endif

__EXPORT uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}


#ifdef CONFIG_STM32_SPI4
__EXPORT void stm32_spi4select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* there can only be one device on this bus, so always select it */
	px4_arch_gpiowrite(GPIO_SPI_CS_FRAM, !selected);
}

__EXPORT uint8_t stm32_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	/* FRAM is always present */
	return SPI_STATUS_PRESENT;
}
#endif

__EXPORT void board_spi_reset(int ms)
{
	/* disable SPI bus */
	px4_arch_configgpio(GPIO_SPI_CS_GYRO_OFF);
	px4_arch_configgpio(GPIO_SPI_CS_ACCEL_MAG_OFF);
	px4_arch_configgpio(GPIO_SPI_CS_BARO_OFF);

	px4_arch_gpiowrite(GPIO_SPI_CS_GYRO_OFF, 0);
	px4_arch_gpiowrite(GPIO_SPI_CS_ACCEL_MAG_OFF, 0);
	px4_arch_gpiowrite(GPIO_SPI_CS_BARO_OFF, 0);

	px4_arch_configgpio(GPIO_SPI3_SCK_OFF);
	px4_arch_configgpio(GPIO_SPI3_MISO_OFF);
	px4_arch_configgpio(GPIO_SPI3_MOSI_OFF);

	px4_arch_gpiowrite(GPIO_SPI3_SCK_OFF, 0);
	px4_arch_gpiowrite(GPIO_SPI3_MISO_OFF, 0);
	px4_arch_gpiowrite(GPIO_SPI3_MOSI_OFF, 0);

	px4_arch_configgpio(GPIO_GYRO_DRDY_OFF);
	px4_arch_configgpio(GPIO_MAG_DRDY_OFF);
	px4_arch_configgpio(GPIO_ACCEL_DRDY_OFF);

	px4_arch_gpiowrite(GPIO_GYRO_DRDY_OFF, 0);
	px4_arch_gpiowrite(GPIO_MAG_DRDY_OFF, 0);
	px4_arch_gpiowrite(GPIO_ACCEL_DRDY_OFF, 0);

	/* wait for the sensor rail to reach GND */
	usleep(ms * 1000);
	syslog(LOG_DEBUG, "reset done, %d ms\n", ms);

	/* wait a bit before starting SPI, different times didn't influence results */
	usleep(100);

	/* reconfigure the SPI pins */
#ifdef CONFIG_STM32_SPI3
	px4_arch_configgpio(GPIO_SPI_CS_GYRO);
	px4_arch_configgpio(GPIO_SPI_CS_ACCEL_MAG);
	px4_arch_configgpio(GPIO_SPI_CS_BARO);

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	px4_arch_gpiowrite(GPIO_SPI_CS_GYRO, 1);
	px4_arch_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
	px4_arch_gpiowrite(GPIO_SPI_CS_BARO, 1);

	px4_arch_configgpio(GPIO_SPI3_SCK);
	px4_arch_configgpio(GPIO_SPI3_MISO);
	px4_arch_configgpio(GPIO_SPI3_MOSI);

#endif
}
