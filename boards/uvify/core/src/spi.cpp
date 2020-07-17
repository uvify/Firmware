/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

// FRAM_CS PD10
// FLOW_CS PB12
// 
// IMU_CS1 PC2  --- ICM20689_CS (MPU9250 switchable)
// IMU_CS2 PE15 --- BMI055_ACC_CS
// IMU_CS3 PD7  --- IMU_BARO_CS
// IMU_CS4 PC15 --- BMI055_GYRO_CS
//
// IMU_DRDY1 PD15 --- ICM20689_DRDY (MPU9250 switchable)
// IMU_DRDY2 PE12 --- BMI055_ACC_INT
// IMU_DRDY3 PC14 --- BMI055_GYRO_INT
//
// VDD_3V3_IMU_EN PE3

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::SPI1, {
		initSPIDevice(DRV_IMU_DEVTYPE_MPU9250, SPI::CS{GPIO::PortC, GPIO::Pin2}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}),
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20689, SPI::CS{GPIO::PortC, GPIO::Pin2}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}),
		initSPIDevice(DRV_GYR_DEVTYPE_BMI055, SPI::CS{GPIO::PortC, GPIO::Pin15}, SPI::DRDY{GPIO::PortC, GPIO::Pin14}),
		initSPIDevice(DRV_ACC_DEVTYPE_BMI055, SPI::CS{GPIO::PortE, GPIO::Pin15}, SPI::DRDY{GPIO::PortE, GPIO::Pin12}),
	}, {GPIO::PortE, GPIO::Pin3}), // power_enable
	initSPIBus(SPI::Bus::SPI2, {
		initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortD, GPIO::Pin10}),
		initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortD, GPIO::Pin7}),
		initSPIDevice(DRV_BARO_DEVTYPE_MS5607, SPI::CS{GPIO::PortD, GPIO::Pin7}),
		initSPIDevice(DRV_FLOW_DEVTYPE_PMW3901, SPI::CS{GPIO::PortB, GPIO::Pin12})
	})
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);

#if defined(BOARD_HAS_BUS_MANIFEST)
__EXPORT bool board_has_bus(enum board_bus_types type, uint32_t bus)
{
	bool rv = true;

	switch (type) {
	case BOARD_SPI_BUS:
#ifdef CONFIG_STM32_SPI4
		rv = bus != 4 || (stm32_gpioread(GPIO_PB4) == 0);
#endif /* CONFIG_STM32_SPI4 */
		break;

	case BOARD_I2C_BUS:
		break;

	default: break;
	}

	return rv;
}
#endif
