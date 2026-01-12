/*
 * bmp390-reg.hpp
 *
 *  Created on: Jan 12, 2026
 *      Author: farhan
 */

#ifndef BMP390_REG_HPP_
#define BMP390_REG_HPP_

#include <stdint.h>

namespace BMP390 {

	constexpr uint8_t REG_CHIP_ID = 0x00;
	constexpr uint8_t REG_REV_ID = 0x01;
	constexpr uint8_t REG_ERR_REG = 0x02;
	constexpr uint8_t REG_STATUS = 0x03;
	constexpr uint8_t REG_DATA_0 = 0x04;
	constexpr uint8_t REG_DATA_1 = 0x05;
	constexpr uint8_t REG_DATA_2 = 0x06;
	constexpr uint8_t REG_DATA_3 = 0x07;
	constexpr uint8_t REG_DATA_4 = 0x08;
	constexpr uint8_t REG_DATA_5 = 0x09;

	constexpr uint8_t REG_SENSORTIME_0 = 0x0C;
	constexpr uint8_t REG_SENSORTIME_1 = 0x0D;
	constexpr uint8_t REG_SENSORTIME_2 = 0x0E;

	constexpr uint8_t REG_EVENT = 0x10;
	constexpr uint8_t REG_INT_STATUS = 0x11;

	constexpr uint8_t REG_FIFO_LENGTH_0 = 0x12;
	constexpr uint8_t REG_FIFO_LENGTH_1 = 0x13;
	constexpr uint8_t REG_FIFO_DATA = 0x14;
	constexpr uint8_t REG_FIFO_WTM_0 = 0x15;
	constexpr uint8_t REG_FIFO_WTM_1 = 0x16;
	constexpr uint8_t REG_FIFO_CONFIG_1 = 0x17;
	constexpr uint8_t REG_FIFO_CONFIG_2 = 0x18;

	constexpr uint8_t REG_INT_CTRL = 0x19;
	constexpr uint8_t REG_IF_CONF = 0x1A;
	constexpr uint8_t REG_PWR_CTRL = 0x1B;
	constexpr uint8_t REG_OSR = 0x1C;
	constexpr uint8_t REG_ODR = 0x1D;
	constexpr uint8_t REG_CONFIG = 0x1F;
	constexpr uint8_t REG_CMD = 0x7E;

	constexpr uint8_t REG_ERROR_REG_CONF_ERR = (1 << 2);
	constexpr uint8_t REG_ERROR_REG_CMD_ERR = (1 << 1);
	constexpr uint8_t REG_ERROR_REG_FATAL_ERR = (1 << 0);

	constexpr uint8_t REG_STATUS_DRDY_TEMP = (1 << 6);
	constexpr uint8_t REG_STATUS_DRDY_PRESS = (1 << 5);
	constexpr uint8_t REG_STATUS_CMD_RDY = (1 << 4);

	constexpr uint8_t REG_EVENT_POR_DETECTED = (1);
	constexpr uint8_t REG_EVENT_ITF_ACT_PT = (1 << 1);

	constexpr uint8_t REG_INT_STATUS_FWM_INT = (1);
	constexpr uint8_t REG_INT_STATUS_FFULL_INT = (1 << 1);
	constexpr uint8_t REG_INT_STATUS_DRDY = (1 << 3);

	constexpr uint8_t REG_FIFO_CONFIG_1_FIFO_MODE = (1);
	constexpr uint8_t REG_FIFO_CONFIG_1_FIFO_STOP_ON_FULL = (1 << 1);
	constexpr uint8_t REG_FIFO_CONFIG_1_FIFO_TIME_EN = (1 << 2);
	constexpr uint8_t REG_FIFO_CONFIG_1_FIFO_PRESS_EN = (1 << 3);
	constexpr uint8_t REG_FIFO_CONFIG_1_FIFO_TEMP_EN = (1 << 4);

	constexpr uint8_t REG_FIFO_CONFIG_2_FIFO_SUBSAMPLING = (0);
	constexpr uint8_t REG_FIFO_CONFIG_2_FIFO_DATA_SELECT_0 = (1 << 3);
	constexpr uint8_t REG_FIFO_CONFIG_2_FIFO_DATA_SELECT_1 = (1 << 4);

	constexpr uint8_t REG_INT_CTRL_INT_OD = (1 << 0);
	constexpr uint8_t REG_INT_CTRL_INT_LEVEL = (1 << 1);
	constexpr uint8_t REG_INT_CTRL_INT_LATCH = (1 << 2);
	constexpr uint8_t REG_INT_CTRL_FWTM_EN = (1 << 3);
	constexpr uint8_t REG_INT_CTRL_FFULL_EN = (1 << 4);
	constexpr uint8_t REG_INT_CTRL_INT_DS = (1 << 5);
	constexpr uint8_t REG_INT_CTRL_DRDY_EN = (1 << 6);

	constexpr uint8_t REG_IF_CONF_SPI3 = (1 << 0);
	constexpr uint8_t REG_IF_CONF_I2C_WDT_SEL = (1 << 1);
	constexpr uint8_t REG_IF_CONF_I2C_WDT_SEL = (1 << 2);

	constexpr uint8_t REG_PWR_CTRL_PRESS_EN = (1 << 0);
	constexpr uint8_t REG_PWR_CTRL_TEMP_EN = (1 << 1);
	constexpr uint8_t REG_PWR_CTRL_MODE_0 = (1 << 4);
	constexpr uint8_t REG_PWR_CTRL_MODE_1 = (1 << 5);

	constexpr uint8_t REG_OSR_OSR_P_0 = (1 << 0);
	constexpr uint8_t REG_OSR_OSR_P_1 = (1 << 1);
	constexpr uint8_t REG_OSR_OSR_P_2 = (1 << 2);

	constexpr uint8_t REG_OSR_OSR_T_0 = (1 << 3);
	constexpr uint8_t REG_OSR_OSR_T_1 = (1 << 4);
	constexpr uint8_t REG_OSR_OSR_T_2 = (1 << 5);

	constexpr uint8_t REG_ODR_ODR_SEL_0 = (1 << 0);
	constexpr uint8_t REG_ODR_ODR_SEL_1 = (1 << 1);
	constexpr uint8_t REG_ODR_ODR_SEL_2 = (1 << 2);
	constexpr uint8_t REG_ODR_ODR_SEL_3 = (1 << 3);
	constexpr uint8_t REG_ODR_ODR_SEL_4 = (1 << 4);

	constexpr uint8_t REG_CONFIG_SHORT_IN = (1 << 0);
    constexpr uint8_t REG_CONFIG_IIR_FILTER_0 = (1 << 1);
    constexpr uint8_t REG_CONFIG_IIR_FILTER_1 = (1 << 2);
    constexpr uint8_t REG_CONFIG_IIR_FILTER_2 = (1 << 3);

    constexpr uint8_t CHIP_ID = 0x60;
    constexpr uint8_t REV_ID = 0x01;

}  // namespace BMP390

#endif /* BMP390_REG_HPP_ */
