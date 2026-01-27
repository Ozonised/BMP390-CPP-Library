/*
 * bmp390.cpp
 *
 *  Created on: Jan 12, 2026
 *      Author: farhan
 */

#include "bmp390.hpp"

/**
 * @brief Determine I2C address.
 *
 * This function configures the BMP390 communication parameters based
 * on the logic levels of the CSB and SDO pins.
 *
 * When CSB is HIGH, the BMP390 operates in I2C mode. In this mode,
 * the I2C slave address is selected by the SDO pin:
 *  - SDO = LOW  -> I2C address 0x76
 *  - SDO = HIGH -> I2C address 0x77
 *
 * @param CSBPinState  Logic level of the CSB pin (true = HIGH).
 * @param SDOPinState  Logic level of the SDO pin (true = HIGH).
 *
 * @note If CSBPinState is LOW, the device is expected to operate in SPI
 *       mode and the I2C address is not configured by this function.
 */
void BMP390::Init(bool CSBPinState, bool SDOPinState)
{
	if (CSBPinState)
	{
		chipAddress = SDOPinState ? 0x77 : 0x76;
	}
}

/**
 * @brief Check for presence of a BMP390 sensor on the bus.
 *
 * @retval BMP390_RET_TYPE_SUCCESS  BMP390 device detected and verified.
 * @retval BMP390_RET_TYPE_FAILURE  Device not detected or communication failed.
 */
BMP390_RET_TYPE BMP390::IsPresent(void)
{
	uint8_t chipId = 0, revId = 0, ret = BMP390_RET_TYPE_FAILURE;

	if (read(hInterface, chipAddress, bmp390::REG_CHIP_ID, &chipId, 1) == BMP390_RET_TYPE_SUCCESS)
	{
		if (read(hInterface, chipAddress, bmp390::REG_REV_ID, &revId, 1) == BMP390_RET_TYPE_SUCCESS)
		{
			if (chipId == bmp390::CHIP_ID && revId == bmp390::REV_ID)
			{
				ret = BMP390_RET_TYPE_SUCCESS;
			}
		}
	}
	return ret;
}

/**
 * @brief Set the power mode of the BMP390 sensor.
 *
 * Supported power modes:
 *  - Sleep  : Sensor is in sleep mode (no measurements)
 *  - Forced : Single measurement is triggered
 *  - Normal : Continuous measurement mode
 *
 * @param mode one of PowerMode values
 *
 * @retval BMP390_RET_TYPE_SUCCESS  Power mode configured successfully.
 * @retval BMP390_RET_TYPE_FAILURE  Register read/write failed.
 */
BMP390_RET_TYPE BMP390::SetPowerMode(bmp390::PowerMode mode)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t pwrCtrl = 0;

	if (read(hInterface, chipAddress, bmp390::REG_PWR_CTRL, &pwrCtrl, 1) == BMP390_RET_TYPE_SUCCESS)
	{
		pwrCtrl &= ~(bmp390::REG_PWR_CTRL_MODE_0 | bmp390::REG_PWR_CTRL_MODE_1);

		switch (mode)
		{
		case bmp390::PowerMode::Sleep:
			break;
		case bmp390::PowerMode::Forced:
			pwrCtrl |= (bmp390::REG_PWR_CTRL_MODE_0);
			break;
		case bmp390::PowerMode::Normal:
			pwrCtrl |= (bmp390::REG_PWR_CTRL_MODE_0 | bmp390::REG_PWR_CTRL_MODE_1);
			break;

		default:
			break;
		}
		ret = write(hInterface, chipAddress, bmp390::REG_PWR_CTRL, &pwrCtrl, 1);
	}
	return ret;
}

/**
 * @brief Enable/Disable pressure measurement
 *
 * @param n
 * 			- 1 : enable pressure measurement
 * 			- 0 : disable pressure measurement
 *
 * @retval BMP390_RET_TYPE_SUCCESS  Success
 * @retval BMP390_RET_TYPE_FAILURE  Register read/write failed.
 */
BMP390_RET_TYPE BMP390::TogglePressureMeasurement(bool n)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t pwrCtrl = 0;

	if (read(hInterface, chipAddress, bmp390::REG_PWR_CTRL, &pwrCtrl, 1) == BMP390_RET_TYPE_SUCCESS)
	{
		if (n == true)
			pwrCtrl |= (bmp390::REG_PWR_CTRL_PRESS_EN);
		else
			pwrCtrl &= ~(bmp390::REG_PWR_CTRL_PRESS_EN);

		ret = write(hInterface, chipAddress, bmp390::REG_PWR_CTRL, &pwrCtrl, 1);
	}
	return ret;
}

/**
 * @brief Enable/Disable temperature measurement
 *
 * @param n
 * 			- 1 : enable temperature measurement
 * 			- 0 : disable temperature measurement
 *
 * @retval BMP390_RET_TYPE_SUCCESS  Success
 * @retval BMP390_RET_TYPE_FAILURE  Register read/write failed.
 */
BMP390_RET_TYPE BMP390::ToggleTemperatureMeasurement(bool n)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t pwrCtrl = 0;

	if (read(hInterface, chipAddress, bmp390::REG_PWR_CTRL, &pwrCtrl, 1) == BMP390_RET_TYPE_SUCCESS)
	{
		if (n == true)
			pwrCtrl |= (bmp390::REG_PWR_CTRL_TEMP_EN);
		else
			pwrCtrl &= ~(bmp390::REG_PWR_CTRL_TEMP_EN);

		ret = write(hInterface, chipAddress, bmp390::REG_PWR_CTRL, &pwrCtrl, 1);
	}
	return ret;
}

/**
 * @brief Configure pressure oversampling setting.
 *
 * This function sets the pressure oversampling ratio by configuring
 * the OSR_P bits in the OSR register. Higher oversampling ratios improve
 * measurement resolution and noise performance at the cost of increased
 * conversion time and power consumption.
 *
 * @param osrp one of TempPressOversampling values.
 *
 * @retval BMP390_RET_TYPE_SUCCESS  Oversampling configuration updated successfully.
 * @retval BMP390_RET_TYPE_FAILURE  Register read/write failed.
 *
 * @note Refer to table 6 in section 3.4.1 of the datasheet
 */
BMP390_RET_TYPE BMP390::SetPressureOversampling(bmp390::TempPressOversampling osrp)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t osr = 0;

	if (read(hInterface, chipAddress, bmp390::REG_OSR, &osr, 1) == BMP390_RET_TYPE_SUCCESS)
	{
		osr &= ~(bmp390::REG_OSR_OSR_P_0 | bmp390::REG_OSR_OSR_P_1 | bmp390::REG_OSR_OSR_P_2);

		switch (osrp)
		{
		case bmp390::TempPressOversampling::x1:
			break;

		case bmp390::TempPressOversampling::x2:
			osr |= bmp390::REG_OSR_OSR_P_1;
			break;

		case bmp390::TempPressOversampling::x4:
			osr |= bmp390::REG_OSR_OSR_P_2;
			break;

		case bmp390::TempPressOversampling::x8:
			osr |= (bmp390::REG_OSR_OSR_P_1 | bmp390::REG_OSR_OSR_P_0);
			break;

		case bmp390::TempPressOversampling::x16:
			osr |= bmp390::REG_OSR_OSR_P_2;
			break;

		case bmp390::TempPressOversampling::x32:
			osr |= (bmp390::REG_OSR_OSR_P_2 | bmp390::REG_OSR_OSR_P_0);
			break;

		default:
			break;
		}
		ret = write(hInterface, chipAddress, bmp390::REG_OSR, &osr, 1);
	}
	return ret;
}

/**
 * @brief Configure temperature oversampling setting.
 *
 * This function sets the temperature oversampling ratio by configuring
 * the OSR_T bits in the OSR register. Higher oversampling ratios improve
 * measurement resolution and noise performance at the cost of increased
 * conversion time and power consumption.
 *
 * @param osrp one of TempPressOversampling values.
 *
 * @retval BMP390_RET_TYPE_SUCCESS  Oversampling configuration updated successfully.
 * @retval BMP390_RET_TYPE_FAILURE  Register read/write failed.
 *
 * @note Refer to table 6 & 7 in section 3.4.1 & 3.4.2 of the datasheet
 */
BMP390_RET_TYPE BMP390::SetTemperatureOversampling(bmp390::TempPressOversampling osrt)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t osr = 0;

	if (read(hInterface, chipAddress, bmp390::REG_OSR, &osr, 1) == BMP390_RET_TYPE_SUCCESS)
	{
		osr &= ~(bmp390::REG_OSR_OSR_T_0 | bmp390::REG_OSR_OSR_T_1 | bmp390::REG_OSR_OSR_T_2);

		switch (osrt)
		{
		case bmp390::TempPressOversampling::x1:
			break;

		case bmp390::TempPressOversampling::x2:
			osr |= bmp390::REG_OSR_OSR_T_1;
			break;

		case bmp390::TempPressOversampling::x4:
			osr |= bmp390::REG_OSR_OSR_T_2;
			break;

		case bmp390::TempPressOversampling::x8:
			osr |= (bmp390::REG_OSR_OSR_T_1 | bmp390::REG_OSR_OSR_T_0);
			break;

		case bmp390::TempPressOversampling::x16:
			osr |= bmp390::REG_OSR_OSR_T_2;
			break;

		case bmp390::TempPressOversampling::x32:
			osr |= (bmp390::REG_OSR_OSR_T_2 | bmp390::REG_OSR_OSR_T_0);
			break;

		default:
			break;
		}
		ret = write(hInterface, chipAddress, bmp390::REG_OSR, &osr, 1);
	}
	return ret;
}

BMP390_RET_TYPE BMP390::SetOutputDataRate(bmp390::TempPressODR odr)
{
	uint8_t ODR = static_cast<uint8_t>(odr);
	return write(hInterface, chipAddress, bmp390::REG_ODR, &ODR, 1);
}

BMP390_RET_TYPE BMP390::GetStatus(uint8_t &status)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	ret = read(hInterface, chipAddress, bmp390::REG_STATUS, &status, 1);
	return ret;
}

BMP390_RET_TYPE BMP390::IsBusy(void)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t status = 0;
	if (GetStatus(status) == BMP390_RET_TYPE_SUCCESS)
	{
		if (status & bmp390::REG_STATUS_CMD_RDY)
		{
			ret = BMP390_RET_TYPE_SUCCESS;
		}
		else
		{
			ret = BMP390_RET_TYPE_BUSY;
		}
	}
	return ret;
}

