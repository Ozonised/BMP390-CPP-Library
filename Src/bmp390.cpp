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
 * @param[in] CSBPinState  Logic level of the CSB pin (true = HIGH).
 * @param[in] SDOPinState  Logic level of the SDO pin (true = HIGH).
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
 * @param[in] mode one of PowerMode values
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
 * @param[in] n
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
 * @param[in] n
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
 * @param[in] osrp one of TempPressOversampling values.
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

		osr |= (static_cast<uint8_t>(osrp) <<bmp390::REG_OSR_OSR_P_POS);

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
 * @param[in] osrp one of TempPressOversampling values.
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

		osr |= (static_cast<uint8_t>(osrt) << bmp390::REG_OSR_OSR_T_POS);

		ret = write(hInterface, chipAddress, bmp390::REG_OSR, &osr, 1);
	}
	return ret;
}

/**
 * @brief Configure the output data rate (ODR).
 *
 * This function sets the output data rate for pressure and temperature
 * measurements by writing to the ODR register. The selected data rate
 * determines how frequently new measurement results are generated
 * when the sensor is operating in Normal mode.
 *
 * @param[in] odr Desired output data rate setting.
 *
 * @retval BMP390_RET_TYPE_SUCCESS  Output data rate configured successfully.
 * @retval BMP390_RET_TYPE_FAILURE  Register write failed.
 *
 * @note The ODR setting is only effective in Normal mode.
 * @note In Forced mode, a single measurement is triggered regardless
 *       of the configured output data rate.
 */
BMP390_RET_TYPE BMP390::SetOutputDataRate(bmp390::TempPressODR odr)
{
	uint8_t ODR = static_cast<uint8_t>(odr);
	return write(hInterface, chipAddress, bmp390::REG_ODR, &ODR, 1);
}

/**
 * @brief Read the BMP390 status register.
 *
 * @param[out] status Reference to a variable where the status register
 *               value will be stored.
 *
 * @retval BMP390_RET_TYPE_SUCCESS  Status register read successfully.
 * @retval BMP390_RET_TYPE_FAILURE  Register read failed.
 *
 * @note The interpretation of individual status bits is defined
 *       in the BMP390 datasheet.
 */
BMP390_RET_TYPE BMP390::GetStatus(uint8_t &status)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	ret = read(hInterface, chipAddress, bmp390::REG_STATUS, &status, 1);
	return ret;
}

/**
 * @brief Check whether the BMP390 is busy executing a command.
 *
 * @retval BMP390_RET_TYPE_SUCCESS  Device is ready (not busy).
 * @retval BMP390_RET_TYPE_BUSY     Device is busy.
 * @retval BMP390_RET_TYPE_FAILURE  Status register read failed.
 *
 * @note This function relies on the CMD_RDY bit in the STATUS register
 *       as defined in the BMP390 datasheet.
 */
BMP390_RET_TYPE BMP390::IsBusy(void)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t status = 0;
	if (GetStatus(status) == BMP390_RET_TYPE_SUCCESS)
	{
		if (status & bmp390::REG_STATUS_CMD_RDY)
		{
			ret = BMP390_RET_TYPE_SUCCESS;
		} else
		{
			ret = BMP390_RET_TYPE_BUSY;
		}
	}
	return ret;
}

BMP390_RET_TYPE BMP390::GetDrdySource(bmp390::DrdySource &src)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t status = 0;

	ret = GetStatus(status);

	if (ret == BMP390_RET_TYPE_SUCCESS)
	{
		if ((status & bmp390::REG_STATUS_DRDY_PRESS) && (status & bmp390::REG_STATUS_DRDY_TEMP))
			src = bmp390::DrdySource::PressTemp;
		else if (status & bmp390::REG_STATUS_DRDY_PRESS)
			src = bmp390::DrdySource::Press;
		else if (status & bmp390::REG_STATUS_DRDY_TEMP)
			src = bmp390::DrdySource::Temp;
		else
			src = bmp390::DrdySource::None;
	}
	return ret;
}

BMP390_RET_TYPE BMP390::SetIIRFilterCoefficient(bmp390::IIRFilterCoefficient coef)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t config = 0;

	if (read(hInterface, chipAddress, bmp390::REG_CONFIG, &config, 1) == BMP390_RET_TYPE_SUCCESS)
	{
		config &= ~(bmp390::REG_CONFIG_IIR_FILTER_2 | bmp390::REG_CONFIG_IIR_FILTER_1 | bmp390::REG_CONFIG_IIR_FILTER_0);

		config |= (static_cast<uint8_t>(coef) << bmp390::REG_CONFIG_IIR_FILTER_POS);

		ret = write(hInterface, chipAddress, bmp390::REG_CONFIG, &config, 1);
	}
	return ret;
}
