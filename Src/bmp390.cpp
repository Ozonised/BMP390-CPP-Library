/*
 * bmp390.cpp
 *
 *  Created on: Jan 12, 2026
 *      Author: farhan
 */

#include "bmp390.hpp"

void BMP390::Init(bool CSBPinState, bool SDOPinState)
{
	if (CSBPinState)
	{
		chipAddress = SDOPinState ? 0x77 : 0x76;
	}
}

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

BMP390_RET_TYPE BMP390::SetPressureOversampling(bmp390::TempPressOversamlping osrp)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t osr = 0;

	if (read(hInterface, chipAddress, bmp390::REG_OSR, &osr, 1) == BMP390_RET_TYPE_SUCCESS)
	{
		osr &= ~(bmp390::REG_OSR_OSR_P_0 | bmp390::REG_OSR_OSR_P_1 | bmp390::REG_OSR_OSR_P_2);

		switch (osrp)
		{
		case bmp390::TempPressOversamlping::x1:
			break;

		case bmp390::TempPressOversamlping::x2:
			osr |= bmp390::REG_OSR_OSR_P_1;
			break;

		case bmp390::TempPressOversamlping::x4:
			osr |= bmp390::REG_OSR_OSR_P_2;
			break;

		case bmp390::TempPressOversamlping::x8:
			osr |= (bmp390::REG_OSR_OSR_P_1 | bmp390::REG_OSR_OSR_P_0);
			break;

		case bmp390::TempPressOversamlping::x16:
			osr |= bmp390::REG_OSR_OSR_P_2;
			break;

		case bmp390::TempPressOversamlping::x32:
			osr |= (bmp390::REG_OSR_OSR_P_2 | bmp390::REG_OSR_OSR_P_0);
			break;

		default:
			break;
		}
		ret = write(hInterface, chipAddress, bmp390::REG_OSR, &osr, 1);
	}
	return ret;
}

BMP390_RET_TYPE BMP390::SetTemperatureOversampling(bmp390::TempPressOversamlping osrt)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t osr = 0;

	if (read(hInterface, chipAddress, bmp390::REG_OSR, &osr, 1) == BMP390_RET_TYPE_SUCCESS)
	{
		osr &= ~(bmp390::REG_OSR_OSR_T_0 | bmp390::REG_OSR_OSR_T_1 | bmp390::REG_OSR_OSR_T_2);

		switch (osrt)
		{
		case bmp390::TempPressOversamlping::x1:
			break;

		case bmp390::TempPressOversamlping::x2:
			osr |= bmp390::REG_OSR_OSR_T_1;
			break;

		case bmp390::TempPressOversamlping::x4:
			osr |= bmp390::REG_OSR_OSR_T_2;
			break;

		case bmp390::TempPressOversamlping::x8:
			osr |= (bmp390::REG_OSR_OSR_T_1 | bmp390::REG_OSR_OSR_T_0);
			break;

		case bmp390::TempPressOversamlping::x16:
			osr |= bmp390::REG_OSR_OSR_T_2;
			break;

		case bmp390::TempPressOversamlping::x32:
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

