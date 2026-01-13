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
