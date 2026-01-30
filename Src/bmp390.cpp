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
 * @brief Enable/Disable temeprature and pressure measurement
 *
 * @param[in] TempEn
 * 			  - 1 : enable temperature measurement
 * 			  - 0 : disable temperature measurement
 *
 * @param[in] PressEn
 * 			  - 1 : enable pressure measurement
 * 			  - 0 : disable pressure measurement
 *
 * @retval BMP390_RET_TYPE_SUCCESS  Success
 * @retval BMP390_RET_TYPE_FAILURE  Register read/write failed.
 */
BMP390_RET_TYPE BMP390::ToggleTemperatureAndPressureMeasurement(bool TempEn, bool PressEn)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t pwrCtrl = 0;

	if (read(hInterface, chipAddress, bmp390::REG_PWR_CTRL, &pwrCtrl, 1) == BMP390_RET_TYPE_SUCCESS)
	{
		if (TempEn == true)
			pwrCtrl |= (bmp390::REG_PWR_CTRL_TEMP_EN);
		else
			pwrCtrl &= ~(bmp390::REG_PWR_CTRL_TEMP_EN);

		if (PressEn == true)
			pwrCtrl |= (bmp390::REG_PWR_CTRL_PRESS_EN);
		else
			pwrCtrl &= ~(bmp390::REG_PWR_CTRL_PRESS_EN);

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
 * @see Table 6 in section 3.4.1 of the datasheet
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
 * @see table 6 & 7 in section 3.4.1 & 3.4.2 of the datasheet
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
 *
 * @see Table 45 in section 4.3.20 for odr settings
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
 *             value will be stored.
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

/**
 * @brief Get the data-ready (DRDY) source.
 *
 * This function reads the BMP390 status register and determines which
 * measurement data is ready based on the DRDY status bits.
 *
 * The returned source indicates whether:
 * - Pressure data is ready
 * - Temperature data is ready
 * - Both pressure and temperature data are ready
 * - No new data is available
 *
 * @param[out] src DrdySource object, contains one of DrdySource value
 *
 * @retval BMP390_RET_TYPE_SUCCESS  Status register read successfully and
 *                                  data-ready source updated.
 * @retval BMP390_RET_TYPE_FAILURE  Failed to read status register.
 *
 * @note If both pressure and temperature DRDY bits are set, the source
 *       is reported as @ref bmp390::DrdySource::PressTemp.
 *
 * @see bmp390::DrdySource
 */
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

/**
 * @brief Configure the IIR filter coefficient.
 *
 * This function sets the Infinite Impulse Response (IIR) filter coefficient
 * used by the BMP390 to smooth pressure and temperature measurements.
 *
 * The IIR filter reduces measurement noise at the cost of increased
 * response time. Higher filter coefficients provide stronger filtering
 * and greater latency.
 *
 * @param[in] coef IIR filter coefficient to be applied.
 *
 * @retval BMP390_RET_TYPE_SUCCESS  IIR filter coefficient configured
 *                                  successfully.
 * @retval BMP390_RET_TYPE_FAILURE  Failed to read or write the CONFIG register.
 *
 * @see bmp390::IIRFilterCoefficient
 * @see Figure 6 in section 3.4.3 for step response and Table 10 in section 3.5 for recommended filter values.
 */
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

/**
 * @brief Configure the electrical behavior of the BMP390 INT pin.
 *
 * This function configures the interrupt pin output type, active level,
 * and latching behavior by updating the INT_CTRL register.
 *
 * @param od[in]    Interrupt pin output driver type.
 *               	- true  : Open-drain output
 *               	- false : Push-pull output
 *
 * @param level[in] Interrupt pin active level.
 *               	- true  : Active-high
 *               	- false : Active-low
 *
 * @param latch[in] Interrupt pin latching behavior.
 *               	- true  : Interrupt is latched until status is cleared
 *               	- false : Interrupt is pulse-based (non-latched)
 *
 * @retval BMP390_RET_TYPE_SUCCESS  INT_CTRL register updated successfully.
 * @retval BMP390_RET_TYPE_FAILURE  Register read or write failed.
 *
 * @note When open-drain mode is enabled, an external pull-up resistor
 *       is required on the INT pin.
 *
 * @note Latching affects both the INT pin and the INT_STATUS register.
 */
BMP390_RET_TYPE BMP390::ConfigInterruptPin(bool od, bool level, bool latch)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t IntCtrl = 0;

	if (read(hInterface, chipAddress, bmp390::REG_INT_CTRL, &IntCtrl, 1) == BMP390_RET_TYPE_SUCCESS)
	{
		IntCtrl = od ? (IntCtrl | bmp390::REG_INT_CTRL_INT_OD) : (IntCtrl & ~(bmp390::REG_INT_CTRL_INT_OD));
		IntCtrl = level ? (IntCtrl | bmp390::REG_INT_CTRL_INT_LEVEL) : (IntCtrl & ~(bmp390::REG_INT_CTRL_INT_LEVEL));
		IntCtrl = latch ? (IntCtrl | bmp390::REG_INT_CTRL_INT_LATCH) : (IntCtrl & ~(bmp390::REG_INT_CTRL_INT_LATCH));

		ret = write(hInterface, chipAddress, bmp390::REG_INT_CTRL, &IntCtrl, 1);
	}
	return ret;
}

/**
 * @brief Enable or disable interrupt sources for the INT pin.
 *
 * This function selects which internal events can trigger an interrupt
 * on the INT pin by enabling or disabling corresponding bits in the
 * INT_CTRL register.
 *
 *
 * @param TempPress[in]     Enable temperature/pressure data-ready interrupt.
 *                      	- true  : Data-ready interrupt enabled
 *                      	- false : Data-ready interrupt disabled
 *
 * @param FifoFull[in]      Enable FIFO full interrupt.
 *                      	- true  : FIFO full interrupt enabled
 *                      	- false : FIFO full interrupt disabled
 *
 * @param FifoWaterMark[in] Enable FIFO watermark interrupt.
 *                      	- true  : FIFO watermark interrupt enabled
 *                      	- false : FIFO watermark interrupt disabled
 *
 * @retval BMP390_RET_TYPE_SUCCESS  INT_CTRL register updated successfully.
 * @retval BMP390_RET_TYPE_FAILURE  Register read or write failed.
 *
 * @note Interrupt sources are reflected in both the INT pin and the
 *       INT_STATUS register.
 */
BMP390_RET_TYPE BMP390::SetInterruptSource(bool TempPress, bool FifoFull, bool FifoWaterMark)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t IntCtrl = 0;

	if (read(hInterface, chipAddress, bmp390::REG_INT_CTRL, &IntCtrl, 1) == BMP390_RET_TYPE_SUCCESS)
	{
		IntCtrl = TempPress ? (IntCtrl | bmp390::REG_INT_CTRL_DRDY_EN) : (IntCtrl & ~(bmp390::REG_INT_CTRL_DRDY_EN));
		IntCtrl = FifoFull ? (IntCtrl | bmp390::REG_INT_CTRL_FFULL_EN) : (IntCtrl & ~(bmp390::REG_INT_CTRL_FFULL_EN));
		IntCtrl = FifoWaterMark ? (IntCtrl | bmp390::REG_INT_CTRL_FWTM_EN) : (IntCtrl & ~(bmp390::REG_INT_CTRL_FWTM_EN));

		ret = write(hInterface, chipAddress, bmp390::REG_INT_CTRL, &IntCtrl, 1);
	}
	return ret;
}

/**
 * @brief Read and decode the active interrupt source(s).
 *
 * This function reads the INT_STATUS register of the BMP390 and extracts
 * the interrupt source flags related to:
 * - Temperature / pressure data ready
 * - FIFO full
 * - FIFO watermark reached
 *
 * @param[out] src InterruptSource object, contains one of InterruptSource value
 *
 * @retval BMP390_RET_TYPE_SUCCESS  INT_STATUS register read successfully
 *                                  and interrupt source updated.
 * @retval BMP390_RET_TYPE_FAILURE  Failed to read INT_STATUS register.
 *
 * @see bmp390::InterruptSource
 */
BMP390_RET_TYPE BMP390::GetInterruptSource(bmp390::InterruptSource &src)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t IntStatus = 0;

	ret = read(hInterface, chipAddress, bmp390::REG_INT_STATUS, &IntStatus, 1);
	if (ret == BMP390_RET_TYPE_SUCCESS)
	{
		IntStatus &= (bmp390::REG_INT_STATUS_DRDY | bmp390::REG_INT_STATUS_FFULL_INT | bmp390::REG_INT_STATUS_FWM_INT);

		src = static_cast<bmp390::InterruptSource>(IntStatus);
	}
	return ret;
}

/**
 * @brief Read and decode BMP390 factory calibration parameters from NVM.
 *
 * This function reads the non-volatile memory (NVM) trimming parameters
 * stored in the BMP390 during production and converts them into
 * floating-point compensation coefficients.
 *
 * The decoded coefficients are stored internally and later used by
 * the temperature and pressure compensation algorithms.
 *
 * @retval BMP390_RET_TYPE_SUCCESS  NVM parameters were read and decoded successfully.
 * @retval BMP390_RET_TYPE_FAILURE  Communication with the device failed.
 *
 * @note This function must be called once after power up and before
 *       performing any temperature or pressure compensation.
 *
 * @see section 3.11.1 for NVM list and 8.4 for calibration
 */
BMP390_RET_TYPE BMP390::ReadNVM(void)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t NvmPar[21];

	ret = read(hInterface, chipAddress, bmp390::REG_NVM_PAR_START, NvmPar, 21);

	if (ret == BMP390_RET_TYPE_SUCCESS)
	{
		uint16_t u16 = static_cast<uint16_t>(static_cast<uint16_t>(NvmPar[1]) << 8 | NvmPar[0]);
		ParT1 = static_cast<double>(u16) / 0.00390625f;

		u16 = static_cast<uint16_t>(static_cast<uint16_t>(NvmPar[3]) << 8 | NvmPar[2]);
		ParT2 = static_cast<double>(u16) / 1073741824.0f;

		ParT3 = static_cast<double>(static_cast<int8_t>(NvmPar[4])) / 281474976710656.0f;

		int16_t s16 = static_cast<int16_t>(static_cast<uint16_t>(NvmPar[6]) << 8 | NvmPar[5]);
		ParP1 = static_cast<double>(s16 - 16384) / 1048576.0f;

		s16 = static_cast<int16_t>(static_cast<uint16_t>(NvmPar[8]) << 8 | NvmPar[7]);
		ParP2 = static_cast<double>(s16 - 16384) / 536870912.0f;

		ParP3 = static_cast<double>(static_cast<int8_t>(NvmPar[9])) / 4294967296.0f;
		ParP4 = static_cast<double>(static_cast<int8_t>(NvmPar[10])) / 137438953472.0f;

		u16 = static_cast<uint16_t>(static_cast<uint16_t>(NvmPar[12]) << 8 | NvmPar[11]);
		ParP5 = static_cast<double>(u16) / 0.125f;

		u16 = static_cast<uint16_t>(static_cast<uint16_t>(NvmPar[14]) << 8 | NvmPar[13]);
		ParP6 = static_cast<double>(u16) / 64.0f;

		ParP7 = static_cast<double>(static_cast<int8_t>(NvmPar[15])) / 256.0f;
		ParP8 = static_cast<double>(static_cast<int8_t>(NvmPar[16])) / 32768.0f;

		s16 = static_cast<int16_t>(static_cast<uint16_t>(NvmPar[18]) << 8 | NvmPar[17]);
		ParP9 = static_cast<double>(s16) / 281474976710656.0f;

		ParP10 = static_cast<double>(static_cast<int8_t>(NvmPar[19])) / 281474976710656.0f;
		ParP11 = static_cast<double>(static_cast<int8_t>(NvmPar[20])) / 36893488147419103232.0f;
	}
	return ret;
}

/**
 * @brief Compensate raw (uncompensated) temperature reading.
 *
 * This function converts the raw 24-bit uncompensated temperature value
 * read from the BMP390 sensor into a compensated temperature value using
 * calibration parameters stored in NVM.
 *
 * The implementation follows the temperature compensation algorithm
 * described in the BMP390 datasheet (Section 8.5).
 *
 * @param[in] UncompTemp 24-bit uncompensated temperature value from the sensor.
 *
 * @return Linearized temperature value (TempLin) in degrees Celsius.
 *
 * @note The calibration parameters ParT1, ParT2, and ParT3 must be
 *       successfully read from the sensor using ReadNVM() before calling this function.
 *
 * @see section 8.5 in the datasheet
 */
double BMP390::CompensateTemperature(uint32_t UncompTemp)
{
	double PartialData1 = static_cast<double>(UncompTemp) - ParT1;
	double PartialData2 = PartialData1 * ParT2;
	/* Update the compensated temperature in structure since this is
	* needed for pressure calculation */
	double TempLin = PartialData2 + (PartialData1 * PartialData1) * ParT3;
	/* Returns compensated temperature */
	return TempLin;
}

/**
 * @brief Read and return the compensated temperature from the BMP390 sensor.
 *
 * This function reads the raw temperature data registers from the BMP390,
 * and applies temperature compensation using calibration data.
 *
 * @param[out] Temperature Reference to a float variable where the
 *                         compensated temperature (in °C) will be stored.
 *
 * @retval BMP390_RET_TYPE_SUCCESS on successful read and compensation,
 * @retval BMP390_RET_TYPE_FAILURE otherwise.
 *
 * @note Calibration parameters must be initialized by calling ReadNVM()
 *       before using this function.
 *
 * @note Use GetTemperatureAndPressure(float &Temperature, float &Pressure), if
 * 		 pressure measurement is enabled.
 */
BMP390_RET_TYPE BMP390::GetTemperature(double &Temperature)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;
	uint8_t RawData[3];
	ret = read(hInterface, chipAddress, bmp390::REG_DATA_3, RawData, 3);

	if (ret == BMP390_RET_TYPE_SUCCESS)
	{
		uint32_t UncompTemp = static_cast<uint32_t>(RawData[2]) << 16 | static_cast<uint32_t>(RawData[1]) << 8 | static_cast<uint32_t>(RawData[0]);

		Temperature = CompensateTemperature(UncompTemp);
	}
	return ret;
}

/**
 * @brief Compensate raw (uncompensated) pressure reading.
 *
 * This function converts the raw 24-bit uncompensated pressure value
 * read from the BMP390 sensor into a compensated pressure value using
 * calibration parameters stored in NVM.
 *
 * The pressure compensation algorithm depends on the linearized
 * temperature value (TempLin) and follows the procedure described
 * in the BMP390 datasheet (Section 8.6).
 *
 * @param[in] UncompPress 24-bit uncompensated pressure value from the sensor.
 * @param[in] TempLin    Linearized temperature value obtained from
 *                       CompensateTemperature().
 *
 * @return Compensated pressure value in Pascals (Pa).
 *
 * @note Calibration parameters ParP1 through ParP11 must be successfully
 *       read from the sensor NVM using ReadNVM() before calling this function.
 *
 * @note The TempLin parameter must correspond to the same measurement
 *       cycle as the pressure reading for accurate compensation.
 *
 * @see section 8.6 in the datasheet
 */
double BMP390::CompensatePressure(uint32_t UncompPress, double TempLin)
{
	/* Variable to store the compensated pressure */
	double CompPress;
	/* Temporary variables used for compensation */
	double PartialData1;
	double PartialData2;
	double PartialData3;
	double PartialData4;
	double PartialOut1;
	double PartialOut2;
	/* Calibration data */
	PartialData1 = ParP6 * TempLin;
	PartialData2 = ParP7 * (TempLin * TempLin);
	PartialData3 = ParP8 * (TempLin * TempLin * TempLin);
	PartialOut1 = ParP5 + PartialData1 + PartialData2 + PartialData3;
	PartialData1 = ParP2 * TempLin;
	PartialData2 = ParP3 * (TempLin * TempLin);
	PartialData3 = ParP4 * (TempLin * TempLin * TempLin);
	PartialOut2 = static_cast<double>(UncompPress) * (ParP1 + PartialData1 + PartialData2 + PartialData3);
	PartialData1 = static_cast<double>(UncompPress) * static_cast<double>(UncompPress);
	PartialData2 = ParP9 + ParP10 * TempLin;
	PartialData3 = PartialData1 * PartialData2;
	PartialData4 = PartialData3 + (static_cast<double>(UncompPress) * static_cast<double>(UncompPress) * static_cast<double>(UncompPress)) * ParP11;
	CompPress = PartialOut1 + PartialOut2 + PartialData4;

	return CompPress;
}

/**
 * @brief Read and return compensated temperature and pressure from the BMP390.
 *
 * This function reads both raw pressure and temperature data from the
 * BMP390 data registers in a single burst read, reconstructs the
 * 24-bit uncompensated values, and applies compensation algorithms
 * using calibration data stored in NVM.
 *
 * Temperature compensation is performed first to obtain the linearized
 * temperature value (TempLin), which is then used for pressure
 * compensation as required by the BMP390 compensation model.
 *
 * @param[out] Temperature Reference to a float variable where the
 *                         compensated temperature (in °C) will be stored.
 * @param[out] Pressure    Reference to a float variable where the
 *                         compensated pressure (in Pa) will be stored.
 *
 * @return BMP390_RET_TYPE_SUCCESS on successful read and compensation,
 *         BMP390_RET_TYPE_FAILURE otherwise.
 *
 * @note Calibration parameters must be initialized by calling ReadNVM()
 *       before using this function.
 *
 * @note This function ensures temperature and pressure values originate
 *       from the same measurement cycle, improving consistency.
 */
BMP390_RET_TYPE BMP390::GetTemperatureAndPressure(double &Temperature, double &Pressure)
{
	BMP390_RET_TYPE ret = BMP390_RET_TYPE_FAILURE;

	uint8_t RawData[6];
	ret = read(hInterface, chipAddress, bmp390::REG_DATA_0, RawData, 6);

	if (ret == BMP390_RET_TYPE_SUCCESS)
	{
		uint32_t UncompPress = static_cast<uint32_t>(RawData[2]) << 16 | static_cast<uint32_t>(RawData[1]) << 8 | static_cast<uint32_t>(RawData[0]);
		uint32_t UncompTemp = static_cast<uint32_t>(RawData[5]) << 16 | static_cast<uint32_t>(RawData[4]) << 8 | static_cast<uint32_t>(RawData[3]);

		Temperature = CompensateTemperature(UncompTemp);
		Pressure = CompensatePressure(UncompPress, Temperature);
	}
	return ret;
}
