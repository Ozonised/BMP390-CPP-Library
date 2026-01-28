/*
 * bmp390.hpp
 *
 *  Created on: Jan 12, 2026
 *      Author: farhan
 */

#ifndef BMP390_HPP_
#define BMP390_HPP_

#include "bmp390-port.hpp"
#include "bmp390-reg.hpp"

typedef BMP390_RET_TYPE (*BMP390_ReadFuncPtr)(void *hInterface, uint8_t chipAddr, uint8_t reg, uint8_t *buf, uint8_t len);
typedef BMP390_RET_TYPE (*BMP390_WriteFuncPtr)(void *hInterface, uint8_t chipAddr, uint8_t reg, uint8_t *buf, uint8_t len);
typedef BMP390_RET_TYPE (*BMP390_DelayMsFuncPtr)(void *hInterface, uint32_t delayMs);

class BMP390 {
	private:
		void *hInterface;
		BMP390_ReadFuncPtr read;
		BMP390_WriteFuncPtr write;
		BMP390_DelayMsFuncPtr delayMs;

		uint8_t chipAddress;

		float ParT1;
		float ParT2;
		float ParT3;

		float ParP1;
		float ParP2;
		float ParP3;
		float ParP4;
		float ParP5;
		float ParP6;
		float ParP7;
		float ParP8;
		float ParP9;
		float ParP10;
		float ParP11;

		BMP390_RET_TYPE GetStatus(uint8_t &status);
		float CompensateTemperature(uint32_t UncompTemp);
		float CompensatePressure(uint32_t UncompPress, float TempLin);
	public:
		BMP390(void *hInterface, BMP390_ReadFuncPtr read, BMP390_WriteFuncPtr write, BMP390_DelayMsFuncPtr delay) :
			hInterface(hInterface), read(read), write(write), delayMs(delay)
		{
			chipAddress = 0;
			ParT1 = 0;
			ParT2 = 0;
			ParT3 = 0;
			ParP1 = 0;
			ParP2 = 0;
			ParP3 = 0;
			ParP4 = 0;
			ParP5 = 0;
			ParP6 = 0;
			ParP7 = 0;
			ParP8 = 0;
			ParP9 = 0;
			ParP10 = 0;
			ParP11 = 0;
		}

		void Init(bool CSBPinState, bool SDOPinState);
		BMP390_RET_TYPE IsPresent(void);
		BMP390_RET_TYPE ReadNVM(void);
		BMP390_RET_TYPE IsBusy(void);
		BMP390_RET_TYPE SetPowerMode(bmp390::PowerMode mode);
		BMP390_RET_TYPE SetPressureOversampling(bmp390::TempPressOversampling osrp);
		BMP390_RET_TYPE SetTemperatureOversampling(bmp390::TempPressOversampling osrt);
		BMP390_RET_TYPE SetOutputDataRate(bmp390::TempPressODR odr);
		BMP390_RET_TYPE SetIIRFilterCoefficient(bmp390::IIRFilterCoefficient coef);
		BMP390_RET_TYPE ConfigInterruptPin(bool od, bool level, bool latch);
		BMP390_RET_TYPE SetInterruptSource(bool TempPress, bool FifoFull, bool FifoWaterMark);
		BMP390_RET_TYPE GetInterruptSource(bmp390::InterruptSource &src);
		BMP390_RET_TYPE TogglePressureMeasurement(bool n);
		BMP390_RET_TYPE ToggleTemperatureMeasurement(bool n);
		BMP390_RET_TYPE GetDrdySource(bmp390::DrdySource &src);
		BMP390_RET_TYPE GetTemperature(float &Temperature);
		BMP390_RET_TYPE GetTemperatureAndPressure(float &Temperature, float &Pressure);
};
#endif /* BMP390_HPP_ */
