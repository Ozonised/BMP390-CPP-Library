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

class BMP390 {
	private:
		void *hInterface;
		BMP390_ReadFuncPtr read;
		BMP390_WriteFuncPtr write;

		uint8_t chipAddress;

		double ParT1;
		double ParT2;
		double ParT3;

		double ParP1;
		double ParP2;
		double ParP3;
		double ParP4;
		double ParP5;
		double ParP6;
		double ParP7;
		double ParP8;
		double ParP9;
		double ParP10;
		double ParP11;

		BMP390_RET_TYPE GetStatus(uint8_t &status);
		double CompensateTemperature(uint32_t UncompTemp);
		double CompensatePressure(uint32_t UncompPress, double TempLin);
	public:
		BMP390(void *hInterface, BMP390_ReadFuncPtr read, BMP390_WriteFuncPtr write) :
			hInterface(hInterface), read(read), write(write)
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
		BMP390_RET_TYPE ToggleTemperatureAndPressureMeasurement(bool TempEn, bool PressEn);
		BMP390_RET_TYPE GetDrdySource(bmp390::DrdySource &src);
		BMP390_RET_TYPE GetTemperature(double &Temperature);
		BMP390_RET_TYPE GetTemperatureAndPressure(double &Temperature, double &Pressure);
		float GetAltitude(double &Pressure);
};
#endif /* BMP390_HPP_ */
