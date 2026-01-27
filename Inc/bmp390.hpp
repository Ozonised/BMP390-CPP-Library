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
		BMP390_RET_TYPE GetStatus(uint8_t &status);
	public:
		BMP390(void *hInterface, BMP390_ReadFuncPtr read, BMP390_WriteFuncPtr write, BMP390_DelayMsFuncPtr delay) :
			hInterface(hInterface), read(read), write(write), delayMs(delay)
		{
			chipAddress = 0;
		}

		void Init(bool CSBPinState, bool SDOPinState);
		BMP390_RET_TYPE IsPresent(void);
		BMP390_RET_TYPE SetPowerMode(bmp390::PowerMode mode);
		BMP390_RET_TYPE TogglePressureMeasurement(bool n);
		BMP390_RET_TYPE ToggleTemperatureMeasurement(bool n);
		BMP390_RET_TYPE SetPressureOversampling(bmp390::TempPressOversampling osrp);
		BMP390_RET_TYPE SetTemperatureOversampling(bmp390::TempPressOversampling osrt);
		BMP390_RET_TYPE SetOutputDataRate(bmp390::TempPressODR odr);
		BMP390_RET_TYPE IsBusy(void);
		BMP390_RET_TYPE GetDrdySource(bmp390::DrdySource &src);
		BMP390_RET_TYPE SetIIRFilterCoefficient(bmp390::IIRFilterCoefficient coef);
};
#endif /* BMP390_HPP_ */
