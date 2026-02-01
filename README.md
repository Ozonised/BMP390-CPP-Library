# A platfrom independent BMP390 CPP library
This is a platfrom independed CPP library for the BMP390 digital barometeric pressure sensor by Bosch.

## Supported Features:
- Checking for device presence.
- Reading NVM and compensating the readings.
- Setting Power mode.
- Setting pressure and temperature oversampling.
- Setting output data rate.
- Configuring IIR filter.
- Setting up Interrupts.
- Reading compensated temperature and pressure data.
- Get estimated altitude above mean sea leve.

### Note:
Fifo mode not supported.

## Do I use it with my favourite microcontroller?
To use it with your favourite microcontroller, you simply need to define the functions for read and write with the following parameters:
1. ```BMP390_RET_TYPE read(void *hInterface, uint8_t chipAddr, uint8_t reg, uint8_t *buf, uint8_t len)``` function to read data.
2. ```BMP390_RET_TYPE BMP390_I2CWrite(void *hInterface, uint8_t chipAddr, uint8_t reg, uint8_t *buf, uint8_t len)``` function to write data.

**Don't worry, I will show you an example for porting to STM32 HAL framework.**

### Porting to STM32 using ST HAL framework:
To use use the library with ST HAL's framework, include the files from [Inc](/Inc) and [Src](/Src) into your project
Now in the main file, create the definitions for the read and write functions:
```CPP
BMP390_RET_TYPE BMP390_I2CRead(void *hInterface, uint8_t chipAddr, uint8_t reg, uint8_t *buf, uint8_t len)
{
	BMP390_RET_TYPE ret = HAL_ERROR;
	ret = HAL_I2C_Mem_Read(static_cast<I2C_HandleTypeDef*>(hInterface), static_cast<uint16_t>(chipAddr << 1),
			static_cast<uint16_t>(reg), 1U, buf, static_cast<uint16_t>(len), 2U);
	if (ret == HAL_TIMEOUT)
		ret = HAL_ERROR;
	return ret;
}

BMP390_RET_TYPE BMP390_I2CWrite(void *hInterface, uint8_t chipAddr, uint8_t reg, uint8_t *buf, uint8_t len)
{
	BMP390_RET_TYPE ret = HAL_ERROR;
	ret = HAL_I2C_Mem_Write(static_cast<I2C_HandleTypeDef*>(hInterface), static_cast<uint16_t>(chipAddr << 1),
			static_cast<uint16_t>(reg), 1U, buf, static_cast<uint16_t>(len), 2U);
	if (ret == HAL_TIMEOUT)
		ret = HAL_ERROR;
	return ret;
}
```
Optional, you can redefine the return types, before including the [bmp390.hpp](/Inc/bmp390.hpp) file:
```CPP
#define BMP390_RET_TYPE HAL_StatusTypeDef
#define BMP390_RET_TYPE_SUCCESS HAL_OK
#define BMP390_RET_TYPE_FAILURE HAL_ERROR
#define BMP390_RET_TYPE_BUSY HAL_BUSY
#include "bmp390.hpp"
```
### Reference code:
```CPP

#define BMP390_RET_TYPE HAL_StatusTypeDef
#define BMP390_RET_TYPE_SUCCESS HAL_OK
#define BMP390_RET_TYPE_FAILURE HAL_ERROR
#define BMP390_RET_TYPE_BUSY HAL_BUSY
#include "bmp390.hpp"
  .
  .
  .
extern I2C_HandleTypeDef hi2c2;
  .
  .
  .
BMP390_RET_TYPE BMP390_I2CRead(void *hInterface, uint8_t chipAddr, uint8_t reg, uint8_t *buf, uint8_t len);
BMP390_RET_TYPE BMP390_I2CWrite(void *hInterface, uint8_t chipAddr, uint8_t reg, uint8_t *buf, uint8_t len);
  .
  .
  .
BMP390 baro(static_cast<void*>(&hi2c2), BMP390_I2CRead, BMP390_I2CWrite);

int main(void)
{
  .
  .
  .
  // MCU initisation
  .
  .
  .
	baro.Init(1, 0);
  delayMilliSecond(20);

	ret = baro.IsPresent();

	while (ret != HAL_OK)
		;

	ret = baro.ReadNVM();
	if (ret == HAL_OK)
	{
		ret = baro.SetPowerMode(bmp390::PowerMode::Normal);

		do
		{
			ret = baro.IsBusy();
		} while (ret == HAL_BUSY || ret == HAL_ERROR);

		ret = baro.SetPressureOversampling(bmp390::TempPressOversampling::x8);

		do
		{
			ret = baro.IsBusy();
		} while (ret == HAL_BUSY || ret == HAL_ERROR);

		ret = baro.SetTemperatureOversampling(bmp390::TempPressOversampling::x1);

		do
		{
			ret = baro.IsBusy();
		} while (ret == HAL_BUSY || ret == HAL_ERROR);

		ret = baro.SetOutputDataRate(bmp390::TempPressODR::ODR_50Hz);

		do
		{
			ret = baro.IsBusy();
		} while (ret == HAL_BUSY || ret == HAL_ERROR);

		ret = baro.SetIIRFilterCoefficient(bmp390::IIRFilterCoefficient::coef3);

		do
		{
			ret = baro.IsBusy();
		} while (ret == HAL_BUSY || ret == HAL_ERROR);

		ret = baro.SetInterruptSource(1, 0, 0);

		do
		{
			ret = baro.IsBusy();
		} while (ret == HAL_BUSY || ret == HAL_ERROR);

		ret = baro.ToggleTemperatureAndPressureMeasurement(1, 1);

	}

	GPIO_PinState CurrentIntPin = GPIO_PIN_RESET, PrevIntPin = GPIO_PIN_RESET;
	float altitude = 0;
	while (1)
	{
		CurrentIntPin = HAL_GPIO_ReadPin(BMP390_INT_GPIO_Port, BMP390_INT_Pin);
		if (CurrentIntPin == GPIO_PIN_SET && PrevIntPin == GPIO_PIN_RESET)
		{
			baro.GetTemperatureAndPressure(Temperature, Pressure);
			altitude = baro.GetAltitude(Pressure);
		}
		PrevIntPin = CurrentIntPin;
	}
}
```
