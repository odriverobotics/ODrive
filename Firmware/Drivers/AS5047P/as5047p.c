// the includes

#include "cmsis_os.h"

// drivers
#include "as5047p.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

uint16_t AS5047D_readPosition(AS5047D_Handle handle){
	CUI_readSpi(handle, (uint16_t)ANGLECOM);

	uint16_t data = 0;

	data = CUI_readSpi(handle, (uint16_t)ANGLECOM);

	return data;

}


uint16_t AS5047D_readSpi(AS5047D_Handle handle, uint16_t command){
	HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);

	uint8_t buff = 0;

	HAL_SPI_Transmit(handle->spiHandle, (uint16_t*)(&command), (uint8_t*)(&buff), 1, 500);

	HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);

	return buff;
}

