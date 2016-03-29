#include "stm32_PCF8574.h"
#include "os_compat.h"
#include "config.h"

//! PCF8574 Salve Address 7-bit mode
//! \b NOTE: R/W bit is EXCLUDE from this address
#define PCF8574_I2C_ADDR       ((0x20)<<1)

void PW_PCF8574_Init( I2C_TypeDef *I2Cx ){
	TM_I2C_Init( I2Cx, TM_I2C_PinsPack_1, 100000 );

	if( PW_PCF8574_isDeviceConneted( I2Cx ) )
		_D(("I: PCF8574 i2c expander chip on board\n"));
}

uint8_t PW_PCF8574_isDeviceConneted( I2C_TypeDef *I2Cx ){
	return TM_I2C_IsDeviceConnected( I2Cx, PCF8574_I2C_ADDR );
}

void PW_PCF8574_WritePort( I2C_TypeDef *I2Cx, uint8_t pins ){
	TM_I2C_WriteMulti( I2Cx, PCF8574_I2C_ADDR, 0, (uint8_t *)&pins, 1 );
}

void PW_PCF8574_ReadPort( I2C_TypeDef *I2Cx, uint8_t *out ){
	*out = TM_I2C_ReadNoRegister( I2Cx, PCF8574_I2C_ADDR );
	TM_I2C_Stop( I2Cx );
}
void PW_PCF8574_WriteBits( I2C_TypeDef *I2Cx, uint8_t mask, uint8_t bits ){
	uint8_t port;

	PW_PCF8574_ReadPort( I2Cx, &port );

    bits &= mask;
    port &= ~mask;
    port |= bits;

    PW_PCF8574_WritePort( I2Cx, port );
}

void PW_PCF8574_ReadBits( I2C_TypeDef *I2Cx, uint8_t mask, uint8_t *bits ){
	uint8_t port = 0;

	PW_PCF8574_ReadPort( I2Cx, &port );
	*bits &= mask;
}
