//*****************************************************************************
//
//! \file stm32_mpl115a2.c
//! \brief Control for MPL115A2 sensor.
//! \version 1.0.0.0
//! \date 12/31/2012
//! \author Nemon
//! \copy
//!
//! Copyright (c)  2011, CooCox
//! All rights reserved.
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions
//! are met:
//!
//!     * Redistributions of source code must retain the above copyright
//! notice, this list of conditions and the following disclaimer.
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution.
//!     * Neither the name of the <ORGANIZATION> nor the names of its
//! contributors may be used to endorse or promote products derived
//! from this software without specific prior written permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//! THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************
#include "tm_stm32_delay.h"
#include "stm32_mpl115a2.h"

#define MPL115A2_I2C_ADDRESS    ((0x60)<<1)

#define MPL115A2_CMD_COEFICNTS  0x04
#define MPL115A2_CMD_CONVERSION 0x12
#define MPL115A2_CMD_RESULT     0x00


//*****************************************************************************
//
//! \brief ONLY FOR driver to calculate Pressure.
//!
//! \param none.
//!
//! calculate Pressure. from Freescale Note and write by: John B. Young
//!
//! \return int[16,4] the pressure value
//
//*****************************************************************************
static signed short PW_MPL115A_calculatePressure(unsigned short Padc, unsigned short Tadc,
                signed short a0, signed short b1, signed short b2, signed short c12) {
	signed long Pressure;
	signed long c12x2, a1, a1x1, y1, a2x2, PComp;
	Padc >>= 6;
	Tadc >>= 6;
	c12x2 = (((signed long)c12) * Tadc) >> 11;
	a1    = (signed long)b1 + c12x2;
	a1x1  = a1 * Padc;
	y1    = (((signed long)a0) << 10) + a1x1;
	a2x2  = (((signed long)b2) * Tadc) >> 1;
	PComp = (y1 + a2x2) >> 9;
	//return (signed short)PComp;
	Pressure = ((((signed long)PComp) * 1041) >> 14) + 800;
	return (unsigned short)Pressure;
}


//*****************************************************************************
//
//! \brief make sensor MPL115A2 init.
//!
//! make sensor MPL115A2 init.
//!
//! \return None
//
//*****************************************************************************
void PW_MPL115A2_Init( const PW_MPL115A2_t *mpl ){
#ifdef MPL115A2_USE_RST
	TM_GPIO_Init( mpl->GPIOx_RST, mpl->GPIO_Pin_RST, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Low );
	TM_GPIO_SetPinHigh( mpl->GPIOx_RST, mpl->GPIO_Pin_RST );
#endif // MPL115A2_USE_RST

#ifdef MPL115A2_USE_SHDN
	TM_GPIO_Init( mpl->GPIOx_SHDN, mpl->GPIO_Pin_SHDN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Low );
	TM_GPIO_SetPinHigh( mpl->GPIOx_SHDN, mpl->GPIO_Pin_SHDN );
#endif // MPL115A2_USE_SHDN
	TM_I2C_Init( mpl->I2Cx, TM_I2C_PinsPack_1, 100000 );
}

//*****************************************************************************
//
//! \brief check if chip is connected.
//!
//!
//! \return 1 - is connected, 0 - not connected
//
//*****************************************************************************
uint8_t PW_MPL115A2_isDeviceConneted( const PW_MPL115A2_t *mpl ){
	uint8_t connected = 0;

#ifdef MPL115A2_USE_RST
	TM_GPIO_SetPinHigh( mpl->GPIOx_RST, mpl->GPIO_Pin_RST );
#endif // MPL115A2_USE_RST

#ifdef MPL115A2_USE_SHDN
	TM_GPIO_SetPinHigh( mpl->GPIOx_SHDN, mpl->GPIO_Pin_SHDN );
	Delay(5000);
#endif // MPL115A2_USE_SHDN

	connected = TM_I2C_IsDeviceConnected( mpl->I2Cx, MPL115A2_I2C_ADDRESS );

#ifdef MPL115A2_USE_RST
	TM_GPIO_SetPinLow( mpl->GPIOx_RST, mpl->GPIO_Pin_RST );
#endif // MPL115A2_USE_RST

#ifdef MPL115A2_USE_SHDN
	TM_GPIO_SetPinLow( mpl->GPIOx_SHDN, mpl->GPIO_Pin_SHDN );
#endif // MPL115A2_USE_SHDN
	return connected;
}

//*****************************************************************************
//
//! \brief get current pressure value.
//!
//! get current pressure value.
//!
//! \return the pressure float value
//
//*****************************************************************************
int PW_MPL115A2_GetPressure( const PW_MPL115A2_t *mpl, signed short *target ){
	unsigned char ucVar[8];
	unsigned char ucRow[8];

#ifdef MPL115A2_USE_RST
	TM_GPIO_SetPinHigh( mpl->GPIOx_RST, mpl->GPIO_Pin_RST );
#endif // MPL115A2_USE_RST

#ifdef MPL115A2_USE_SHDN
	TM_GPIO_SetPinHigh( mpl->GPIOx_SHDN, mpl->GPIO_Pin_SHDN );
	Delay(5000);
#endif // MPL115A2_USE_SHDN

	TM_I2C_ReadMulti( mpl->I2Cx, MPL115A2_I2C_ADDRESS, MPL115A2_CMD_COEFICNTS, ucVar, sizeof(ucVar) );

	TM_I2C_WriteNoRegister( mpl->I2Cx, MPL115A2_I2C_ADDRESS, MPL115A2_CMD_CONVERSION );

	Delay(800000);// 7000);

	TM_I2C_ReadMulti( mpl->I2Cx, MPL115A2_I2C_ADDRESS, MPL115A2_CMD_COEFICNTS, ucRow, sizeof(ucRow) );

#ifdef MPL115A2_USE_RST
	TM_GPIO_SetPinLow( mpl->GPIOx_RST, mpl->GPIO_Pin_RST );
#endif // MPL115A2_USE_RST

#ifdef MPL115A2_USE_SHDN
	TM_GPIO_SetPinLow( mpl->GPIOx_SHDN, mpl->GPIO_Pin_SHDN );
	Delay(5000);
#endif // MPL115A2_USE_SHDN

	*target = PW_MPL115A_calculatePressure(
			((unsigned short)ucRow[0]<<8)|((unsigned short)ucRow[1]),
			((unsigned short)ucRow[2]<<8)|((unsigned short)ucRow[3]),
			((unsigned short)ucVar[0]<<8)|((unsigned short)ucVar[1]),
			((unsigned short)ucVar[2]<<8)|((unsigned short)ucVar[3]),
			((unsigned short)ucVar[4]<<8)|((unsigned short)ucVar[5]),
			((unsigned short)ucVar[6]<<8)|((unsigned short)ucVar[7])
			);
	return 1;
}

