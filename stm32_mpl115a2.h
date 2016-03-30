//*****************************************************************************
//
//! \file stm_mpl115a2.h
//! \brief Driver for MPL115A2 Sensor.
//! \version 1.0.0.0
//! \date 12/31/2012
//! \author Nemon
//! \copy
//!
//! Copyright (c)  2011, CooCoX
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

#ifndef STM32_MPL115A2_H
#define STM32_MPL115A2_H


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C" {
#endif
#include "tm_stm32_gpio.h"
#include "tm_stm32_i2c.h"
#include "defines.h"

//*****************************************************************************
//
//! \addtogroup CoX_Shield_Lib
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Sensor
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Pressure_Sensor
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup MPL115A2
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup MPL115A2_Exported_APIs  MPL115A2 Driver APIs
//! \brief API Refrence of MPL115A2 Driver.
//! @{
//
//*****************************************************************************

/**
 * @brief  MPL115A2 working struct
 * @note   filled by user before chip initialization
 */
typedef struct {
	I2C_TypeDef*	I2Cx;                 /*!< I2Cx port to be used for I/O functions */

#ifdef MPL115A2_USE_RST
	GPIO_TypeDef*	GPIOx_RST;
	uint16_t		GPIO_Pin_RST;         /*!< GPIO RST Pin to be used for I/O functions */
#endif // MPL115A2_USE_RST

#ifdef MPL115A2_USE_SHDN
	GPIO_TypeDef*	GPIOx_SHDN;
	uint16_t		GPIO_Pin_SHDN;        /*!< GPIO SHDN Pin to be used for I/O functions */
#endif // MPL115A2_USE_SHDN

} PW_MPL115A2_t;

//*****************************************************************************
//
//! \brief make sensor init.
//!
//! make sensor init.
//!
//! \return None
//
//*****************************************************************************
extern void PW_MPL115A2_Init( const PW_MPL115A2_t *mpl );

//*****************************************************************************
//
//! \brief check if chip is connected.
//!
//!
//! \return 1 - is connected, 0 - not connected
//
//*****************************************************************************
uint8_t PW_MPL115A2_isDeviceConneted( const PW_MPL115A2_t *mpl );

//*****************************************************************************
//
//! \brief get current pressure value.
//!
//! get current pressure value.
//!
//! \return int[16,4] the pressure value
//
//*****************************************************************************
extern int PW_MPL115A2_GetPressure( const PW_MPL115A2_t *mpl, signed short *target );

//*****************************************************************************
//
//! \brief translate int[16,4] pressure value to float.
//!
//! \param uiValue is pressure value of int[16,4].
//!
//! translate int[16,4] pressure value to float.
//!
//! \return float pressure value
//
//*****************************************************************************
#define MPL115A2TranslateValue(uiValue) \
                                ((float)(uiValue))/(1<<4)


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // STM32_MPL115A2_H
