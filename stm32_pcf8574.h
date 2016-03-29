#ifndef STM32_PCF8574_H
#define STM32_PCF8574_H
#include <stdint.h>
#include "tm_stm32_i2c.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief  Initializes I2C inteface to communication with PCF8574
 * @note   This function also check if chip is present or not
 * @param  I2Cx: Pointer to I2Cx port you will use for initialization
 * @retval None
 */
void PW_PCF8574_Init( I2C_TypeDef *I2Cx );

/**
 * @brief  Checks if device is connected to I2C bus
 * @param  *I2Cx: I2C used
 * @retval Device status:
 *            - 0: Device is not connected
 *            - > 0: Device is connected
 */
uint8_t PW_PCF8574_isDeviceConneted( I2C_TypeDef *I2Cx );

/**
 * @brief  Writes port configuration to device
 * @param  *I2Cx: I2C used
 * @param  port: pins as bitfield
 * @retval None
 */
void PW_PCF8574_WritePort( I2C_TypeDef *I2Cx, uint8_t port );

/**
 * @brief  Reads port configuration from device
 * @param  *I2Cx: I2C used
 * @param  *port: pins as bitfield
 * @retval None
 */
void PW_PCF8574_ReadPort( I2C_TypeDef *I2Cx, uint8_t *port );

/**
 * @brief Write the special bit(s).
//! if you want to set PIO5 to 1, PIO3 to 0.
//! then, bit mask is (1<<5) | (1<<3)  that is 0x28
//!       bit value is (1<<5) | (0<<3) that is 0x20
//! so, you can call WriteBits like below:
//!  PCF8574_WriteBits(0x28, 0x20);
 * @param  *I2Cx: I2C used
 * @param  bits: pins as bit fields
 * @retval None
 */
void PW_PCF8574_WriteBits( I2C_TypeDef *I2Cx, uint8_t mask, uint8_t bits );

/**
 * @brief  Read the special bit(s) value.
//! if you want to check the PIO5 value.
//! then, bit mask is (1<<5) that is 0x20
//! so, you can call WriteBits like below:
//!  retv = PCF8574_ReadBits(0x20, &Buf);
 * @param  *I2Cx: I2C used
 * @param  bits: pins as bit fields
 * @retval None
 */
void PW_PCF8574_ReadBits( I2C_TypeDef *I2Cx, uint8_t mask, uint8_t *bits );

#ifdef __cplusplus
}
#endif

#endif  // STM32_PCF8574_H
