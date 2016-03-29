#ifndef PW_STM32_AT24xx_H
#define PW_STM32_AT24xx_H
#include <stdint.h>
#include <tm_stm32_i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initializes I2C inteface to communication with AT24xx
 * @note   This function also check if is present or not
 * @param  I2Cx: Pointer to I2Cx port you will use for initialization
 * @retval None
 */
void PW_AT24xx_Init(I2C_TypeDef *I2Cx);

/**
 * @brief  Checks if device is connected to I2C bus
 * @param  *I2Cx: I2C used
 * @retval Device status:
 *            - 0: Device is not connected
 *            - > 0: Device is connected
 */
uint8_t PW_AT24xx_isDeviceConneted(I2C_TypeDef *I2Cx);

/**
 * @brief  Writes data to device
 * @param  *I2Cx: I2C used
 * @param  addr: address range (0 ... 8 * 255)
 * @param  data: pointer to data which need to be write
 * @param  size: size of data
 * @retval None
 */
void PW_AT24xx_Write(I2C_TypeDef *I2Cx, uint16_t addr, const uint8_t *buf, int size);

/**
 * @brief  Reads data from device
 * @param  *I2Cx: I2C used
 * @param  addr: address range (0 ... 8 * 255)
 * @param  buf: pointer to data which need to be read
 * @param  size: size of read data
 * @retval None
 */
void PW_AT24xx_Read(I2C_TypeDef *I2Cx, uint16_t addr, uint8_t *buf, int size);

#ifdef __cplusplus
}
#endif

#endif  // PW_STM32_AT24xx_H
