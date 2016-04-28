#ifndef UG2864_LL_H
#define UG2864_LL_H 100

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************/
/**************************************************************************/
/**************************************************************************/
/*                                                                        */
/*    Edit file name to ug2864_ll.h and edit values for your platform     */
/*                                                                        */
/**************************************************************************/
/**************************************************************************/
/**************************************************************************/

/**
 * @defgroup UG2864_LL
 * @brief    Low level, platform dependent, part for communicate with ENC module and platform.
 * @{
 *
 * This is a low-level part of ENC module library.
 *
 * It provides communication between ENC module and platform. There are some function, which needs to be implemented by user and with care.
 *
 * \par SPI configuration
 *
 * ENC28J60 module works with SPI communication with device.
 *
 * - \ref UG2864_LL_SPIInit: Function, which is called when SPI should be initialized
 * - \ref ENC8J60_LL_SPIRxTx: Function, which is called when data should be sent to ENC28J60 device
 *
 *
\code
//Code below show example procedure and does need to be directly for your platform.
//You have to make sure to properly configure UART and RX interrupt for it.

//USART Initialization function, which is called from ESP stack
uint8_t UG2864_LL_SPIInit(void) {

    //Init SPI peripheral
	TM_SPI_InitFull( UG2864_SPI, TM_SPI_PinsPack_1, TM_SPI_Mode_0, SPI_BAUDRATEPRESCALER_2, SPI_MODE_MASTER, SPI_FIRSTBIT_MSB );
	TM_SPI_SetDataSize( UG2864_SPI, TM_SPI_DataSize_8b );

    //Init CS pin
    UG2864_CS_INIT;

    //Return 0 = Successful
    return 0;
}

//SPI send/receive function, which is called from ESP stack
uint8_t UG2864_LL_SPIRxTx(uint8_t txbyte){
    //send/receive data via SPI
    TM_SPI_Send( UG2864_SPI, txbyte );
    //Return RX byte
    return TM_SPI_Send( UG2864_SPI, 0xff );
}

\endcode
 * 
 * \par Chip Select configuration
 *
 * You need to implement 3 macros or functions for CS capability for communication purpose.
 *
 * Take a look for \ref UG2864_CS_INIT, \ref UG2864_CS_LOW and \ref UG2864_CS_HIGH macros.
 *
\code
//Examples for 3 defines
//Implementation must be made according to your platform
//This is just a technical pseudo code and does not need to work on any platform

//Init CS pin as output
//Use do-while if you must specify more than 1 statement in define
#define UG2864_CS_INIT        TM_GPIO_Init(GPIOA, GPIO_PIN_4, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
//Set CS pin high
#define UG2864_CS_HIGH        TM_GPIO_SetPinHigh(GPIOA, GPIO_PIN_4)
//Set CS pin low
#define UG2864_CS_LOW         TM_GPIO_SetPinLow(GPIOA, GPIO_PIN_4)
\endcode
 */
#include "stm32fxxx_hal.h"
#include "tm_stm32_gpio.h"

/**
 * @brief  Initializes CS pin on platform
 * @note   Function is called from UG2864 module when needed
 * @note   Declared as macro 
 */
#define UG2864_CS_INIT    TM_GPIO_Init(GPIOB, GPIO_PIN_12, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High)
	
/**
 * @brief  Sets CS pin LOW
 * @note   Function is called from UG2864 module when needed
 * @note   Declared as macro 
 */
#define UG2864_CS_LOW     TM_GPIO_SetPinLow(GPIOB, GPIO_PIN_12)

/**
 * @brief  Sets CS pin HIGH
 * @note   Function is called from UG2864 module when needed
 * @note   Declared as macro 
 */
#define UG2864_CS_HIGH    __UG2864_CS_HIGH()

/**
 * @brief  Initializes DC pin on platform (data <-> command)
 * @note   Function is called from UG2864 module when needed
 * @note   Declared as macro
 */
#define UG2864_DC_INIT    TM_GPIO_Init(GPIOB, GPIO_PIN_10, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High)

/**
 * @brief  Sets DC pin LOW
 * @note   Function is called from UG2864 module when needed
 * @note   Declared as macro
 */
#define UG2864_DC_LOW     TM_GPIO_SetPinLow(GPIOB, GPIO_PIN_10)

/**
 * @brief  Sets DC pin HIGH
 * @note   Function is called from UG2864 module when needed
 * @note   Declared as macro
 */
#define UG2864_DC_HIGH    TM_GPIO_SetPinHigh(GPIOB, GPIO_PIN_10)

/**
 * @brief  Initializes backlight pin on platform
 * @note   Function is called from UG2864 module when needed
 * @note   Declared as macro
 */
#define UG2864_BACKLIGHT_INIT    TM_GPIO_Init(GPIOC, GPIO_PIN_2, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High)

/**
 * @brief  Sets backlight pin LOW
 * @note   Function is called from UG2864 module when needed
 * @note   Declared as macro
 */
#define UG2864_BACKLIGHT_LOW     TM_GPIO_SetPinLow(GPIOC, GPIO_PIN_2)

/**
 * @brief  Sets backlight pin HIGH
 * @note   Function is called from UG2864 module when needed
 * @note   Declared as macro
 */
#define UG2864_BACKLIGHT_HIGH    TM_GPIO_SetPinHigh(GPIOC, GPIO_PIN_2)

/**
 * @brief  Initializes reset pin on platform
 * @note   Function is called from UG2864 module when needed
 * @note   Declared as macro
 */
#define UG2864_RST_INIT    TM_GPIO_Init(GPIOB, GPIO_PIN_11, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High)

/**
 * @brief  Sets reset pin LOW
 * @note   Function is called from UG2864 module when needed
 * @note   Declared as macro
 */
#define UG2864_RST_LOW     TM_GPIO_SetPinLow(GPIOB, GPIO_PIN_11)

/**
 * @brief  Sets reset pin HIGH
 * @note   Function is called from UG2864 module when needed
 * @note   Declared as macro
 */
#define UG2864_RST_HIGH    TM_GPIO_SetPinHigh(GPIOB, GPIO_PIN_11)

/**
 * @brief  Initializes SPI peripheral for UG2864 communication
 * @note   This function is called from driver
 * @retval Initialization status:
 *           - 0: Initialization OK
 *           - > 0: Initialization failed
 */
uint8_t UG2864_LL_SPIInit(void);

/**
 * @brief  Sends data to UG2864 module using SPI
 * @param  txbyte: byte to sent
 */
void UG2864_LL_SPITx(uint8_t txbyte);

/**
 * @brief  Sends data to UG2864 module using SPI with DMA
 * @param  fb: pointer to memory
 * @param size: buffer size
 */
void UG2864_LL_SPIFlush( const char *fb, int size );

/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
