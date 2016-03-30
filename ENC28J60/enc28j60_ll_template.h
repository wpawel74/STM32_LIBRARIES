#ifndef ENC28J60_LL_H
#define ENC28J60_LL_H 100

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************/
/**************************************************************************/
/**************************************************************************/
/*                                                                        */
/*    Edit file name to enc28j60_ll.h and edit values for your platform   */
/*                                                                        */
/**************************************************************************/
/**************************************************************************/
/**************************************************************************/

/**
 * @defgroup ENC28J60_LL
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
 * - \ref ENC28J60_LL_SPIInit: Function, which is called when SPI should be initialized
 * - \ref ENC8J60_LL_SPIRxTx: Function, which is called when data should be sent to ENC28J60 device
 *
 *
\code
//Code below show example procedure and does need to be directly for your platform.
//You have to make sure to properly configure UART and RX interrupt for it.

//USART Initialization function, which is called from ESP stack
uint8_t ENC28J60_LL_SPIInit(void) {

    //Init SPI peripheral
	TM_SPI_InitFull( ENC28J60_SPI, TM_SPI_PinsPack_1, TM_SPI_Mode_0, SPI_BAUDRATEPRESCALER_2, SPI_MODE_MASTER, SPI_FIRSTBIT_MSB );
	TM_SPI_SetDataSize( ENC28J60_SPI, TM_SPI_DataSize_8b );

    //Init CS pin
    ENC28J60_CS_INIT;

    //Return 0 = Successful
    return 0;
}

//SPI send/receive function, which is called from ESP stack
uint8_t ENC28J60_LL_SPIRxTx(uint8_t txbyte){
    //send/receive data via SPI
    TM_SPI_Send( ENC28J60_SPI, txbyte );
    //Return RX byte
    return TM_SPI_Send( ENC28J60_SPI, 0xff );
}

\endcode
 * 
 * \par Chip Select configuration
 *
 * You need to implement 3 macros or functions for CS capability for communication purpose.
 *
 * Take a look for \ref ENC28J60_CS_INIT, \ref ENC28J60_CS_LOW and \ref ENC28J60_CS_HIGH macros.
 *
\code
//Examples for 3 defines
//Implementation must be made according to your platform
//This is just a technical pseudo code and does not need to work on any platform

//Init CS pin as output
//Use do-while if you must specify more than 1 statement in define
#define ENC28J60_CS_INIT        TM_GPIO_Init(GPIOA, GPIO_PIN_4, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
//Set CS pin high
#define ENC28J60_CS_HIGH        TM_GPIO_SetPinHigh(GPIOA, GPIO_PIN_4)
//Set CS pin low
#define ENC28J60_CS_LOW         TM_GPIO_SetPinLow(GPIOA, GPIO_PIN_4)
\endcode
 */
#include "stm32fxxx_hal.h"
#include "tm_stm32_gpio.h"

/**
 * @brief  Initializes CS pin on platform
 * @note   Function is called from ENC stack module when needed
 * @note   Declared as macro 
 */
#define ENC28J60_CS_INIT    TM_GPIO_Init(GPIOA, GPIO_PIN_4, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High)
	
/**
 * @brief  Sets CS pin LOW
 * @note   Function is called from ENC stack module when needed
 * @note   Declared as macro 
 */
#define ENC28J60_CS_LOW     TM_GPIO_SetPinLow(GPIOA, GPIO_PIN_4)

/**
 * @brief  Sets CS pin HIGH
 * @note   Function is called from ENC stack module when needed
 * @note   Declared as macro 
 */
#define ENC28J60_CS_HIGH    TM_GPIO_SetPinHigh(GPIOA, GPIO_PIN_4)


/**
 * @brief  Initializes SPI peripheral for ENC28J60 communication
 * @note   This function is called from ENC stack
 * @retval Initialization status:
 *           - 0: Initialization OK
 *           - > 0: Initialization failed
 */
uint8_t ENC28J60_LL_SPIInit(void);

/**
 * @brief  Sends/Receive data from ENC stack to ENC28J60 module using SPI
 * @param  txbyte: byte to sent
 * @retval Received byte
 */
uint8_t ENC28J60_LL_SPIRxTx(uint8_t txbyte);


/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
