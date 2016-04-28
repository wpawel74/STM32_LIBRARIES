/**	
   ----------------------------------------------------------------------
    Copyright (c) 2016

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
 */
#include "stm32f1xx.h"

/**************************************************************************/
/**************************************************************************/
/**************************************************************************/
/*                                                                        */
/*   Edit file name to ug2864_ll.c and edit values for your platform      */
/*                                                                        */
/**************************************************************************/
/**************************************************************************/
/**************************************************************************/
#include "ug2864_ll_template.h"
#include "tm_stm32_spi.h"
#include "stm32fxxx_hal.h"

#define UG2864_SPI					SPI2

static DMA_HandleTypeDef			hdma_spi2_tx;
static SPI_HandleTypeDef			spi2;

#define UG2864_PORT					GPIOB
#define UG2864_SCK					GPIO_PIN_13
#define UG2864_DO					GPIO_PIN_15

static int completed = 0;

void __UG2864_CS_HIGH(void){
	while( !__HAL_SPI_GET_FLAG(&spi2, SPI_FLAG_TXE ) );
	while( __HAL_SPI_GET_FLAG(&spi2, SPI_FLAG_BSY ) );
	TM_GPIO_SetPinHigh(GPIOB, GPIO_PIN_12);
}

/**
* @brief This function handles DMA1 Stream5 global interrupt.
*/
void DMA1_Stream5_IRQHandler(void){

	/* USER CODE BEGIN DMA1_Stream5_IRQn 0 */
	completed = 1;
	/* USER CODE END DMA1_Stream5_IRQn 0 */

	HAL_DMA_IRQHandler(&hdma_spi2_tx);

	/* USER CODE BEGIN DMA1_Stream5_IRQn 1 */
	/* USER CODE END DMA1_Stream5_IRQn 1 */
}

uint8_t UG2864_LL_SPIInit(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	__GPIOB_CLK_ENABLE();
	__SPI2_CLK_ENABLE();

	GPIO_InitStructure.Pin = ( UG2864_SCK | UG2864_DO );
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init( UG2864_PORT, &GPIO_InitStructure );

	spi2.Instance = UG2864_SPI;
	spi2.Init.Mode = SPI_MODE_MASTER;
	spi2.Init.Direction = SPI_DIRECTION_1LINE;
	spi2.Init.DataSize = SPI_DATASIZE_8BIT;
	spi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	spi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	spi2.Init.NSS = SPI_NSS_SOFT;
	spi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	spi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	spi2.Init.TIMode = SPI_TIMODE_DISABLED;
	spi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
	spi2.Init.CRCPolynomial = 7;
	HAL_SPI_Init(&spi2);

	__HAL_RCC_DMA1_CLK_ENABLE();
	__DMA1_CLK_ENABLE();

	hdma_spi2_tx.Instance = DMA1_Channel5;
	HAL_DMA_DeInit(&hdma_spi2_tx);

	hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_spi2_tx.Init.Mode = DMA_NORMAL;
	hdma_spi2_tx.Init.Priority = DMA_PRIORITY_HIGH;
	HAL_DMA_Init(&hdma_spi2_tx);

	__HAL_LINKDMA(&spi2, hdmatx, hdma_spi2_tx);

	/* Return 0 = Successful */
	return 0;
}

void UG2864_LL_SPITx(uint8_t txbyte) {
	/* send/receive data via SPI */
	UG2864_CS_LOW;
	while( __HAL_SPI_GET_FLAG(&spi2, SPI_FLAG_BSY) );	//wait until transmit register will be empty
	UG2864_DC_LOW;

	HAL_SPI_Transmit( &spi2, &txbyte, 1, 100 );

	while( __HAL_SPI_GET_FLAG(&spi2, SPI_FLAG_BSY) );	//wait until transmit register will be empty
	UG2864_DC_HIGH;
	UG2864_CS_HIGH;
}

void UG2864_LL_SPIFlush( const char *fb, int size ) {

	UG2864_LL_SPITx(0x00 | 0x0);  // low col = 0
	UG2864_LL_SPITx(0x10 | 0x0);  // hi col = 0
//	UG2864_LL_SPITx(0x40 | 0x0); // line #0
	UG2864_LL_SPITx(0xb0 | 0x0); // page start #0

	UG2864_CS_LOW;
	__HAL_DMA_DISABLE( &hdma_spi2_tx );
	completed = 0;
	HAL_DMA_Start( &hdma_spi2_tx, (uint32_t)fb, (uint32_t)(&(UG2864_SPI->DR)), size );
	while( !__HAL_DMA_GET_FLAG( &hdma_spi2_tx, DMA_FLAG_TC5 ) );
	UG2864_CS_HIGH;
}
