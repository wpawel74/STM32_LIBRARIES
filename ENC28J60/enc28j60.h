#ifndef ENC28J60_H
#define ENC28J60_H 100

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32fxxx_hal.h"
#include "tm_stm32_gpio.h"
#include "defines.h"
#include "attributes.h"

/**
 * @defgroup ENC28J60_Macros
 * @brief    Library defines
 * @{
 */

#define ENC28J60_MAXFRAME	1500

// PHY registers
#define PHCON1 				0x00
#define PHSTAT1 			0x01
#define PHID1 				0x02
#define PHID2 				0x03
#define PHCON2 				0x10
#define PHSTAT2 			0x11
#define PHIE 				0x12
#define PHIR 				0x13
#define PHLCON 				0x14

// PHCON1
#define PHCON1_PRST			0x8000
#define PHCON1_PLOOPBK		0x4000
#define PHCON1_PPWRSV		0x0800
#define PHCON1_PDPXMD		0x0100

// PHSTAT1
#define PHSTAT1_PFDPX		0x1000
#define PHSTAT1_PHDPX		0x0800
#define PHSTAT1_LLSTAT		0x0004
#define PHSTAT1_JBSTAT		0x0002

// PHCON2
#define PHCON2_FRCLNK		0x4000
#define PHCON2_TXDIS		0x2000
#define PHCON2_JABBER		0x0400
#define PHCON2_HDLDIS		0x0100

// PHSTAT2
#define PHSTAT2_TXSTAT		0x2000
#define PHSTAT2_RXSTAT		0x1000
#define PHSTAT2_COLSTAT		0x0800
#define PHSTAT2_LSTAT		0x0400
#define PHSTAT2_DPXSTAT		0x0200
#define PHSTAT2_PLRITY		0x0010

// PHIE
#define PHIE_PLNKIE			0x0010
#define PHIE_PGEIE			0x0002

// PHIR
#define PHIR_PLNKIF			0x0010
#define PHIR_PGIF			0x0004

// PHLCON
#define PHLCON_LACFG3		0x0800
#define PHLCON_LACFG2		0x0400
#define PHLCON_LACFG1		0x0200
#define PHLCON_LACFG0		0x0100
#define PHLCON_LBCFG3		0x0080
#define PHLCON_LBCFG2		0x0040
#define PHLCON_LBCFG1		0x0020
#define PHLCON_LBCFG0		0x0010
#define PHLCON_LFRQ1		0x0008
#define PHLCON_LFRQ0		0x0004
#define PHLCON_STRCH		0x0002

/**
 * @defgroup ENC28J60_Functions
 * @brief    ENC28J60 Functions
 * @{
 */

/**
 * @brief  Initializes ENC28J60 chip
 * @note   This function configure SPI interface to communication and CS line
 * @param  *macadr: pointer to MAC address (6 bytes)
 * @retval None
 */
void enc28j60_init(uint8_t *macadr);

/**
 * @brief  Sends packet to ethernet using ENC28J60
 * @note
 * @param  *data: pointer to packet data
 * @param  len: size of data
 * @retval None
 */
void enc28j60_send_packet(const uint8_t *data, uint16_t len);

/**
 * @brief  Receive packet from ethernet using ENC28J60
 * @note
 * @param  *data: pointer to buffer
 * @param  len: size of buffer
 * @retval uint16_t : length of packet
 */
uint16_t enc28j60_recv_packet(uint8_t *buf, uint16_t buflen);

/**
 * @brief  Read PHY register
 * @note
 * @param  adr: address
 * @retval uint16_t: content of register
 */
uint16_t enc28j60_read_phy(uint8_t adr);

/**
 * @brief  Write PHY register
 * @note
 * @param  adr: address
 * @param  data: value of register (16 bits)
 * @retval None
 */
void enc28j60_write_phy(uint8_t adr, uint16_t data);

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
