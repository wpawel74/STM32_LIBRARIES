#include "tm_stm32_spi.h"
#include "tm_stm32_gpio.h"
#include "tm_stm32_delay.h"
#include "enc28j60.h"
#include "enc28j60_ll.h"

#define ENC28J60_BUFSIZE	0x2000
#define ENC28J60_RXSIZE		0x1A00
#define ENC28J60_BUFEND		(ENC28J60_BUFSIZE - 1)

#define ENC28J60_RXSTART	0
#define ENC28J60_RXEND		(ENC28J60_RXSIZE - 1)
#define ENC28J60_TXSTART	ENC28J60_RXSIZE

#define ENC28J60_SPI_RCR	0x00
#define ENC28J60_SPI_RBM	0x3A
#define ENC28J60_SPI_WCR	0x40
#define ENC28J60_SPI_WBM	0x7A
#define ENC28J60_SPI_BFS	0x80
#define ENC28J60_SPI_BFC	0xA0
#define ENC28J60_SPI_SC		0xFF

#define ENC28J60_ADDR_MASK	0x1F
#define ENC28J60_COMMON_CR	0x1B

/*
 * Main registers
 */
#define EIE 				0x1B
#define EIR 				0x1C
#define ESTAT 				0x1D
#define ECON2 				0x1E
#define ECON1 				0x1F


// Buffer read pointer
#define ERDPTL 				0x00
#define ERDPTH 				0x01
#define ERDPT				ERDPTL

// Buffer write pointer
#define EWRPTL 				0x02
#define EWRPTH 				0x03
#define EWRPT				EWRPTL

// Tx packet start pointer
#define ETXSTL 				0x04
#define ETXSTH 				0x05
#define ETXST				ETXSTL

// Tx packet end pointer
#define ETXNDL 				0x06
#define ETXNDH 				0x07
#define ETXND				ETXNDL

// Rx FIFO start pointer
#define ERXSTL 				0x08
#define ERXSTH 				0x09
#define ERXST				ERXSTL

// Rx FIFO end pointer
#define ERXNDL 				0x0A
#define ERXNDH 				0x0B
#define ERXND				ERXNDL

// Rx FIFO read pointer
#define ERXRDPTL 			0x0C
#define ERXRDPTH 			0x0D
#define ERXRDPT				ERXRDPTL

// Rx FIFO write pointer
#define ERXWRPTL 			0x0E
#define ERXWRPTH 			0x0F
#define ERXWRPT				ERXWRPTL

// DMA source block start pointer
#define EDMASTL 			0x10
#define EDMASTH 			0x11
#define EDMAST				EDMASTL

// DMA source block end pointer
#define EDMANDL 			0x12
#define EDMANDH 			0x13
#define EDMAND				EDMANDL

// DMA destination pointer
#define EDMADSTL 			0x14
#define EDMADSTH 			0x15
#define	EDMADST				EDMADSTL

// DMA checksum
#define EDMACSL 			0x16
#define EDMACSH 			0x17
#define EDMACS				EDMACSL

// Hash table registers
#define EHT0 				(0x00 | 0x20)
#define EHT1 				(0x01 | 0x20)
#define EHT2 				(0x02 | 0x20)
#define EHT3 				(0x03 | 0x20)
#define EHT4 				(0x04 | 0x20)
#define EHT5 				(0x05 | 0x20)
#define EHT6 				(0x06 | 0x20)
#define EHT7 				(0x07 | 0x20)

// Pattern match registers
#define EPMM0 				(0x08 | 0x20)
#define EPMM1 				(0x09 | 0x20)
#define EPMM2 				(0x0A | 0x20)
#define EPMM3 				(0x0B | 0x20)
#define EPMM4 				(0x0C | 0x20)
#define EPMM5 				(0x0D | 0x20)
#define EPMM6 				(0x0E | 0x20)
#define EPMM7 				(0x0F | 0x20)
#define EPMCSL 				(0x10 | 0x20)
#define EPMCSH 				(0x11 | 0x20)
#define EPMOL 				(0x14 | 0x20)
#define EPMOH 				(0x15 | 0x20)

// Wake-on-LAN interrupt registers
#define EWOLIE 				(0x16 | 0x20)
#define EWOLIR 				(0x17 | 0x20)

// Receive filters mask
#define ERXFCON 			(0x18 | 0x20)

#define ERXFCON_UCEN		0x80
#define ERXFCON_ANDOR		0x40
#define ERXFCON_CRCEN		0x20
#define ERXFCON_PMEN		0x10
#define ERXFCON_MPEN		0x08
#define ERXFCON_HTEN		0x04
#define ERXFCON_MCEN		0x02
#define ERXFCON_BCEN		0x01

// Packet counter
#define EPKTCNT 			(0x19 | 0x20)

// MAC control registers
#define MACON1 				(0x00 | 0x40 | 0x80)
#define MACON2 				(0x01 | 0x40 | 0x80)
#define MACON3 				(0x02 | 0x40 | 0x80)
#define MACON4 				(0x03 | 0x40 | 0x80)

// MAC Back-to-back gap
#define MABBIPG 			(0x04 | 0x40 | 0x80)

// MAC Non back-to-back gap
#define MAIPGL 				(0x06 | 0x40 | 0x80)
#define MAIPGH 				(0x07 | 0x40 | 0x80)

// Collision window & rexmit timer
#define MACLCON1 			(0x08 | 0x40 | 0x80)
#define MACLCON2 			(0x09 | 0x40 | 0x80)

// Max frame length
#define MAMXFLL 			(0x0A | 0x40 | 0x80)
#define MAMXFLH 			(0x0B | 0x40 | 0x80)
#define MAMXFL				MAMXFLL

// MAC-PHY support register
#define MAPHSUP 			(0x0D | 0x40 | 0x80)
#define MICON 				(0x11 | 0x40 | 0x80)

// MII registers
#define MICMD 				(0x12 | 0x40 | 0x80)
#define MIREGADR 			(0x14 | 0x40 | 0x80)

#define MIWRL 				(0x16 | 0x40 | 0x80)
#define MIWRH 				(0x17 | 0x40 | 0x80)
#define MIWR				MIWRL

#define MIRDL 				(0x18 | 0x40 | 0x80)
#define MIRDH 				(0x19 | 0x40 | 0x80)
#define MIRD				MIRDL

// MAC Address
#define MAADR1 				(0x00 | 0x60 | 0x80)
#define MAADR0 				(0x01 | 0x60 | 0x80)
#define MAADR3 				(0x02 | 0x60 | 0x80)
#define MAADR2 				(0x03 | 0x60 | 0x80)
#define MAADR5 				(0x04 | 0x60 | 0x80)
#define MAADR4 				(0x05 | 0x60 | 0x80)

// Built-in self-test
#define EBSTSD 				(0x06 | 0x60)
#define EBSTCON 			(0x07 | 0x60)
#define EBSTCSL 			(0x08 | 0x60)
#define EBSTCSH 			(0x09 | 0x60)
#define MISTAT 				(0x0A | 0x60 | 0x80)

// Revision ID
#define EREVID 				(0x12 | 0x60)

// Clock output control register
#define ECOCON 				(0x15 | 0x60)

// Flow control registers
#define EFLOCON 			(0x17 | 0x60)
#define EPAUSL 				(0x18 | 0x60)
#define EPAUSH 				(0x19 | 0x60)

// EIE
#define EIE_INTIE			0x80
#define EIE_PKTIE			0x40
#define EIE_DMAIE			0x20
#define EIE_LINKIE			0x10
#define EIE_TXIE			0x08
#define EIE_WOLIE			0x04
#define EIE_TXERIE			0x02
#define EIE_RXERIE			0x01

// EIR
#define EIR_PKTIF			0x40
#define EIR_DMAIF			0x20
#define EIR_LINKIF			0x10
#define EIR_TXIF			0x08
#define EIR_WOLIF			0x04
#define EIR_TXERIF			0x02
#define EIR_RXERIF			0x01

// ESTAT
#define ESTAT_INT			0x80
#define ESTAT_LATECOL		0x10
#define ESTAT_RXBUSY		0x04
#define ESTAT_TXABRT		0x02
#define ESTAT_CLKRDY		0x01

// ECON2
#define ECON2_AUTOINC		0x80
#define ECON2_PKTDEC		0x40
#define ECON2_PWRSV			0x20
#define ECON2_VRPS			0x08

// ECON1
#define ECON1_TXRST			0x80
#define ECON1_RXRST			0x40
#define ECON1_DMAST			0x20
#define ECON1_CSUMEN		0x10
#define ECON1_TXRTS			0x08
#define ECON1_RXEN			0x04
#define ECON1_BSEL1			0x02
#define ECON1_BSEL0			0x01

// EWOLIE
#define EWOLIE_UCWOLIE		0x80
#define EWOLIE_AWOLIE		0x40
#define EWOLIE_PMWOLIE		0x10
#define EWOLIE_MPWOLIE		0x08
#define EWOLIE_HTWOLIE		0x04
#define EWOLIE_MCWOLIE		0x02
#define EWOLIE_BCWOLIE		0x01

// EWOLIR
#define EWOLIR_UCWOLIF		0x80
#define EWOLIR_AWOLIF		0x40
#define EWOLIR_PMWOLIF		0x10
#define EWOLIR_MPWOLIF		0x08
#define EWOLIR_HTWOLIF		0x04
#define EWOLIR_MCWOLIF		0x02
#define EWOLIR_BCWOLIF		0x01

// ERXFCON
#define ERXFCON_UCEN		0x80
#define ERXFCON_ANDOR		0x40
#define ERXFCON_CRCEN		0x20
#define ERXFCON_PMEN		0x10
#define ERXFCON_MPEN		0x08
#define ERXFCON_HTEN		0x04
#define ERXFCON_MCEN		0x02
#define ERXFCON_BCEN		0x01

// MACON1
#define MACON1_LOOPBK		0x10
#define MACON1_TXPAUS		0x08
#define MACON1_RXPAUS		0x04
#define MACON1_PASSALL		0x02
#define MACON1_MARXEN		0x01

// MACON2
#define MACON2_MARST		0x80
#define MACON2_RNDRST		0x40
#define MACON2_MARXRST		0x08
#define MACON2_RFUNRST		0x04
#define MACON2_MATXRST		0x02
#define MACON2_TFUNRST		0x01

// MACON3
#define MACON3_PADCFG2		0x80
#define MACON3_PADCFG1		0x40
#define MACON3_PADCFG0		0x20
#define MACON3_TXCRCEN		0x10
#define MACON3_PHDRLEN		0x08
#define MACON3_HFRMEN		0x04
#define MACON3_FRMLNEN		0x02
#define MACON3_FULDPX		0x01

// MACON4
#define MACON4_DEFER		0x40
#define MACON4_BPEN			0x20
#define MACON4_NOBKOFF		0x10
#define MACON4_LONGPRE		0x02
#define MACON4_PUREPRE		0x01

// MAPHSUP
#define MAPHSUP_RSTINTFC	0x80
#define MAPHSUP_RSTRMII		0x08

// MICON
#define MICON_RSTMII		0x80

// MICMD
#define MICMD_MIISCAN		0x02
#define MICMD_MIIRD			0x01

// EBSTCON
#define EBSTCON_PSV2		0x80
#define EBSTCON_PSV1		0x40
#define EBSTCON_PSV0		0x20
#define EBSTCON_PSEL		0x10
#define EBSTCON_TMSEL1		0x08
#define EBSTCON_TMSEL0		0x04
#define EBSTCON_TME			0x02
#define EBSTCON_BISTST		0x01

// MISTAT
#define MISTAT_NVALID		0x04
#define MISTAT_SCAN			0x02
#define MISTAT_BUSY			0x01

// ECOCON
#define ECOCON_COCON2		0x04
#define ECOCON_COCON1		0x02
#define ECOCON_COCON0		0x01

// EFLOCON
#define EFLOCON_FULDPXS		0x04
#define EFLOCON_FCEN1		0x02
#define EFLOCON_FCEN0		0x01

/*
 * SPI
 */
volatile static uint8_t enc28j60_current_bank = 0;
volatile static uint16_t enc28j60_rxrdpt = 0;

#define enc28j60_rx()				ENC28J60_LL_SPIRxTx(0xff)
#define enc28j60_tx(data)			ENC28J60_LL_SPIRxTx(data)

// Generic SPI read command
static uint8_t enc28j60_read_op(uint8_t cmd, uint8_t adr){
	uint8_t data;

	ENC28J60_CS_LOW;
	enc28j60_tx(cmd | (adr & ENC28J60_ADDR_MASK));
	if(adr & 0x80) // throw out dummy byte 
		enc28j60_rx(); // when reading MII/MAC register
	data = enc28j60_rx();
	ENC28J60_CS_HIGH;
	return data;
}

// Generic SPI write command
static void enc28j60_write_op(uint8_t cmd, uint8_t adr, uint8_t data){
	ENC28J60_CS_LOW;
	enc28j60_tx(cmd | (adr & ENC28J60_ADDR_MASK));
	enc28j60_tx(data);
	ENC28J60_CS_HIGH;
}

// Initiate software reset
static void enc28j60_soft_reset(void){
	ENC28J60_CS_LOW;
	enc28j60_tx(ENC28J60_SPI_SC);
	ENC28J60_CS_HIGH;
	
	enc28j60_current_bank = 0;
	Delayms(1);
}

/*
 * Memory access
 */

// Set register bank
void enc28j60_set_bank(uint8_t adr){
	uint8_t bank;

	if( (adr & ENC28J60_ADDR_MASK) < ENC28J60_COMMON_CR ){
		bank = (adr >> 5) & 0x03; //BSEL1|BSEL0=0x03
		if(bank != enc28j60_current_bank){
			enc28j60_write_op(ENC28J60_SPI_BFC, ECON1, 0x03);
			enc28j60_write_op(ENC28J60_SPI_BFS, ECON1, bank);
			enc28j60_current_bank = bank;
		}
	}
}

// Read register
uint8_t enc28j60_rcr(uint8_t adr){
	enc28j60_set_bank(adr);
	return enc28j60_read_op(ENC28J60_SPI_RCR, adr);
}

// Read register pair
uint16_t enc28j60_rcr16(uint8_t adr){
	enc28j60_set_bank(adr);
	return enc28j60_read_op(ENC28J60_SPI_RCR, adr) |
		(enc28j60_read_op(ENC28J60_SPI_RCR, adr+1) << 8);
}

// Write register
void enc28j60_wcr(uint8_t adr, uint8_t arg){
	enc28j60_set_bank(adr);
	enc28j60_write_op(ENC28J60_SPI_WCR, adr, arg);
}

// Write register pair
void enc28j60_wcr16(uint8_t adr, uint16_t arg){
	enc28j60_set_bank(adr);
	enc28j60_write_op(ENC28J60_SPI_WCR, adr, arg);
	enc28j60_write_op(ENC28J60_SPI_WCR, adr+1, arg>>8);
}

// Clear bits in register (reg &= ~mask)
void enc28j60_bfc(uint8_t adr, uint8_t mask){
	enc28j60_set_bank(adr);
	enc28j60_write_op(ENC28J60_SPI_BFC, adr, mask);
}

// Set bits in register (reg |= mask)
void enc28j60_bfs(uint8_t adr, uint8_t mask){
	enc28j60_set_bank(adr);
	enc28j60_write_op(ENC28J60_SPI_BFS, adr, mask);
}

// Read Rx/Tx buffer (at ERDPT)
void enc28j60_read_buffer(uint8_t *buf, uint16_t len){
	ENC28J60_CS_LOW;
	enc28j60_tx(ENC28J60_SPI_RBM);
	while(len--)
		*(buf++) = enc28j60_rx();
	ENC28J60_CS_HIGH;
}

// Write Rx/Tx buffer (at EWRPT)
void enc28j60_write_buffer(const uint8_t *buf, uint16_t len){
	ENC28J60_CS_LOW;
	enc28j60_tx(ENC28J60_SPI_WBM);
	while(len--)
		enc28j60_tx(*(buf++));
	ENC28J60_CS_HIGH;
}

// Read PHY register
uint16_t enc28j60_read_phy(uint8_t adr){
	enc28j60_wcr(MIREGADR, adr);
	enc28j60_bfs(MICMD, MICMD_MIIRD);
	while(enc28j60_rcr(MISTAT) & MISTAT_BUSY)
		;
	enc28j60_bfc(MICMD, MICMD_MIIRD);
	return enc28j60_rcr16(MIRD);
}

// Write PHY register
void enc28j60_write_phy(uint8_t adr, uint16_t data){
	enc28j60_wcr(MIREGADR, adr);
	enc28j60_wcr16(MIWR, data);
	while(enc28j60_rcr(MISTAT) & MISTAT_BUSY)
		;
}

/*
 * Init & packet Rx/Tx
 */
void enc28j60_init(uint8_t *macadr){

	ENC28J60_LL_SPIInit();

	ENC28J60_CS_INIT;

	// Reset ENC28J60
	enc28j60_soft_reset();

	// Setup Rx/Tx buffer
	enc28j60_wcr16(ERXST, ENC28J60_RXSTART);
	enc28j60_rcr16(ERXST);
	enc28j60_wcr16(ERXRDPT, ENC28J60_RXSTART);
	enc28j60_wcr16(ERXND, ENC28J60_RXEND);
	enc28j60_rxrdpt = ENC28J60_RXSTART;

	// Setup MAC
	enc28j60_wcr(MACON1, MACON1_TXPAUS| // Enable flow control
				 MACON1_RXPAUS|MACON1_MARXEN); // Enable MAC Rx
	enc28j60_wcr(MACON2, 0); // Clear reset
	enc28j60_wcr(MACON3, MACON3_PADCFG0| // Enable padding,
				 MACON3_TXCRCEN|MACON3_FRMLNEN|MACON3_FULDPX); // Enable crc & frame len chk
	enc28j60_wcr16(MAMXFL, ENC28J60_MAXFRAME);
	enc28j60_wcr(MABBIPG, 0x15); // Set inter-frame gap
	enc28j60_wcr(MAIPGL, 0x12);
	enc28j60_wcr(MAIPGH, 0x0c);
	enc28j60_wcr(MAADR5, macadr[0]); // Set MAC address
	enc28j60_wcr(MAADR4, macadr[1]);
	enc28j60_wcr(MAADR3, macadr[2]);
	enc28j60_wcr(MAADR2, macadr[3]);
	enc28j60_wcr(MAADR1, macadr[4]);
	enc28j60_wcr(MAADR0, macadr[5]);

	enc28j60_wcr(ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_BCEN); // Filter policy: only unicast frames and

	// Setup PHY
	enc28j60_write_phy(PHCON1, PHCON1_PDPXMD); // Force full-duplex mode
	enc28j60_write_phy(PHCON2, PHCON2_HDLDIS); // Disable loopback
	enc28j60_write_phy(PHLCON, PHLCON_LACFG2| // Configure LED ctrl
						PHLCON_LBCFG2|PHLCON_LBCFG1|PHLCON_LBCFG0|
						PHLCON_LFRQ0|PHLCON_STRCH);

	// Enable Rx packets
	enc28j60_bfs(ECON1, ECON1_RXEN);
}

void enc28j60_send_packet(const uint8_t *data, uint16_t len){
	while(enc28j60_rcr(ECON1) & ECON1_TXRTS){
		// TXRTS may not clear - ENC28J60 bug. We must reset
		// transmit logic in cause of Tx error
		if(enc28j60_rcr(EIR) & EIR_TXERIF) {
			enc28j60_bfs(ECON1, ECON1_TXRST);
			enc28j60_bfc(ECON1, ECON1_TXRST);
		}
	}

	enc28j60_wcr16(EWRPT, ENC28J60_TXSTART);
	enc28j60_write_buffer((uint8_t*)"\x00", 1);
	enc28j60_write_buffer(data, len);

	enc28j60_wcr16(ETXST, ENC28J60_TXSTART);
	enc28j60_wcr16(ETXND, ENC28J60_TXSTART + len);

	enc28j60_bfs(ECON1, ECON1_TXRTS); // Request packet send
}

uint16_t enc28j60_recv_packet(uint8_t *buf, uint16_t buflen){
	uint16_t len = 0, rxlen, status, temp;

	if(enc28j60_rcr(EPKTCNT)){
		enc28j60_wcr16(ERDPT, enc28j60_rxrdpt);

		enc28j60_read_buffer((void*)&enc28j60_rxrdpt, sizeof(enc28j60_rxrdpt));
		enc28j60_read_buffer((void*)&rxlen, sizeof(rxlen));
		enc28j60_read_buffer((void*)&status, sizeof(status));

		if(status & 0x80){
			//success
			len = rxlen - 4; //throw out crc
			if(len > buflen) len = buflen;
			enc28j60_read_buffer(buf, len);	
		}

		// Set Rx read pointer to next packet
		temp = (enc28j60_rxrdpt - 1) & ENC28J60_BUFEND;
		enc28j60_wcr16(ERXRDPT, temp);

		// Decrement packet counter
		enc28j60_bfs(ECON2, ECON2_PKTDEC);
	}
	return len;
}
