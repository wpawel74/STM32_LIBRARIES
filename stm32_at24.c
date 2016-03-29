#include "os_compat.h"
#include "config.h"
#include "sensors.h"
#include "tm_stm32_i2c.h"
#include "tm_stm32_delay.h"
#include "stm32_at24.h"
#include <stdio.h>
#include <string.h>

#define AT24xx_DEBUG
//#define AT24xx_DUMP

//! AT24xx Salve Address 7-bit mode
//! \b NOTE: R/W bit is EXCLUDE from this address
#define AT24xx_I2C_ADDR				((0x50)<<1)

#define AT24xx_PAGE_SIZE			255
#define AT24xx_MAX_PAGES			8
#define AT24xx_MAX_WRITE_BYTES		8
#define AT24xx_MAX_READ_BYTES		32

#ifdef AT24xx_DUMP
static const char *hex2string( const char *data, int size ){
	static char buff[1024];
	int			it = 0;
	char 		*out = buff;
	bzero( buff, sizeof(buff) );
	for( it = 0; it < size; it++ )
		out += snprintf( out, sizeof(buff) - (out - buff), "%.2x ", data[it] );
	return buff;
}

static void debug_hex2string( const char *title, const char *data, int size, int col ) {
	int it, raw_no = 0;
	_D(("%s (size=%d cols->%d)\n", title, size, col));
	for( it = 0; it < size + col; it += col, raw_no++ ){
		int col_no = it + col > size ? col - (it + col - size): col;
		if( col_no > 0 ) {
			const char *out = hex2string( data + it, col_no );
			_D((" [%2d] %s\n", it, out));
		}
	}
}

static void AT24xx_dump( I2C_TypeDef *I2Cx, uint16_t addr, int size ){
	unsigned char buf[64];
	do {
		char address[40];
		PW_AT24xx_Read(I2Cx, addr, buf, size > sizeof(buf)? sizeof(buf): size );
		snprintf( address, sizeof(address), "0x%x ", addr );
		debug_hex2string( address, buf, size > sizeof(buf)? sizeof(buf): size, 32 );

		addr += size > sizeof(buf)? sizeof(buf): size;
		size -= size > sizeof(buf)? sizeof(buf): size;
		Delay(10);
	} while( size > 0 );
}
#endif // AT24xx_DUMP

void PW_AT24xx_Init(I2C_TypeDef *I2Cx){
	TM_I2C_Init( I2Cx, TM_I2C_PinsPack_1, 100000 );

	if( PW_AT24xx_isDeviceConneted( I2Cx ) )
		_D(("I: AT24xx INFO: pages %d, page size %d bytes\n", AT24xx_MAX_PAGES, AT24xx_PAGE_SIZE));

#if 0 // ONLY FOR TEST!!!
	unsigned char buf[ 40 ];
	int it;
	for( it = 0; it < sizeof(buf); it++ )
		buf[it] = it;
	PW_AT24xx_Write(I2Cx, 0, buf, sizeof(buf));

	AT24xx_dump( I2Cx, 0, 10 );
#endif // AT24xx_DUMP
}

uint8_t PW_AT24xx_isDeviceConneted(I2C_TypeDef *I2Cx){
	return TM_I2C_IsDeviceConnected( I2Cx, AT24xx_I2C_ADDR );
}

void PW_AT24xx_Write(I2C_TypeDef *I2Cx, uint16_t address, const uint8_t *buf, int size){

	do {
		int wbytes = (size > AT24xx_MAX_WRITE_BYTES) ? AT24xx_MAX_WRITE_BYTES: size;

		if( address > (AT24xx_PAGE_SIZE * AT24xx_MAX_PAGES) )
			return;

		if( ((address + wbytes) / AT24xx_PAGE_SIZE) != (address / AT24xx_PAGE_SIZE) )
			wbytes = AT24xx_PAGE_SIZE - (wbytes % AT24xx_PAGE_SIZE);

#ifdef AT24xx_DEBUG
		int it;
		_D(("D: AT24xx WR(0x%x) page(%d) addr(%d)",address, ((address >> 8) & 0x07), address & 0xff));
		for( it = 0; it < wbytes; it++ )
			_D((":%02x",buf[it]));
		_D(("\n"));
#endif // AT24xx_DEBUG

		TM_I2C_WriteMulti( I2Cx, AT24xx_I2C_ADDR | (((address>>8) & 0x07) << 1),
						address, (uint8_t *)buf, wbytes );

		size -= wbytes;
		address += wbytes;
		buf += wbytes;

		Delay(6000);
	} while ( size > 0 );
}

static void PW_AT24xx_ReadData(I2C_TypeDef *I2Cx, uint16_t address, uint8_t *buf, int size){
	uint8_t addr_in_page = (address & 0xff);

	TM_I2C_ReadMulti( I2Cx, AT24xx_I2C_ADDR, addr_in_page, buf, size );

#ifdef AT24xx_DEBUG
	int it;
	_D(("D: AT24xx RD(0x%x) page(%d) addr(%d)",address, ((address >> 8) & 0x07), addr_in_page));
	for( it = 0; it < size; it++ )
		_D((":%02x",buf[it]));
	_D(("\n"));
#endif // AT24xx_DEBUG
}

void PW_AT24xx_Read(I2C_TypeDef *I2Cx, uint16_t address, uint8_t *buf, int size){

	do {
		int rbytes = (size > AT24xx_MAX_READ_BYTES) ? AT24xx_MAX_READ_BYTES: size;

		if( address > (AT24xx_PAGE_SIZE * AT24xx_MAX_PAGES) )
			return;

		if( ((address + rbytes) / AT24xx_PAGE_SIZE) != (address / AT24xx_PAGE_SIZE) )
			rbytes = AT24xx_PAGE_SIZE - (rbytes % AT24xx_PAGE_SIZE);

		PW_AT24xx_ReadData(I2Cx, address, buf, rbytes);

		size -= rbytes;
		address += rbytes;
		buf += rbytes;
	} while( size > 0 );
}
