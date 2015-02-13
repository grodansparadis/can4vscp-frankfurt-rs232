
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include "hexutils.h"


///////////////////////////////////////////////////////////////////////////////
// hexStringToBytes
//
// utility function to convert hex character representation to their
// nibble (4 bit) values
//

static uint8_t nibbleFromChar(char c)
{
	if(c >= '0' && c <= '9') return c - '0';
	if(c >= 'a' && c <= 'f') return c - 'a' + 10;
	if(c >= 'A' && c <= 'F') return c - 'A' + 10;
	return 255;
}

///////////////////////////////////////////////////////////////////////////////
// hexStringToBytes
//
// Convert a string of characters representing a hex buffer into a series of
// bytes 
//

uint8_t hexStringToBytes(uint8_t *buf, uint8_t size, char *inhex)
{
	uint8_t *p;
	int len, i;

    len = strlen(inhex) / 2;
    if ( len > size ) return 0; // Buffer is to small
	for (i=0, p = (uint8_t *)inhex; i<len; i++) {
		buf[ i ] = (nibbleFromChar(*p) << 4) | nibbleFromChar(*(p+1));
		p += 2;
	}

	return i;
}

static char byteMap[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };
static int byteMapLen = sizeof(byteMap);

///////////////////////////////////////////////////////////////////////////////
// nibbleToChar
//
// Utility function to convert nibbles (4 bit values) into a hex character
// representation
//

static char nibbleToChar(uint8_t nibble)
{
	if (nibble < byteMapLen) return byteMap[nibble];
	return '*';
}

///////////////////////////////////////////////////////////////////////////////
// bytesToHexString
//
// Convert a buffer of binary values into a hex string representation
//

uint8_t bytesToHexString( uint8_t *buf, uint8_t size, uint8_t *bytes, uint8_t buflen)
{
	int i;

	if ( size < (buflen*2 + 1) ) return 0;
	for (i=0; i<buflen; i++) {
		buf[i*2] = nibbleToChar(bytes[i] >> 4);
		buf[i*2+1] = nibbleToChar(bytes[i] & 0x0f);
	}
    buf[i] = '\0';
	return i;
}