/* 
 * File:   hexutils.h
 * Author: grodansparadis
 *
 * Created on den 9 december 2014, 15:40
 */

#ifndef HEXUTILS_H
#define	HEXUTILS_H

#ifdef	__cplusplus
extern "C" {
#endif

static uint8_t nibbleFromChar(char c);
uint8_t hexStringToBytes(uint8_t *buf, uint8_t size, char *inhex);
static char nibbleToChar(uint8_t nibble);
uint8_t bytesToHexString(uint8_t *buf, uint8_t size, uint8_t *bytes, uint8_t buflen);


#ifdef	__cplusplus
}
#endif

#endif	/* HEXUTILS_H */

