// File:  main.h

/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol) 
 * 	http://www.vscp.org
 *
 * 	Version: See project header
 * 	akhe@eurosource.se
 *
 *  Copyright (C) 1995-2013 Ake Hedman, Grodans Paradis AB
 *                          <akhe@grodansparadis.com>
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 
 *	This file is part of VSCP - Very Simple Control Protocol 	
 *	http://www.vscp.org
 *
 * ******************************************************************************
*/

#ifndef FRANKFURT_RS232_MAIN_H
#define FRANKFURT_RS232_MAIN_H

#define	TRUE                            1
#define	FALSE                           0

#define	STATUS_LED_OFF                  0
#define	STATUS_LED_ON                   1
#define	STATUS_LED_VERY_SLOW_BLINK      2
#define	STATUS_LED_SLOW_BLINK           3
#define	STATUS_LED_NORMAL_BLINK         4
#define	STATUS_LED_FAST_BLINK           5
#define	STATUS_LED_VERY_FAST_BLINK      6


#define SIZE_SERIAL_INPUT_BUFFER        128u
#define SIZE_CAN_INPUT_FIFO             38      // Max total 512 bytes 
                                                // (13* this value) is total memory
                                                // for buffer    
// Working modes
#define WORKING_MODE_VERBOSE            0
#define WORKING_MODE_VSCP_DRIVER        1
#define WORKING_MODE_SL_DRIVER          2
#define WORKING_MODE_VSCP_NODE          3

#define NUMERICAL_PRINTOUTMODE_HEX      TRUE
#define NUMERICAL_PRINTOUTMODE_DECIMAL  FALSE

// Default
#define DEFAULT_REGISTER_RW_TIMEOUT     10u // Reg r/w timeout

// Baudrates 40MHz
#define BAUDRATE_19200                  128 // BRGH=1 0,16%
#define BAUDRATE_38400                  64  // BRGH=1 0.16%
#define BAUDRATE_57600                  42  // BRGH=1 0.94%
#define BAUDRATE_115200                 21  // BRGH=1 1.36%
#define BAUDRATE_128000                 19  // BRGH=1 2.34%
#define BAUDRATE_256000                 9   // BRGH=1 2.34%
#define BAUDRATE_500000                 4   // BRGH=1 0%
#define BAUDRATE_625000                 3   // BRGH=1 (0/0 for 0% error)
#define BAUDRATE_1000000                2   // BRGH=1 16.67%

//
// 10 MHz with PLL => 40 MHz
// 1:4 prescaler => 1.25 MHz ( 0.800 uS cycle )
// 1 ms == 1000 uS
// 65535 - 1250 = 64285 = 0xfb1d
//
// Timer2 use 156 and prescaler 1:8
//
#define TIMER0_RELOAD_VALUE             0xfb1d

// Our capabilities
#define OUR_CAPS_MAX_CANAL_FRAMES       1
#define OUR_CAPS_MAX_VSCP_FRAMES        1

// VSCP driver commands
#define VSCP_DRIVER_COMMAND_NOOP        0
#define VSCP_DRIVER_COMMAND_OPEN        1
#define VSCP_DRIVER_COMMAND_LISTEN      2
#define VSCP_DRIVER_COMMAND_LOOPBACK    3
#define VSCP_DRIVER_COMMAND_CLOSE       4
#define VSCP_DRIVER_COMMAND_SET_FILTER  5

#define SLCAN_TIMESTAMP_NOT_USED        0
#define SLCAN_TIMESTAMP_USE             1

// EEPROM Storage
#define MODULE_EEPROM_BOOTLOADER_FLAG   0x00	// Reserved for bootloader
                                                // != 00 is go bootloader
#define MODULE_EEPROM_INIT_BYTE1        0x01
#define MODULE_EEPROM_INIT_BYTE2        0x02
#define MODULE_EEPROM_STARTUP_MODE      0x03    // Working mode to start in
#define MODULE_EEPROM_RW_DELAY          0x04    // R/W Delay
#define MOUDLE_EEPROM_SLCAN_TIMESTAMP   0x05    // SLCAN timestamp
#define MOUDLE_EEPROM_PRINTOUT_IN_HEX   0x06    // HEX/DECIMAL  printout
#define MODULE_EEPROM_STARTUP_OPEN      0x07    // Config/Open/listen/loopback
#define MODULE_EEPROM_SLCAN_TIMESTAMP   0x08    // True if timestamp should be sent
#define MODULE_EEPROM_RW_TIMEOUT        0x09    // Register/read/write timeout

// Filters 0-15
#define MODULE_EEPROM_FILTER0           0x20	// Filter 0 - 4 bytes
#define MODULE_EEPROM_FILTER1           0x24	// Filter 1 - 4 bytes
#define MODULE_EEPROM_FILTER2           0x28	// Filter 2 - 4 bytes
#define MODULE_EEPROM_FILTER3           0x2C	// Filter 3 - 4 bytes
#define MODULE_EEPROM_FILTER4           0x30	// Filter 4 - 4 bytes
#define MODULE_EEPROM_FILTER5           0x34	// Filter 5 - 4 bytes
#define MODULE_EEPROM_FILTER6           0x38	// Filter 6 - 4 bytes
#define MODULE_EEPROM_FILTER7           0x3C	// Filter 7 - 4 bytes
#define MODULE_EEPROM_FILTER8           0x40	// Filter 8 - 4 bytes
#define MODULE_EEPROM_FILTER9           0x44	// Filter 9 - 4 bytes
#define MODULE_EEPROM_FILTER10          0x48	// Filter 10 - 4 bytes
#define MODULE_EEPROM_FILTER11          0x4C	// Filter 11 - 4 bytes
#define MODULE_EEPROM_FILTER12          0x50	// Filter 12 - 4 bytes
#define MODULE_EEPROM_FILTER13          0x54	// Filter 13 - 4 bytes
#define MODULE_EEPROM_FILTER14          0x58	// Filter 14 - 4 bytes
#define MODULE_EEPROM_FILTER15          0x5C	// Filter 15 - 4 bytes

// Mask 0-1
#define MODULE_EEPROM_MASK0             0x60	// Mask 0 - 4 bytes
#define MODULE_EEPROM_MASK1             0x64	// Mask 1 - 4 bytes

// Function Prototypes
void doModeVerbose( void );
void doModeVscp( void );
void doModeSLCAN( void );
void doModeVscpNode();
void init( void );
void init_app_ram( void );
void init_app_eeprom( void );

uint8_t calcCRC( uint8_t *p, uint8_t len );
void sendEscapedUartData( uint8_t c, uint8_t *pcrc );
void sendVSCPDriverErrorFrame( uint8_t errorcode );
void sendVSCPDriverAck( void );
void sendVSCPDriverNack( void );
BOOL readRegister( uint8_t nodeid,
                        uint8_t reg,
                        uint16_t timeout,
                        uint8_t *value);
BOOL readRegisterExtended( uint8_t nodeid,
                                uint16_t page,
                                uint8_t reg,
                                uint16_t timeout,
                                uint8_t *value );
BOOL writeRegister( uint8_t nodeid,
                        uint8_t reg,
                        uint16_t timeout,
                        uint8_t *value);
BOOL writeRegisterExtended(uint8_t nodeid,
                                uint16_t page,
                                uint8_t reg,
                                uint16_t timeout,
                                uint8_t *value );
void printBinary( uint8_t value );
BOOL receivePrintEventVerbose( void );
BOOL receiveSendEventCANAL( void );
BOOL receiveSendMultiEventCANAL(void);
BOOL receiveSendEventVSCP( void );
BOOL receiveSendEventSLCAN( void );
BOOL receiveVSCPModeCanalMsg( void );
BOOL receiveVSCPModeMultiCanalMsg(void);
BOOL sendVSCPModeCapabilities(void);
void sendVSCPDriverCommandReply( uint8_t cmdReplyCode, uint8_t cmdCode );
void printHelp( void );
void findNodes( void );
void printGUID( uint8_t nodeid );
void printMDF( uint8_t nodeid );
void printNodeFirmwareVersion( uint8_t nodeid );

void printStatistics( void );
void printErrors( void );

void vscp_restoreDefaults( void );
void printFirmwareVersion( void );
void printMode( void );

void setFilter( uint8_t filter, uint32_t val );

/*!
	Send Extended ID CAN frame
	@param id CAN extended ID for frame.
	@param size Number of databytes 0-8
	@param pData Pointer to databytes of frame.
	@return TRUE (!=0) on success, FALSE (==0) on failure.
*/
int8_t sendCANFrame( uint32_t id, uint8_t size, uint8_t *pData );

/*!
	Get extended ID CAN frame
	@param pid Pointer to CAN extended ID for frame.
	@param psize Pointer to number of databytes 0-8
	@param pData Pointer to databytes of frame.
	@return TRUE (!=0) on success, FALSE (==0) on failure.
*/
int8_t getCANFrame( uint32_t *pid, uint8_t *psize, uint8_t *pData );


/*!
    Get a VSCP frame frame
    @param pvscpclass Pointer to variable that will get VSCP class.
    @param pvscptype Ponter to variable which will get VSCP type.
    @param pNodeId Pointer to variable which will get nodeid.
        @param pPriority Pointer to variable which will get priority (0-7).
    @param pSize Pointer to variable that will get datasize.
    @param pData pinter to array that will get event data.
    @return TRUE on success.
 */
int8_t getVSCPFrame(uint16_t *pvscpclass,
                        uint8_t *pvscptype,
                        uint8_t *pNodeId,
                        uint8_t *pPriority,
                        uint8_t *pSize,
                        uint8_t *pData);

/*!
    Send a VSCP frame
    @param vscpclass VSCP class for event.
    @param vscptype VSCP type for event.
        @param nodeid Nodeid for originating node.
        @param priority Priotity for event.
    @param size Size of data portion.
    @param pData Pointer to event data.
    @return TRUE on success.
 */
int8_t sendVSCPFrame(uint16_t vscpclass,
                        uint8_t vscptype,
                        uint8_t nodeid,
                        uint8_t priority,
                        uint8_t size,
                        uint8_t *pData);



#endif
