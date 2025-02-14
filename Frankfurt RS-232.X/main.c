/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol)
 * 	http://www.vscp.org
 *
 *  Frankfurt RS-232
 *  ================
 *
 *  Copyright (C)1995-2020 Ake Hedman, Grodans Paradis AB
 *                          http://www.grodansparadis.com
 *                          <akhe@grodansparadis.com>
 *
 *  This work is licensed under the Creative Common
 *  Attribution-NonCommercial-ShareAlike 3.0 Unported license. The full
 *  license is available in the top folder of this project (LICENSE) or here
 *  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode
 *  It is also available in a human readable form here
 *  http://creativecommons.org/licenses/by-nc-sa/3.0/
 *
 *	This file is part of VSCP - Very Simple Control Protocol
 *	http://www.vscp.org
 *
 * ******************************************************************************
 */

#include <xc.h>
#include <timers.h>
#include <delays.h>
#include <eeprom.h>
#include <inttypes.h>
#include "ECAN.h"
#include <plib/usart.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fifo.h"
#include "version.h"
#include <vscp_class.h>
#include <vscp_type.h>
#include <vscp_serial.h>
#include "hexutils.h"
#include "crc8.h"
#include "main.h"

#define _XTAL_FREQ 40000000

#if defined(_18F2580)
    #define VSCP_PUTS_USART     putsUSART
    #define VSCP_WRITE_USART    putcUSART
    #define VSCP_BUSY_USART     BusyUSART
    #define VSCP_READ_USART     ReadUSART
    #define VSCP_OPEN_USART     OpenUSART
    #define VSCP_CLOSE_USART    CloseUSART
#else
    #define VSCP_PUTS_USART     puts1USART
    #define VSCP_WRITE_USART    putc1USART
    #define VSCP_BUSY_USART     Busy1USART
    #define VSCP_READ_USART     Read1USART
    #define VSCP_OPEN_USART     Open1USART
    #define VSCP_CLOSE_USART    Close1USART
#endif


uint8_t mode;                       // Unit working mode

// For interrupt CAN receive
uint32_t id;
uint8_t dlc;
uint8_t data[8];
ECAN_RX_MSG_FLAGS flags;

// Buffers
uint8_t serial_inputBuffer[ SIZE_SERIAL_INPUT_BUFFER ];
uint8_t can_inputBuffer[ 13 * SIZE_CAN_INPUT_FIFO ];
// Frame size = 13  ext-id(4) +  dlc(1) + data(8)

// fifos
fifo_t serialInputFifo;
fifo_t canInputFifo;

volatile uint32_t timer = 0;        // Millisecond timer
volatile uint32_t timekeeper = 0;   // Nill to measure time
volatile uint8_t ledFunctionality;  // Init. LED functionality
volatile uint16_t status_led_cnt;   // status LED counter
// increase externally by one every
// millisecond


BOOL bHex = FALSE;                  // Numerical printouts in hex.
BOOL bOpen = FALSE;                 // TRUE if i/f is open.
BOOL bSilent = FALSE;               // Open but no receive.
uint8_t rwtimeout;                  // Reg read/write timeout.
BOOL bLocalEcho = FALSE;            // True for local echo.
BOOL bTimestamp = FALSE;            // Send timestamped frames (CAN4VSCP)

volatile uint8_t fifo_canrxcount = 0; // Number of CAN messages in fifo

// Statistics
uint32_t cntTxFrames = 0;           // Number of sent CAN frames
uint32_t cntTxBytes = 0;            // Number of sent bytes
uint32_t cntRxFrames = 0;           // Number of received frames
uint32_t cntRxBytes = 0;            // Number of received frames

// Error statistics
uint32_t can_receiveOverruns = 0;   // CAN receive overruns (to user)
uint32_t can_transmitOverruns = 0;  // CAN send overruns.
uint32_t uart_receiveOverruns = 0;  // UART receive overruns (user)
uint32_t uart_transmitOverruns = 0; // UART send overruns.

uint8_t pos = 0;
char cmdbuf[80];
char wrkbuf[80];

// VSCP driver mode
BOOL stateVscpDriver = STATE_VSCP_SERIAL_DRIVER_WAIT_FOR_FRAME_START;
BOOL bDLE = FALSE;      // True if escape character has been received.
uint8_t sequencyno = 0; // Sequency number. Increases for every frame sent.

// * * * *   Capabilities   * * * *
vscp_serial_caps caps;   // Init. structure in main

// SLCAN mode
BOOL bInitiated = FALSE;
uint8_t nTimeStamp = SLCAN_TIMESTAMP_NOT_USED;

// Global event data
uint16_t vscpClass;
uint8_t vscpType;
uint8_t vscpNodeId;
uint8_t vscpPriority;
uint8_t vscpSize;
uint8_t vscpData[8];

#if defined(_18F2580)

#if defined(RELEASE)

#pragma config WDT = ON, WDTPS = 128
#pragma config OSC = HSPLL
#pragma config BOREN = BOACTIVE
#pragma config STVREN = ON
#pragma config BORV = 2		// 2.8V
#pragma config LVP = ON
#pragma config CPB = ON
#pragma config BBSIZ = 1024
#pragma config WRTD  = OFF

#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
#pragma config EBTR3 = OFF

#pragma config EBTRB = OFF

#else

#pragma config WDT = OFF
#pragma config OSC = HSPLL
#pragma config PWRT = ON
#pragma config BOREN = BOACTIVE
#pragma config STVREN = ON
#pragma config BORV = 2		// 2.8V
#pragma config LVP = OFF
#pragma config CPB = OFF
#pragma config WRTD  = OFF

#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
#pragma config EBTR3 = OFF

#pragma config EBTRB = OFF

#endif

#else if defined(_18F25K80) || defined(_18F26K80) || defined(_18F45K80) || defined(_18F46K80) || defined(_18F65K80) || defined(_18F66K80)


// CONFIG1L
#pragma config SOSCSEL = DIG    // RC0/RC is I/O
#pragma config RETEN = OFF      // Ultra low-power regulator is Disabled (Controlled by REGSLP bit).
#pragma config INTOSCSEL = HIGH // LF-INTOSC in High-power mode during Sleep.
#pragma config XINST = OFF      // No extended instruction set

// CONFIG1H
#pragma config FOSC = HS2       // Crystal 10 MHz
#pragma config PLLCFG = ON      // 4 x PLL

// CONFIG2H
#pragma config WDTPS = 1048576  // Watchdog prescaler
#pragma config BOREN = SBORDIS  // Brown out enabled
#pragma config BORV  = 0        // 3V

// CONFIG3H
#pragma config CANMX = PORTB    // ECAN TX and RX pins are located on RB2 and RB3, respectively.
#pragma config MSSPMSK = MSK7   // 7 Bit address masking mode.
#pragma config MCLRE = ON       // MCLR Enabled, RE3 Disabled.

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset enabled
#pragma config BBSIZ = BB2K     // Boot block size 2K

#ifdef DEBUG
#pragma config WDTEN = OFF      // WDT disabled in hardware; SWDTEN bit disabled.
#else
#pragma config WDTEN = ON       // WDT enabled in hardware;
#endif


#endif


///////////////////////////////////////////////////////////////////////////////
// Interrupt
//

void interrupt low_priority Interrupt()
{
    uint8_t c;

    // Check if the interrupt is caused by UART RX
    if ( 1 == PIR1bits.RCIF ) {

        c = VSCP_READ_USART();

        if ( 1 != fifo_write( &serialInputFifo, &c, 1 ) ) {
            // we have an overrun
            uart_receiveOverruns++;
        }

        // clear RX flag
        PIR1bits.RCIF = 0;

    }
    // Time0 overflow - 1ms timer
    else if (1 == INTCONbits.TMR0IF) {

        WriteTimer0(TIMER0_RELOAD_VALUE);

        timer++;
        timekeeper++;

        // Status LED
        status_led_cnt++;
        if ( ( STATUS_LED_VERY_SLOW_BLINK == ledFunctionality ) &&
                ( status_led_cnt > 1000 ) ) {
            LATC1 = ~LATC1;
            status_led_cnt = 0;
        }
        else if ( ( STATUS_LED_SLOW_BLINK == ledFunctionality ) &&
                ( status_led_cnt > 400 ) ) {
            LATC1 = ~LATC1;
            status_led_cnt = 0;
        }
        else if ( ( STATUS_LED_NORMAL_BLINK == ledFunctionality ) &&
                ( status_led_cnt > 100 ) ) {
            LATC1 = ~LATC1;
            status_led_cnt = 0;
        }
        else if ( ( STATUS_LED_FAST_BLINK == ledFunctionality ) &&
                (status_led_cnt > 70)) {
            LATC1 = ~LATC1;
            status_led_cnt = 0;
        }
        else if ( ( STATUS_LED_VERY_FAST_BLINK == ledFunctionality ) &&
                ( status_led_cnt > 40 ) ) {
            LATC1 = ~LATC1;
            status_led_cnt = 0;
        }
        else if ( STATUS_LED_ON == ledFunctionality ) {
            LATC1 = 1;
            status_led_cnt = 0;
        }
        else if ( STATUS_LED_OFF == ledFunctionality ) {
            LATC1 = 0;
            status_led_cnt = 0;
        }

        INTCONbits.TMR0IF = 0; // Clear Timer0 Interrupt Flag
    }

    // CAN error
    if ( 1 == ERRIF ) {

        // Check if we have CAN receive overflows
        if ( COMSTATbits.RXBNOVFL ) {
            can_receiveOverruns++;
            COMSTATbits.RXBNOVFL = 0;
        }

        ERRIF = 0;
    }

    // The error bits are checked outside of the error interrupt
    // to close error conditions correctly

    // Check CAN bus state and set LED status accordingly
    if (COMSTATbits.TXBO) {
        // Bus off
        ledFunctionality = STATUS_LED_VERY_FAST_BLINK;
    }
    else if (COMSTATbits.TXBP || COMSTATbits.RXBP) {
        // Bus passive
        ledFunctionality = STATUS_LED_FAST_BLINK;
    }
    else if (COMSTATbits.EWARN) {
        // Bus warning (RX/TX)
        ledFunctionality = STATUS_LED_NORMAL_BLINK;
    }
    else {
        // OK
        ledFunctionality = STATUS_LED_ON;
    }

    // Check for CAN RX interrupt
    if ( RXBnIF ) {

        if ( ECANReceiveMessage((unsigned long *)&id,
                                (BYTE*)&data,
                                (BYTE*)&dlc,
                                &flags) ) {

            // Put in canInputFifo
            if ( flags & ECAN_RX_OVERFLOW ) {
                if ( flags & ECAN_RX_XTD_FRAME ) can_receiveOverruns++;
            }
            else if ( !( flags & ECAN_RX_RTR_FRAME ) &&
                    ( flags & ECAN_RX_XTD_FRAME ) ) {

                if  ( ( fifo_getFree( &canInputFifo ) >= (5+dlc) ) ) {
                    uint8_t fail = FALSE;
                    if ( 4 != fifo_write( &canInputFifo, (uint8_t *)&id, 4 ) ) {
                        fail = TRUE;
                    }
                    if ( 1 != fifo_write( &canInputFifo, &dlc, 1 ) ) {
                        fail = TRUE;
                    }
                    if ( dlc != fifo_write( &canInputFifo, (uint8_t *)&data, dlc ) ) {
                        fail = TRUE;
                    }

                    fifo_canrxcount++;   // Another CAN frame in the fifo

                    if ( fail ) {
                        can_receiveOverruns++;
                    }

                }
                else {
                    can_receiveOverruns++;
                }

            }

        }

        // Not needed rested in ECAN
        RXBnIF = 0; // Clear CAN0 Interrupt Flag

    }



}

///////////////////////////////////////////////////////////////////////////////
// main
//

int main(int argc, char** argv)
{
    // Init. capabilities
    caps.maxVscpFrames = 1;
    caps.maxCanalFrames = 1;        // Max frames that will be sent to host
                                    // can be increase after a capability
                                    // request.

    // Init. fifos
    fifo_init( &serialInputFifo, serial_inputBuffer, sizeof ( serial_inputBuffer)); // UART receive
    fifo_init( &canInputFifo, can_inputBuffer, sizeof ( can_inputBuffer ) );        // CAN receive

    // Init. CRC table
    init_crc8();

    // If EEPROM is not initialized then restore defaults
    // before the system is initialized
    if ( ( 0x55 != eeprom_read(MODULE_EEPROM_INIT_BYTE1) ) ||
            ( 0xaa != eeprom_read(MODULE_EEPROM_INIT_BYTE2) ) ) {
        vscp_restoreDefaults();
    }

    // Initialize
    init();

    // This delay is needed to prevent garbage on serial line on startup
    // Delay http://microchip.wikidot.com/faq:26
    __delay_ms(10);

    //mode = WORKING_MODE_VSCP_DRIVER;

    VSCP_PUTS_USART((char*) "\r\nFrankfurt RS-232 CAN4VSCP module\r\n");
    VSCP_PUTS_USART((char*) "Copyright (C) 2014-2015 Paradise of the Frog AB, Sweden\r\n");
    VSCP_PUTS_USART((char*) "http://www.paradiseofthefrog.com\r\n");
    printFirmwareVersion();
    printMode();

    // Wait for init. sequency to bring interface to verbose mode
    // 'v' pressed within three seconds
    if ( WORKING_MODE_VERBOSE != mode ) {

        uint8_t c;

        VSCP_PUTS_USART((char *) "Press 'v' within three seconds to enter verbose mode\r\n");

        timekeeper = 0;
        ledFunctionality = STATUS_LED_VERY_FAST_BLINK;
        while (timekeeper < 3000) {
            ClrWdt(); // Feed the dog
            di();       // Disable interrupt
            if (1 == fifo_read(&serialInputFifo, &c, 1)) {
                ei(); // Enable interrupt
                if ('v' == c) {
                    mode = WORKING_MODE_VERBOSE;
                    VSCP_PUTS_USART((char *) "Temporary verbose mode set\r\n");
                    break;
                }
            }
            ei(); // Enable interrupt
        }
    }

    ledFunctionality = STATUS_LED_ON;

    // Set mask filter in requested startup state

    // Set the port in the requested start up state
    switch ( eeprom_read( MODULE_EEPROM_STARTUP_OPEN ) ) {

        case STARTUP_IFMODE_OPEN:
            bSilent = FALSE;
            ECANSetOperationMode( ECAN_OP_MODE_NORMAL );
            break;

        case STARTUP_IFMODE_SILENT:
            bSilent = TRUE;
            ECANSetOperationMode( ECAN_OP_MODE_NORMAL );
            break;

        case STARTUP_IFMODE_LISTEN:
            bSilent = FALSE;
            ECANSetOperationMode( ECAN_OP_MODE_LISTEN );
            break;

        case STARTUP_IFMODE_LOOPBACK:
            bSilent = FALSE;
            ECANSetOperationMode( ECAN_OP_MODE_LOOP );
            break;

        default:
            ECANSetOperationMode( ECAN_INIT_DISABLE );
            break;
    }


    ///////////////////////////////////////////////////////////////////////////
    //                            Work loop
    ///////////////////////////////////////////////////////////////////////////

    while ( TRUE ) {

        ClrWdt(); // Feed the dog

        // Check for UART overflow. This should not happen in normal
        // mode but does while debugging.
        if ( RCSTAbits.OERR ) {
            RCSTAbits.CREN = 0;
            RCSTAbits.CREN = 1;
        }

        di();
        if ( fifo_getFree( &serialInputFifo ) < 100 ) {
            PORTCbits.RC4 = 0;
        }
        else {
            PORTCbits.RC4 = 1;
        }
        ei();

        if (WORKING_MODE_VERBOSE == mode) {
            doModeVerbose();
        }
        else if (WORKING_MODE_VSCP_DRIVER == mode) {
            doModeVscp();
        }
        else if (WORKING_MODE_SL_DRIVER == mode) {
            doModeSLCAN();
        }
        else if (WORKING_MODE_VSCP_NODE == mode) {
            doModeVscpNode();
        }
        else {
            doModeVscp();      // Just in case
        }

    } // While

    return (EXIT_SUCCESS);
}



///////////////////////////////////////////////////////////////////////////////
// Init. - Initialization Routine
//

void init()
{
    //uint8_t msgdata[ 8 ];
    init_app_ram();

    // Setup system clock
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
    OSCTUNEbits.PLLEN = 1; // Turn on PLL

    // RA4 input (VCAP for PIC18F26K80))
#if defined(_18F2580)    
    TRISA4 = 1;
#endif    

    TRISB2 = 0; // CAN TX
    TRISB3 = 1; // CAN RX

    TRISC1 = 0; // Status LED
    TRISC4 = 0; // Output: CTS
    TRISC5 = 1; // Input: RTS
    TRISC6 = 0; // UART TX pin set as output
    TRISC7 = 1; // UART RX pin set as input

    PORTCbits.RC4 = 1; // Activate clear to send

    // Initialise UART
    // 230400 (10), 500000 (4) and 625000 (3) 115200 (20)
    VSCP_OPEN_USART( USART_TX_INT_OFF &
                    USART_RX_INT_ON &
                    USART_ASYNCH_MODE &
                    USART_EIGHT_BIT &
                    USART_CONT_RX &
                    USART_BRGH_HIGH,
                    BAUDRATE_115200 );

    //baudUSART( BAUD_8_BIT_RATE | BAUD_AUTO_OFF );

    RCIF = 0; // Reset RX pin flag
    RCIP = 0; // Not high priority
    RCIE = 1; // Enable RX interrupt
    PEIE = 1; // Enable peripheral interrupt (serial port is a peripheral)

    // Initialize 1 ms timer
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_8);
    WriteTimer0(TIMER0_RELOAD_VALUE);

    // Initialize microsecond timer  10 MHz
#if defined(_18F2580)
    OpenTimer3( T3_SOURCE_INT & T3_PS_1_1 & T3_16BIT_RW & T3_SYNC_EXT_OFF & TIMER_INT_OFF );
#else
    OpenTimer3( T3_SOURCE_FOSC_4 & T3_PS_1_1 & T3_16BIT_RW & T3_SYNC_EXT_OFF & TIMER_INT_OFF, 0 );
#endif
    WriteTimer3( 0x0000 );

    // Initialize CAN
    ECANInitialize();

    // Must be in config. mode to change many of settings.
    //ECANSetOperationMode(ECAN_OP_MODE_CONFIG);

    // Return to Normal mode to communicate.
    //ECANSetOperationMode(ECAN_OP_MODE_NORMAL);

    /*
    msgdata[ 0 ] = 1;
    msgdata[ 1 ] = 2;
    msgdata[ 2 ] = 3;
    msgdata[ 3 ] = 4;
    msgdata[ 4 ] = 5;
    msgdata[ 5 ] = 6;
    msgdata[ 6 ] = 7;
    msgdata[ 7 ] = 8;

    if ( sendCANFrame( 0x123, 8, msgdata ) ) {
        ;
    }
     */

    // CAN interrupt
    RXBnIE = 1;
    ERRIE = 1;

    ei(); // Remember the master switch for interrupt?

}

///////////////////////////////////////////////////////////////////////////////
// init._app._ram
//

void init_app_ram(void)
{
    // Nill error counters
    can_receiveOverruns = 0;
    can_transmitOverruns = 0;
    uart_receiveOverruns = 0;
    uart_transmitOverruns = 0;
    timer = 0;

    bHex = eeprom_read(MOUDLE_EEPROM_PRINTOUT_IN_HEX);
    mode = eeprom_read(MODULE_EEPROM_STARTUP_MODE);
    bLocalEcho = eeprom_read(MODULE_LOCAL_ECHO);
    bTimestamp = eeprom_read(MODULE_TIMESTAMP);

    rwtimeout = eeprom_read(MODULE_EEPROM_RW_TIMEOUT);

    // Set filter/mask setup values

    // Must be in Config mode to change settings.
    ECANSetOperationMode(ECAN_OP_MODE_CONFIG);

    for (uint8_t i=0; i<15; i++ ) {
        setFilter( i,
                    ((uint32_t)( eeprom_read( MODULE_EEPROM_FILTER0 + i*4 )) << 24 ) |
                    ((uint32_t)( eeprom_read( MODULE_EEPROM_FILTER0 + 1 + i*4 )) << 16 ) |
                    ((uint32_t)( eeprom_read( MODULE_EEPROM_FILTER0 + 2 + i*4 )) << 8 ) |
                    ((uint32_t)( eeprom_read( MODULE_EEPROM_FILTER0 + 3 + i*4 )) ), FALSE );
    }

    ECANSetRXM0Value( ((uint32_t)( eeprom_read( MODULE_EEPROM_MASK0 )) << 24 ) |
                        ((uint32_t)( eeprom_read( MODULE_EEPROM_MASK0 + 1 )) << 16 ) |
                        ((uint32_t)( eeprom_read( MODULE_EEPROM_MASK0 + 2 )) << 8 ) |
                        (( eeprom_read( MODULE_EEPROM_MASK0 + 3 )) ),
                      ECAN_MSG_XTD );

    ECANSetRXM1Value( ((uint32_t)( eeprom_read( MODULE_EEPROM_MASK1 )) << 24 ) |
                        ((uint32_t)( eeprom_read( MODULE_EEPROM_MASK1 + 1 )) << 16 ) |
                        ((uint32_t)( eeprom_read( MODULE_EEPROM_MASK1 + 2 )) << 8 ) |
                        ((uint32_t)( eeprom_read( MODULE_EEPROM_MASK1 + 3 )) ),
                      ECAN_MSG_XTD );

    ECANSetOperationMode(ECAN_INIT_DISABLE);
}

///////////////////////////////////////////////////////////////////////////////
// init_app_eeprom
//

void init_app_eeprom(void)
{
    uint8_t i;

    eeprom_write( MODULE_EEPROM_INIT_BYTE1, 0x55 );
    eeprom_write( MODULE_EEPROM_INIT_BYTE2, 0xAA );
    eeprom_write( MODULE_EEPROM_STARTUP_MODE, WORKING_MODE_VERBOSE );
    eeprom_write( MOUDLE_EEPROM_SLCAN_TIMESTAMP, SLCAN_TIMESTAMP_NOT_USED );
    eeprom_write( MOUDLE_EEPROM_PRINTOUT_IN_HEX, NUMERICAL_PRINTOUTMODE_DECIMAL );
    eeprom_write( MODULE_EEPROM_RW_TIMEOUT, DEFAULT_REGISTER_RW_TIMEOUT );

    // Set all filters to 0xff
    for ( i = MODULE_EEPROM_FILTER0; i < (MODULE_EEPROM_FILTER15 + 4); i++ ) {
        eeprom_write( MODULE_EEPROM_FILTER0, 0xFF );
    }

    // Set all masks 0x00 - All allowed
    for (i = MODULE_EEPROM_MASK0; i < (MODULE_EEPROM_MASK1 + 4); i++) {
        eeprom_write( MODULE_EEPROM_MASK0, 0x00 );
    }

    eeprom_write( MODULE_LOCAL_ECHO, 0 );
    eeprom_write( MODULE_TIMESTAMP, 0 );

}


///////////////////////////////////////////////////////////////////////////////
// checkCANBusState
//
// Set the global ledFunctionality as of status.
//

void checkCANBusState(void)
{
    // Check CAN bus state and set LED status accordingly
    if ( COMSTATbits.TXBO ) {
        // Bus off
        ledFunctionality = STATUS_LED_VERY_FAST_BLINK;
    }
    else if ( COMSTATbits.TXBP || COMSTATbits.RXBP ) {
        // Bus passive
        ledFunctionality = STATUS_LED_FAST_BLINK;
    }
    else if ( COMSTATbits.EWARN ) {
        // Bus warning (RX/TX)
        ledFunctionality = STATUS_LED_NORMAL_BLINK;
    }
    else {
        // OK
        ledFunctionality = STATUS_LED_ON;
    }
}


///////////////////////////////////////////////////////////////////////////////
// doModeVerbose
//

void doModeVerbose(void)
{
    uint8_t c;
    uint8_t i;

    // Fetch possible VSCP event
    if (!bSilent) {
        receivePrintEventVerbose();
    }

    // Disable interrupt
    di();

    if (1 == fifo_read(&serialInputFifo, &c, 1)) {

        // Enable interrupt again
        ei();

        // If local echo
        if ( bLocalEcho ) {
            VSCP_WRITE_USART( c );
            while (VSCP_BUSY_USART());
            if ( 0x0d == c ) {
                VSCP_WRITE_USART( 0x0a );
                while (VSCP_BUSY_USART());
            }
        }

        // Save
        cmdbuf[ pos++ ] = c;
        if (pos >= sizeof( cmdbuf ) ) {
            // We have garbage. Start all over again to
            // look for valid content
            pos = 0;
        }

        // Check if we have a command
        if (0x0d == c) {

            cmdbuf[ pos ] = 0;

            // Enter bootloader
            if (cmdbuf == stristr(cmdbuf, "BOOT")) {
                VSCP_PUTS_USART((char *) "Will enter bootloader now...\r\n");
                eeprom_write(MODULE_EEPROM_BOOTLOADER_FLAG, 0xFF);
                Reset();
            }
            // Open interface
            else if (cmdbuf == stristr(cmdbuf, "OPEN")) {
                bSilent = FALSE;
                ECANSetOperationMode(ECAN_OP_MODE_NORMAL);
                VSCP_PUTS_USART((char *) "+OK\r\n");
            }
            // Open interface but don't show receive frames
            else if (cmdbuf == stristr(cmdbuf, "SILENT")) {
                bSilent = TRUE;
                ECANSetOperationMode(ECAN_OP_MODE_NORMAL);
                VSCP_PUTS_USART((char *) "+OK\r\n");
            }
            // Close interface
            else if (cmdbuf == stristr(cmdbuf, "CLOSE")) {
                bSilent = TRUE;
                ECANSetOperationMode( ECAN_OP_MODE_CONFIG );
                VSCP_PUTS_USART((char *) "+OK\r\n");
            }
            // Open interface in listen only mode
            else if (cmdbuf == stristr(cmdbuf, "LOOPBACK")) {
                bSilent = FALSE;
                ECANSetOperationMode(ECAN_OP_MODE_LOOP);
                VSCP_PUTS_USART((char *) "+OK\r\n");
            }
            // Open interface in loopback mode
            else if (cmdbuf == stristr(cmdbuf, "LISTEN")) {
                bSilent = FALSE;
                ECANSetOperationMode(ECAN_OP_MODE_LISTEN);
                VSCP_PUTS_USART((char *) "+OK\r\n");
            }
            // Print version
            else if (cmdbuf == stristr(cmdbuf, "VERSION")) {
                printFirmwareVersion();
                VSCP_PUTS_USART((char *) "+OK\r\n");
            }
            // Set interface mode
            else if (cmdbuf == stristr(cmdbuf, "IFMODE")) {
                ECAN_OP_MODE ifmode = ECANGetOperationMode();
                if ((ECAN_OP_MODE_NORMAL == ifmode) & !bSilent) {
                    VSCP_PUTS_USART((char *) "+OK - Normal mode\r\n");
                }
                else if ((ECAN_OP_MODE_NORMAL == ifmode) & bSilent) {
                    VSCP_PUTS_USART((char *) "+OK - Silent mode\r\n");
                }
                else if (ECAN_OP_MODE_SLEEP == ifmode) {
                    VSCP_PUTS_USART((char *) "+OK - Sleep mode\r\n");
                }
                else if (ECAN_OP_MODE_LOOP == ifmode) {
                    VSCP_PUTS_USART((char *) "+OK - Loopback mode\r\n");
                }
                else if (ECAN_OP_MODE_LISTEN == ifmode) {
                    VSCP_PUTS_USART((char *) "+OK - Listen only mode\r\n");
                }
                else if (ECAN_OP_MODE_CONFIG == ifmode) {
                    VSCP_PUTS_USART((char *) "+OK - Closed mode\r\n");
                }
                else if (ECAN_OP_MODE_BITS == ifmode) {
                    VSCP_PUTS_USART((char *) "+OK - Bits mode\r\n");
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - Unknown mode\r\n");
                }
            }
            // Send a VSCP event
            // Format priority,class,type,nodeid,count,data,,,
            else if (cmdbuf == stristr(cmdbuf, "TX")) {

                char *p = strtok(cmdbuf, ",");

                // Priority
                vscpPriority = 3; // Normal priority
                if (NULL != p) {
                    vscpPriority = atoi(p);
                }

                // Class
                vscpClass = 0;
                if (NULL != (p = strtok(NULL, ","))) {
                    vscpClass = atoi(p);
                }

                // Type
                vscpType = 0;
                if (NULL != (p = strtok(NULL, ","))) {
                    vscpType = atoi(p);
                }

                // Node ID
                vscpNodeId = 0;
                if (NULL != (p = strtok(NULL, ","))) {
                    vscpNodeId = atoi(p);
                }

                // Data Count
                vscpSize = 0;
                if (NULL != (p = strtok(NULL, ","))) {
                    vscpSize = atoi(p);
                }

                memset( vscpData, 0, 8 );
                for (i = 0; i < vscpSize; i++) {
                    if (NULL != (p = strtok(NULL, ","))) {
                        vscpData[ i ] = atoi(p);
                    }
                    else {
                        break;
                    }
                }

                if ( sendVSCPFrame(vscpClass,
                                    vscpType,
                                    vscpNodeId,
                                    vscpPriority,
                                    vscpSize,
                                    vscpData ) ) {
                    // Statistics
                    cntTxFrames++;
                    cntTxBytes += vscpSize;
                    VSCP_PUTS_USART((char *) "+OK\r\n");
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - Failed to send event.\r\n");
                }
            }
            // Receive event
            else if (cmdbuf == stristr(cmdbuf, "RX")) {
                if (receivePrintEventVerbose()) {
                    VSCP_PUTS_USART((char *) "+OK\r\n");
                }
                else {
                    VSCP_PUTS_USART((char *) "+OK - no events\r\n");
                }
            }
            // Print out statistics
            else if (cmdbuf == stristr(cmdbuf, "STAT")) {
                printStatistics();
                VSCP_PUTS_USART((char *) "+OK\r\n");
            }
            // List error counters
            else if (cmdbuf == stristr(cmdbuf, "ERR")) {
                printErrors();
                VSCP_PUTS_USART((char *) "+OK\r\n");
            }
            // Help
            else if (cmdbuf == stristr(cmdbuf, "HELP")) {
                printHelp();
                VSCP_PUTS_USART((char *) "+OK\r\n");
            }
            // Find nodes on CAN4VSCP bus
            else if (cmdbuf == stristr(cmdbuf, "FIND")) {
                findNodes();
                VSCP_PUTS_USART((char *) "+OK\r\n");
            }
            // Read register of node
            //      RREG nodeid [page:]reg count
            else if (cmdbuf == stristr(cmdbuf, "RREG")) {

                uint8_t i;
                uint8_t nodeid;
                uint8_t page = 0;
                uint8_t reg;
                uint8_t value;
                uint8_t count = 1;

                if ( ECAN_OP_MODE_NORMAL != ECANGetOperationMode() ) {
                    VSCP_PUTS_USART( (char *)STR_ERR_ONLY_IF_OPEN );
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                strcpy(cmdbuf, cmdbuf + 5);
                char *p = strtok(cmdbuf, " ");
                if (NULL != p) {
                    nodeid = atoi(p);
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - Needs nodeid\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                if (NULL != (p = strtok(NULL, " "))) {

                    char *preg;
                    if (preg = strchr(p, ':')) {
                        page = atoi(p);
                        reg = atoi(preg + 1);
                    }
                    else {
                        reg = atoi(p);
                    }

                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - Needs [page:]register\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                // Count
                if (NULL != (p = strtok(NULL, " "))) {
                    count = atoi(p);
                    if (0 == count) count = 1;
                }

                BOOL rv = TRUE;
                for ( i = 0; i < count; i++ ) {

                    if ( readRegisterExtended( nodeid,
                                                page,
                                                (reg + i) & 0xff,
                                                rwtimeout,
                                                &value ) ) {
                        VSCP_PUTS_USART((char *) "+OK - nodeid=");
                        sprintf(wrkbuf, bHex ? "0x%02X - " : "%d - ", nodeid);
                        VSCP_PUTS_USART(wrkbuf);
                        VSCP_PUTS_USART((char *) "Value for reg ");
                        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", page);
                        VSCP_PUTS_USART(wrkbuf);
                        VSCP_WRITE_USART(':');
                        while (VSCP_BUSY_USART());
                        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", (reg + i) & 0xff);
                        VSCP_PUTS_USART(wrkbuf);
                        VSCP_PUTS_USART((char *) " = ");
                        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", value);
                        VSCP_PUTS_USART(wrkbuf);
                        while (VSCP_BUSY_USART());
                        VSCP_PUTS_USART((char *)"\t\'");
                        if ((value > 32) && (value < 127)) {
                            VSCP_WRITE_USART(value);
                        }
                        else {
                            VSCP_WRITE_USART('.');
                        }
                        while (VSCP_BUSY_USART());
                        VSCP_PUTS_USART((char *)"\' \t");
                        printBinary(value);
                        VSCP_PUTS_USART((char *) "\r\n");
                    }
                    else {
                        rv = FALSE;
                        VSCP_PUTS_USART((char *) "-ERROR - nodeid=");
                        sprintf(wrkbuf, bHex ? "0x%02X - " : "%d - ", nodeid);
                        VSCP_PUTS_USART(wrkbuf);
                        VSCP_PUTS_USART((char *) "Unable to read register ");
                        while (VSCP_BUSY_USART());
                        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", page);
                        VSCP_PUTS_USART(wrkbuf);
                        VSCP_WRITE_USART(':');
                        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", (reg + i) & 0xff);
                        VSCP_PUTS_USART(wrkbuf);

                        VSCP_PUTS_USART((char *) "\r\n");
                    }
                }

                if (rv) {
                    VSCP_PUTS_USART((char *) "+OK\r\n");
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - One or more register(s) could not be read.\r\n");
                }

            }
            // Write register of node
            //      WREG nodeid [page:]reg value1 [value2 value3 value4 value5...]
            else if (cmdbuf == stristr(cmdbuf, "WREG")) {

                uint8_t nodeid;
                uint8_t page = 0;
                uint8_t reg;
                uint8_t value;

                if ( ECAN_OP_MODE_NORMAL != ECANGetOperationMode() ) {
                    VSCP_PUTS_USART( (char *)STR_ERR_ONLY_IF_OPEN );
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                strcpy(cmdbuf, cmdbuf + 5);
                char *p = strtok(cmdbuf, " ");
                if (NULL != p) {
                    nodeid = atoi(p);
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - Needs nodeid\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                if ( NULL != ( p = strtok(NULL, " ") ) ) {

                    char *preg;
                    if (preg = strchr(p, ':')) {
                        page = atoi(p);
                        reg = atoi(preg + 1);
                    }
                    else {
                        reg = atoi(p);
                    }

                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - Needs [page:]register\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                // Value
                if (NULL != (p = strtok(NULL, " "))) {
                    value = atoi(p);
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - Need a value\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                if (writeRegisterExtended(nodeid,
                        page,
                        reg,
                        rwtimeout,
                        &value)) {
                    VSCP_PUTS_USART((char *) "+OK - Value written successfully for ");
                    VSCP_PUTS_USART((char *) "nodeid=");
                    sprintf(wrkbuf, bHex ? "0x%02X\n" : "%d\n", nodeid);
                    VSCP_PUTS_USART(wrkbuf);
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - Failed to write value for ");
                    VSCP_PUTS_USART((char *) "nodeid=");
                    sprintf(wrkbuf, bHex ? "0x%02X\n" : "%d\n", nodeid);
                }

            }
            // Read full node info
            //      INFO node-id
            else if (cmdbuf == stristr(cmdbuf, "INFO")) {

                uint8_t nodeid;
                uint8_t value;

                if ( ECAN_OP_MODE_NORMAL != ECANGetOperationMode() ) {
                    VSCP_PUTS_USART( (char *)STR_ERR_ONLY_IF_OPEN );
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                strcpy(cmdbuf, cmdbuf + 5);
                nodeid = atoi(cmdbuf);

                VSCP_PUTS_USART((char *) "Info for node id = ");
                sprintf(wrkbuf, bHex ? "0x%02X" : "%d", nodeid);
                VSCP_PUTS_USART(wrkbuf);
                VSCP_PUTS_USART((char *) "\r\n");

                if (!readRegisterExtended(nodeid,
                            0,
                            0xd0,
                            rwtimeout,
                            &value)) {
                    VSCP_PUTS_USART((char *) "-ERROR - Node not found.\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                printNodeFirmwareVersion(nodeid);
                printGUID(nodeid);
                printMDF(nodeid);
                VSCP_PUTS_USART((char *) "+OK\r\n");
            }
            // Set filter
            //  FILTER filterno,prio,class,type,nodeid
            //  filterno = 0-15
            else if (cmdbuf == stristr(cmdbuf, "FILTER")) {

                // RXF0 - RXF15
                uint8_t filterno;
                uint8_t filter_priority;
                uint16_t filter_class;
                uint8_t filter_type;
                uint8_t filter_nodeid;
                BOOL bPersistent = FALSE;

                strcpy(cmdbuf, cmdbuf + 8);
                char *p = strtok(cmdbuf, ",");
                if (NULL != p) {
                    filterno = atoi(p);
                    if (filterno > 15) {
                        VSCP_PUTS_USART((char *) "-ERROR - Filter number can only be set to a value between 0-15.\r\n");
                        memset( cmdbuf, 0, sizeof( cmdbuf ) );
                        pos = 0; // Start again
                        return;
                    }
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - No filter number specified.\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                // Prio
                p = strtok(NULL, ",");
                if (NULL != p) {
                    filter_priority = atoi(p);
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - filter for priority is missing\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                // Class
                p = strtok(NULL, ",");
                if (NULL != p) {
                    filter_class = atoi(p);
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - filter for class is missing\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                // Type
                p = strtok(NULL, ",");
                if (NULL != p) {
                    filter_type = atoi(p);
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - filter for type is missing\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                // Node id
                p = strtok(NULL, ",");
                if (NULL != p) {
                    filter_nodeid = atoi(p);
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - filter for nide id is missing\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                // persistent i.e. stored in EEPROM
                p = strtok(NULL, ",");
                if ( NULL != stristr(p, "PERSISTENT") ) {
                    bPersistent = TRUE;
                }

                // Must be in Config mode to change settings.
                ECANSetOperationMode(ECAN_OP_MODE_CONFIG);

                uint32_t id = ((uint32_t)filter_priority << 26) |
                        ((uint32_t)filter_class << 16) |
                        ((uint32_t)filter_type << 8) |
                        filter_nodeid;

                setFilter(filterno, id, bPersistent );

                // Go back to normal mode
                ECANSetOperationMode(ECAN_OP_MODE_NORMAL);

                VSCP_PUTS_USART((char *) "+OK\r\n");
            }
            // Set Mask
            //  MASK maskno,prio,class,type,nodeid
            //  maskno = 0 or 1
            else if (cmdbuf == stristr(cmdbuf, "MASK")) {

                uint8_t maskno;
                uint8_t mask_priority;
                uint16_t mask_class;
                uint8_t mask_type;
                uint8_t mask_nodeid;
                BOOL bPersistent = FALSE;

                strcpy(cmdbuf, cmdbuf + 8);
                char *p = strtok(cmdbuf, ",");
                if (NULL != p) {
                    maskno = atoi(p);
                    if (maskno > 1) {
                        VSCP_PUTS_USART((char *) "-ERROR - Mask number can only be set as 0 or 1.\r\n");
                        memset( cmdbuf, 0, sizeof( cmdbuf ) );
                        pos = 0; // Start again
                        return;
                    }
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - No mask number specified.\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                // Prio
                p = strtok(NULL, ",");
                if (NULL != p) {
                    mask_priority = atoi(p);
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - mask for priority is missing\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                // Class
                p = strtok(NULL, ",");
                if (NULL != p) {
                    mask_class = atoi(p);
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - mask for class is missing\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                // Type
                p = strtok(NULL, ",");
                if (NULL != p) {
                    mask_type = atoi(p);
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - mask for type is missing\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }

                // Node id
                p = strtok(NULL, ",");
                if (NULL != p) {
                    mask_nodeid = atoi(p);
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - mask for nide id is missing\r\n");
                    memset( cmdbuf, 0, sizeof( cmdbuf ) );
                    pos = 0; // Start again
                    return;
                }
                uint32_t id = ((uint32_t)mask_priority << 26) |
                        ((uint32_t)mask_class << 16) |
                        ((uint32_t)mask_type << 8) |
                        mask_nodeid;

                // Must be in Config mode to change many of settings.
                ECANSetOperationMode(ECAN_OP_MODE_CONFIG);

                maskno ? ECANSetRXM0Value(id, ECAN_MSG_XTD) :
                        ECANSetRXM1Value(id, ECAN_MSG_XTD);

                // Go back to normal mode
                ECANSetOperationMode(ECAN_OP_MODE_NORMAL);

                if ( bPersistent ) {
                    if ( 0 == maskno ) {
                        eeprom_write( MODULE_EEPROM_MASK0, ( ( id >> 24 ) & 0xff ) );
                        eeprom_write( MODULE_EEPROM_MASK0 + 1, ( ( id >> 16 ) & 0xff ) );
                        eeprom_write( MODULE_EEPROM_MASK0 + 2, ( ( id >> 8 ) & 0xff ) );
                        eeprom_write( MODULE_EEPROM_MASK0 + 3, ( id & 0xff ) );
                    }
                    else {
                        eeprom_write( MODULE_EEPROM_MASK1, ( ( id >> 24 ) & 0xff ) );
                        eeprom_write( MODULE_EEPROM_MASK1 + 1, ( ( id >> 16 ) & 0xff ) );
                        eeprom_write( MODULE_EEPROM_MASK1 + 2, ( ( id >> 8 ) & 0xff ) );
                        eeprom_write( MODULE_EEPROM_MASK1 + 3, ( id & 0xff ) );
                    }
                }

                VSCP_PUTS_USART((char *) "+OK\r\n");

            }
            // Set Configuration
            //      RWTIMEOUT n  - rreg/wreg timeout
            //      STARTIF [CONFIG|CLOSE|OPEN|LISTEN|LOOPBACK]
            //      MODE [VERBOSE|VSCP|SLCAN]
            //      HEX
            //      DECIMAL
            //      ECHO
            //      TIMESTAMP
            //      BAUDRATE
            //      DEAFULTS
            //      ?
            else if (cmdbuf == stristr(cmdbuf, "SET ")) {

                // Remove "SET "
                strcpy(cmdbuf, cmdbuf + 4);

                // Hex - Numbers in Hex from now on
                if (cmdbuf == stristr(cmdbuf, "HEX")) {
                    bHex = TRUE;
                    eeprom_write(MOUDLE_EEPROM_PRINTOUT_IN_HEX, NUMERICAL_PRINTOUTMODE_HEX);
                    VSCP_PUTS_USART((char *) "+OK - Numerical output now in hexadecimal\r\n");
                }
                // Decimal - numbers in decimal from now on
                else if (cmdbuf == stristr(cmdbuf, "DECIMAL")) {
                    bHex = FALSE;
                    eeprom_write(MOUDLE_EEPROM_PRINTOUT_IN_HEX, NUMERICAL_PRINTOUTMODE_DECIMAL);
                    VSCP_PUTS_USART((char *) "+OK - Numerical output now in decimal\r\n");
                }
                else if (0 != stristr(cmdbuf, "RWTIMEOUT ")) {
                    strcpy(cmdbuf, cmdbuf + 10);
                    rwtimeout = atoi(cmdbuf);
                    if (rwtimeout < DEFAULT_REGISTER_RW_TIMEOUT) {
                        rwtimeout = DEFAULT_REGISTER_RW_TIMEOUT;
                    }
                    eeprom_write(MODULE_EEPROM_RW_TIMEOUT, rwtimeout);
                    VSCP_PUTS_USART((char *) "+OK\r\n");
                }
                // Interface state to use at start up
                else if (0 != stristr(cmdbuf, "STARTIF ")) {
                    strcpy(cmdbuf, cmdbuf + 8);
                    if (0 != stristr(cmdbuf, "CLOSE")) {
                        eeprom_write( MODULE_EEPROM_STARTUP_OPEN, STARTUP_IFMODE_CLOSE );
                    }
                    else if (0 != stristr(cmdbuf, "OPEN")) {
                        eeprom_write( MODULE_EEPROM_STARTUP_OPEN, STARTUP_IFMODE_OPEN );
                    }
                    else if (0 != stristr(cmdbuf, "SILENT")) {
                        eeprom_write( MODULE_EEPROM_STARTUP_OPEN, STARTUP_IFMODE_SILENT );
                    }
                    else if (0 != stristr(cmdbuf, "LISTEN")) {
                        eeprom_write( MODULE_EEPROM_STARTUP_OPEN, STARTUP_IFMODE_LISTEN );
                    }
                    else if (0 != stristr(cmdbuf, "LOOPBACK")) {
                        eeprom_write( MODULE_EEPROM_STARTUP_OPEN, STARTUP_IFMODE_LOOPBACK );
                    }
                }
                else if (0 != stristr(cmdbuf, "MODE ")) {
                    strcpy(cmdbuf, cmdbuf + 5);
                    if (0 != stristr(cmdbuf, "VERBOSE")) {
                        mode = WORKING_MODE_VERBOSE;
                        eeprom_write(MODULE_EEPROM_STARTUP_MODE, WORKING_MODE_VERBOSE);
                        VSCP_PUTS_USART((char *) "+OK - Mode is now verbose\r\n");
                    }
                    else if (0 != stristr(cmdbuf, "VSCP")) {
                        mode = WORKING_MODE_VSCP_DRIVER;
                        eeprom_write(MODULE_EEPROM_STARTUP_MODE, WORKING_MODE_VSCP_DRIVER);
                        VSCP_PUTS_USART((char *) "+OK - Mode is now VSCP Driver\r\n");
                    }
                    else if (0 != stristr(cmdbuf, "SLCAN")) {
                        mode = WORKING_MODE_SL_DRIVER;
                        eeprom_write(MODULE_EEPROM_STARTUP_MODE, WORKING_MODE_SL_DRIVER);
                        VSCP_PUTS_USART((char *) "+OK - Mode is now SLCAN\r\n");
                    }
                }
                // Enable/disable local echo  'echo on|off'
                else if (cmdbuf == stristr(cmdbuf, "ECHO ")) {
                    strcpy(cmdbuf, cmdbuf + 5);
                    if (0 != stristr(cmdbuf, "ON")) {
                        bLocalEcho = TRUE;
                        eeprom_write( MODULE_LOCAL_ECHO, 1 );
                        VSCP_PUTS_USART((char *) "+OK - Local echo on\r\n");
                    }
                    else if (0 != stristr(cmdbuf, "OFF")) {
                        bLocalEcho = FALSE;
                        eeprom_write( MODULE_LOCAL_ECHO, 0 );
                        VSCP_PUTS_USART((char *) "+OK - Local echo off\r\n");
                    }
                    else {
                        VSCP_PUTS_USART((char *) "+ERROR - Wrong argument to 'set echo'.\r\n");
                    }
                }
                // Enable/disable timestamp  'echo on|off'
                else if (cmdbuf == stristr(cmdbuf, "TIMESTAMP ")) {
                    strcpy(cmdbuf, cmdbuf + 10);
                    if (0 != stristr(cmdbuf, "ON")) {
                        bTimestamp = TRUE;
                        eeprom_write( MODULE_TIMESTAMP, 1 );
                        VSCP_PUTS_USART((char *) "+OK - Timestamp on\r\n");
                    }
                    else if (0 != stristr(cmdbuf, "OFF")) {
                        bTimestamp = FALSE;
                        eeprom_write( MODULE_TIMESTAMP, 0 );
                        VSCP_PUTS_USART((char *) "+OK - Timestamp off\r\n");
                    }
                    else {
                        VSCP_PUTS_USART((char *) "+ERROR - Wrong argument to 'set timestamp'.\r\n");
                    }
                }
                // Set baudrate
                else if (cmdbuf == stristr(cmdbuf, "BAUDRATE ")) {

                    uint32_t baudcode;
                    strcpy( cmdbuf, cmdbuf + 9 );

                    baudcode = atoi( cmdbuf );
                    if ( baudcode < SET_BAUDRATE_MAX ) {
                        VSCP_PUTS_USART((char *) "+OK - New baudrate will be set.\r\n");
                        changeBaudrate( baudcode );
                    }
                    else {
                        VSCP_PUTS_USART((char *) "+ERROR - Invalid baudrate.\r\n");
                    }

                }
                // Set defaults
                else if (cmdbuf == stristr(cmdbuf, "DEFAULTS")) {
                    vscp_restoreDefaults();
                }
                // Show current settings
                else if (cmdbuf == stristr(cmdbuf, "?")) {

                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - Unknown 'SET' command\r\n");
                }

            }
            else {
                if ( 0x0d == cmdbuf[ 0 ]  ) {
                    VSCP_PUTS_USART((char *) "+OK\r\n");
                }
                else {
                    VSCP_PUTS_USART((char *) "-ERROR - Unknown command\r\n");
                }
            }

            memset( cmdbuf, 0, sizeof( cmdbuf ) );
            pos = 0; // Start again
        }
    }

    // Enable interrupt again
    ei();
}

///////////////////////////////////////////////////////////////////////////////
// doModeVscp
//

void doModeVscp( void )
{
    uint8_t c;

    // Fetch possible multi message
    if ( caps.maxCanalFrames > 1 ) {
        // Send multiple CANAL frames in one serial frame for speed
        // if it is possible to do so.
        receiveSendMultiEventCANAL();
    }
    else {
        // If not possible to send several CANAL frames in one serial
        // frame this is a speedier alternative.
        receiveSendEventCANAL();
    }

    // Disable interrupt
    di();

    if ( 1 == fifo_read( &serialInputFifo, &c, 1 ) ) {

        // Enable interrupts again
        ei();

        // Check if the buffer pointer is out of bounds
        if ( pos >= sizeof( cmdbuf ) ) {
            // Houston we got a problem...
            pos = 0;
            bDLE = FALSE;
            stateVscpDriver = STATE_VSCP_SERIAL_DRIVER_WAIT_FOR_FRAME_START;
        }

        if ( STATE_VSCP_SERIAL_DRIVER_WAIT_FOR_FRAME_START == stateVscpDriver ) {

            if ( bDLE ) {

                bDLE = FALSE;

                // Check for frame start
                if ( STX == c ) {
                    stateVscpDriver = STATE_VSCP_SERIAL_DRIVER_WAIT_FOR_FRAME_END;
                    pos = 0;
                }
            }
            else {
                bDLE = TRUE;
                return;
            }
        }
        else if ( STATE_VSCP_SERIAL_DRIVER_WAIT_FOR_FRAME_END == stateVscpDriver ) {

            // Check if last char was DLE
            if ( bDLE ) {

                // Yes last char was DLE this is an escape sequence
                bDLE = FALSE;

                // Check for frame end
                if ( ETX == c ) {
                    stateVscpDriver = STATE_VSCP_SERIAL_DRIVER_FRAME_RECEIVED;
                }
                // Check for escaped DLE
                else if (DLE == c) {
                    // Save
                    cmdbuf[ pos++ ] = c;
                    if ( pos >= sizeof( cmdbuf ) ) {
                        // We have garbage. Start all over again to
                        // look for valid content
                        bDLE = FALSE;
                        pos = 0;
                        stateVscpDriver = STATE_VSCP_SERIAL_DRIVER_WAIT_FOR_FRAME_START;
                    }
                    return;
                }
            }
            else {
                if (DLE == c) {
                    bDLE = TRUE;
                    return;
                }
                else {
                    // This is data. Save it until the full frame is
                    // received.
                    cmdbuf[ pos++ ] = c;
                    if ( pos >= sizeof( cmdbuf ) ) {
                        // We have garbage. Start all over again to
                        // look for valid content
                        bDLE = FALSE;
                        pos = 0;
                        stateVscpDriver = STATE_VSCP_SERIAL_DRIVER_WAIT_FOR_FRAME_START;
                    }
                    return;
                }
            }
        }

        if ( STATE_VSCP_SERIAL_DRIVER_FRAME_RECEIVED == stateVscpDriver ) {

            // Next state is waiting for frame
            stateVscpDriver = STATE_VSCP_SERIAL_DRIVER_WAIT_FOR_FRAME_START;

            // 0 - Frame type
            // 1 - Channel
            // 2 - Sequency number
            // 3 - Size for payload MSB
            // 4 - Size for payload LSB
            // 5-n - Payload
            // len-1 - crc for frame

            // Check that the checksum is correct
            // (Correct if calculation over frame including crc is zero)
            if (calcCRC(cmdbuf, pos)) {
                sendVSCPDriverNack();     // Failed
                return;
            }

            // * * * *  N O O P  * * * *
            if (VSCP_SERIAL_DRIVER_FRAME_TYPE_NOOP ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                sendVSCPDriverAck(); // OK
            }
            // * * * *  V S C P  E V E N T  * * * *
            else if (VSCP_SERIAL_DRIVER_FRAME_TYPE_VSCP_EVENT ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                sendVSCPDriverNack(); // Failed
            }
            // * * * *  C A N A L  E V E N T  * * * *
            else if (VSCP_SERIAL_DRIVER_FRAME_TYPE_CANAL ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                if ( receiveVSCPModeCanalMsg() ) {
                    sendVSCPDriverAck();    // OK
                }
                else {
                    sendVSCPDriverNack();   // Failed
                }
            }
            // * * * *  M U L T I  F R A M E  C A N A L  * * * *
            else if (VSCP_SERIAL_DRIVER_FRAME_TYPE_MULTI_FRAME_CANAL ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                if (receiveVSCPModeMultiCanalMsg()) {
                    sendVSCPDriverAck(); // OK
                }
                else {
                    sendVSCPDriverNack(); // Failed
                }
            }
            // * * * *  M U L T I  F R A M E  V S C P  * * * *
            else if (VSCP_SERIAL_DRIVER_FRAME_TYPE_MULTI_FRAME_CANAL ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                sendVSCPDriverNack(); // Not supported
            }
            // * * * *  C O N F I G U R E  * * * *
            else if (VSCP_SERIAL_DRIVER_FRAME_TYPE_CONFIGURE ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                
                // * * * Noop * * *
                if ( cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                        VSCP_DRIVER_CONFIG_NOOP) {
                    sendVSCPDriverAck();
                }
                // * * * Change driver mode * * *
                else if ( cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                        VSCP_DRIVER_CONFIG_MODE ) {
                     
                    if ( 2 == ( ( ((uint16_t)cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_SIZE_PAYLOAD_MSB ]) << 8 ) |
                                cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_SIZE_PAYLOAD_LSB ] ) ) {
                         
                        if ( cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 1 ] < 4 ) {
                            mode = cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 1 ];
                            eeprom_write( MODULE_EEPROM_STARTUP_MODE,
                                            cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 1 ]);
                             
                            sendVSCPDriverAck();
                        }
                        else {
                            sendVSCPDriverCommandReply(0x66, 0x66); 
                            sendVSCPDriverNack();
                        }
                    }
                    else {
                        // Wrong payload size
                        sendVSCPDriverNack();
                    }
                }
                // * * * Activate/deactivate timestamp * * *
                else if ( cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                            VSCP_DRIVER_CONFIG_TIMESTAMP ) {

                    if ( 2 == ( ( (uint16_t)cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_SIZE_PAYLOAD_MSB ] << 8 ) |
                                cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_SIZE_PAYLOAD_LSB ] ) ) {

                        if ( cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 1 ] ) {
                            bTimestamp = TRUE;
                        }
                        else {
                            bTimestamp = FALSE;
                        }
                        // Save persistent
                        eeprom_write( MODULE_TIMESTAMP,
                                        cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 1 ] );
                        sendVSCPDriverAck();
                    }
                    else {
                        sendVSCPDriverNack(); // Not supported
                    }
                }
                // * * * Baudrate * * *
                else if ( cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                            VSCP_DRIVER_CONFIG_BAUDRATE ) {

                    if ( 2 == ( ( (uint16_t)cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_SIZE_PAYLOAD_MSB ] << 8 ) |
                                cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_SIZE_PAYLOAD_LSB ] ) ) {

                        if ( cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 1 ] < SET_BAUDRATE_MAX ) {
                            sendVSCPDriverAck();
                            changeBaudrate( cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 1 ] );
                        }
                        else {
                            sendVSCPDriverNack();
                        }
                    }
                    else {
                        sendVSCPDriverNack(); // Not supported
                    }
                }
                else {
                    // Wrong payload size
                    sendVSCPDriverNack();
                }
            } // configuration
            // * * * *  P O L L  * * * *
            else if (VSCP_SERIAL_DRIVER_FRAME_TYPE_POLL ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                sendVSCPDriverNack(); // Not supported
            }
            // * * * *  CAPABILITIES  * * * *
            else if (VSCP_SERIAL_DRIVER_FRAME_TYPE_CAPS_REQUEST ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                caps.maxVscpFrames = cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ];
                caps.maxCanalFrames = cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 1 ];
                sendVSCPModeCapabilities();     // Send our capabilities
            }
            // * * * *  C O M M A N D  * * * *
            else if (VSCP_SERIAL_DRIVER_FRAME_TYPE_COMMAND ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {

                // * * * Noop * * *
                if (cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                        VSCP_DRIVER_COMMAND_NOOP) {
                    sendVSCPDriverCommandReply(0, VSCP_DRIVER_COMMAND_NOOP);
                }
                // * * * Open * * *
                else if (cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                        VSCP_DRIVER_COMMAND_OPEN) {
                    ECANSetOperationMode(ECAN_OP_MODE_NORMAL);
                    sendVSCPDriverCommandReply(0, VSCP_DRIVER_COMMAND_OPEN);
                }
                // * * * Loopback * * *
                else if (cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                        VSCP_DRIVER_COMMAND_LISTEN) {
                    ECANSetOperationMode(ECAN_OP_MODE_LOOP);
                    sendVSCPDriverCommandReply(0, VSCP_DRIVER_COMMAND_LISTEN);
                }
                // * * * Listen * * *
                else if (cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                        VSCP_DRIVER_COMMAND_LOOPBACK) {
                    ECANSetOperationMode(ECAN_OP_MODE_LISTEN);
                    sendVSCPDriverCommandReply(0, VSCP_DRIVER_COMMAND_LOOPBACK);
                }
                // * * * Close * * *
                else if (cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                        VSCP_DRIVER_COMMAND_CLOSE) {
                    ECANSetOperationMode(ECAN_INIT_DISABLE);
                    sendVSCPDriverCommandReply(0, VSCP_DRIVER_COMMAND_CLOSE);
                }
                // * * * Set filter * * *
                else if (cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                        VSCP_DRIVER_COMMAND_SET_FILTER) {

                    if ( ( ( ( (uint16_t)cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_SIZE_PAYLOAD_MSB ] << 8 ) |
                            cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_SIZE_PAYLOAD_MSB ] ) < 6 ) &&
                            ( cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 1 ] < 16 ) ) {
                        // Payload is to small
                        sendVSCPDriverCommandReply(1, VSCP_DRIVER_COMMAND_SET_FILTER);
                    }
                    else {
                        // Must be in Config mode to change settings.
                        ECANSetOperationMode(ECAN_OP_MODE_CONFIG);

                        uint8_t filter = (((uint32_t)cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 2 ]) << 24 ) |
                                        (((uint32_t)cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 3 ]) << 16 ) |
                                        (((uint32_t)cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 4 ]) << 8 ) |
                                        cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 5 ];
                        setFilter( cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 1 ],
                                    filter,
                                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 6 ] );

                        // Go back to normal mode
                        ECANSetOperationMode(ECAN_OP_MODE_NORMAL);

                        sendVSCPDriverCommandReply(0, VSCP_DRIVER_COMMAND_SET_FILTER);
                    }
                }
                // * * * Set mask * * *
                else if (cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                        VSCP_DRIVER_COMMAND_SET_MASK) {

                    if ( ( ( ( (uint16_t)cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_SIZE_PAYLOAD_MSB ] << 8 ) |
                            cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_SIZE_PAYLOAD_MSB ] ) < 6 ) &&
                            ( cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 1 ] < 2 ) ) {
                        // Payload is to small
                        sendVSCPDriverCommandReply(1, VSCP_DRIVER_COMMAND_SET_FILTER);
                    }
                    else {
                        // Must be in Config mode to change settings.
                        ECANSetOperationMode(ECAN_OP_MODE_CONFIG);

                        uint8_t mask =  (((uint32_t)cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 2 ]) << 24 ) |
                                        (((uint32_t)cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 3 ]) << 16 ) |
                                        (((uint32_t)cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 4 ]) << 8 ) |
                                        cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 5 ];
                        setFilter( cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 1 ],
                                    mask,
                                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 6 ] );

                        // Go back to normal mode
                        ECANSetOperationMode(ECAN_OP_MODE_NORMAL);

                        sendVSCPDriverCommandReply(1, VSCP_DRIVER_COMMAND_SET_FILTER);
                    }
                }
                else {
                    sendVSCPDriverNack(); // Not supported
                }

            }
            else {
                // This is an unknown operation
                sendVSCPDriverErrorFrame(VSCP_SERIAL_DRIVER_ERROR_UNKNOWN_OPERATION);
            }

            // Get ready for a new frame
            pos = 0;
            stateVscpDriver = STATE_VSCP_SERIAL_DRIVER_WAIT_FOR_FRAME_START;

        }

    }

    // Enable interrupt again (a just in case thingi sort of...)
    ei();
}

///////////////////////////////////////////////////////////////////////////////
// doModeSl
//

void doModeSLCAN(void)
{
    uint8_t c;
    BOOL rv = FALSE;

    // Fetch possible CAN message
    receiveSendEventSLCAN();

    // Disable interrupt
    di();

    if ( 1 == fifo_read( &serialInputFifo, &c, 1 ) ) {

        // Enable interrupts again
        ei();

        if (0x0d != c) {
            cmdbuf[ pos ] = c;
            pos++;
            return;
        }

        switch (c) {

                // Get custom string
            case 'J':
                if (1 == strlen(cmdbuf)) {
                    VSCP_PUTS_USART((char *) "JFrankfurt RS-232\r\n"); // Mimic Lawicel adapter
                    rv = TRUE;
                }
                break;

                // Get version number
            case 'V':
                if (1 == strlen(cmdbuf)) {
                    VSCP_PUTS_USART((char *) "V1011\r\n"); // Mimic Lawicel adapter
                    rv = TRUE;
                }
                break;

                // Get serial number
            case 'N':
                if (1 == strlen(cmdbuf)) {
                    VSCP_PUTS_USART((char *) "N1977\r\n"); // Mimic Lawicel adapter
                    rv = TRUE;
                }
                break;

                // Set standard CAN bitrate
                // Sn where n is 0-8
                //    WE ACCEPT EVERYTHING
            case 'S':
                bInitiated = TRUE;
                rv = TRUE;
                break;

                // Set BTR0/BTR1 CAN bitrate
            case 's':
                if (!bOpen) {
                    bInitiated = TRUE;
                    rv = TRUE;
                }

                // Open the channel
            case 'O':
                if (1 == strlen(cmdbuf)) {
                    if (!bOpen && bInitiated) {
                        ECANSetOperationMode(ECAN_OP_MODE_NORMAL);
                        bOpen = TRUE;
                        rv = TRUE;
                    }
                }
                break;

                // Close the CAN channel
            case 'C':
                if (1 == strlen(cmdbuf)) {
                    if (bOpen) {
                        ECANSetOperationMode(ECAN_INIT_DISABLE);
                        bOpen = FALSE;
                    }
                    rv = TRUE;
                }
                break;

                // Transmit a standard frame
                // WE DONT SEND THEM
            case 't':
                break;

                // Transmit an extended frame  "T%08X%1X"
            case 'T':
                if (bOpen && (strlen(cmdbuf) >= 10)) {

                    uint32_t id;
                    uint8_t dlc;

                    if (hexStringToBytes(wrkbuf, sizeof ( wrkbuf), cmdbuf) < 5) {
                        break;
                    }

                    id = ((uint32_t)wrkbuf[0] << 24) |
                         ((uint32_t)wrkbuf[1] << 16) |
                         ((uint32_t)wrkbuf[2] << 8) |
                                    wrkbuf[3];
                    dlc = vscpData[4];

                    // Check if we have valid data
                    if (dlc > 8) break; // range
                    if (strlen(cmdbuf) < (10 + 2 * dlc)) break; // buffer contents

                    memcpy(vscpData, vscpData + 5, dlc);

                    rv = sendCANFrame(id, dlc, vscpData);

                }
                break;

                // Transmit a standard RTR - WE DO NOT SUPPORT
            case 'r':
                break;

                // Transmit an extended RTR - WE DO NOT SUPPORT
            case 'R':
                break;

                // Read status flag
            case 'F':
                if (bOpen && (strlen(cmdbuf) == 1)) {
                    VSCP_PUTS_USART((char *) "F00\r\n");
                }
                break;

                // Set acceptance code - WE PRETEND TO SUPPORT
            case 'M':
                rv = TRUE;
                break;

                // Set acceptance mask - WE PRETEND TO SUPPORT
            case 'm':
                rv = TRUE;
                break;

                // Enable timestamp
            case 'Z':
                if (2 == strlen(cmdbuf)) {
                    if (!bOpen) {
                        if ('0' == cmdbuf[1]) {
                            nTimeStamp = SLCAN_TIMESTAMP_NOT_USED;
                            // Timestamp disabled
                            eeprom_write(MOUDLE_EEPROM_SLCAN_TIMESTAMP, SLCAN_TIMESTAMP_NOT_USED);
                            rv = TRUE;
                        } else if ('1' == cmdbuf[1]) {
                            nTimeStamp = SLCAN_TIMESTAMP_USE;
                            // Low resolution timestamp enabled
                            eeprom_write(MOUDLE_EEPROM_SLCAN_TIMESTAMP,
                                    SLCAN_TIMESTAMP_USE);
                            rv = TRUE;
                        } else {
                            rv = FALSE;
                        }
                    }
                }
                break;

            // BOOT loader entry
            case 'B':
                eeprom_write(MODULE_EEPROM_INIT_BYTE1, 0xFF);
                Reset();
                break;

            // Go to VSCP/Verbose mode '@vscp' or '@verb'
            case '@':
                if ( 'v' == cmdbuf[1] &&
                        's' == cmdbuf[2] &&
                        'c' == cmdbuf[3] &&
                        'p' == cmdbuf[4] ) {
                    mode = WORKING_MODE_VSCP_DRIVER;
                }
                else if ( 'v' == cmdbuf[1] &&
                        'e' == cmdbuf[2] &&
                        'r' == cmdbuf[3] &&
                        'b' == cmdbuf[4] ) {
                    mode = WORKING_MODE_VERBOSE;
                }
                break;

            default:
            {
                // CR for CR
                if (0 == strlen(cmdbuf)) {
                    rv = TRUE;
                }
            }
                break;

        } // switch

        if (rv) {
            // OK, send CR
            VSCP_PUTS_USART((char *) "\r");
        }
        else {
            // Failure, send BELL
            VSCP_PUTS_USART((char *) "\a");
        }

        // Get ready for next command
        memset( cmdbuf, 0, sizeof( cmdbuf ) );
        pos = 0;

    }

    // Enable interrupts again
    ei();
}

///////////////////////////////////////////////////////////////////////////////
// doModeVscpNode
//

void doModeVscpNode(void)
{
    ;
}

///////////////////////////////////////////////////////////////////////////////
// sendEscapedUartData
//

void sendEscapedUartData(uint8_t c, uint8_t *pcrc)
{
    if (DLE == c) {

        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(DLE);
        //if (NULL != pcrc) crc8(pcrc, DLE);

        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(DLE);
        if (NULL != pcrc) crc8(pcrc, DLE);

    }
    else {

        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(c);
        if (NULL != pcrc) crc8(pcrc, c);

    }
}

///////////////////////////////////////////////////////////////////////////////
// sendVSCPDriverErrorFrame
//

void sendVSCPDriverErrorFrame(uint8_t errorcode)
{
    uint8_t crc = 0;

    // Start of frame
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(DLE);
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(STX);

    // Operation
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(VSCP_SERIAL_DRIVER_FRAME_TYPE_ERROR);
    crc8(&crc, VSCP_SERIAL_DRIVER_FRAME_TYPE_ERROR);

    // Channel
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(0);
    crc8(&crc, 0);

    // Sequency number
    sendEscapedUartData(cmdbuf[ 2 ], &crc);

    // Payload length
    while (VSCP_BUSY_USART()); // MSB
    VSCP_WRITE_USART(0);
    crc8(&crc, 0);
    while (VSCP_BUSY_USART()); // LSB
    VSCP_WRITE_USART(1);
    crc8(&crc, 1);

    // Payload == error code
    sendEscapedUartData(errorcode, &crc);

    // CRC
    sendEscapedUartData(crc, NULL);

    // End of frame
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(DLE);
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(ETX);
}

///////////////////////////////////////////////////////////////////////////////
// sendVSCPDriverAck
//

void sendVSCPDriverAck(void)
{
    uint8_t crc = 0;

    // Start of frame
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(DLE);
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(STX);

    // Operation
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(VSCP_SERIAL_DRIVER_FRAME_TYPE_ACK);
    crc8(&crc, VSCP_SERIAL_DRIVER_FRAME_TYPE_ACK);

    // Channel
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(0);
    crc8(&crc, 0);

    // Sequency number
    sendEscapedUartData(cmdbuf[ 2 ], &crc);

    // Payload length
    while (VSCP_BUSY_USART()); // MSB
    VSCP_WRITE_USART(0);
    crc8(&crc, 0);
    while (VSCP_BUSY_USART()); // LSB
    VSCP_WRITE_USART(0);
    crc8(&crc, 0);

    // Checksum
    sendEscapedUartData(crc, NULL);

    // End of frame
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(DLE);
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(ETX);
}

///////////////////////////////////////////////////////////////////////////////
// sendVSCPDriverNack
//

void sendVSCPDriverNack(void) {
    uint8_t crc = 0;

    // Start of frame
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(DLE);
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(STX);

    // Operation
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(VSCP_SERIAL_DRIVER_FRAME_TYPE_NACK);
    crc8(&crc, VSCP_SERIAL_DRIVER_FRAME_TYPE_NACK);

    // Channel
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(0);
    crc8(&crc, 0);

    // Sequency number
    sendEscapedUartData(cmdbuf[ 2 ], &crc);

    // Payload length
    while (VSCP_BUSY_USART()); // MSB
    VSCP_WRITE_USART(0);
    crc8(&crc, 0);
    while (VSCP_BUSY_USART()); // LSB
    VSCP_WRITE_USART(0);
    crc8(&crc, 0);

    // Checksum
    sendEscapedUartData(crc, NULL);

    // End of frame
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(DLE);
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(ETX);
}

///////////////////////////////////////////////////////////////////////////////
// sendVSCPDriverCommandReply
//

void sendVSCPDriverCommandReply(uint8_t cmdReplyCode, uint8_t cmdCode)
{
    uint8_t crc = 0;

    // Start of frame
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(DLE);
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(STX);

    // Operation
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(VSCP_SERIAL_DRIVER_FRAME_TYPE_COMMAND_REPLY);
    crc8(&crc, VSCP_SERIAL_DRIVER_FRAME_TYPE_COMMAND_REPLY);

    // Channel
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(0);
    crc8(&crc, 0);

    // Sequency number
    sendEscapedUartData(cmdbuf[ 2 ], &crc);

    // Payload length
    while (VSCP_BUSY_USART()); // MSB
    VSCP_WRITE_USART(0);
    crc8(&crc, 0);
    while (VSCP_BUSY_USART()); // LSB
    VSCP_WRITE_USART(2);
    crc8(&crc, 2);

    // Reply code 0 == OK
    while (VSCP_BUSY_USART());
    sendEscapedUartData(cmdReplyCode, &crc);

    // Rely on command
    while (VSCP_BUSY_USART());
    sendEscapedUartData(cmdCode, &crc);

    // Checksum
    sendEscapedUartData(crc, NULL);

    // End of frame
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(DLE);
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(ETX);
}

///////////////////////////////////////////////////////////////////////////////
// receivePrintEventVerbose
//

BOOL receivePrintEventVerbose(void)
{
    uint8_t i;

    if ( getVSCPFrame(&vscpClass,
                        &vscpType,
                        &vscpNodeId,
                        &vscpPriority,
                        &vscpSize,
                        vscpData)) {

        // Statistics
        cntRxFrames++;
        cntRxBytes += vscpSize;

        VSCP_PUTS_USART((char *) "<Prio=");
        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", vscpPriority);
        VSCP_PUTS_USART(wrkbuf);
        if ( bTimestamp ) {
            VSCP_PUTS_USART((char *) ",timestamp=");
            sprintf(wrkbuf, bHex ? "0x%08lX" : "%lu", (timer<<16) | ReadTimer3()*10 );
            VSCP_PUTS_USART(wrkbuf);
        }
        VSCP_PUTS_USART((char *) ",class=");
        sprintf(wrkbuf, bHex ? "0x%04X" : "%d", vscpClass);
        VSCP_PUTS_USART(wrkbuf);
        VSCP_PUTS_USART((char *) ",type=");
        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", vscpType);
        VSCP_PUTS_USART(wrkbuf);
        VSCP_PUTS_USART((char *) ",nodeid=");
        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", vscpNodeId);
        VSCP_PUTS_USART(wrkbuf);
        VSCP_PUTS_USART((char *) ",size=");
        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", vscpSize);
        VSCP_PUTS_USART(wrkbuf);
        if (vscpSize > 0) {
            VSCP_PUTS_USART((char *) ",Data=");
            for (i = 0; i < vscpSize; i++) {
                //itoa(wrkbuf, vscpData[i], bHex ? 16 : 10);
                sprintf(wrkbuf, bHex ? "0x%02X" : "%d", vscpData[i]);
                VSCP_PUTS_USART(wrkbuf);
                if (i < (vscpSize - 1)) {
                    VSCP_PUTS_USART((char *) ",");
                }
            }
        } else {
            VSCP_PUTS_USART((char *) ",Data=none");
        }
        VSCP_PUTS_USART((char *) ">\r\n");
        return TRUE;
    }

    return FALSE;
}

///////////////////////////////////////////////////////////////////////////////
// receiveSendEventCANAL
//

BOOL receiveSendEventCANAL(void)
{
    uint8_t i;
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];

    if ( getCANFrame( &id, &dlc, &data ) ) {

        uint8_t crc = 0;

        // Statistics
        cntRxFrames++;
        cntRxBytes += dlc;

        // Start of frame
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(DLE);
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(STX);

        // Operation
        while (VSCP_BUSY_USART());
        if ( bTimestamp ) {
            VSCP_WRITE_USART( VSCP_SERIAL_DRIVER_FRAME_TYPE_CANAL_TIMESTAMP );
            crc8(&crc, VSCP_SERIAL_DRIVER_FRAME_TYPE_CANAL_TIMESTAMP );
        }
        else {
            VSCP_WRITE_USART(VSCP_SERIAL_DRIVER_FRAME_TYPE_CANAL);
            crc8(&crc, VSCP_SERIAL_DRIVER_FRAME_TYPE_CANAL);
        }

        // Channel
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(0);
        crc8(&crc, 0);

        // Sequency number
        sendEscapedUartData(sequencyno, &crc);
        sequencyno++;

        // Payload length
        sendEscapedUartData(0, &crc);
        sendEscapedUartData(5 + dlc, &crc);


        // * * * *  P A Y L O A D  * * * *

        // id
        sendEscapedUartData( ((id >> 24) & 0xff), &crc);
        sendEscapedUartData(((id >> 16) & 0xff), &crc);
        sendEscapedUartData(((id >> 8) & 0xff), &crc);
        sendEscapedUartData((id & 0xff), &crc);

        // timestamp
        if ( bTimestamp ) {
            uint32_t timestamp = (timer<<16) | ReadTimer3()*10;
            sendEscapedUartData( ((timestamp >> 24) & 0xff), &crc);
            sendEscapedUartData(((timestamp >> 16) & 0xff), &crc);
            sendEscapedUartData(((timestamp >> 8) & 0xff), &crc);
            sendEscapedUartData((timestamp & 0xff), &crc);
        }

        // dlc
        sendEscapedUartData(dlc, &crc);

        // data
        for (i = 0; i < dlc; i++) {
            sendEscapedUartData( data[ i ], &crc );
        }

        // * * * *  P A Y L O A D  * * * *

        // Checksum
        sendEscapedUartData(crc, NULL);

        // End of frame
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(DLE);
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(ETX);

        return TRUE;
    }

    return FALSE;
}

///////////////////////////////////////////////////////////////////////////////
// receiveSendMultiEventCANAL
//

BOOL receiveSendMultiEventCANAL(void)
{
    uint8_t i;
    uint8_t pos=0;
    uint8_t msgcount=0;
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];

    if ( getCANFrame( &id, &dlc, &data ) ) {

        uint8_t crc = 0;

        // Statistics
        cntRxFrames++;
        cntRxBytes += dlc;

        // Start of frame
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(DLE);
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(STX);

        // Operation
        while (VSCP_BUSY_USART());
        if ( bTimestamp ) {
            VSCP_WRITE_USART( VSCP_SERIAL_DRIVER_FRAME_TYPE_MULTI_FRAME_CANAL_TIMESTAMP );
            crc8(&crc, VSCP_SERIAL_DRIVER_FRAME_TYPE_MULTI_FRAME_CANAL_TIMESTAMP );
        }
        else {
            VSCP_WRITE_USART( VSCP_SERIAL_DRIVER_FRAME_TYPE_MULTI_FRAME_CANAL );
            crc8(&crc, VSCP_SERIAL_DRIVER_FRAME_TYPE_MULTI_FRAME_CANAL  );
        }

        // Channel
        while ( VSCP_BUSY_USART() );
        VSCP_WRITE_USART(0);
        crc8(&crc, 0);

        // Sequency number
        sendEscapedUartData(sequencyno, &crc);
        sequencyno++;




        // * * * *  P A Y L O A D  * * * *

        do {

            // id
            wrkbuf[ pos ] = (id >> 24) & 0xff;
            pos++;
            wrkbuf[ pos ] = (id >> 16) & 0xff;
            pos++;
            wrkbuf[ pos ] = (id >> 8) & 0xff;
            pos++;
            wrkbuf[ pos ] = id & 0xff;
            pos++;

            if ( bTimestamp ) {
                uint32_t timestamp = (timer<<16) | ReadTimer3()*10;
                wrkbuf[ pos ] = (timestamp >> 24) & 0xff;
                pos++;
                wrkbuf[ pos ] = (timestamp >> 16) & 0xff;
                pos++;
                wrkbuf[ pos ] = (timestamp >> 8) & 0xff;
                pos++;
                wrkbuf[ pos ] = timestamp & 0xff;
                pos++;
            }

            // dlc
            wrkbuf[ pos ] = dlc;
            pos++;

            // data
            for (i = 0; i < dlc; i++) {
                wrkbuf[ pos ] = data[ i ];
                pos++;
            }

            msgcount++; // Another message

        } while ( ( msgcount < caps.maxCanalFrames ) &&     // Max host accepts
                  ( msgcount < 5 ) &&                       // We send max five
                    getCANFrame( &id, &dlc, &data ) );

        // * * * *  P A Y L O A D  * * * *

        // Payload length
        sendEscapedUartData(0, &crc);
        sendEscapedUartData(pos, &crc);

        // Send payload
        for (i = 0; i < pos; i++) {
            sendEscapedUartData( wrkbuf[ i ], &crc);
        }

        // Checksum
        sendEscapedUartData(crc, NULL);

        // End of frame
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(DLE);
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(ETX);

        return TRUE;
    }

    return FALSE;

}


///////////////////////////////////////////////////////////////////////////////
// receiveSendEventVSCP
//

BOOL receiveSendEventVSCP(void)
{
    uint8_t i;

    if ( getVSCPFrame( &vscpClass,
                        &vscpType,
                        &vscpNodeId,
                        &vscpPriority,
                        &vscpSize,
                        vscpData ) ) {

        uint8_t crc = 0;

        // Statistics
        cntRxFrames++;
        cntRxBytes += vscpSize;

        // Start of frame
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(DLE);
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(STX);

        // Operation
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(VSCP_SERIAL_DRIVER_FRAME_TYPE_VSCP_EVENT);
        crc8(&crc, VSCP_SERIAL_DRIVER_FRAME_TYPE_VSCP_EVENT);

        // Channel
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(0);
        crc8(&crc, 0);

        // Sequency number
        sendEscapedUartData(sequencyno, &crc);
        sequencyno++;

        // Payload length
        sendEscapedUartData((6 + vscpSize) >> 8, &crc);
        sendEscapedUartData((6 + vscpSize)&0xff, &crc);


        // * * * *  P A Y L O A D  * * * *


        sendEscapedUartData(0, &crc);

        sendEscapedUartData(vscpPriority, &crc);

        sendEscapedUartData(vscpClass >> 8, &crc);

        sendEscapedUartData(vscpClass & 0xff, &crc);

        sendEscapedUartData(vscpType >> 8, &crc);

        sendEscapedUartData(vscpType & 0xff, &crc);

        sendEscapedUartData(vscpNodeId, &crc);

        for (i = 0; i < vscpSize; i++) {
            sendEscapedUartData(vscpData[i], &crc);
        }

        // Checksum
        sendEscapedUartData(crc, NULL);

        // End of frame
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(DLE);
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(ETX);

        return TRUE;
    }

    return FALSE;
}


///////////////////////////////////////////////////////////////////////////////
// sendVSCPModeCapabilities
//

BOOL sendVSCPModeCapabilities(void)
{
    uint8_t crc = 0;

    // Start of frame
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(DLE);
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(STX);

    // Operation
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(VSCP_SERIAL_DRIVER_FRAME_TYPE_CAPS_RESPONSE);
    crc8(&crc, VSCP_SERIAL_DRIVER_FRAME_TYPE_CAPS_RESPONSE);

    // Channel
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(0);
    crc8(&crc, 0);

    // Sequency number
    sendEscapedUartData(sequencyno, &crc );
    sequencyno++;

    // Payload length
    sendEscapedUartData( 0, &crc );
    sendEscapedUartData( 2, &crc );

    // payload
    sendEscapedUartData( OUR_CAPS_MAX_VSCP_FRAMES, &crc );
    sendEscapedUartData( 0, &crc );

    // Checksum
    sendEscapedUartData(crc, NULL);

    // End of frame
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(DLE);
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART(ETX);

    return TRUE;
}

///////////////////////////////////////////////////////////////////////////////
// receiveSendEventSLCAN
//

BOOL receiveSendEventSLCAN(void)
{
    int i;
    uint8_t dlc;
    uint32_t id;

    if (getCANFrame(&id, &dlc, vscpData)) {

        // Statistics
        cntRxFrames++;
        cntRxBytes += dlc;

        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART('T');

        ultoa(wrkbuf, id, 16);
        // First print leading zeros so length is eight characers
        for (i = 0; i < ((uint8_t) (8 - sizeof (wrkbuf))); i++) {
            while (VSCP_BUSY_USART());
            VSCP_WRITE_USART('0');
        }
        VSCP_PUTS_USART(wrkbuf);

        // Send dlc - always one digit
        itoa(wrkbuf, dlc, 16);
        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(wrkbuf[0]);

        // Sen data
        for (i = 0; i < dlc; i++) {
            itoa(wrkbuf, vscpData[i], 16);
            // Leading zero if needed
            if (2 != strlen(wrkbuf)) {
                while (VSCP_BUSY_USART());
                VSCP_WRITE_USART('0');
            }
            // Data
            VSCP_PUTS_USART(wrkbuf);
        }

        // If timestamp is active add it to
        if (nTimeStamp) {
            ultoa(wrkbuf, timer, 16);
            // First print leading zeros so length is eight characters
            for (i = 0; i < ((uint8_t) (8 - sizeof (wrkbuf))); i++) {
                while (VSCP_BUSY_USART());
                VSCP_WRITE_USART('0');
            }
            VSCP_PUTS_USART(wrkbuf);
        }

        while (VSCP_BUSY_USART());
        VSCP_WRITE_USART(0x0d);
    }

    return FALSE;
}

///////////////////////////////////////////////////////////////////////////////
// receiveVSCPModeCanalMsg
//

BOOL receiveVSCPModeCanalMsg(void)
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];

    id = ((uint32_t)cmdbuf[VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD] << 26) |
         ((uint32_t)cmdbuf[VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 1] << 16) |
         ((uint32_t)cmdbuf[VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 2] << 8) |
                    cmdbuf[VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 3]; // node address (our address)

    dlc = ( cmdbuf[VSCP_SERIAL_DRIVER_POS_FRAME_SIZE_PAYLOAD_LSB] - 4 );

    if ( dlc > 8 ) {
        dlc = 8;
    }
    memcpy( data, cmdbuf + VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 4, dlc );

    return sendCANFrame(id, dlc, data);
}


///////////////////////////////////////////////////////////////////////////////
// receiveVSCPModeMultiCanalMsg
//

BOOL receiveVSCPModeMultiCanalMsg(void)
{

    return FALSE;

}




///////////////////////////////////////////////////////////////////////////////
// readRegister
//

BOOL readRegister(uint8_t nodeid, uint8_t reg, uint16_t timeout, uint8_t *value)
{
    vscpData[ 0 ] = nodeid;     // Node id
    vscpData[ 1 ] = reg;        // First byte of MDF

    if (sendVSCPFrame(VSCP_CLASS1_PROTOCOL,     // Class
            VSCP_TYPE_PROTOCOL_READ_REGISTER,   // Read register
            0, // Node id (ours)
            0, // High priority
            2, // reg
            vscpData)) {

        timekeeper = 0;
        while (timekeeper < timeout) {

            ClrWdt(); // Feed the dog
            checkCANBusState();

            if (getVSCPFrame(&vscpClass,
                    &vscpType,
                    &vscpNodeId,
                    &vscpPriority,
                    &vscpSize,
                    vscpData)) {
                if ((nodeid == vscpNodeId) &&
                        (VSCP_CLASS1_PROTOCOL == vscpClass) &&
                        (VSCP_TYPE_PROTOCOL_RW_RESPONSE == vscpType) &&
                        (2 == vscpSize) &&
                        (reg == vscpData[ 0 ])) {
                    *value = vscpData[ 1 ];
                    return TRUE;
                }
            }
        }
    }

    return FALSE;
}

///////////////////////////////////////////////////////////////////////////////
// readRegisterExtended
//

BOOL readRegisterExtended(uint8_t nodeid,
                            uint16_t page,
                            uint8_t reg,
                            uint16_t timeout,
                            uint8_t *value)
{
    vscpData[ 0 ] = nodeid;         // Node id
    vscpData[ 1 ] = page >> 8;      // Page MSB
    vscpData[ 2 ] = page & 0xff;    // Page LSB
    vscpData[ 3 ] = reg;            // Offset into page
    vscpData[ 4 ] = 1;              // Number of regs to read

    if ( sendVSCPFrame(VSCP_CLASS1_PROTOCOL,         // class
                        VSCP_TYPE_PROTOCOL_EXTENDED_PAGE_READ,  // Read register
                        0, // Node id (ours)
                        0, // High priority
                        5, // size
                        vscpData ) ) {

        timekeeper = 0;
        while (timekeeper < timeout) {

            ClrWdt(); // Feed the dog
            checkCANBusState();

            if ( getVSCPFrame( &vscpClass,
                                &vscpType,
                                &vscpNodeId,
                                &vscpPriority,
                                &vscpSize,
                                vscpData ) ) {
                if ( ( nodeid == vscpNodeId ) &&
                        ( VSCP_CLASS1_PROTOCOL == vscpClass ) &&
                        ( VSCP_TYPE_PROTOCOL_EXTENDED_PAGE_RESPONSE == vscpType ) &&
                        ( 5 == vscpSize ) &&
                        ( 0 == vscpData[ 0 ] ) &&
                        ( (page >> 8) == vscpData[ 1 ] ) &&
                        ( (page & 0xff) == vscpData[ 2 ] ) &&
                        ( reg == vscpData[ 3 ] ) ) {
                    *value = vscpData[ 4 ];
                    return TRUE;
                }
            }
        }
    }

    return FALSE;
}


///////////////////////////////////////////////////////////////////////////////
// writeRegister
//

BOOL writeRegister(uint8_t nodeid,
                    uint8_t reg,
                    uint16_t timeout,
                    uint8_t *value)
{
    vscpData[ 0 ] = nodeid;     // Node id
    vscpData[ 1 ] = reg;        // reg
    vscpData[ 2 ] = *value;     // value


    if (sendVSCPFrame(VSCP_CLASS1_PROTOCOL,     // class
            VSCP_TYPE_PROTOCOL_WRITE_REGISTER,  // Extended Read register
            0, // Node id (ours)
            0, // High priority
            3, // size
            vscpData)) {

        timekeeper = 0;
        while (timekeeper < timeout) {

            ClrWdt(); // Feed the dog
            checkCANBusState();

            if (getVSCPFrame(&vscpClass,
                    &vscpType,
                    &vscpNodeId,
                    &vscpPriority,
                    &vscpSize,
                    vscpData)) {
                if ((nodeid == vscpNodeId) &&
                        (VSCP_CLASS1_PROTOCOL == vscpClass) &&
                        (VSCP_TYPE_PROTOCOL_RW_RESPONSE == vscpType) &&
                        (2 == vscpSize) &&
                        (reg == vscpData[ 0 ])) {
                    *value = vscpData[ 1 ];
                    return TRUE;
                }
            }
        }
    }

    return FALSE;
}


///////////////////////////////////////////////////////////////////////////////
// writeRegisterExtended
//

BOOL writeRegisterExtended(uint8_t nodeid,
                            uint16_t page,
                            uint8_t reg,
                            uint16_t timeout,
                            uint8_t *value)
{
    vscpData[ 0 ] = nodeid;         // Node id
    vscpData[ 1 ] = page >> 8;      // Page MSB
    vscpData[ 2 ] = page & 0xff;    // Page LSB
    vscpData[ 3 ] = reg;            // Offset into page
    vscpData[ 4 ] = *value;         // Value to write

    if (sendVSCPFrame(VSCP_CLASS1_PROTOCOL,         // class
            VSCP_TYPE_PROTOCOL_EXTENDED_PAGE_WRITE, // Extended Read register
            0, // Node id (ours)
            0, // High priority
            5, // size
            vscpData)) {

        timekeeper = 0;
        while (timekeeper < timeout) {

            ClrWdt(); // Feed the dog
            checkCANBusState();

            if (getVSCPFrame(&vscpClass,
                    &vscpType,
                    &vscpNodeId,
                    &vscpPriority,
                    &vscpSize,
                    vscpData)) {

                if ((nodeid == vscpNodeId) &&
                        (5 == vscpSize) &&
                        (0 == vscpData[ 0 ]) &&
                        ((page >> 8) == vscpData[ 1 ]) &&
                        ((page & 0xff) == vscpData[ 2 ]) &&
                        (reg == vscpData[ 3 ]) &&
                        (*value == vscpData[ 4 ])) {
                    *value = vscpData[ 1 ];
                    return TRUE;
                }

            }

        }

    }

    return FALSE;
}

///////////////////////////////////////////////////////////////////////////////
// calcCRC
//

uint8_t calcCRC(uint8_t *p, uint8_t len)
{
    uint8_t j;
    uint8_t crc = 0;

    for (j = 0; j < len; j++) {
        crc8(&crc, p[j]);
    }

    return crc;
}

///////////////////////////////////////////////////////////////////////////////
// test
//

void test(void)
{
    vscpData[ 0 ] = 1;      // Node id
    vscpData[ 1 ] = 0;      // Page MSB
    vscpData[ 2 ] = 0;      // Page LSB
    vscpData[ 3 ] = 0xD0;   // Offset
    vscpData[ 4 ] = 16;     // Number of regs to read

    sendVSCPFrame(VSCP_CLASS1_PROTOCOL, // class
            VSCP_TYPE_PROTOCOL_EXTENDED_PAGE_READ, // Read register
            0, // Node id (ours)
            0, // High priority
            5, // size
            vscpData);
}

///////////////////////////////////////////////////////////////////////////////
// printBinary
//

void printBinary(uint8_t value)
{
    int i;

    for (i = 7; i >= 0; i--) {
        if (value & (1 << i)) {
            while (VSCP_BUSY_USART());
            VSCP_WRITE_USART('1');
        } else {
            while (VSCP_BUSY_USART());
            VSCP_WRITE_USART('0');
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// printStatistics
//

void printStatistics(void)
{
    VSCP_PUTS_USART((char *) "Sent CAN frames: ");
    sprintf(wrkbuf, bHex ? "0x%08X" : "%lu", cntTxFrames);
    VSCP_PUTS_USART(wrkbuf);
    VSCP_PUTS_USART((char *) "\r\n");

    VSCP_PUTS_USART((char *) "Sent CAN bytes: ");
    sprintf(wrkbuf, bHex ? "0x%08X" : "%lu", cntTxBytes);
    VSCP_PUTS_USART(wrkbuf);
    VSCP_PUTS_USART((char *) "\r\n");

    VSCP_PUTS_USART((char *) "Received CAN frames: ");
    sprintf(wrkbuf, bHex ? "0x%08X" : "%lu", cntRxFrames);
    VSCP_PUTS_USART(wrkbuf);
    VSCP_PUTS_USART((char *) "\r\n");

    VSCP_PUTS_USART((char *) "Received CAN bytes: ");
    sprintf(wrkbuf, bHex ? "0x%08X" : "%lu", cntRxBytes);
    VSCP_PUTS_USART(wrkbuf);
    VSCP_PUTS_USART((char *) "\r\n");
}

///////////////////////////////////////////////////////////////////////////////
// printErrors
//

void printErrors(void)
{
    VSCP_PUTS_USART((char *) "CAN Receive overruns: ");
    sprintf(wrkbuf, bHex ? "0x%08lX" : "%lu", can_receiveOverruns);
    VSCP_PUTS_USART(wrkbuf);
    VSCP_PUTS_USART((char *) "\r\n");

    VSCP_PUTS_USART((char *) "CAN Transmit overruns: ");
    sprintf(wrkbuf, bHex ? "0x%08lX" : "%lu", can_transmitOverruns);
    VSCP_PUTS_USART(wrkbuf);
    VSCP_PUTS_USART((char *) "\r\n");

    VSCP_PUTS_USART((char *) "UART Receive overruns: ");
    sprintf(wrkbuf, bHex ? "0x%08lX" : "%lu", uart_receiveOverruns);
    VSCP_PUTS_USART(wrkbuf);
    VSCP_PUTS_USART((char *) "\r\n");

    VSCP_PUTS_USART((char *) "UART Receive overruns: ");
    sprintf(wrkbuf, bHex ? "0x%08lX" : "%lu", uart_transmitOverruns);
    VSCP_PUTS_USART(wrkbuf);
    VSCP_PUTS_USART((char *) "\r\n");

    if (COMSTATbits.EWARN) {
        VSCP_PUTS_USART((char *) "Transmitter or Receiver is in Error State Warning\r\n");
    }

    VSCP_PUTS_USART((char *) "Transmit Error Counter: ");
    sprintf(wrkbuf, bHex ? "0x%02X" : "%d", TXERRCNT);
    VSCP_PUTS_USART(wrkbuf);
    VSCP_PUTS_USART((char *) "\r\n");

    if (COMSTATbits.TXWARN) {
        VSCP_PUTS_USART((char *) "Transmitter in Error State Warning (128 > TXERRCNT > 96)\r\n");
    }

    if (COMSTATbits.TXBO) {
        VSCP_PUTS_USART((char *) "Transmitter in Error State Bus OFF (TXERRCNT ? 256)\r\n");
    }

    if (COMSTATbits.TXBP) {
        VSCP_PUTS_USART((char *) "Transmitter in Error State Bus Passive (TXERRCNT ? 128)\r\n");
    }

    VSCP_PUTS_USART((char *) "Receive Error Counter: ");
    sprintf(wrkbuf, bHex ? "0x%02X" : "%d", RXERRCNT);
    VSCP_PUTS_USART(wrkbuf);
    VSCP_PUTS_USART((char *) "\r\n");

    if (COMSTATbits.RXBP) {
        VSCP_PUTS_USART((char *) "Receiver in Error State Bus Passive (RXERRCNT > 127)\r\n");
    }

    if (COMSTATbits.RXWARN) {
        VSCP_PUTS_USART((char *) "Receiver  in Error State Warning (128 > RXERRCNT > 96)\r\n");
    }

}


///////////////////////////////////////////////////////////////////////////////
// printHelp
//

void printHelp(void)
{
    VSCP_PUTS_USART((char *) "Help for the Frankfurt RS-232 module\r\n");
    VSCP_PUTS_USART((char *) "------------------------------------\r\n");
    VSCP_PUTS_USART((char *) "BOOT - Enter bootloader.\r\n");
    VSCP_PUTS_USART((char *) "OPEN - Open CAN interface in normal mode.\r\n");
    VSCP_PUTS_USART((char *) "SILENT - Open CAN interface in silent mode.\r\n");
    VSCP_PUTS_USART((char *) "LISTEN - Open CAN interface in listen only mode.\r\n");
    VSCP_PUTS_USART((char *) "LOOPBACK - Open CAN interface in loopback mode.\r\n");
    VSCP_PUTS_USART((char *) "CLOSE - Close CAN interface.\r\n");
    VSCP_PUTS_USART((char *) "VERSION - Display firmware version information.\r\n");
    VSCP_PUTS_USART((char *) "IFMODE - Display selected interface mode.\r\n");
    VSCP_PUTS_USART((char *) "TX - Send CAN frame .\r\n");
    VSCP_PUTS_USART((char *) "     Format: priority,class,type,nodeid,count,data,,,\r\n");
    VSCP_PUTS_USART((char *) "RX - Read CAN frame.\r\n");
    VSCP_PUTS_USART((char *) "STAT - Display CAN statistics.\r\n");
    VSCP_PUTS_USART((char *) "ERR - Display CAN error information.\r\n");
    VSCP_PUTS_USART((char *) "HELP - Display this help information.\r\n");
    VSCP_PUTS_USART((char *) "FIND - Find available CAN4VSCP nodes on bus.\r\n");
    VSCP_PUTS_USART((char *) "RREG - Read register(s) of node (Format: rreg nodeid [page:]reg [count]).\r\n");
    VSCP_PUTS_USART((char *) "WREG - Write register of node (Format: wreg nodeid [page:]reg content).\r\n");
    VSCP_PUTS_USART((char *) "INFO - Get info about an existent node on the bus (Format: info nickname).\r\n");
    VSCP_PUTS_USART((char *) "FILTER - Set filter .\r\n");
    VSCP_PUTS_USART((char *) "         Format: filter filterno,prio,class,type,nodeid  (filterno = 0-15).\r\n");
    VSCP_PUTS_USART((char *) "MASK - Set mask .\r\n");
    VSCP_PUTS_USART((char *) "       Format: mask maskno,prio,class,type,nodeid (maskno = 0 or 1).\r\n");
    VSCP_PUTS_USART((char *) "SET - Persistent functionality.\r\n");
    VSCP_PUTS_USART((char *) "    HEX - Display numericals in hexadecimal.\r\n");
    VSCP_PUTS_USART((char *) "    DECIMAL - Display numericals in decimal.\r\n");
    VSCP_PUTS_USART((char *) "    RWTIMEOUT - Set register read/write timeout. Default=20 ms .\r\n");
    VSCP_PUTS_USART((char *) "                Format: set rwtimeout timeout.\r\n");
    VSCP_PUTS_USART((char *) "    STARTIF - Set interface state to use on startup.\r\n");
    VSCP_PUTS_USART((char *) "    MODE - Set adapter mode that should be used on startup.\r\n");
    VSCP_PUTS_USART((char *) "           Modes: verbose|vscp|slcan\r\n");
}


///////////////////////////////////////////////////////////////////////////////
// findNodes
//

void findNodes(void)
{
    uint8_t nFound = 0;
    uint8_t i;
    uint8_t value;
    BOOL bDot = FALSE;

    if ( ECAN_OP_MODE_NORMAL != ECANGetOperationMode() ) {
        VSCP_PUTS_USART( (char *)STR_ERR_ONLY_IF_OPEN );
        return;
    }

    VSCP_PUTS_USART((char *) "----------------------------------------------------------------\r\n");

    for (i = 1; i != 0; i++) {

        ClrWdt(); // Feed the dog

        if (readRegister( i,
                            0xE0,
                            rwtimeout,
                            &value ) ) {
            if ( bDot ) {
                VSCP_PUTS_USART((char *) "\r\n");
            }
            VSCP_PUTS_USART((char *) "Node found with node id = ");
            itoa(wrkbuf, vscpNodeId, bHex ? 16 : 10);
            VSCP_PUTS_USART(wrkbuf);
            VSCP_PUTS_USART((char *) "\r\n");
            printNodeFirmwareVersion(i);
            printGUID(i);
            printMDF(i);
            VSCP_PUTS_USART((char *) "----------------------------------------------------------------\r\n");
            nFound++; // Another one found
        }
        else {
            bDot = TRUE;
            VSCP_WRITE_USART('.');
            VSCP_BUSY_USART();
        }

    }

    VSCP_PUTS_USART((char *) "\r\n");
    itoa(wrkbuf, nFound, 10);
    VSCP_PUTS_USART(wrkbuf);
    VSCP_PUTS_USART((char *) " node(s) found\r\n");

}


///////////////////////////////////////////////////////////////////////////////
// printGUID
//

void printGUID(uint8_t nodeid)
{
    uint8_t i;
    uint8_t value;
    char buf[3];

    memset( wrkbuf, 0, sizeof( wrkbuf ) );
    VSCP_PUTS_USART((char *) "GUID = ");

    for (i = 0; i < 16; i++) {
        if (readRegister(nodeid,
                0xD0 + i,
                rwtimeout,
                &value)) {
            itoa(buf, value, 16);
            if (1 == strlen(buf)) {
                strcat(wrkbuf, "0");
                strcat(wrkbuf, buf);
            } else {
                strcat(wrkbuf, buf);
            }

        } else {
            strcat(wrkbuf, "--");
        }

        if (i < 15) {
            strcat(wrkbuf, ":");
        }

    }

    VSCP_PUTS_USART(wrkbuf);
    VSCP_PUTS_USART((char *) "\r\n");
}


///////////////////////////////////////////////////////////////////////////////
// printMDF
//

void printMDF(uint8_t nodeid)
{
    uint8_t i;
    uint8_t value;
    char *p = wrkbuf;
    memset( wrkbuf, 0, sizeof( wrkbuf ) );
    VSCP_PUTS_USART((char *) "MDF = http://");

    for (i = 0; i < 32; i++) {
        if (readRegister(nodeid,
                0xE0 + i,
                rwtimeout,
                &value)) {
            *p = value;
            p++;

        }
        else {
            *p = '?';
            p++;
        }
    }

    VSCP_PUTS_USART(wrkbuf);
    VSCP_PUTS_USART((char *) "\r\n");
}

///////////////////////////////////////////////////////////////////////////////
// printNodeFirmwareVersion
//

void printNodeFirmwareVersion(uint8_t nodeid)
{
    uint8_t i;
    uint8_t value;
    char buf[3];

    memset( wrkbuf, 0, sizeof(wrkbuf) );
    VSCP_PUTS_USART((char *) "Firmware version = ");

    for (i = 0; i < 3; i++) {
        if (readRegister(nodeid,
                0x94 + i,
                rwtimeout,
                &value)) {
            itoa(buf, value, 16);
            strcat(wrkbuf, buf);
        }
        else {
            strcat(wrkbuf, "?");
        }

        if (i < 2) {
            strcat(wrkbuf, ".");
        }

    }

    VSCP_PUTS_USART(wrkbuf);
    VSCP_PUTS_USART((char *) "\r\n");
}

///////////////////////////////////////////////////////////////////////////////
// vscp_restoreDefaults
//

void vscp_restoreDefaults(void)
{
    init_app_eeprom();
    init_app_ram();
}

///////////////////////////////////////////////////////////////////////////////
// printFirmwareVersion
//

void printFirmwareVersion(void)
{
    char wrkbuf[20];
    VSCP_PUTS_USART((char *) "Version: ");
    itoa(wrkbuf, FIRMWARE_MAJOR_VERSION, 10);
    VSCP_PUTS_USART(wrkbuf);
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART('.');
    itoa(wrkbuf, FIRMWARE_MINOR_VERSION, 10);
    VSCP_PUTS_USART(wrkbuf);
    while (VSCP_BUSY_USART());
    VSCP_WRITE_USART('.');
    itoa(wrkbuf, FIRMWARE_SUB_MINOR_VERSION, 10);
    VSCP_PUTS_USART(wrkbuf);
    VSCP_PUTS_USART((char *) "\r\n");
}

///////////////////////////////////////////////////////////////////////////////
// printMode
//

void printMode(void)
{
    VSCP_PUTS_USART((char *) "Mode: ");
    if (WORKING_MODE_VERBOSE == mode) {
        VSCP_PUTS_USART((char *) "Verbose");
    } else if (WORKING_MODE_VSCP_DRIVER == mode) {
        VSCP_PUTS_USART((char *) "VSCP Driver");
    } else if (WORKING_MODE_SL_DRIVER == mode) {
        VSCP_PUTS_USART((char *) "SL Driver");
    } else if (WORKING_MODE_VSCP_NODE == mode) {
        VSCP_PUTS_USART((char *) "VSCP Node");
    } else {
        VSCP_PUTS_USART((char *) "Unknown (Verbose used)");
    }
    VSCP_PUTS_USART((char *) "\r\n");
}

///////////////////////////////////////////////////////////////////////////////
// setFilter
//

void setFilter(uint8_t nFilter, uint32_t val, BOOL bPersistent )
{
    uint8_t sidh = (long) val >> 21L;
    uint8_t sidl = (((long) val >> 13L) & 0xe0) |
            ((long) (val) & 0x03L) |
            0x08;
    uint8_t eidh = (long) val >> 8L;
    uint8_t eidl = val;

    switch (nFilter) {

        case 0:
            RXF0SIDH = sidh;
            RXF0SIDL = sidl;
            RXF0EIDH = eidh;
            RXF0EIDL = eidl;
            break;

        case 1:
            RXF1SIDH = sidh;
            RXF1SIDL = sidl;
            RXF1EIDH = eidh;
            RXF1EIDL = eidl;
            break;

        case 2:
            RXF2SIDH = sidh;
            RXF2SIDL = sidl;
            RXF2EIDH = eidh;
            RXF2EIDL = eidl;
            break;

        case 3:
            RXF3SIDH = sidh;
            RXF3SIDL = sidl;
            RXF3EIDH = eidh;
            RXF3EIDL = eidl;
            break;

        case 4:
            RXF4SIDH = sidh;
            RXF4SIDL = sidl;
            RXF4EIDH = eidh;
            RXF4EIDL = eidl;
            break;

        case 5:
            RXF5SIDH = sidh;
            RXF5SIDL = sidl;
            RXF5EIDH = eidh;
            RXF5EIDL = eidl;
            break;

        case 6:
            RXF6SIDH = sidh;
            RXF6SIDL = sidl;
            RXF6EIDH = eidh;
            RXF6EIDL = eidl;
            break;

        case 7:
            RXF7SIDH = sidh;
            RXF7SIDL = sidl;
            RXF7EIDH = eidh;
            RXF7EIDL = eidl;
            break;

        case 8:
            RXF8SIDH = sidh;
            RXF8SIDL = sidl;
            RXF8EIDH = eidh;
            RXF8EIDL = eidl;
            break;

        case 9:
            RXF9SIDH = sidh;
            RXF9SIDL = sidl;
            RXF9EIDH = eidh;
            RXF9EIDL = eidl;
            break;

        case 10:
            RXF10SIDH = sidh;
            RXF10SIDL = sidl;
            RXF10EIDH = eidh;
            RXF10EIDL = eidl;
            break;

        case 11:
            RXF11SIDH = sidh;
            RXF11SIDL = sidl;
            RXF11EIDH = eidh;
            RXF11EIDL = eidl;
            break;

        case 12:
            RXF12SIDH = sidh;
            RXF12SIDL = sidl;
            RXF12EIDH = eidh;
            RXF12EIDL = eidl;
            break;

        case 13:
            RXF13SIDH = sidh;
            RXF13SIDL = sidl;
            RXF13EIDH = eidh;
            RXF13EIDL = eidl;
            break;

        case 14:
            RXF14SIDH = sidh;
            RXF14SIDL = sidl;
            RXF14EIDH = eidh;
            RXF14EIDL = eidl;
            break;

        case 15:
            RXF15SIDH = sidh;
            RXF15SIDL = sidl;
            RXF15EIDH = eidh;
            RXF15EIDL = eidl;
            break;
    }

    if ( bPersistent ) {
        eeprom_write( MODULE_EEPROM_FILTER0 + 0 + 4*nFilter, ( ( id >> 24 ) & 0xff ) );
        eeprom_write( MODULE_EEPROM_FILTER0 + 1 + 4*nFilter, ( ( id >> 16 ) & 0xff ) );
        eeprom_write( MODULE_EEPROM_FILTER0 + 2 + 4*nFilter, ( ( id >> 8 ) & 0xff ) );
        eeprom_write( MODULE_EEPROM_FILTER0 + 3 + 4*nFilter, ( id & 0xff ) );
    }

}

///////////////////////////////////////////////////////////////////////////////
// changeBaudrate
//

void setMask( uint8_t nMask, uint32_t mask, BOOL bPersistent )
{
    // Must be in Config mode to change many of settings.
    ECANSetOperationMode(ECAN_OP_MODE_CONFIG);

    nMask ? ECANSetRXM0Value(mask, ECAN_MSG_XTD) :
                ECANSetRXM1Value(mask, ECAN_MSG_XTD);

    // Go back to normal mode
    ECANSetOperationMode(ECAN_OP_MODE_NORMAL);

    if ( bPersistent ) {
        if ( 0 == nMask ) {
            eeprom_write( MODULE_EEPROM_MASK0, ( ( id >> 24 ) & 0xff ) );
            eeprom_write( MODULE_EEPROM_MASK0 + 1, ( ( id >> 16 ) & 0xff ) );
            eeprom_write( MODULE_EEPROM_MASK0 + 2, ( ( id >> 8 ) & 0xff ) );
            eeprom_write( MODULE_EEPROM_MASK0 + 3, ( id & 0xff ) );
        }
        else {
            eeprom_write( MODULE_EEPROM_MASK1, ( ( id >> 24 ) & 0xff ) );
            eeprom_write( MODULE_EEPROM_MASK1 + 1, ( ( id >> 16 ) & 0xff ) );
            eeprom_write( MODULE_EEPROM_MASK1 + 2, ( ( id >> 8 ) & 0xff ) );
            eeprom_write( MODULE_EEPROM_MASK1 + 3, ( id & 0xff ) );
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// changeBaudrate
//

void changeBaudrate( uint8_t nBaud )
{
    switch( nBaud ) {

        case SET_BAUDRATE_128000:
            nBaud = BAUDRATE_128000;
            break;

        case SET_BAUDRATE_230400:
            nBaud = BAUDRATE_230400;
            break;

        case SET_BAUDRATE_256000:
            nBaud = BAUDRATE_256000;
            break;

        case SET_BAUDRATE_460800:
            nBaud = BAUDRATE_460800;
            break;

        case SET_BAUDRATE_500000:
            nBaud = BAUDRATE_500000;
            break;

        case SET_BAUDRATE_625000:
            nBaud = BAUDRATE_625000;
            break;

        case SET_BAUDRATE_921600:
            nBaud = BAUDRATE_921600;
            break;

        case SET_BAUDRATE_1000000:
            nBaud = BAUDRATE_1000000;
            break;

        case SET_BAUDRATE_9600:
            nBaud = BAUDRATE_9600;
            break;

        case SET_BAUDRATE_19200:
            nBaud = BAUDRATE_19200;
            break;

        case SET_BAUDRATE_38400:
            nBaud = BAUDRATE_38400;
            break;

        case SET_BAUDRATE_57600:
            nBaud = BAUDRATE_57600;
            break;

        case SET_BAUDRATE_115200:
        default:
            nBaud = BAUDRATE_115200;
            break;
    }

//#if defined(_18F2580)
    VSCP_CLOSE_USART();
//#endif
    VSCP_OPEN_USART( USART_TX_INT_OFF &
                    USART_RX_INT_ON &
                    USART_ASYNCH_MODE &
                    USART_EIGHT_BIT &
                    USART_BRGH_HIGH,
                    nBaud );
}

///////////////////////////////////////////////////////////////////////////////
// sendVSCPFrame
//

int8_t sendVSCPFrame(uint16_t vscpclass,
                        uint8_t vscptype,
                        uint8_t nodeid,
                        uint8_t priority,
                        uint8_t size,
                        uint8_t *pData)
{
    uint32_t id = ((uint32_t)priority << 26) |
            ((uint32_t)vscpclass << 16) |
            ((uint32_t)vscptype << 8) |
            nodeid; // nodeaddress (our address)

    if (!sendCANFrame(id, size, pData)) {
        // Failed to send message
        return FALSE;
    }

    return TRUE;
}


///////////////////////////////////////////////////////////////////////////////
// getVSCPFrame
//

int8_t getVSCPFrame(uint16_t *pvscpclass,
                        uint8_t *pvscptype,
                        uint8_t *pNodeId,
                        uint8_t *pPriority,
                        uint8_t *pSize,
                        uint8_t *pData)
{
    uint32_t id;

    if (!getCANFrame(&id, pSize, pData)) {
        return FALSE;
    }

    *pNodeId = id & 0x0ff;
    *pvscptype = (id >> 8) & 0xff;
    *pvscpclass = (id >> 16) & 0x1ff;
    *pPriority = (uint16_t) (0x07 & (id >> 26));

    return TRUE;
}


///////////////////////////////////////////////////////////////////////////////
// sendCANFrame
//

int8_t sendCANFrame(uint32_t id, uint8_t dlc, uint8_t *pdata)
{
    if (!ECANSendMessage(id, pdata, dlc, ECAN_TX_XTD_FRAME)) {
        // Failed to send frame
        can_transmitOverruns++;
        return FALSE;
    }

    return TRUE;
}

///////////////////////////////////////////////////////////////////////////////
// getCANFrame
//

int8_t getCANFrame(uint32_t *pid, uint8_t *pdlc, uint8_t *pdata)
{
    if ( fifo_canrxcount ) {

        // Get id
        di(); // Disable interrupt
        if ( 4 != fifo_read( &canInputFifo, (uint8_t *)pid, 4 ) ) {
            ei();   // Enable interrupt
            return FALSE;
        }

        // Get dlc
        if ( 1 != fifo_read( &canInputFifo, pdlc, 1 ) ) {
            ei();   // Enable interrupt
            return FALSE;
        }

        // Out of bounds check
        if ( *pdlc > 8 ) {
            *pdlc = 0;
        }

        // Get data
        if ( *pdlc != fifo_read( &canInputFifo, pdata, *pdlc ) ) {
            ei();   // Enable interrupt
            return FALSE;
        }

        fifo_canrxcount--;   // One less CAN frame in fifo

        ei();   // Enable interrupt

        return TRUE;
    }

    return FALSE;

}
