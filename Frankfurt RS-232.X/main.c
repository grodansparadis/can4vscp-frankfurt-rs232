/* 
 * File:   main.c
 * Author: grodansparadis
 *
 * Created on den 28 november 2014, 12:17
 */

#include <xc.h>
#include <p18cxxx.h>
#include <timers.h>
#include <delays.h>
#include <eeprom.h>
#include <inttypes.h>
#include "ecan.h"
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


// Buffers
uint8_t inputBuffer[ SIZE_SERIAL_INPUT_BUFFER ];
uint8_t caninputBuffer[ 13 * SIZE_CAN_INPUT_FIFO ];

// fifos
fifo_t serialInputFifo;
fifo_t canInputFifo;

volatile uint32_t timer = 0; // Millisecond timer
volatile uint32_t timekeeper = 0; // Nill to measure time
uint8_t ledFunctionality; // Init LED functionality
volatile uint16_t status_led_cnt; // status LED counter
// increase externally bye one every
// millisecond
uint8_t mode; // Unit working mode

BOOL bHex = FALSE; // Numerical printouts in hex

BOOL bOpen = FALSE;     // TRUE if i/f is open
BOOL bSilent = FALSE;   // Open but no receive
uint8_t rwtimeout;      // Reg read/write timeout

volatile uint8_t canrxcount = 0;    // Number of CAN messages in fifo

// Statistics
uint32_t cntTxFrames = 0;   // Number of sent CAN frames
uint32_t cntTxBytes = 0;    // Number of sent bytes
uint32_t cntRxFrames = 0;   // Number of receiveed frames
uint32_t cntRxBytes = 0;    // Number of received frames

// Error statistics
uint32_t can_receiveOverruns = 0; // CAN receive overruns (to user)
uint32_t can_transmitOverruns = 0; // CAN send overruns.
uint32_t uart_receiveOverruns = 0; // UART receive overruns (user)
uint32_t uart_transmitOverruns = 0; // UART send overruns.

uint8_t pos = 0;
char cmdbuf[80];
char wrkbuf[80];

// VSCP driver mode
BOOL stateVscpDriver = STATE_VSCP_SERIAL_DRIVER_WAIT_FOR_FRAME_START;
BOOL bDLE = FALSE; // True if escpae charcter has been received.
uint8_t sequencyno = 0; // Sequency number. Increaces for every fram

// * * * *   Capabilities   * * * *
vscp_serial_caps caps;   // Init structure in main

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

#if defined(RELEASE)

#pragma config WDT = ON, WDTPS = 128
#pragma config OSC = HSPLL
#pragma config BOREN = BOACTIVE
#pragma config STVREN = ON
#pragma config BORV = 3
#pragma config LVP = ON
#pragma config CPB = ON
#pragma config BBSIZ = 2048
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
#pragma config BORV = 3
#pragma config LVP = OFF
#pragma config CPB = OFF
#pragma config WRTD  = OFF

#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
#pragma config EBTR3 = OFF

#pragma config EBTRB = OFF

#endif


///////////////////////////////////////////////////////////////////////////////
// Interrupt
//

void interrupt low_priority Interrupt()
{
    uint8_t c;

    // Check if the interrupt is caused by RX pin
    if ( 1 == PIR1bits.RCIF ) {

        c = ReadUSART();

        if (1 != fifo_write(&serialInputFifo, &c, 1)) {
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
        if ((STATUS_LED_VERY_SLOW_BLINK == ledFunctionality) &&
                (status_led_cnt > 1000)) {
            LATC1 = ~LATC1;
            status_led_cnt = 0;
        }
        else if ((STATUS_LED_SLOW_BLINK == ledFunctionality) &&
                (status_led_cnt > 400)) {
            LATC1 = ~LATC1;
            status_led_cnt = 0;
        }
        else if ((STATUS_LED_NORMAL_BLINK == ledFunctionality) &&
                (status_led_cnt > 100)) {
            LATC1 = ~LATC1;
            status_led_cnt = 0;
        }
        else if ((STATUS_LED_FAST_BLINK == ledFunctionality) &&
                (status_led_cnt > 70)) {
            LATC1 = ~LATC1;
            status_led_cnt = 0;
        }
        else if ((STATUS_LED_VERY_FAST_BLINK == ledFunctionality) &&
                (status_led_cnt > 40)) {
            LATC1 = ~LATC1;
            status_led_cnt = 0;
        }
        else if (STATUS_LED_ON == ledFunctionality) {
            LATC1 = 1;
            status_led_cnt = 0;
        }
        else if (STATUS_LED_OFF == ledFunctionality) {
            LATC1 = 0;
            status_led_cnt = 0;
        }

        INTCONbits.TMR0IF = 0; // Clear Timer0 Interrupt Flag
    }

    // CAN error
    if ( 1 == IRXIF ) {

        IRXIF = 0;
    }

    // Check for CAN RX interrrupt
    if ( RXBnIF ) {

        uint32_t id;
        uint8_t dlc;
        uint8_t data[8];
        ECAN_RX_MSG_FLAGS flags;

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
                    fifo_write( &canInputFifo, (uint8_t *)&id, 4 );
                    fifo_write( &canInputFifo, &dlc, 1 );
                    fifo_write( &canInputFifo, (uint8_t *)&data, dlc );

                    canrxcount++;   // Another CAN frame in the fifo
                }
                else {
                    can_receiveOverruns++;
                }

            }

        }

        // Not needed restetted in ECAN
        RXBnIF = 0; // Clear CAN0 Interrupt Flag
    }

    

}

///////////////////////////////////////////////////////////////////////////////
// main
//

int main(int argc, char** argv)
{
    // Init capabilities
    caps.maxVscpFrames = 1;
    caps.maxCanalFrames = 1;        // Max frames that will be sent to host
                                    // can be increase after a capability
                                    // request.

    // Init fifos
    fifo_init( &serialInputFifo, inputBuffer, sizeof ( inputBuffer));       // UART receive
    fifo_init( &canInputFifo, caninputBuffer, sizeof ( caninputBuffer));    // CAN receive

    // Init CRC table
    init_crc8();

    // If EEPROM is not initialized then restore defaults
    // before the system is initialized
    if ( ( 0x55 != readEEPROM(MODULE_EEPROM_INIT_BYTE1) ) ||
            ( 0xaa != readEEPROM(MODULE_EEPROM_INIT_BYTE2) ) ) {
        vscp_restoreDefaults();
    }

    // Initialize
    init();

    // This delay is needed to prevent garbage on serial line on startup
    // Delay http://microchip.wikidot.com/faq:26
    __delay_ms(10);

    //mode = WORKING_MODE_VSCP_DRIVER;

    putsUSART((char *) "\r\nFrankfurt RS-232 CAN4VSCP module\r\n");
    putsUSART((char *) "Copyright (C) 2014-2015 Grodans Paradis AB, Sweden\r\n");
    putsUSART((char *) "http://www.paradiseofthefrog.com\r\n");
    printFirmwareVersion();
    printMode();

    // Wait for init sequency to bring interface to verbose mode
    // v' pressed withing three seconds
    if (0 && (WORKING_MODE_VERBOSE != mode)) {

        uint8_t c;

        putsUSART((char *) "Press 'v' within three seconds to enter verbose mode\r\n");

        timekeeper = 0;
        ledFunctionality = STATUS_LED_VERY_FAST_BLINK;
        while (timekeeper < 3000) {
            ClrWdt(); // Feed the dog
            di();       // Disablee interrupt
            if (1 == fifo_read(&serialInputFifo, &c, 1)) {
                ei(); // Enable interrupt
                if ('v' == c) {
                    mode = WORKING_MODE_VERBOSE;
                    putsUSART((char *) "Temporary verbose mode set\r\n");
                    break;
                }
            }
            ei(); // Enable interrupt
        }
    }

    ledFunctionality = STATUS_LED_ON;

    // Work loop  
    while (TRUE) {

        ClrWdt(); // Feed the dog

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

        // Check if we have CAN receive overflows
        if (COMSTATbits.RXBnOVFL) {
            can_receiveOverruns++;
            COMSTATbits.RXBnOVFL = 0;
        }

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
            doModeVerbose(); // Just in case
        }

    } // While

    return (EXIT_SUCCESS);
}



///////////////////////////////////////////////////////////////////////////////
// Init - Initialization Routine
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

    TRISBbits.RB2 = 0; // CAN TX
    TRISBbits.RB3 = 1; // CAN RX

    TRISCbits.RC1 = 0; // Status LED
    TRISCbits.RC6 = 0; // UART TX pin set as output
    TRISCbits.RC7 = 1; // UART RX pin set as input

    // Initialize UART
    // 230400 (10), 500000 (4) and 625000 (3) 115200 (20)
    OpenUSART( USART_TX_INT_OFF &
                USART_RX_INT_ON &
                USART_ASYNCH_MODE &
                USART_EIGHT_BIT &
                USART_BRGH_HIGH,
                BAUDRATE_115200 );

    RCIF = 0; // Reset RX pin flag
    RCIP = 0; // Not high priority
    RCIE = 1; // Enable RX interrupt
    PEIE = 1; // Enable pheripheral interrupt (serial port is a pheripheral)

    // Initialize 1 ms timer
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_8);
    WriteTimer0(TIMER0_RELOAD_VALUE);

    // Initialize CAN
    ECANInitialize();

    // Must be in Config mode to change many of settings.
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
// init_app_ram
//

void init_app_ram(void)
{
    // Nill error counters
    can_receiveOverruns = 0;
    can_transmitOverruns = 0;
    uart_receiveOverruns = 0;
    uart_transmitOverruns = 0;

    bHex = readEEPROM(MOUDLE_EEPROM_PRINTOUT_IN_HEX);
    mode = readEEPROM(MODULE_EEPROM_STARTUP_MODE);

    rwtimeout = readEEPROM(MODULE_EEPROM_RW_TIMEOUT);
}

///////////////////////////////////////////////////////////////////////////////
// init_app_eeprom
//

void init_app_eeprom(void)
{
    uint8_t i;

    writeEEPROM(MODULE_EEPROM_INIT_BYTE1, 0x55);
    writeEEPROM(MODULE_EEPROM_INIT_BYTE2, 0xAA);
    writeEEPROM(MODULE_EEPROM_STARTUP_MODE, WORKING_MODE_VERBOSE);
    writeEEPROM(MOUDLE_EEPROM_SLCAN_TIMESTAMP, SLCAN_TIMESTAMP_NOT_USED);
    writeEEPROM(MOUDLE_EEPROM_PRINTOUT_IN_HEX, NUMERICAL_PRINTOUTMODE_DECIMAL);
    writeEEPROM(MODULE_EEPROM_RW_TIMEOUT, DEFAULT_REGISTER_RW_TIMEOUT);

    // Set all filters to 0xff
    for (i = MODULE_EEPROM_FILTER0; i < (MODULE_EEPROM_FILTER15 + 4); i++) {
        writeEEPROM(MODULE_EEPROM_INIT_BYTE1, 0xFF);
    }

    // Set all masks 0x00
    for (i = MODULE_EEPROM_MASK0; i < (MODULE_EEPROM_MASK1 + 4); i++) {
        writeEEPROM(MODULE_EEPROM_INIT_BYTE1, 0xFF);
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

        // Save
        cmdbuf[ pos ] = c;
        pos++;

        if (pos >= SIZE_SERIAL_INPUT_BUFFER) {

            // We have garbage. Start all over again to
            // look for valid content
            pos = 0;

        }

        // Check if we have a command
        if (0x0d == c) {

            cmdbuf[ pos ] = 0;

            // Enter bootloader
            if (cmdbuf == stristr(cmdbuf, "BOOT")) {
                putsUSART((char *) "Will enter bootloader now...\r\n");
                writeEEPROM(MODULE_EEPROM_BOOTLOADER_FLAG, 0xFF);
                Reset();
            }
            // Open interface
            else if (cmdbuf == stristr(cmdbuf, "OPEN")) {
                bSilent = FALSE;
                ECANSetOperationMode(ECAN_OP_MODE_NORMAL);
                putsUSART((char *) "+OK\r\n");
            }
            // Open interface but don't show receive frames
            else if (cmdbuf == stristr(cmdbuf, "SILENT")) {
                bSilent = TRUE;
                ECANSetOperationMode(ECAN_OP_MODE_NORMAL);
                putsUSART((char *) "+OK\r\n");
            }
            // Close interface
            else if (cmdbuf == stristr(cmdbuf, "CLOSE")) {
                bSilent = TRUE;
                ECANSetOperationMode(ECAN_INIT_DISABLE);
                putsUSART((char *) "+OK\r\n");
            }
            // Open interface in listen only mode
            else if (cmdbuf == stristr(cmdbuf, "LOOPBACK")) {
                bSilent = FALSE;
                ECANSetOperationMode(ECAN_OP_MODE_LOOP);
                putsUSART((char *) "+OK\r\n");
            }
            // Open interface in loopback mode
            else if (cmdbuf == stristr(cmdbuf, "LISTEN")) {
                bSilent = FALSE;
                ECANSetOperationMode(ECAN_OP_MODE_LISTEN);
                putsUSART((char *) "+OK\r\n");
            }
            // Print version
            else if (cmdbuf == stristr(cmdbuf, "VERSION")) {
                printFirmwareVersion();
                putsUSART((char *) "+OK\r\n");
            }
            else if (cmdbuf == stristr(cmdbuf, "IFMODE")) {
                ECAN_OP_MODE ifmode = ECANGetOperationMode();
                if ((ECAN_OP_MODE_NORMAL == ifmode) & !bSilent) {
                    putsUSART((char *) "+OK - Normal mode\r\n");
                }
                else if ((ECAN_OP_MODE_NORMAL == ifmode) & bSilent) {
                    putsUSART((char *) "+OK - Silent mode\r\n");
                }
                else if (ECAN_OP_MODE_SLEEP == ifmode) {
                    putsUSART((char *) "+OK - Sleep mode\r\n");
                }
                else if (ECAN_OP_MODE_LOOP == ifmode) {
                    putsUSART((char *) "+OK - Loopback mode\r\n");
                }
                else if (ECAN_OP_MODE_LISTEN == ifmode) {
                    putsUSART((char *) "+OK - Listen only mode\r\n");
                }
                else if (ECAN_OP_MODE_CONFIG == ifmode) {
                    putsUSART((char *) "+OK - Closed mode\r\n");
                }
                else if (ECAN_OP_MODE_BITS == ifmode) {
                    putsUSART((char *) "+OK - Bits mode\r\n");
                }
                else {
                    putsUSART((char *) "-ERROR - Unknown mode\r\n");
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

                memset(vscpData, 0, 8);
                for (i = 0; i < vscpSize; i++) {
                    if (NULL != (p = strtok(NULL, ","))) {
                        vscpData[ i ] = atoi(p);
                    } else {
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
                    putsUSART((char *) "+OK\r\n");
                }
                else {
                    putsUSART((char *) "-ERROR - Failed to send event.\r\n");
                }
            }
            // Receive event
            else if (cmdbuf == stristr(cmdbuf, "RX")) {
                if (receivePrintEventVerbose()) {
                    putsUSART((char *) "+OK\r\n");
                }
                else {
                    putsUSART((char *) "+OK - no events\r\n");
                }
            }
            // Print out statistics
            else if (cmdbuf == stristr(cmdbuf, "STAT")) {
                printStatistics();
                putsUSART((char *) "+OK\r\n");
            }
            // List error counters
            else if (cmdbuf == stristr(cmdbuf, "ERR")) {
                printErrors();
                putsUSART((char *) "+OK\r\n");
            }
            // Help
            else if (cmdbuf == stristr(cmdbuf, "HELP")) {
                printHelp();
                putsUSART((char *) "+OK\r\n");
            }
            // Find nodes on CAN4VSCP bus
            else if (cmdbuf == stristr(cmdbuf, "FIND")) {
                findNodes();
                putsUSART((char *) "+OK\r\n");
            }
            // Read register of node
            //      RREG node [page:]reg count
            else if (cmdbuf == stristr(cmdbuf, "RREG")) {

                uint8_t i;
                uint8_t nodeid;
                uint8_t page = 0;
                uint8_t reg;
                uint8_t value;
                uint8_t count = 1;

                strcpy(cmdbuf, cmdbuf + 5);
                char *p = strtok(cmdbuf, " ");
                if (NULL != p) {
                    nodeid = atoi(p);
                } else {
                    putsUSART((char *) "-ERROR - Needs nodeid\r\n");
                    return;
                }

                if (NULL != (p = strtok(NULL, " "))) {

                    char *preg;
                    if (preg = strchr(p, ':')) {
                        page = atoi(p);
                        reg = atoi(preg + 1);
                    } else {
                        reg = atoi(p);
                    }


                }
                else {
                    putsUSART((char *) "-ERROR - Needs [page:]register\r\n");
                    return;
                }

                // Count
                if (NULL != (p = strtok(NULL, " "))) {
                    count = atoi(p);
                    if (0 == count) count = 1;
                }

                BOOL rv = TRUE;
                for (i = 0; i < count; i++) {

                    if (readRegisterExtended(nodeid,
                            page,
                            (reg + i) & 0xff,
                            rwtimeout,
                            &value)) {
                        putsUSART((char *) "Value for reg ");
                        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", page);
                        putsUSART(wrkbuf);
                        while (BusyUSART());
                        WriteUSART(':');
                        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", (reg + i) & 0xff);
                        putsUSART(wrkbuf);
                        putsUSART((char *) " = ");
                        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", value);
                        putsUSART(wrkbuf);
                        while (BusyUSART());
                        WriteUSART(' ');
                        while (BusyUSART());
                        if ((value > 32) && (value < 127)) {
                            WriteUSART(value);
                        }
                        else {
                            WriteUSART('.');
                        }
                        while (BusyUSART());
                        WriteUSART(' ');
                        printBinary(value);
                        putsUSART((char *) "\r\n");
                    }
                    else {
                        rv = FALSE;
                        putsUSART((char *) "-ERROR - Unable to read register ");
                        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", page);
                        putsUSART(wrkbuf);
                        while (BusyUSART());
                        WriteUSART(':');
                        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", (reg + i) & 0xff);
                        putsUSART(wrkbuf);
                        putsUSART((char *) "\r\n");
                    }
                }

                if (rv) {
                    putsUSART((char *) "+OK\r\n");
                }
                else {
                    putsUSART((char *) "-ERROR - One or more register(s) could not be read.\r\n");
                }

            }
            // Write register of node
            //      WREG node [page:]reg value1 [value2 value3 value4 value5...]
            else if (cmdbuf == stristr(cmdbuf, "WREG")) {

                uint8_t nodeid;
                uint8_t page = 0;
                uint8_t reg;
                uint8_t value;

                strcpy(cmdbuf, cmdbuf + 5);
                char *p = strtok(cmdbuf, " ");
                if (NULL != p) {
                    nodeid = atoi(p);
                }
                else {
                    putsUSART((char *) "-ERROR - Needs nodeid\r\n");
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
                    putsUSART((char *) "-ERROR - Needs [page:]register\r\n");
                    return;
                }

                // Value
                if (NULL != (p = strtok(NULL, " "))) {
                    value = atoi(p);
                }
                else {
                    putsUSART((char *) "-ERROR - Need a value\r\n");
                    return;
                }

                if (writeRegisterExtended(nodeid,
                        page,
                        reg,
                        rwtimeout,
                        &value)) {
                    putsUSART((char *) "+OK - Value written successfully\r\n");
                }
                else {
                    putsUSART((char *) "-ERROR - Failed to write value\r\n");
                }

            }
            // Read full node info
            //      INFO node-id
            else if (cmdbuf == stristr(cmdbuf, "INFO")) {
                uint8_t nodeid;

                strcpy(cmdbuf, cmdbuf + 5);
                nodeid = atoi(cmdbuf);

                putsUSART((char *) "Info for node id = ");
                sprintf(wrkbuf, bHex ? "0x%02X" : "%d", nodeid);
                putsUSART(wrkbuf);
                putsUSART((char *) "\r\n");
                printNodeFirmwareVersion(nodeid);
                printGUID(nodeid);
                printMDF(nodeid);
                putsUSART((char *) "+OK\r\n");
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

                strcpy(cmdbuf, cmdbuf + 8);
                char *p = strtok(cmdbuf, ",");
                if (NULL != p) {
                    filterno = atoi(p);
                    if (filterno > 15) {
                        putsUSART((char *) "-ERROR - Filter number can only be set to a value between 0-15.\r\n");
                        return;
                    }
                }
                else {
                    putsUSART((char *) "-ERROR - No filter number specified.\r\n");
                    return;
                }

                // Prio
                p = strtok(NULL, ",");
                if (NULL != p) {
                    filter_priority = atoi(p);
                }
                else {
                    putsUSART((char *) "-ERROR - filter for priority is missing\r\n");
                    return;
                }

                // Class
                p = strtok(NULL, ",");
                if (NULL != p) {
                    filter_class = atoi(p);
                }
                else {
                    putsUSART((char *) "-ERROR - filter for class is missing\r\n");
                    return;
                }

                // Type
                p = strtok(NULL, ",");
                if (NULL != p) {
                    filter_type = atoi(p);
                }
                else {
                    putsUSART((char *) "-ERROR - filter for type is missing\r\n");
                    return;
                }

                // Node id
                p = strtok(NULL, ",");
                if (NULL != p) {
                    filter_nodeid = atoi(p);
                }
                else {
                    putsUSART((char *) "-ERROR - filter for nide id is missing\r\n");
                    return;
                }

                // Must be in Config mode to change many of settings.
                ECANSetOperationMode(ECAN_OP_MODE_CONFIG);

                uint32_t id = ((uint32_t) filter_priority << 26) |
                        ((uint32_t) filter_class << 16) |
                        ((uint32_t) filter_type << 8) |
                        filter_nodeid;
                setFilter(filterno, id);
                putsUSART((char *) "+OK\r\n");
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

                strcpy(cmdbuf, cmdbuf + 8);
                char *p = strtok(cmdbuf, ",");
                if (NULL != p) {
                    maskno = atoi(p);
                    if (maskno > 1) {
                        putsUSART((char *) "-ERROR - Mask number can only be set as 0 or 1.\r\n");
                        return;
                    }
                }
                else {
                    putsUSART((char *) "-ERROR - No mask number specified.\r\n");
                    return;
                }

                // Prio
                p = strtok(NULL, ",");
                if (NULL != p) {
                    mask_priority = atoi(p);
                }
                else {
                    putsUSART((char *) "-ERROR - mask for priority is missing\r\n");
                    return;
                }

                // Class
                p = strtok(NULL, ",");
                if (NULL != p) {
                    mask_class = atoi(p);
                }
                else {
                    putsUSART((char *) "-ERROR - mask for class is missing\r\n");
                    return;
                }

                // Type
                p = strtok(NULL, ",");
                if (NULL != p) {
                    mask_type = atoi(p);
                }
                else {
                    putsUSART((char *) "-ERROR - mask for type is missing\r\n");
                    return;
                }

                // Node id
                p = strtok(NULL, ",");
                if (NULL != p) {
                    mask_nodeid = atoi(p);
                }
                else {
                    putsUSART((char *) "-ERROR - mask for nide id is missing\r\n");
                    return;
                }

                uint32_t id = ((uint32_t) mask_priority << 26) |
                        ((uint32_t) mask_class << 16) |
                        ((uint32_t) mask_type << 8) |
                        mask_nodeid;

                // Must be in Config mode to change many of settings.
                ECANSetOperationMode(ECAN_OP_MODE_CONFIG);

                maskno ? ECANSetRXM0Value(id, ECAN_MSG_XTD) :
                        ECANSetRXM1Value(id, ECAN_MSG_XTD);
                putsUSART((char *) "+OK\r\n");
            }
            // Set Configuration
            //      RWTIMEOUT n  - rreg/wreg timeout
            //      STARTIF [CONFIG|CLOSE|OPEN|LISTEN|LOOPBACK]
            //      MODE [VERBOSE|VSCP|SLCAN]
            //      HEX
            //      DECIMAL
            else if (cmdbuf == stristr(cmdbuf, "SET ")) {

                // Remove "SET "
                strcpy(cmdbuf, cmdbuf + 4);

                // Hex - Numbers in Hex from now on
                if (cmdbuf == stristr(cmdbuf, "HEX")) {
                    bHex = TRUE;
                    writeEEPROM(MOUDLE_EEPROM_PRINTOUT_IN_HEX, NUMERICAL_PRINTOUTMODE_HEX);
                    putsUSART((char *) "+OK - Numerical output now in hexadecimal\r\n");
                }
                // Decimal - numbers in decimal from now on
                else if (cmdbuf == stristr(cmdbuf, "DECIMAL")) {
                    bHex = FALSE;
                    writeEEPROM(MOUDLE_EEPROM_PRINTOUT_IN_HEX, NUMERICAL_PRINTOUTMODE_DECIMAL);
                    putsUSART((char *) "+OK - Numerical output now in decimal\r\n");
                }
                else if (0 != stristr(cmdbuf, "RWTIMEOUT ")) {
                    strcpy(cmdbuf, cmdbuf + 10);
                    rwtimeout = atoi(cmdbuf);
                    if (rwtimeout < DEFAULT_REGISTER_RW_TIMEOUT) {
                        rwtimeout = DEFAULT_REGISTER_RW_TIMEOUT;
                    }
                    writeEEPROM(MODULE_EEPROM_RW_TIMEOUT, rwtimeout);
                    putsUSART((char *) "+OK\r\n");
                }
                else if (0 != stristr(cmdbuf, "STARTIF ")) {
                    strcpy(cmdbuf, cmdbuf + 8);
                }
                else if (0 != stristr(cmdbuf, "MODE ")) {
                    strcpy(cmdbuf, cmdbuf + 5);
                    if (0 != stristr(cmdbuf, "VERBOSE")) {
                        mode = WORKING_MODE_VERBOSE;
                        writeEEPROM(MODULE_EEPROM_STARTUP_MODE, WORKING_MODE_VERBOSE);
                        putsUSART((char *) "+OK - Mode is now verbose\r\n");
                    }
                    else if (0 != stristr(cmdbuf, "VSCP")) {
                        mode = WORKING_MODE_VSCP_DRIVER;
                        writeEEPROM(MODULE_EEPROM_STARTUP_MODE, WORKING_MODE_VSCP_DRIVER);
                        putsUSART((char *) "+OK - Mode is now VSCP Driver\r\n");
                    }
                    else if (0 != stristr(cmdbuf, "SLCAN")) {
                        mode = WORKING_MODE_SL_DRIVER;
                        writeEEPROM(MODULE_EEPROM_STARTUP_MODE, WORKING_MODE_SL_DRIVER);
                        putsUSART((char *) "+OK - Mode is now SLCAN\r\n");
                    }
                }
                // filterno,prio,class,type,nodeid (filterno = 0-15)
                else if (0 != stristr(cmdbuf, "FILTER ")) {
                    strcpy(cmdbuf, cmdbuf + 7);
                }
                // maskno,prio,class,type,nodeid (maskno = 0 or 1)
                else if (0 != stristr(cmdbuf, "MASK ")) {
                    strcpy(cmdbuf, cmdbuf + 5);
                }
                else {
                    putsUSART((char *) "-ERROR - Unknown 'SET' command\r\n");
                }

            }
            else {
                if ( 0x0d == cmdbuf[ 0 ]  ) {
                    putsUSART((char *) "+OK\r\n");
                }
                else {
                    putsUSART((char *) "-ERROR - Unknown command\r\n");
                }
            }

            memset(cmdbuf, 0, sizeof ( cmdbuf));
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

    // Fetch possible VSCP event
    if ( !bSilent ) {
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
    }

    // Disable interrupt
    di();

    if ( 1 == fifo_read(&serialInputFifo, &c, 1 ) ) {

        // Enable interrupts again
        ei();

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
            if (bDLE) {

                bDLE = FALSE;

                // Check for frame end
                if (ETX == c) {
                    stateVscpDriver = STATE_VSCP_SERIAL_DRIVER_FRAME_RECEIVED;
                }                    // Check for escaped DLE
                else if (DLE == c) {
                    // Save
                    cmdbuf[ pos++ ] = c;
                    return;
                }
            }
            else {
                if (DLE == c) {
                    bDLE = TRUE;
                    return;
                }
                else {
                    // Save
                    cmdbuf[ pos++ ] = c;

                    if (pos >= SIZE_SERIAL_INPUT_BUFFER) {
                        // We have garbage. Start all over again to
                        // look for valid content
                        pos = 0;
                        stateVscpDriver = STATE_VSCP_SERIAL_DRIVER_WAIT_FOR_FRAME_START;
                    }

                    return;
                }
            }
        }




        if (STATE_VSCP_SERIAL_DRIVER_FRAME_RECEIVED == stateVscpDriver) {

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
            //          Correct if zero
            if (calcCRC(cmdbuf, pos)) {
                sendVSCPDriverErrorFrame(VSCP_SERIAL_DRIVER_ERROR_CHECKSUM);
            }

            // * * * *  N O O P  * * * *
            if (VSCP_SERIAL_DRIVER_OPERATION_NOOP ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                sendVSCPDriverAck(); // OK
            }
            // * * * *  V S C P  E V E N T  * * * *
            else if (VSCP_SERIAL_DRIVER_OPERATION_VSCP_EVENT ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {

            }
            // * * * *  C A N A L  E V E N T  * * * *
            else if (VSCP_SERIAL_DRIVER_OPERATION_CANAL ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                if ( receiveVSCPModeCanalMsg() ) {
                    sendVSCPDriverAck(); // OK
                }
                else {
                    sendVSCPDriverNack(); // Failed
                }
            }
            // * * * *  M U L T I  F R A M E  C A N A L  * * * *
            else if (VSCP_SERIAL_DRIVER_OPERATION_MULTI_FRAME_CANAL ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                if (receiveVSCPModeMultiCanalMsg()) {
                    sendVSCPDriverAck(); // OK
                }
                else {
                    sendVSCPDriverNack(); // Failed
                }
            }
            // * * * *  M U L T I  F R A M E  V S C P  * * * *
            else if (VSCP_SERIAL_DRIVER_OPERATION_MULTI_FRAME_CANAL ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                sendVSCPDriverNack(); // Not supported
            }
            // * * * *  C O N F I G U R E  * * * *
            else if (VSCP_SERIAL_DRIVER_OPERATION_CONFIGURE ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                sendVSCPDriverNack(); // Not supported
            }
            // * * * *  P O L L  * * * *
            else if (VSCP_SERIAL_DRIVER_OPERATION_POLL ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                sendVSCPDriverNack(); // Not supported
            }
            // * * * *  CAPABILITIES  * * * *
            else if (VSCP_SERIAL_DRIVER_OPERATION_CAPS_REQUEST ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                caps.maxVscpFrames = cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ];
                caps.maxCanalFrames = cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 1 ];
                sendVSCPModeCapabilities();     // SAend our capabilities
            }
            // * * * *  C O M M A N D  * * * *
            else if (VSCP_SERIAL_DRIVER_OPERATION_COMMAND ==
                    cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_TYPE ]) {
                // Noop
                if (cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                        VSCP_DRIVER_COMMAND_NOOP) {
                    sendVSCPDriverCommandReply(0, VSCP_DRIVER_COMMAND_NOOP);
                }
                // Open
                else if (cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                        VSCP_DRIVER_COMMAND_OPEN) {
                    ECANSetOperationMode(ECAN_OP_MODE_NORMAL);
                    sendVSCPDriverCommandReply(0, VSCP_DRIVER_COMMAND_OPEN);
                }
                // Loopback
                else if (cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                        VSCP_DRIVER_COMMAND_LISTEN) {
                    ECANSetOperationMode(ECAN_OP_MODE_LOOP);
                    sendVSCPDriverCommandReply(0, VSCP_DRIVER_COMMAND_LISTEN);
                }                    
                // Listen
                else if (cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                        VSCP_DRIVER_COMMAND_LOOPBACK) {
                    ECANSetOperationMode(ECAN_OP_MODE_LISTEN);
                    sendVSCPDriverCommandReply(0, VSCP_DRIVER_COMMAND_LOOPBACK);
                }
                // Close
                else if (cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                        VSCP_DRIVER_COMMAND_CLOSE) {
                    ECANSetOperationMode(ECAN_INIT_DISABLE);
                    sendVSCPDriverCommandReply(0, VSCP_DRIVER_COMMAND_CLOSE);
                }
                // Set filter
                else if (cmdbuf[ VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD ] ==
                        VSCP_DRIVER_COMMAND_SET_FILTER) {
                    sendVSCPDriverCommandReply(0, VSCP_DRIVER_COMMAND_NOOP);
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

    // Enable interrupt again
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

    if (1 == fifo_read(&serialInputFifo, &c, 1)) {

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
                    putsUSART((char *) "JFrankfurt RS-232\r\n"); // Mimic Lawicel adapter
                    rv = TRUE;
                }
                break;

                // Get version number
            case 'V':
                if (1 == strlen(cmdbuf)) {
                    putsUSART((char *) "V1011\r\n"); // Mimic Lawicel adapter
                    rv = TRUE;
                }
                break;

                // Get serial number
            case 'N':
                if (1 == strlen(cmdbuf)) {
                    putsUSART((char *) "N1977\r\n"); // Mimic Lawicel adapter
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

                    id = (uint32_t) wrkbuf[0] << 24 + (uint32_t) wrkbuf[1] << 16 + (uint32_t) wrkbuf[2] << 8 + wrkbuf[3];
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
                    putsUSART((char *) "F00\r\n");
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
                            writeEEPROM(MOUDLE_EEPROM_SLCAN_TIMESTAMP, SLCAN_TIMESTAMP_NOT_USED);
                            rv = TRUE;
                        } else if ('1' == cmdbuf[1]) {
                            nTimeStamp = SLCAN_TIMESTAMP_USE;
                            // Low resolution timestamp enabled
                            writeEEPROM(MOUDLE_EEPROM_SLCAN_TIMESTAMP,
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
                writeEEPROM(MODULE_EEPROM_INIT_BYTE1, 0xFF);
                Reset();
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
            putsUSART((char *) "\r");
        }
        else {
            // Failure, send BELL
            putsUSART((char *) "\a");
        }

        // Get ready for next command
        memset(cmdbuf, 0, sizeof ( cmdbuf));
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

        while BusyUSART();
        WriteUSART(DLE);
        //if (NULL != pcrc) crc8(pcrc, DLE);

        while BusyUSART();
        WriteUSART(DLE);
        if (NULL != pcrc) crc8(pcrc, DLE);

    }
    else {

        while BusyUSART();
        WriteUSART(c);
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
    while BusyUSART();
    WriteUSART(DLE);
    while BusyUSART();
    WriteUSART(STX);

    // Operation
    while BusyUSART();
    WriteUSART(VSCP_SERIAL_DRIVER_OPERATION_ERROR);
    crc8(&crc, VSCP_SERIAL_DRIVER_OPERATION_ERROR);

    // Channel
    while BusyUSART();
    WriteUSART(0);
    crc8(&crc, 0);

    // Sequency number
    sendEscapedUartData(cmdbuf[ 2 ], &crc);

    // Payload length
    while BusyUSART(); // MSB
    WriteUSART(0);
    crc8(&crc, 0);
    while BusyUSART(); // LSB
    WriteUSART(1);
    crc8(&crc, 1);

    // Payload == error code
    sendEscapedUartData(errorcode, &crc);

    // CRC
    sendEscapedUartData(crc, NULL);

    // End of frame
    while BusyUSART();
    WriteUSART(DLE);
    while BusyUSART();
    WriteUSART(ETX);
}

///////////////////////////////////////////////////////////////////////////////
// sendVSCPDriverAck
//

void sendVSCPDriverAck(void)
{
    uint8_t crc = 0;

    // Start of frame
    while BusyUSART();
    WriteUSART(DLE);
    while BusyUSART();
    WriteUSART(STX);

    // Operation
    while BusyUSART();
    WriteUSART(VSCP_SERIAL_DRIVER_OPERATION_ACK);
    crc8(&crc, VSCP_SERIAL_DRIVER_OPERATION_ACK);

    // Channel
    while BusyUSART();
    WriteUSART(0);
    crc8(&crc, 0);

    // Sequency number
    sendEscapedUartData(cmdbuf[ 2 ], &crc);

    // Payload length
    while BusyUSART(); // MSB
    WriteUSART(0);
    crc8(&crc, 0);
    while BusyUSART(); // LSB
    WriteUSART(0);
    crc8(&crc, 0);

    // Checksum
    sendEscapedUartData(crc, NULL);

    // End of frame
    while BusyUSART();
    WriteUSART(DLE);
    while BusyUSART();
    WriteUSART(ETX);
}

///////////////////////////////////////////////////////////////////////////////
// sendVSCPDriverNack
//

void sendVSCPDriverNack(void) {
    uint8_t crc = 0;

    // Start of frame
    while BusyUSART();
    WriteUSART(DLE);
    while BusyUSART();
    WriteUSART(STX);

    // Operation
    while BusyUSART();
    WriteUSART(VSCP_SERIAL_DRIVER_OPERATION_NACK);
    crc8(&crc, VSCP_SERIAL_DRIVER_OPERATION_NACK);

    // Channel
    while BusyUSART();
    WriteUSART(0);
    crc8(&crc, 0);

    // Sequency number
    sendEscapedUartData(cmdbuf[ 2 ], &crc);

    // Payload length
    while BusyUSART(); // MSB
    WriteUSART(0);
    crc8(&crc, 0);
    while BusyUSART(); // LSB
    WriteUSART(0);
    crc8(&crc, 0);

    // Checksum
    sendEscapedUartData(crc, NULL);

    // End of frame
    while BusyUSART();
    WriteUSART(DLE);
    while BusyUSART();
    WriteUSART(ETX);
}

///////////////////////////////////////////////////////////////////////////////
// sendVSCPDriverCommandReply
//

void sendVSCPDriverCommandReply(uint8_t cmdReplyCode, uint8_t cmdCode)
{
    uint8_t crc = 0;

    // Start of frame
    while BusyUSART();
    WriteUSART(DLE);
    while BusyUSART();
    WriteUSART(STX);

    // Operation
    while BusyUSART();
    WriteUSART(VSCP_SERIAL_DRIVER_OPERATION_COMMAND_REPLY);
    crc8(&crc, VSCP_SERIAL_DRIVER_OPERATION_COMMAND_REPLY);

    // Channel
    while BusyUSART();
    WriteUSART(0);
    crc8(&crc, 0);

    // Sequency number
    sendEscapedUartData(cmdbuf[ 2 ], &crc);

    // Payload length
    while BusyUSART(); // MSB
    WriteUSART(0);
    crc8(&crc, 0);
    while BusyUSART(); // LSB
    WriteUSART(2);
    crc8(&crc, 2);

    // Reply code 0 == OK
    while BusyUSART();
    sendEscapedUartData(cmdReplyCode, &crc);

    // Rely on command
    while BusyUSART();
    sendEscapedUartData(cmdCode, &crc);

    // Checksum
    sendEscapedUartData(crc, NULL);

    // End of frame
    while BusyUSART();
    WriteUSART(DLE);
    while BusyUSART();
    WriteUSART(ETX);
}

///////////////////////////////////////////////////////////////////////////////
// receivePrintEventVerbose
//

BOOL receivePrintEventVerbose(void)
{
    uint8_t i;

    if (getVSCPFrame(&vscpClass,
                        &vscpType,
                        &vscpNodeId,
                        &vscpPriority,
                        &vscpSize,
                        vscpData)) {

        // Statistics
        cntRxFrames++;
        cntRxBytes += vscpSize;

        putsUSART((char *) "<Prio=");
        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", vscpPriority);
        putsUSART(wrkbuf);
        putsUSART((char *) ",class=");
        sprintf(wrkbuf, bHex ? "0x%04X" : "%d", vscpClass);
        putsUSART(wrkbuf);
        putsUSART((char *) ",type=");
        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", vscpType);
        putsUSART(wrkbuf);
        putsUSART((char *) ",nodeid=");
        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", vscpNodeId);
        putsUSART(wrkbuf);
        putsUSART((char *) ",size=");
        sprintf(wrkbuf, bHex ? "0x%02X" : "%d", vscpSize);
        putsUSART(wrkbuf);
        if (vscpSize > 0) {
            putsUSART((char *) ",Data=");
            for (i = 0; i < vscpSize; i++) {
                //itoa(wrkbuf, vscpData[i], bHex ? 16 : 10);
                sprintf(wrkbuf, bHex ? "0x%02X" : "%d", vscpData[i]);
                putsUSART(wrkbuf);
                if (i < (vscpSize - 1)) {
                    putsUSART((char *) ",");
                }
            }
        } else {
            putsUSART((char *) ",Data=none");
        }
        putsUSART((char *) ">\r\n");
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
        while BusyUSART();
        WriteUSART(DLE);
        while BusyUSART();
        WriteUSART(STX);

        // Operation
        while BusyUSART();
        WriteUSART(VSCP_SERIAL_DRIVER_OPERATION_CANAL);
        crc8(&crc, VSCP_SERIAL_DRIVER_OPERATION_CANAL);

        // Channel
        while BusyUSART();
        WriteUSART(0);
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
        while BusyUSART();
        WriteUSART(DLE);
        while BusyUSART();
        WriteUSART(ETX);

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
        while BusyUSART();
        WriteUSART(DLE);
        while BusyUSART();
        WriteUSART(STX);

        // Operation
        while BusyUSART();
        WriteUSART(VSCP_SERIAL_DRIVER_OPERATION_MULTI_FRAME_CANAL);
        crc8(&crc, VSCP_SERIAL_DRIVER_OPERATION_MULTI_FRAME_CANAL);

        // Channel
        while BusyUSART();
        WriteUSART(0);
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
        while BusyUSART();
        WriteUSART(DLE);
        while BusyUSART();
        WriteUSART(ETX);

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
        while BusyUSART();
        WriteUSART(DLE);
        while BusyUSART();
        WriteUSART(STX);

        // Operation
        while BusyUSART();
        WriteUSART(VSCP_SERIAL_DRIVER_OPERATION_VSCP_EVENT);
        crc8(&crc, VSCP_SERIAL_DRIVER_OPERATION_VSCP_EVENT);

        // Channel
        while BusyUSART();
        WriteUSART(0);
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
        while BusyUSART();
        WriteUSART(DLE);
        while BusyUSART();
        WriteUSART(ETX);

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
    while BusyUSART();
    WriteUSART(DLE);
    while BusyUSART();
    WriteUSART(STX);

    // Operation
    while BusyUSART();
    WriteUSART(VSCP_SERIAL_DRIVER_OPERATION_CAPS_RESPONSE);
    crc8(&crc, VSCP_SERIAL_DRIVER_OPERATION_CAPS_RESPONSE);

    // Channel
    while BusyUSART();
    WriteUSART(0);
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
    while BusyUSART();
    WriteUSART(DLE);
    while BusyUSART();
    WriteUSART(ETX);

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

        while BusyUSART();
        WriteUSART('T');

        ultoa(wrkbuf, id, 16);
        // First print leading zeros so length is eight characers
        for (i = 0; i < ((uint8_t) (8 - sizeof (wrkbuf))); i++) {
            while BusyUSART();
            WriteUSART('0');
        }
        putsUSART(wrkbuf);

        // Send dlc - always one digit
        itoa(wrkbuf, dlc, 16);
        while BusyUSART();
        WriteUSART(wrkbuf[0]);

        // Sen data
        for (i = 0; i < dlc; i++) {
            itoa(wrkbuf, vscpData[i], 16);
            // Leading zero if needed
            if (2 != strlen(wrkbuf)) {
                while BusyUSART();
                WriteUSART('0');
            }
            // Data
            putsUSART(wrkbuf);
        }

        // If timestamp is active add it to
        if (nTimeStamp) {
            ultoa(wrkbuf, timer, 16);
            // First print leading zeros so length is eight characers
            for (i = 0; i < ((uint8_t) (8 - sizeof (wrkbuf))); i++) {
                while BusyUSART();
                WriteUSART('0');
            }
            putsUSART(wrkbuf);
        }

        while BusyUSART();
        WriteUSART(0x0d);
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

    id = ((uint32_t) cmdbuf[VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD] << 26) |
            ((uint32_t) cmdbuf[VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 1] << 16) |
            ((uint32_t) cmdbuf[VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 2] << 8) |
            cmdbuf[VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 3]; // nodeaddress (our address)
    dlc = cmdbuf[VSCP_SERIAL_DRIVER_POS_FRAME_SIZE_PAYLOAD_LSB] - 4;
    memcpy(data, cmdbuf + VSCP_SERIAL_DRIVER_POS_FRAME_PAYLOAD + 4, dlc);

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

    if (sendVSCPFrame(VSCP_CLASS1_PROTOCOL,         // class
            VSCP_TYPE_PROTOCOL_EXTENDED_PAGE_READ,  // Read register
            0, // Node id (ours)
            0, // High priority
            5, // size
            vscpData)) {

        timekeeper = 0;
        while (timekeeper < timeout) {

            ClrWdt(); // Feed the dog

            if (getVSCPFrame(&vscpClass,
                    &vscpType,
                    &vscpNodeId,
                    &vscpPriority,
                    &vscpSize,
                    vscpData)) {
                if ((nodeid == vscpNodeId) &&
                        (VSCP_CLASS1_PROTOCOL == vscpClass) &&
                        (VSCP_TYPE_PROTOCOL_EXTENDED_PAGE_RESPONSE == vscpType) &&
                        (5 == vscpSize) &&
                        (0 == vscpData[ 0 ]) &&
                        ((page >> 8) == vscpData[ 1 ]) &&
                        ((page & 0xff) == vscpData[ 2 ]) &&
                        (reg == vscpData[ 3 ])) {
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
    vscpData[ 0 ] = 1; // Node id
    vscpData[ 1 ] = 0; // Page MSB
    vscpData[ 2 ] = 0; // Page LSB
    vscpData[ 3 ] = 0xD0; // Offset
    vscpData[ 4 ] = 16; // Number of regs to read

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
            while (BusyUSART());
            WriteUSART('1');
        } else {
            while (BusyUSART());
            WriteUSART('0');
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// printStatistics
//

void printStatistics(void)
{
    putsUSART((char *) "Sent CAN frames: ");
    sprintf(wrkbuf, bHex ? "0x%08X" : "%lu", cntTxFrames);
    putsUSART(wrkbuf);
    putsUSART((char *) "\r\n");

    putsUSART((char *) "Sent CAN bytes: ");
    sprintf(wrkbuf, bHex ? "0x%08X" : "%lu", cntTxBytes);
    putsUSART(wrkbuf);
    putsUSART((char *) "\r\n");

    putsUSART((char *) "Received CAN frames: ");
    sprintf(wrkbuf, bHex ? "0x%08X" : "%lu", cntRxFrames);
    putsUSART(wrkbuf);
    putsUSART((char *) "\r\n");

    putsUSART((char *) "Received CAN bytes: ");
    sprintf(wrkbuf, bHex ? "0x%08X" : "%lu", cntRxBytes);
    putsUSART(wrkbuf);
    putsUSART((char *) "\r\n");
}

///////////////////////////////////////////////////////////////////////////////
// printErrors
//

void printErrors(void)
{
    putsUSART((char *) "CAN Receive overruns: ");
    sprintf(wrkbuf, bHex ? "0x%08lX" : "%lu", can_receiveOverruns);
    putsUSART(wrkbuf);
    putsUSART((char *) "\r\n");

    putsUSART((char *) "CAN Transmit overruns: ");
    sprintf(wrkbuf, bHex ? "0x%08lX" : "%lu", can_transmitOverruns);
    putsUSART(wrkbuf);
    putsUSART((char *) "\r\n");

    putsUSART((char *) "UART Receive overruns: ");
    sprintf(wrkbuf, bHex ? "0x%08lX" : "%lu", uart_receiveOverruns);
    putsUSART(wrkbuf);
    putsUSART((char *) "\r\n");

    putsUSART((char *) "UART Receive overruns: ");
    sprintf(wrkbuf, bHex ? "0x%08lX" : "%lu", uart_transmitOverruns);
    putsUSART(wrkbuf);
    putsUSART((char *) "\r\n");

    putsUSART((char *) "Transmit Error Counter: ");
    sprintf(wrkbuf, bHex ? "0x%02X" : "%d", TXERRCNT);
    putsUSART(wrkbuf);
    putsUSART((char *) "\r\n");

    if (COMSTATbits.TXWARN) {
        putsUSART((char *) "Transmitter in Error State Warning (128 > TERRCNT ? 96)\r\n");
    }

    if (COMSTATbits.TXBO) {
        putsUSART((char *) "Transmitter in Error State Bus OFF (TERRCNT ? 256)\r\n");
    }

    if (COMSTATbits.TXBP) {
        putsUSART((char *) "Transmitter in Error State Bus Passive (TERRCNT ? 128)\r\n");
    }


    putsUSART((char *) "Receive Error Counter: ");
    sprintf(wrkbuf, bHex ? "0x%02X" : "%d", RXERRCNT);
    putsUSART(wrkbuf);
    putsUSART((char *) "\r\n");

    if (COMSTATbits.RXWARN) {
        putsUSART((char *) "Receiver in Error State Warning (128 > RERRCNT ? 96)\r\n");
    }

    if (COMSTATbits.RXBP) {
        putsUSART((char *) "Receiver in Error State Bus Passive (RERRCNT ? 128)\r\n");
    }

    if (COMSTATbits.EWARN) {
        putsUSART((char *) "Transmitter or Receiver is in Error State Warning\r\n");
    }

}


///////////////////////////////////////////////////////////////////////////////
// printHelp
//

void printHelp(void)
{
    putsUSART((char *) "Help for the Frankfurt RS-232 module\r\n");
    putsUSART((char *) "------------------------------------\r\n");
    putsUSART((char *) "BOOT - Enter bootloader.\r\n");
    putsUSART((char *) "OPEN - Open CAN interface in normal mode.\r\n");
    putsUSART((char *) "SILENT - Open CAN interface in silent mode.\r\n");
    putsUSART((char *) "LISTEN - Open CAN interface in listen only mode.\r\n");
    putsUSART((char *) "LOOPBACK - Open CAN interface in loopback mode.\r\n");
    putsUSART((char *) "CLOSE - Close CAN interface.\r\n");
    putsUSART((char *) "VERSION - Display firmware version information.\r\n");
    putsUSART((char *) "IFMODE - Display selected interface mode.\r\n");
    putsUSART((char *) "TX - Send CAN frame .\r\n");
    putsUSART((char *) "     Format: priority,class,type,nodeid,count,data,,,\r\n");
    putsUSART((char *) "RX - Read CAN frame.\r\n");
    putsUSART((char *) "STAT - Display CAN statistics.\r\n");
    putsUSART((char *) "ERR - Display CAN error information.\r\n");
    putsUSART((char *) "HELP - Display this help information.\r\n");
    putsUSART((char *) "FIND - Find available CAN4VSCP nodes on bus.\r\n");
    putsUSART((char *) "RREG - Read register(s) of node (Format: rreg [page:]reg [count]).\r\n");
    putsUSART((char *) "WREG - Write register of node (Format: wreg [page:]reg content).\r\n");
    putsUSART((char *) "INFO - Get info about an existent node on the bus (Format: info nickname).\r\n");
    putsUSART((char *) "FILTER - Set filter .\r\n");
    putsUSART((char *) "         Format: filter filterno,prio,class,type,nodeid  (filterno = 0-15).\r\n");
    putsUSART((char *) "MASK - Set mask .\r\n");
    putsUSART((char *) "       Format: mask maskno,prio,class,type,nodeid (maskno = 0 or 1).\r\n");
    putsUSART((char *) "SET - Persistent functionality.\r\n");
    putsUSART((char *) "    HEX - Display numericals in hexadecimal.\r\n");
    putsUSART((char *) "    DECIMAL - Display numericals in decimal.\r\n");
    putsUSART((char *) "    RWTIMEOUT - Set register read/write timeout. Default=10 ms .\r\n");
    putsUSART((char *) "                Format: set rwtimeout timeout.\r\n");
    putsUSART((char *) "    STARTIF - Set interface state to use on startup.\r\n");
    putsUSART((char *) "    MODE - Set adapter mode that should be used on startup.\r\n");
    putsUSART((char *) "           Modes: verbose|vscp|slcan\r\n");
}


///////////////////////////////////////////////////////////////////////////////
// findNodes
//

void findNodes(void)
{
    uint8_t nFound = 0;
    uint8_t i;
    uint8_t value;

    putsUSART((char *) "----------------------------------------------------------------\r\n");

    for (i = 1; i != 0; i++) {

        ClrWdt(); // Feed the dog

        if (readRegister(i,
                0xE0,
                rwtimeout,
                &value)) {
            putsUSART((char *) "Node found with node id = ");
            itoa(wrkbuf, vscpNodeId, bHex ? 16 : 10);
            putsUSART(wrkbuf);
            putsUSART((char *) "\r\n");
            printNodeFirmwareVersion(i);
            printGUID(i);
            printMDF(i);
            putsUSART((char *) "----------------------------------------------------------------\r\n");
            nFound++; // Another one found
        }

    }

    itoa(wrkbuf, nFound, 10);
    putsUSART(wrkbuf);
    putsUSART((char *) " nodes found\r\n");

}


///////////////////////////////////////////////////////////////////////////////
// printGUID
//

void printGUID(uint8_t nodeid)
{
    uint8_t i;
    uint8_t value;
    char buf[3];

    memset(wrkbuf, 0, sizeof (wrkbuf));
    putsUSART((char *) "GUID = ");

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

    putsUSART(wrkbuf);
    putsUSART((char *) "\r\n");
}


///////////////////////////////////////////////////////////////////////////////
// printMDF
//

void printMDF(uint8_t nodeid)
{
    uint8_t i;
    uint8_t value;
    char *p = wrkbuf;
    memset(wrkbuf, 0, sizeof (wrkbuf));
    putsUSART((char *) "MDF = http://");

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

    putsUSART(wrkbuf);
    putsUSART((char *) "\r\n");
}

///////////////////////////////////////////////////////////////////////////////
// printNodeFirmwareVersion
//

void printNodeFirmwareVersion(uint8_t nodeid)
{
    uint8_t i;
    uint8_t value;
    char buf[3];

    memset(wrkbuf, 0, sizeof (wrkbuf));
    putsUSART((char *) "Firmware version = ");

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

    putsUSART(wrkbuf);
    putsUSART((char *) "\r\n");
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
    putsUSART((char *) "Version: ");
    itoa(wrkbuf, FIRMWARE_MAJOR_VERSION, 10);
    putsUSART(wrkbuf);
    while (BusyUSART());
    WriteUSART('.');
    itoa(wrkbuf, FIRMWARE_MINOR_VERSION, 10);
    putsUSART(wrkbuf);
    while (BusyUSART());
    WriteUSART('.');
    itoa(wrkbuf, FIRMWARE_SUB_MINOR_VERSION, 10);
    putsUSART(wrkbuf);
    putsUSART((char *) "\r\n");
}

///////////////////////////////////////////////////////////////////////////////
// printMode
//

void printMode(void)
{
    putsUSART((char *) "Mode: ");
    if (WORKING_MODE_VERBOSE == mode) {
        putsUSART((char *) "Verbose");
    } else if (WORKING_MODE_VSCP_DRIVER == mode) {
        putsUSART((char *) "VSCP Driver");
    } else if (WORKING_MODE_SL_DRIVER == mode) {
        putsUSART((char *) "SL Driver");
    } else if (WORKING_MODE_VSCP_NODE == mode) {
        putsUSART((char *) "VSCP Node");
    } else {
        putsUSART((char *) "Unknown (Verbose used)");
    }
    putsUSART((char *) "\r\n");
}

///////////////////////////////////////////////////////////////////////////////
// setFilter
//

void setFilter(uint8_t filter, uint32_t val)
{
    uint8_t sidh = (long) val >> 21L;
    uint8_t sidl = (((long) val >> 13L) & 0xe0) |
            ((long) (val) & 0x03L) |
            0x08;
    uint8_t eidh = (long) val >> 8L;
    uint8_t eidl = val;

    switch (filter) {

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
    uint32_t id = ((uint32_t) priority << 26) |
            ((uint32_t) vscpclass << 16) |
            ((uint32_t) vscptype << 8) |
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
/*
    ECAN_RX_MSG_FLAGS flags;

    if (ECANReceiveMessage((unsigned long *) pid,
                            (BYTE*) pdata,
                            (BYTE*) pdlc,
                            &flags) ) {

        // RTR not interesting
        if (flags & ECAN_RX_RTR_FRAME) return FALSE;

        // Must be extended frame
        if (!(flags & ECAN_RX_XTD_FRAME)) return FALSE;

        return TRUE;
    }

    return FALSE;
 */

    if ( canrxcount ) {
        
        di(); // Disable interrupt

        // Get id
        if ( 4 != fifo_read( &canInputFifo, (uint8_t *)pid, 4 ) ) {
            ei();   // Enable interrupt
            return FALSE;
        }

        // Get dlc
        if ( 1 != fifo_read( &canInputFifo, pdlc, 1 ) ) {
            ei();   // Enable interrupt
            return FALSE;
        }

        // Get data
        if ( *pdlc != fifo_read( &canInputFifo, pdata, *pdlc ) ) {
            ei();   // Enable interrupt
            return FALSE;
        }

        canrxcount--;   // One less CAN frame in fifo

        ei();   // Enable interrupt
    
        return TRUE;
    }

    return FALSE;
    
}

