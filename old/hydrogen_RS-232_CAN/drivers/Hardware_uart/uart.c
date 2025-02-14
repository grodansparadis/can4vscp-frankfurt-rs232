
/****************************************************************************
 Title  :   C  file for the UART FUNCTIONS library (uart.c)
 Author:    Chris efstathiou hendrix@otenet.gr
 Date:      30/Aug/2002
 Software:  AVR-GCC with AVR-AS
 Target:    any AVR device
 Comments:  This software is FREE.

*****************************************************************************/
#ifndef _IO_REG_MACRO_MODE_
#define _IO_REG_MACRO_MODE_  1     /* In case you have the new assignment mode io headers */
#endif

#ifndef  _SFR_ASM_COMPAT
#define  _SFR_ASM_COMPAT     1     /* This is for GCC 3.2 */
#endif

#include <io.h>
#include <sig-avr.h>
#include <progmem.h>
#include <eeprom.h>
#include "uart.h"

#if SREG > 0x3F
#define IO_TO_MEM_OFFSET   0
#elif SREG <= 0x3F
#define IO_TO_MEM_OFFSET   0x20       
#endif


/* IF DYNAMIC BAUD RATE IS NOT MANDATORY */
/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#if UART_DYNAMIC_BAUD_CHANGE == 0 && NUMBER_OF_UARTS >= 1
/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

#define UART0_DIVIDER   16

#define UART0_BAUDRATE_ACCURATE   (F_CPU/(UART0_BAUDRATE/100))  
#define UART0_BAUDRATE_REAL       (((F_CPU/UART0_BAUDRATE)/UART0_DIVIDER)*UART0_DIVIDER)

/*22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#if  ((UART0_BAUDRATE_ACCURATE/UART0_BAUDRATE_REAL)-100) > 2
/*22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

#if UART_DOUBLE_SPEED_CAPABLE == 1

#undef UART0_DIVIDER 
#define UART0_DIVIDER 8
#define UART0_BAUDRATE_X2   1
#if  ((UART0_BAUDRATE_ACCURATE/UART0_BAUDRATE_REAL)-100) > 1
#error " UART0_BAUDRATE_ERROR_TOO_HIGH! "
#endif 

#elif UART_DOUBLE_SPEED_CAPABLE == 0  /* #if UART_DOUBLE_SPEED_CAPABLE == 1 */

#if  ((UART0_BAUDRATE_ACCURATE/UART0_BAUDRATE_REAL)-100) > 2
#error " UART0_BAUDRATE_ERROR_TOO_HIGH! "
#endif 

#endif  /* #elif UART_DOUBLE_SPEED_CAPABLE == 0 */

/*22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#endif
/*22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

#define UBRR0_VALUE ( ((F_CPU/UART0_BAUDRATE)/UART0_DIVIDER)-1 )


#if UBRR0_VALUE > 255 && UBRR0_VALUE <= 0XFFF && UART0_UBRRH > 0
#define UBRRH0_VALUE ( (UBRR0_VALUE>>8) & 0X0F )
#elif  UBRR0_VALUE > 0XFFF && UART0_UBRRH > 0
#error " UART0_ERROR BAUDRATE TOO LOW! "
#elif UBRR0_VALUE > 255 
#error " UART0_ERROR BAUDRATE TOO LOW! "
#endif

/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#endif  /* #if UART_DYNAMIC_BAUD_CHANGE == 0 && NUMBER_OF_UARTS >= 1 */
/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#if  UART_DYNAMIC_BAUD_CHANGE == 0 && NUMBER_OF_UARTS >= 2
/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

#define UART1_DIVIDER   16

#define UART1_BAUDRATE_ACCURATE   (F_CPU/(UART1_BAUDRATE/100))  
#define UART1_BAUDRATE_REAL       (((F_CPU/UART1_BAUDRATE)/UART1_DIVIDER)*UART1_DIVIDER)

/*22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#if  ((UART1_BAUDRATE_ACCURATE/UART1_BAUDRATE_REAL)-100) > 2
/*22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

#if UART_DOUBLE_SPEED_CAPABLE == 1

#undef UART1_DIVIDER 
#define UART1_DIVIDER 8
#define UART1_BAUDRATE_X2   1
#if  ((UART1_BAUDRATE_ACCURATE/UART1_BAUDRATE_REAL)-100) > 1
#error " UART1_BAUDRATE_ERROR_TOO_HIGH! "
#endif 

#elif UART_DOUBLE_SPEED_CAPABLE == 0  /* #if UART_DOUBLE_SPEED_CAPABLE == 1 */

#if  ((UART1_BAUDRATE_ACCURATE/UART1_BAUDRATE_REAL)-100) > 2
#error " UART1_BAUDRATE_ERROR_TOO_HIGH! "
#endif 

#endif  /* #elif UART_DOUBLE_SPEED_CAPABLE == 0 */

/*22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#endif
/*22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

#define UBRR1_VALUE ( ((F_CPU/UART1_BAUDRATE)/UART1_DIVIDER)-1 )


#if UBRR1_VALUE > 255 && UBRR1_VALUE <= 0XFFF && UART1_UBRRH > 0
#define UBRRH1_VALUE ( (UBRR1_VALUE>>8) & 0X0F )
#elif  UBRR1_VALUE > 0XFFF && UART1_UBRRH > 0
#error " UART1_ERROR BAUDRATE TOO LOW! "
#elif UBRR1_VALUE > 255 
#error " UART1_ERROR BAUDRATE TOO LOW! "
#endif

/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#endif  /* UART_DYNAMIC_BAUD_CHANGE == 0 && NUMBER_OF_UARTS >= 2 */
/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

/* Mistyped switches error catching */
#if UART_DYNAMIC_BAUD_CHANGE > 1 || UART_DYNAMIC_BAUD_CHANGE < 0
#error "UART_DYNAMIC_BAUD_CHANGE" CAN BE 0 OR 1 ! 
#endif

#if UART0_ERROR_CHECK_SUPPORT  > 1 || UART0_ERROR_CHECK_SUPPORT  < 0
#error  "UART0_ERROR_CHECK_SUPPORT"  CAN BE 0 OR 1 ! 
#endif

#if UART0_AT_COMMAND_SUPPORT > 1 || UART0_AT_COMMAND_SUPPORT < 0
#error "UART0_AT_COMMAND_SUPPORT"  CAN BE 0 OR 1 ! 
#endif

#if UART0_RAW_DATA_SUPPORT > 1 || UART0_RAW_DATA_SUPPORT < 0
#error "UART0_RAW_DATA_SUPPORT" CAN BE 0 OR 1 ! 
#endif

#if UART1_ERROR_CHECK_SUPPORT > 1 || UART1_ERROR_CHECK_SUPPORT < 0
#error  "UART1_ERROR_CHECK_SUPPORT"  CAN BE 0 OR 1 ! 
#endif

#if UART1_AT_COMMAND_SUPPORT > 1 || UART1_AT_COMMAND_SUPPORT < 0
#error "UART1_AT_COMMAND_SUPPORT"  CAN BE 0 OR 1 ! 
#endif

#if UART1_RAW_DATA_SUPPORT > 1 || UART1_RAW_DATA_SUPPORT < 0
#error "UART1_RAW_DATA_SUPPORT" CAN BE 0 OR 1 ! 
#endif

#if UART_MULTI_COMMAND_SUPPORT > 1 || UART_MULTI_COMMAND_SUPPORT < 0
#error "UART_MULTI_COMMAND_SUPPORT"  CAN BE 0 OR 1 ! 
#endif
 


/*####################################################################################################*/ 
/* uart globals */
/*####################################################################################################*/ 
/* TYPE DEFINITIONS  */
typedef unsigned char  u08;
typedef          char  s08;
typedef unsigned short u16;
typedef          short s16;


#if NUMBER_OF_UARTS == 2



volatile u08    uart_udr;
volatile u08    uart_ucr;
volatile u08    uart_usr;
volatile u08    uart_ubrrl;
#if UART0_UBRRH > 0 && UART1_UBRRH > 0
volatile u08    uart_ubrrh;
#endif
#if UART0_UCSRC > 0
volatile u08    uart_ucsrc;
#endif
volatile        u08    uart;
   
/* TxD variables */
volatile u08    tx_busy[NUMBER_OF_UARTS];
volatile u08    *tx_ptr[NUMBER_OF_UARTS];
volatile u08    tx_on[NUMBER_OF_UARTS];
volatile u08    echo[NUMBER_OF_UARTS];
volatile u08    data_location[NUMBER_OF_UARTS];

/* RxD variables */
volatile u08    uart_rx_buffer[NUMBER_OF_UARTS][(UART_RX_BUFFER_SIZE+1)];
volatile u08    rx_c[NUMBER_OF_UARTS];
volatile u08    rx_ISR_counter[NUMBER_OF_UARTS];
volatile u08    uart_rx_count[NUMBER_OF_UARTS];
volatile u08    uart_error[NUMBER_OF_UARTS];
volatile u08    uart_string_received[NUMBER_OF_UARTS];
volatile u08    uart_at_cmd_detected[NUMBER_OF_UARTS];
volatile u08    rx_mode[NUMBER_OF_UARTS];
volatile u08    temp1[NUMBER_OF_UARTS];

/*####################################################################################################*/ 
/* PRIVATE ROUTINES  */
/*####################################################################################################*/

/* private function prototypes */
#if UART_MULTI_COMMAND_SUPPORT == 1
static unsigned char execute_uart_command(unsigned int command);
#endif
static void          flush_rx_buffer(void);
/* IN MEGA 128 THE I/O SPACE ABOVE 0X64 CAN ONLY BE REACHED USING THE LDD/LDS/LD, ST/STS/STD COMMANDS */
static void          outpvar(unsigned char value, unsigned char port);
static void          sbivar(unsigned char port, unsigned char bit);
static void          cbivar(unsigned char port, unsigned char bit);
static unsigned char inpvar(unsigned char port);
#if UART0_ERROR_CHECK_SUPPORT == 1 || UART1_ERROR_CHECK_SUPPORT == 1
static unsigned char bit_is_set_var(unsigned char port, unsigned char bit);
#endif
/*####################################################################################################*/ 
static void outpvar(unsigned char value, unsigned char port)
{
   *((unsigned char*)(port+IO_TO_MEM_OFFSET))=value;

return;
}
/*####################################################################################################*/

static void sbivar(unsigned char port, unsigned char bit)
{

    *((unsigned char*)(port+IO_TO_MEM_OFFSET)) |= (1<<bit);

return;
}
/*####################################################################################################*/

static void cbivar(unsigned char port, unsigned char bit)
{

    *((unsigned char*)(port+IO_TO_MEM_OFFSET)) &= ~(1<<bit);

return;
}
/*####################################################################################################*/

static unsigned char inpvar(unsigned char port)
{

return( *((unsigned char*)(port+IO_TO_MEM_OFFSET)) );
}
/*####################################################################################################*/

#if UART0_ERROR_CHECK_SUPPORT == 1 || UART1_ERROR_CHECK_SUPPORT == 1
static unsigned char bit_is_set_var(unsigned char port, unsigned char bit)
{

return( *((unsigned char*)(port+IO_TO_MEM_OFFSET)) & (1<<bit) );
}
#endif
/*####################################################################################################*/

static void flush_rx_buffer(void)
{
/* clear uart_rx_buffer any rx system messages */
    
    for(uart_rx_count[uart]=0;uart_rx_count[uart]<=UART_RX_BUFFER_SIZE;uart_rx_count[uart]++)
      { 
         *(uart_rx_buffer[uart]+uart_rx_count[uart])='\0';
      }   
    uart_rx_count[uart]=0;
    uart_string_received[uart]=0;
    
}
/*####################################################################################################*/

#if UART_MULTI_COMMAND_SUPPORT == 1
static unsigned char execute_uart_command(unsigned int command)
#elif UART_MULTI_COMMAND_SUPPORT == 0
unsigned char        uart_command(unsigned int command)
#endif
{
   
switch(command)
     {
        case(UART_RX_OFF)         : cbivar(uart_ucr, UART_RXCIE);
                                    cbivar(uart_ucr, UART_RXEN);
                                    break;                          

        case(UART_GET_STRING)     : cbivar(uart_ucr, UART_RXCIE);
                                    inpvar(uart_udr);
                                    flush_rx_buffer(); 
                                    uart_error[uart]=0;
                                    rx_mode[uart]=UART_GET_AT_CMD;
                                    uart_at_cmd_detected[uart]=1;
                                    sbivar(uart_ucr, UART_RXCIE);
                                    sbivar(uart_ucr, UART_RXEN);
                                    break;                           
#if UART0_AT_COMMAND_SUPPORT == 1 || UART1_AT_COMMAND_SUPPORT == 1
        case(UART_GET_AT_CMD)     : cbivar(uart_ucr, UART_RXCIE);
                                    inpvar(uart_udr);
                                    flush_rx_buffer(); 
                                    uart_error[uart]=0;
                                    rx_mode[uart]=UART_GET_AT_CMD;
                                    uart_at_cmd_detected[uart]=0;
                                    temp1[uart]=0;
                                    sbivar(uart_ucr, UART_RXCIE);
                                    sbivar(uart_ucr, UART_RXEN);
                                    break;                                                          
#endif          
#if UART0_RAW_DATA_SUPPORT == 1 || UART1_RAW_DATA_SUPPORT == 1
        case(UART_GET_RAW_DATA)   : cbivar(uart_ucr, UART_RXCIE);
                                    inpvar(uart_udr);
                                    flush_rx_buffer();
                                    uart_error[uart]=0;
                                    rx_mode[uart]=UART_GET_RAW_DATA;
                                    sbivar(uart_ucr, UART_RXCIE);
                                    sbivar(uart_ucr, UART_RXEN);
                                    break;  
#endif
        case(UART_GET_RX_COUNT)   : return(uart_rx_count[uart]);
                                    break;                              

        case(UART_STRING_RECEIVED): return(uart_string_received[uart]);
                                    break;  

        case(UART_AT_CMD_RECEIVED): return(uart_string_received[uart]);
                                    break;  

#if UART0_ERROR_CHECK_SUPPORT == 1 || UART1_ERROR_CHECK_SUPPORT == 1           
        case(UART_GET_ERROR)      : return(uart_error[uart]);
                                    break;  
#endif

        case(UART_TX_ON)          : sbivar(uart_ucr, UART_TXCIE);
                                    sbivar(uart_ucr, UART_TXEN);
                                    tx_on[uart]=1;
                                    break;                 
           
        case(UART_TX_OFF)         : while(tx_busy[uart]);
                                    cbivar(uart_ucr, UART_TXCIE);
                                    cbivar(uart_ucr, UART_TXEN);
                                    tx_on[uart]=0;
                                    break;  

        case(UART_ECHO_ON)        : echo[uart]=1;
                                    break;                                                    
           
        case(UART_ECHO_OFF)       : echo[uart]=0;
                                    break;  

        case(SELECT_UART0)        : uart=0;
                                    uart_udr=UART0_UDR;
                                    uart_ucr=UART0_UCR;
                                    uart_usr=UART0_USR;
                                    uart_ubrrl=UART0_UBRRL;
#if UART0_UBRRH > 0 
                                    uart_ubrrh=UART0_UBRRH;
#endif
#if UART0_UCSRC > 0
                                    uart_ucsrc=UART0_UCSRC;
#endif
                                    break;

        case(SELECT_UART1)        : uart=1;
                                    uart_udr=UART1_UDR;
                                    uart_ucr=UART1_UCR;
                                    uart_usr=UART1_USR;
                                    uart_ubrrl=UART1_UBRRL;
#if UART1_UBRRH > 0 
                                    uart_ubrrh=UART1_UBRRH;
#endif 
#if UART1_UCSRC > 0
                                    uart_ucsrc=UART1_UCSRC;
#endif                                 
                                    break;
          
                                 
           
           /* IF NONE OF THE ABOVE */
        default                   : break;  
           
         }  

return(0); 
}
/*####################################################################################################*/
/* PUBLIC  CONTROL  FUNCTIONS */
/*####################################################################################################*/

/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#if   UART_DYNAMIC_BAUD_CHANGE == 0
/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
void uart_init(void)
{

#if UART0_UBRRH > 0 && UART1_UBRRH > 0
#if UART1_UBRRH == UART0_UBRRH && (defined(UBRRH0_VALUE) || defined(UBRRH1_VALUE)) 
unsigned char temp=0;
#endif
#endif

for(uart=0; uart<NUMBER_OF_UARTS; uart++)
  {
     /* INITIALIZE  UART0 */
     while(tx_busy[uart]);
     asm("cli");
     rx_c[uart]='\0';
     flush_rx_buffer();
  } 
     /* DISABLE RxD / Txd and associated interrupts  */
     outp(0,UART0_UCR);
     /* DISABLE RxD / Txd and associated interrupts  */
     outp(0,UART1_UCR);
     /* SELECT PROPERLY UART0 */
     uart_command(SELECT_UART0);

#if defined(UART0_BAUDRATE_X2)
    sbi(UART0_USR, UART_U2X);
#endif

#if UART0_UBRRH > 0 && defined(UBRRH0_VALUE) 

#if UART1_UBRRH > UART0_UBRRH || UART1_UBRRH < UART0_UBRRH
    outp((unsigned char)UBRRH0_VALUE&0X0F, UART0_UBRRH);
#else
    temp=(inp(UART0_UBRRH)&0xF0);
    temp|=(unsigned char)UBRRH0_VALUE&0X0F;
    outp(temp, UART0_UBRRH);
#endif

#endif /* #if UART0_UBRRH > 0 && defined(UBRRH0_VALUE) */

    outp((unsigned char)UBRR0_VALUE, UART0_UBRRL);

#if defined(UART1_BAUDRATE_X2)
    sbi(UART1_USR, UART_U2X);
#endif

#if UART1_UBRRH > 0 && defined(UBRRH1_VALUE) 

#if UART1_UBRRH > UART0_UBRRH || UART1_UBRRH < UART0_UBRRH
    outp((unsigned char)UBRRH1_VALUE&0X0F, UART1_UBRRH);
#else
    temp=(inp(UART1_UBRRH)&0x0F);
    temp|=(unsigned char)UBRRH1_VALUE&0XF0;
    outp(temp, UART1_UBRRH);
#endif

#endif

    outp((unsigned char)UBRR1_VALUE, UART1_UBRRL);
    asm("sei");    

return;
}
/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#elif UART_DYNAMIC_BAUD_CHANGE == 1
/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

unsigned char uart_init(unsigned long int baud_rate, unsigned int command)
/* initialize uart */
{
unsigned char divider=0;
union word_union {
                     signed int  ubrr_value; 
                     unsigned char ubrr_byte[2];
                 } baud;  
    
    while(tx_busy[uart]);
    asm("cli");
/*  EXECUTE THE GIVEN UART COMMANDS */
    uart_command(command);
/* INITIALIZE THE UART AND SET BAUD RATE */
    rx_c[uart]='\0';
    flush_rx_buffer();
    /* DISABLE RxD / Txd and associated interrupts  */
    outpvar(0,uart_ucr);

    divider=16;
    /* set baud rate */
    baud.ubrr_value = (((F_CPU/baud_rate)/divider)-1);   /* UBRR=25 for 4 mhz and 9600 baud  */
    if( (((F_CPU/(baud_rate/100))/((baud.ubrr_value+1)*divider))-100) > 2  )
     { 
#if UART_DOUBLE_SPEED_CAPABLE == 1
        divider=8;
        sbivar(uart_usr, UART_U2X);
        baud.ubrr_value = (((F_CPU/baud_rate)/divider)-1);
        if( (( (F_CPU/(baud_rate/100))/((baud.ubrr_value+1)*divider) )-100) > 1  )
         { 
             return(UART_HIGH_BAUDRATE_ERROR); 
         }
#else 
        return(UART_HIGH_BAUDRATE_ERROR);
#endif
     }     
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#if UART0_UBRRH > 0 && UART1_UBRRH > 0 
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

    if(baud.ubrr_byte[1] > 0x0F) { return(UART_BAUDRATE_TOO_LOW); }

#if UART1_UBRRH == UART0_UBRRH 
         unsigned char temp=0;
         if(uart==0)
          { 
             temp=(inpvar(uart_ubrrh)&0xF0);
             temp|=baud.ubrr_byte[1];
             outpvar(temp, uart_ubrrh);
          }
         else if(uart==1)
               { 
                  temp=(inpvar(uart_ubrrh)&0x0F);
                  temp|=(baud.ubrr_byte[1]<<4);
                  outpvar(temp, uart_ubrrh);
               }
#elif  UART1_UBRRH > UART0_UBRRH || UART1_UBRRH < UART0_UBRRH /* #if UART1_UBRRH == UART0_UBRRH */
     outpvar(baud.ubrr_byte[1], uart_ubrrh);
#endif /* #elif  UART1_UBRRH > UART0_UBRRH || UART1_UBRRH < UART0_UBRRH */

/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#elif UART0_UBRRH == 0 || UART1_UBRRH == 0 /* #if UART0_UBRRH > 0 && UART1_UBRRH > 0 */
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

    if(baud.ubrr_byte[1] > 0) { return(UART_BAUDRATE_TOO_LOW); }

/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#endif  /* #elif UART0_UBRRH == 0 */
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

    outpvar(baud.ubrr_byte[0], uart_ubrrl);
    asm("sei");    

return(0);
}
/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#endif
/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

/*####################################################################################################*/
#if UART_MULTI_COMMAND_SUPPORT == 1
unsigned char uart_command(unsigned int command)
{   
unsigned char temp=0, return_value=0;

       do
        {
           if( command & (1<<temp) )
            { 
               return_value=execute_uart_command( (1<<temp) ); 
            }
           temp++;
        }
       while( temp<16 );

return(return_value);
}
#endif
/*####################################################################################################*/
/* PUBLIC   INTERRUPT MODE DATA FUNCTIONS */
/*####################################################################################################*/

void  uart_get_buf(unsigned char *data, unsigned char size)
{
unsigned char x=0, y=0;   


        cbivar(uart_ucr, UART_RXCIE);
        for(x=0; (x<(UART_RX_BUFFER_SIZE+1) && x<size); x++)
          {
             *(data+x)= *(uart_rx_buffer[uart]+x);
             *(data+x+1)='\0';
          }

        /* ADJUST THE RX BUFFER POINTERS SO THE NEW FOUND SPACE IS ANOUNCED */
        uart_rx_count[uart]=uart_rx_count[uart]-x;
        uart_string_received[uart]=uart_string_received[uart]-x;
 
       /* MOVE THE UNREAD BUFFER CONTENTS AT THE BEGGINING OF THE RX BUFFER */
        for(; x <= UART_RX_BUFFER_SIZE; x++, y++)
          {
             *(uart_rx_buffer[uart]+y)= *(uart_rx_buffer[uart]+x);
             *(uart_rx_buffer[uart]+y+1)='\0';  
          }
        
        sbivar(uart_ucr, UART_RXCIE);
  
 

return;  
}
/*####################################################################################################*/

void uart_putc(unsigned char tx_data)
/* send buffer <tx_data> to uart */
{   
 
  while(tx_busy[uart]);
  tx_busy[uart]=1;
  outpvar(tx_data, uart_udr); 

return;  
}
/*####################################################################################################*/

void uart_puts(u08 *tx_data)
/* send  <tx_data> to uart */
{   
   if(*tx_data && tx_data && tx_on[uart]) /* check for TX_on, a NULL pointer or an EMPTY string */
    {
      while(tx_busy[uart]);
      tx_busy[uart]=1; data_location[uart]=RAM;
      tx_ptr[uart]=tx_data; outpvar(*tx_data, uart_udr);
    }
}
/*####################################################################################################*/

void uart_puts_p(const char *progmem_tx_data)
/* send progmem  <progmem_tx_data> to uart */
{   
  if(tx_on[uart]) /* If TX_on send the string else terminate */
   {
      while(tx_busy[uart]); 
      tx_busy[uart]=1; data_location[uart]=FLASH;
      tx_ptr[uart]=(u08*)progmem_tx_data;
      outpvar(PRG_RDB(tx_ptr[uart]), uart_udr);
   }
}
/*####################################################################################################*/

void uart_puts_e(unsigned char *eeprom_tx_data)
/* send progmem  <eeprom_tx_data> to uart */
{   
  
  if(tx_on[uart]) /* If TX_on send the string else terminate */
   {
      while(tx_busy[uart]); 
      tx_busy[uart]=1; data_location[uart]=EEPROM;
      tx_ptr[uart]=(u08*)eeprom_tx_data;
      outpvar(eeprom_rb((u16)tx_ptr[uart]), uart_udr);
   }
}
/*####################################################################################################*/

void uart_puti(int value, unsigned char dot_position)
{
unsigned char uart_data[6]={'0','0','0','0','0','0' }, position=sizeof(uart_data), radix=10; 

    /* convert int to ascii  */ 
    if(value<0) { uart_putc('-'); value=-value; }    
    do { position--; *(uart_data+position)=(value%radix)+'0'; value/=radix;  } while(value); 

   /* some fractional digit corrections  */
   if( dot_position>=sizeof(uart_data) ) dot_position=(sizeof(uart_data)-1);
   while( (sizeof(uart_data)-dot_position)<=position )  position--; 

   /* start displaying the number */
   for(;position<=(sizeof(uart_data)-1);position++)
     {
       if( position==sizeof(uart_data)-dot_position ) uart_putc(',');  
       uart_putc(uart_data[position]);
     }

}
/*####################################################################################################*/

unsigned char uart_getc(void)
{   
u08 buffer=rx_ISR_counter[uart];  
 
  while(buffer==rx_ISR_counter[uart]); 
 
return(rx_c[uart]);  
}

/*####################################################################################################*/
/* INTERRUPT SERVICE ROUTINES  */
/*####################################################################################################*/

UART0_TX_INTERRUPT_ISR()        /* signal handler for uart txd ready interrupt */
{

   tx_ptr[0]++;
   if(data_location[0]==EEPROM)
    {
       if(eeprom_rb((u16)tx_ptr[0])) { outpvar(eeprom_rb((u16)tx_ptr[0]),UART0_UDR ); }
       else { data_location[0]=0; tx_busy[0]=0; }
    }
   else if(data_location[0]==FLASH)
         {
            if( PRG_RDB(tx_ptr[0]) )  { outpvar(PRG_RDB(tx_ptr[0]), UART0_UDR); }
            else { data_location[0]=0; tx_busy[0]=0; }
         }
   else if(data_location[0]==RAM)
         {
            if(*tx_ptr[0]) { outpvar(*tx_ptr[0], UART0_UDR); }
            else { data_location[0]=0; tx_busy[0]=0; }
         }
   else  { tx_busy[0]=0; }
}
/*####################################################################################################*/

UART0_RX_INTERRUPT_ISR()        /* signal handler for receive complete interrupt */
{
#if UART0_ERROR_CHECK_SUPPORT == 1
  if( bit_is_set_var(UART0_USR, UART_FE) ) { uart_error[0]=FRAMING_ERROR; }
#endif
  rx_c[0]=inpvar(UART0_UDR);
#if UART0_ERROR_CHECK_SUPPORT == 1
  if( bit_is_set_var(UART0_USR, UART_OVR) ) { uart_error[0]=OVERUN_ERROR; }
#endif
  rx_ISR_counter[0]++;         
  if(echo[0]) if(tx_on[0]) if(!tx_busy[0]) outpvar(rx_c[0],UART0_UDR);
#if UART0_RAW_DATA_SUPPORT == 1
  if(rx_mode[0]==UART_GET_RAW_DATA)
   {
      if( uart_rx_count[0] >= UART_RX_BUFFER_SIZE )
       { 
          uart_error[0]=BUFFER_OVERUN;
          rx_mode[0]=0;
       }
      else{
             *(uart_rx_buffer[0]+uart_rx_count[0])=rx_c[0];
             uart_rx_count[0]++;
          }     
   }
  else
#endif
       if(rx_mode[0]==UART_GET_AT_CMD)
        {
#if UART0_AT_COMMAND_SUPPORT == 1
           if(uart_at_cmd_detected[0])
            {
#endif
               if( rx_c[0]==UART0_RX_TERMINATION_CHAR  )
                {
                    uart_string_received[0]=(uart_rx_count[0]+1);     /* valid rx data arrived */
                    rx_mode[0]=0;
                } 
               else {
                       if(uart_rx_count[0] >= UART_RX_BUFFER_SIZE)  
                        { 
#if UART0_ERROR_CHECK_SUPPORT == 1
                           uart_error[0]=BUFFER_OVERUN;
#endif
                           uart_string_received[0]=UART_RX_BUFFER_SIZE;
                           rx_mode[0]=0;
                        }
                       else { *(uart_rx_buffer[0]+uart_rx_count[0])=rx_c[0]; uart_rx_count[0]++; }
                    }
#if UART0_AT_COMMAND_SUPPORT == 1
            }
           else {  /* else wait for AT combination before copying rx data to uart_rx_buffer */
                   if(rx_c[0]=='A') { temp1[0]=1;  }
                   else if(rx_c[0]=='T' && temp1[0]==1)
                         {
                            uart_at_cmd_detected[0]=1;
                         }
                   else  { temp1[0]=0; }
                }
#endif
        }

}
/*####################################################################################################*/

UART1_TX_INTERRUPT_ISR()       /* signal handler for uart txd ready interrupt */
{
   tx_ptr[1]++;
   if(data_location[1]==EEPROM)
    {
       if(eeprom_rb((u16)tx_ptr[1])) { outpvar(eeprom_rb((u16)tx_ptr[1]), UART1_UDR); }
       else { data_location[1]=0; tx_busy[1]=0; }
    }
   else if(data_location[1]==FLASH)
         {
            if( PRG_RDB(tx_ptr[1]) )  { outpvar(PRG_RDB(tx_ptr[1]), UART1_UDR); }
            else { data_location[1]=0; tx_busy[1]=0; }
         }
   else if(data_location[1]==RAM)
         {
            if(*tx_ptr[1]) { outpvar(*tx_ptr[1], UART1_UDR); }
            else { data_location[1]=0; tx_busy[1]=0; }
         }
   else  { tx_busy[1]=0; }
}
/*####################################################################################################*/

UART1_RX_INTERRUPT_ISR()        /* signal handler for receive complete interrupt */
{
#if UART1_ERROR_CHECK_SUPPORT == 1
  if( bit_is_set_var(UART1_USR, UART_FE) ) { uart_error[1]=FRAMING_ERROR; }
#endif
  rx_c[1]=inpvar(UART1_UDR);
#if UART1_ERROR_CHECK_SUPPORT == 1
  if( bit_is_set_var(UART1_USR, UART_OVR) ) { uart_error[1]=OVERUN_ERROR; }
#endif
  rx_ISR_counter[1]++;         
  if(echo[1]) if(tx_on[1]) if(!tx_busy[1]) outpvar(rx_c[1],UART1_UDR);
#if UART1_RAW_DATA_SUPPORT == 1
  if(rx_mode[1]==UART_GET_RAW_DATA)
   {
      if( uart_rx_count[1] >= UART_RX_BUFFER_SIZE )
       { 
          uart_error[1]=BUFFER_OVERUN;
          rx_mode[1]=0;
       }
      else{
             *(uart_rx_buffer[1]+uart_rx_count[1])=rx_c[1];
             uart_rx_count[1]++;
          }     
   }
  else
#endif
       if(rx_mode[1]==UART_GET_AT_CMD)
        {
#if UART1_AT_COMMAND_SUPPORT == 1
           if(uart_at_cmd_detected[1])
            {
#endif
               if( rx_c[1]==UART1_RX_TERMINATION_CHAR  )
                {
                    uart_string_received[1]=(uart_rx_count[1]+1);     /* valid rx data arrived */
                    rx_mode[1]=0;
                } 
               else {
                       if(uart_rx_count[1] >= UART_RX_BUFFER_SIZE)  
                        { 
#if UART1_ERROR_CHECK_SUPPORT == 1
                           uart_error[1]=BUFFER_OVERUN;
#endif
                           uart_string_received[1]=UART_RX_BUFFER_SIZE;
                           rx_mode[1]=0;
                        }
                       else { *(uart_rx_buffer[1]+uart_rx_count[1])=rx_c[1]; uart_rx_count[1]++; }
                    }
#if UART1_AT_COMMAND_SUPPORT == 1
            }
           else {  /* else wait for AT combination before copying rx data to uart_rx_buffer */
                   if(rx_c[1]=='A') { temp1[1]=1;  }
                   else if(rx_c[1]=='T' && temp1[1]==1)
                         {
                            uart_at_cmd_detected[1]=1;
                         }
                   else  { temp1[1]=0; }
                }
#endif
        }

}

/*####################################################################################################*/
#elif NUMBER_OF_UARTS == 1
/*####################################################################################################*/

/* TxD variables */
volatile u08    tx_busy;
volatile u08    *tx_ptr;
         u08    tx_on;
         u08    echo;
         u08    data_location;
         u08    tx_c=0;
 
/* RxD variables */
volatile u08    uart_rx_buffer[(UART_RX_BUFFER_SIZE+1)];
volatile u08    rx_c;
volatile u08    rx_ISR_counter;
         u08    uart_rx_count;
         u08    uart_string_received;
         u08    uart_at_cmd_detected;
         u08    uart_error;
         u08    rx_mode;
         u08    temp1=0;

/*####################################################################################################*/ 
/* PRIVATE ROUTINES  */
/*####################################################################################################*/

/* private function prototypes */
#if UART_MULTI_COMMAND_SUPPORT == 1
static unsigned char execute_uart_command(unsigned int command);
#endif
static void          flush_rx_buffer(void);

/*####################################################################################################*/

static void flush_rx_buffer(void)
{
/* clear uart_rx_buffer any rx system messages */
    
    for(uart_rx_count=0;uart_rx_count<=UART_RX_BUFFER_SIZE;uart_rx_count++)
      { 
         *(uart_rx_buffer+uart_rx_count)='\0';
      }   
    uart_rx_count=0;
    uart_string_received=0;
    
}
/*####################################################################################################*/
#if UART_MULTI_COMMAND_SUPPORT == 1
static unsigned char execute_uart_command(unsigned int command)
#elif UART_MULTI_COMMAND_SUPPORT == 0
unsigned char        uart_command(unsigned int command)
#endif
{

   
switch(command)
     {
        case(UART_RX_OFF)         :  cbi(UART0_UCR, UART_RXCIE);
                                     cbi(UART0_UCR, UART_RXEN);
                                     break;                          

        case(UART_GET_STRING)     :  cbi(UART0_UCR, UART_RXCIE);
                                     inp(UART0_UDR);
                                     flush_rx_buffer(); 
                                     uart_error=0;
                                     rx_mode=UART_GET_AT_CMD;
                                     uart_at_cmd_detected=1;
                                     sbi(UART0_UCR, UART_RXCIE);
                                     sbi(UART0_UCR, UART_RXEN);
                                     break;                           
#if UART0_AT_COMMAND_SUPPORT == 1 
        case(UART_GET_AT_CMD)     :  cbi(UART0_UCR, UART_RXCIE);
                                     inp(UART0_UDR);
                                     flush_rx_buffer(); 
                                     uart_error=0;
                                     rx_mode=UART_GET_AT_CMD;
                                     uart_at_cmd_detected=0;
                                     temp1=0;
                                     sbi(UART0_UCR, UART_RXCIE);
                                     sbi(UART0_UCR, UART_RXEN);
                                     break;                                                          
#endif
#if UART0_RAW_DATA_SUPPORT == 1         
        case(UART_GET_RAW_DATA)   :  cbi(UART0_UCR, UART_RXCIE);
                                     inp(UART0_UDR);
                                     flush_rx_buffer();
                                     uart_error=0;
                                     rx_mode=UART_GET_RAW_DATA;
                                     sbi(UART0_UCR, UART_RXCIE);
                                     sbi(UART0_UCR, UART_RXEN);
                                     break;  
#endif
        case(UART_GET_RX_COUNT)   :  return(uart_rx_count);
                                     break;  

        case(UART_STRING_RECEIVED): return(uart_string_received);
                                    break;  


        case(UART_AT_CMD_RECEIVED): return(uart_string_received);
                                    break;  
                            
#if UART0_ERROR_CHECK_SUPPORT == 1          
        case(UART_GET_ERROR)      : return(uart_error);
                                    break;  
#endif
        case(UART_TX_ON)          : sbi(UART0_UCR, UART_TXCIE);
                                    sbi(UART0_UCR, UART_TXEN);
                                    tx_on=1;
                                    break;                 
           
        case(UART_TX_OFF)         : while(tx_busy);
                                    cbi(UART0_UCR, UART_TXCIE);
                                    cbi(UART0_UCR, UART_TXEN);
                                    tx_on=0;
                                    break;  

        case(UART_ECHO_ON)        : echo=1;
                                    break;                                                    
           
        case(UART_ECHO_OFF)       : echo=0;
                                    break;  

              
           /* IF NONE OF THE ABOVE */
        default :                   break;  
           
         }  
return(0); 
}
/*####################################################################################################*/
/* PUBLIC  CONTROL  FUNCTIONS */
/*####################################################################################################*/

/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#if   UART_DYNAMIC_BAUD_CHANGE == 0
/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
void uart_init(void)
/* initialize uart */
{
  
    while(tx_busy);
    asm("cli");

/* INITIALIZE THE UART AND SET BAUD RATE */
    rx_c='\0';
    flush_rx_buffer();

    /* DISABLE RxD / Txd and associated interrupts  */
    outp(0,UART0_UCR);
 
#if defined(UART0_BAUDRATE_X2)
    sbi(UART0_USR, UART_U2X);
#endif

#if UART0_UBRRH > 0 && defined(UBRRH0_VALUE) 
    outp((unsigned char)UBRRH0_VALUE, UART0_UBRRH);
#endif

    outp((unsigned char)UBRR0_VALUE, UART0_UBRRL);

    asm("sei");    

return;
}

/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#elif UART_DYNAMIC_BAUD_CHANGE == 1
/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

unsigned char uart_init(unsigned long int baud_rate, unsigned int command)
/* initialize uart */
{
unsigned char divider=0;
union word_union {
                     unsigned int  ubrr_value; 
                     unsigned char ubrr_byte[2];
                 } baud;
    
    while(tx_busy);
    asm("cli");
/*  EXECUTE THE GIVEN UART COMMANDS */
    uart_command(command);

/* INITIALIZE THE UART AND SET BAUD RATE */
    rx_c='\0';
    flush_rx_buffer();

    /* DISABLE RxD / Txd and associated interrupts  */
    outp(0,UART0_UCR);

    divider=16;
    /* set baud rate */
    baud.ubrr_value = (((F_CPU/baud_rate)/divider)-1);   /* UBRR=25 for 4 mhz and 9600 baud  */
    if( (((F_CPU/(baud_rate/100))/((baud.ubrr_value+1)*divider))-100) > 2  )
     { 
#if UART_DOUBLE_SPEED_CAPABLE == 1
        divider=8;
        sbi(UART0_USR, UART_U2X);
        baud.ubrr_value = (((F_CPU/baud_rate)/divider)-1);
        if( (((F_CPU/(baud_rate/100))/((baud.ubrr_value+1)*divider))-100) > 1  )
         { 
             return(UART_HIGH_BAUDRATE_ERROR); 
         }
#else 
        return(UART_HIGH_BAUDRATE_ERROR);
#endif
     }

/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/    
#if UART0_UBRRH > 0
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

    if(baud.ubrr_byte[1] > 0x0F) { return(UART_BAUDRATE_TOO_LOW); }
    outp(baud.ubrr_byte[1], UART0_UBRRH);

/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#elif UART0_UBRRH == 0
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

    if(baud.ubrr_byte[1] > 0) { return(UART_BAUDRATE_TOO_LOW); }

/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#endif
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
    outp(baud.ubrr_byte[0], UART0_UBRRL);

    asm("sei");    

return(0);
}

/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#endif  /* #elif UART_DYNAMIC_BAUD_CHANGE == 1 */
/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

/*####################################################################################################*/
#if UART_MULTI_COMMAND_SUPPORT == 1
unsigned char uart_command(unsigned int command)
{   
unsigned char temp=0, return_value=0;

       do
        {
           if( command & (1<<temp) ) { return_value=execute_uart_command( (1<<temp) ); }
           temp++;
        }
       while( temp<16 );

return(return_value);
}
#endif

/*####################################################################################################*/
/* PUBLIC   INTERRUPT MODE DATA FUNCTIONS */
/*####################################################################################################*/

void  uart_get_buf(unsigned char *data, unsigned char size)
{
unsigned char x=0, y=0;   

  
        cbi(UART0_UCR, UART_RXCIE);
        for(x=0; (x<(UART_RX_BUFFER_SIZE+1) && x<size); x++)
          {
             *(data+x)= *(uart_rx_buffer+x);
             *(data+x+1)='\0';
          }
        /* ADJUST THE RX BUFFER POINTERS SO THE NEW FOUND SPACE IS ANOUNCED */
        uart_rx_count=uart_rx_count-x;
        uart_string_received=uart_string_received-x; 

        /* MOVE THE UNREAD BUFFER CONTENTS AT THE BEGGINING OF THE RX BUFFER */
        for(; x <= UART_RX_BUFFER_SIZE; x++, y++)
          {
             *(uart_rx_buffer+y)= *(uart_rx_buffer+x);
             *(uart_rx_buffer+y+1)='\0';  
          }
        
        sbi(UART0_UCR, UART_RXCIE);


return;  
}
/*####################################################################################################*/

void uart_putc(unsigned char tx_data)
/* send buffer <tx_data> to uart */
{   
 
  while(tx_busy);
  tx_busy=1;
  outp(tx_data, UART0_UDR); 

return;  
}
/*####################################################################################################*/

void uart_puts(u08 *tx_data)
/* send  <tx_data> to uart */
{   
   if(*tx_data && tx_data && tx_on) /* check for TX_on, a NULL pointer or an EMPTY string */
    {
      while(tx_busy);
      tx_busy=1; data_location=RAM;
      tx_ptr=tx_data; outp(*tx_data, UART0_UDR);
    }
}
/*####################################################################################################*/

void uart_puts_p(const char *progmem_tx_data)
/* send progmem  <progmem_tx_data> to uart */
{   
  if(tx_on) /* If TX_on send the string else terminate */
   {
      while(tx_busy); 
      tx_busy=1; data_location=FLASH;
      tx_ptr=(u08*)progmem_tx_data;
      outp(PRG_RDB(tx_ptr), UART0_UDR);
   }
}
/*####################################################################################################*/

void uart_puts_e(unsigned char *eeprom_tx_data)
/* send progmem  <eeprom_tx_data> to uart */
{   
  
  if(tx_on) /* If TX_on send the string else terminate */
   {
      while(tx_busy); 
      tx_busy=1; data_location=EEPROM;
      tx_ptr=(u08*)eeprom_tx_data;
      outp(eeprom_rb((u16)tx_ptr), UART0_UDR);
   }
}
/*####################################################################################################*/

void uart_puti(int value, unsigned char dot_position)
{
unsigned char uart_data[6]={'0','0','0','0','0','0' }, position=sizeof(uart_data), radix=10; 

    /* convert int to ascii  */ 
    if(value<0) { uart_putc('-'); value=-value; }    
    do { position--; *(uart_data+position)=(value%radix)+'0'; value/=radix;  } while(value); 

   /* some fractional digit corrections  */
   if( dot_position>=sizeof(uart_data) ) dot_position=(sizeof(uart_data)-1);
   while( (sizeof(uart_data)-dot_position)<=position )  position--; 

   /* start displaying the number */
   for(;position<=(sizeof(uart_data)-1);position++)
     {
       if( position==sizeof(uart_data)-dot_position ) uart_putc(',');  
       uart_putc(uart_data[position]);
     }

}
/*####################################################################################################*/

unsigned char uart_getc(void)
{   
u08 buffer=rx_ISR_counter;  
 
  while(buffer==rx_ISR_counter); 
 
return(rx_c);  
}

/*####################################################################################################*/
/* INTERRUPT SERVICE ROUTINES  */
/*####################################################################################################*/

UART0_TX_INTERRUPT_ISR()        /* signal handler for uart txd ready interrupt */
{

   tx_ptr++;
   if(data_location==EEPROM)
    {
       if((tx_c=eeprom_rb((u16)tx_ptr)) != 0xFF) { outp(tx_c,UART0_UDR ); }
       else { data_location=0; tx_busy=0; }
    }
   else if(data_location==FLASH)
         {
            if( (tx_c=PRG_RDB(tx_ptr)) )  { outp(tx_c, UART0_UDR); }
            else { data_location=0; tx_busy=0; }
         }
   else if(data_location==RAM)
         {
            if(*tx_ptr) { outp(*tx_ptr, UART0_UDR); }
            else { data_location=0; tx_busy=0; }
         }
   else  { tx_busy=0; }
}
/*####################################################################################################*/

UART0_RX_INTERRUPT_ISR()        /* signal handler for receive complete interrupt */
{
#if UART0_ERROR_CHECK_SUPPORT == 1
  if( bit_is_set(UART0_USR, UART_FE) ) { uart_error=FRAMING_ERROR; }
#endif
  rx_c=inp(UART0_UDR);
#if UART0_ERROR_CHECK_SUPPORT == 1
  if( bit_is_set(UART0_USR, UART_OVR) ) { uart_error=OVERUN_ERROR; }
#endif
  rx_ISR_counter++;         
  if(echo) if(tx_on) if(!tx_busy) outp(rx_c,UART0_UDR);
#if UART0_RAW_DATA_SUPPORT == 1
  if(rx_mode==UART_GET_RAW_DATA)
   {
      if( uart_rx_count >= UART_RX_BUFFER_SIZE )
       { 
          uart_error=BUFFER_OVERUN;
          rx_mode=0;
       }
      else{
             *(uart_rx_buffer+uart_rx_count)=rx_c;
             uart_rx_count++;
          }     
   }
  else
#endif
       if(rx_mode==UART_GET_AT_CMD)
        {
#if UART0_AT_COMMAND_SUPPORT == 1
           if(uart_at_cmd_detected)
            {
#endif
               if( rx_c==UART0_RX_TERMINATION_CHAR  )
                {
                    uart_string_received=(uart_rx_count+1);     /* valid rx data arrived */
                    rx_mode=0;
                } 
               else {
                       if(uart_rx_count >= UART_RX_BUFFER_SIZE)  
                        { 
#if UART0_ERROR_CHECK_SUPPORT == 1
                           uart_error=BUFFER_OVERUN;
#endif
                           uart_string_received=UART_RX_BUFFER_SIZE;
                           rx_mode=0;
                        }
                       else { *(uart_rx_buffer+uart_rx_count)=rx_c; uart_rx_count++; }
                    }
#if UART0_AT_COMMAND_SUPPORT == 1
            }
           else {  /* else wait for AT combination before copying rx data to uart_rx_buffer */
                   if(rx_c=='A') { temp1=1;  }
                   else if(rx_c=='T' && temp1==1)
                         {
                            uart_at_cmd_detected=1;
                         }
                   else  { temp1=0; }
                }
#endif
        }

}
#endif
/*######################################################################################################*/
/*                                         T H E   E N D                                                */
/*######################################################################################################*/


