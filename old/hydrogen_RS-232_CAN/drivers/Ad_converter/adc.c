/****************************************************************************
 Title  :   C  file for the ADC FUNCTIONS library (adc.c)
 Author:    Chris efstathiou hendrix@otenet.gr
 Date:      03/Nov/2002
 Software:  AVR-GCC with AVR-AS
 Target:    any AVR device
 Comments:  This software is FREE.

*****************************************************************************/
/*
IMPORTANT:
It is my understanding that if you dont want to burn the AD Converter
never ever connect the ADC inputs to VCC. 
Also do not connect AREF or GND directly to the ADC inputs.
Use a resistor in series (say 10Kohm) otherwise you might burn the ADC as i did.
*/ 

#ifndef _IO_REG_MACRO_MODE_
#define _IO_REG_MACRO_MODE_  1     /* In case you have the new assignment mode io headers */
#endif

#ifndef  _SFR_ASM_COMPAT
#define  _SFR_ASM_COMPAT     1     /* This is for GCC 3.2 */
#endif

#include  <io.h>
#include <sig-avr.h>
#include "adc.h"


#if ADC_INT_MODE_NEEDED == 0 && ADC_SLEEP_MODE_NEEDED == 0 && ADC_SCAN_MODE_NEEDED == 0 \
    && ADC_NO_INT_MODE_NEEDED == 0
#error "ADC DRIVER DEACTIVATED "
#endif  


/* DEVICE SPECIFIC SETTINGS */
#if defined(__AVR_ATmega16__) || defined(__AVR_ATmega32__) || defined(__AVR_ATmega64__) || \
      defined(__AVR_ATmega8535__) || defined(__AVR_ATmega8515__) || defined(__AVR_ATmega163__) || \
      defined(__AVR_ATmega323__)

#define  ADC_CONTROL_REG            ADCSR
#define  ADC_MUX_REG                ADMUX
#define  NUMBER_OF_ADC_CHANNELS     8
#define  ADC_PORT                   A
#define  ADC_PORT_IS_BIDIRECTIONAL  1
#define  ADC_SPECIFIC_SLEEP_MODE    1

#elif defined(__AVR_ATmega128__)

#define  ADC_CONTROL_REG            ADCSR
#define  ADC_MUX_REG                ADMUX
#define  NUMBER_OF_ADC_CHANNELS     8
#define  ADC_PORT                   F
#define  ADC_PORT_IS_BIDIRECTIONAL  1
#define  ADC_SPECIFIC_SLEEP_MODE    1

#elif defined(__AVR_ATmega103__)

#define  ADC_CONTROL_REG            ADCSR
#define  ADC_MUX_REG                ADMUX
#define  NUMBER_OF_ADC_CHANNELS     8
#define  ADC_PORT                   F
#define  ADC_PORT_IS_BIDIRECTIONAL  0
#define  ADC_SPECIFIC_SLEEP_MODE    0

#elif defined(__AVR_ATmega8__) 

#define  ADC_CONTROL_REG            ADCSR
#define  ADC_MUX_REG                ADMUX
#define  NUMBER_OF_ADC_CHANNELS     8
#define  ADC_PORT                   C
#define  ADC_PORT_IS_BIDIRECTIONAL  1
#define  ADC_SPECIFIC_SLEEP_MODE    1

#elif defined(__AVR_AT90S8535__) || defined(__AVR_ATmega161__) 

#define  ADC_CONTROL_REG            ADCSR
#define  ADC_MUX_REG                ADMUX
#define  NUMBER_OF_ADC_CHANNELS     8
#define  ADC_PORT                   A
#define  ADC_PORT_IS_BIDIRECTIONAL  1
#define  ADC_SPECIFIC_SLEEP_MODE    0

#elif defined(__AVR_AT90S4433__) || defined(__AVR_AT90S2333__)

#define  ADC_CONTROL_REG            ADCSR
#define  ADC_MUX_REG                ADMUX
#define  NUMBER_OF_ADC_CHANNELS     6
#define  ADC_PORT                   C
#define  ADC_PORT_IS_BIDIRECTIONAL  1
#define  ADC_SPECIFIC_SLEEP_MODE    0


#else
#error SORRY YOU DONT HAVE AN AD CONVERTER!

#endif

#if ADC_DYNAMIC_CLOCK == 0

#define ADC_CLOCK_REAL (ADC_CLOCK_KHZ*1000L)

#if   F_CPU/2 < ((ADC_CLOCK_REAL*13)/10)
#define ADC_PRESCALER   1
#elif F_CPU/4 < ((ADC_CLOCK_REAL*13)/10)
#define ADC_PRESCALER   2 
#elif F_CPU/8 < ((ADC_CLOCK_REAL*13)/10)
#define ADC_PRESCALER   3
#elif F_CPU/16 < ((ADC_CLOCK_REAL*13)/10)
#define ADC_PRESCALER   4 
#elif F_CPU/32 < ((ADC_CLOCK_REAL*13)/10)
#define ADC_PRESCALER   5 
#elif F_CPU/64 < ((ADC_CLOCK_REAL*13)/10)
#define ADC_PRESCALER   6 
#else
#define ADC_PRESCALER   7
 
#endif

#endif  /* #if ADC_DYNAMIC_CLOCK == 0 */


/* REGISTER NAME FORMING */
#ifndef CONCAT1
#define CONCAT1(a, b) CONCAT2(a, b)
#endif

#ifndef CONCAT2
#define CONCAT2(a, b) a ## b
#endif

#define ADC_PORT_OUT_REG       CONCAT1(PORT, ADC_PORT)
#define ADC_PORT_DDR_REG       CONCAT1(DDR, ADC_PORT)
#define ADC_PORT_PIN_REG       CONCAT1(PIN, ADC_PORT)


/* Normally you shouldn't need to change the below  lines  */
#ifndef  GLOBAL_INT_REG
#define  GLOBAL_INT_REG        SREG       /* the global interrupt enable register name    */
#endif
#ifndef  GLOBAL_INT_BIT
#define  GLOBAL_INT_BIT        7          /* the global interrupt enable bit position     */
#endif


/*######################################################################################################*/
/*                                Global variables                                                      */
/*######################################################################################################*/
volatile union adc_union {
                           volatile unsigned int  adc_value; 
                           volatile unsigned char adc_byte[2];
                         } adc;

#if    ADC_INT_MODE_NEEDED == 1 || ADC_SCAN_MODE_NEEDED == 1 || ADC_SLEEP_MODE_NEEDED == 1 
volatile unsigned char adc_auto_repeat=0;
volatile unsigned char adc_data_valid=0;
volatile unsigned char adc_init_flag=0;
#endif

volatile unsigned int  adc_value[NUMBER_OF_ADC_CHANNELS];
volatile unsigned char adc_channel=0;
unsigned char          adc_mode=0;
unsigned char          adc_scan_mask=0;

#if ADC_DYNAMIC_CLOCK == 1 
unsigned char adc_prescaler=0;
#endif

/*Local function prototypes */
static void write_admux(void);




/*######################################################################################################*/
/*                                local function definitions                                            */
/*######################################################################################################*/
static void write_admux(void)
{
register unsigned char temp1=0;

    for(temp1=0; temp1<=2; temp1++)
      {
         if(adc_channel&(1<<temp1)) { sbi(ADC_MUX_REG, temp1); }
         else { cbi(ADC_MUX_REG, temp1); }
      }       

return;
}
/*######################################################################################################*/
/*                                Public function definitions                                           */
/*######################################################################################################*/
void  adc_init(unsigned char mode, unsigned int adc_clock, unsigned char channel)
{
#if ADC_DYNAMIC_CLOCK == 1   
unsigned int  f_cpu=0, adc_div=0;
unsigned char temp_adc_div=0, x=0;
#endif

/*-------------------------------------------------------------------------------------------------------*/   
#if    ADC_INT_MODE_NEEDED == 1  || ADC_SCAN_MODE_NEEDED == 1
   while(adc_init_flag);
#endif
#if   ADC_DYNAMIC_CLOCK == 1
   /* set the ADC prescaler bits  */
   f_cpu=F_CPU/1000;
   temp_adc_div=f_cpu/adc_clock;
   adc_div=256;
   adc_prescaler=8;
   do
    { 
      adc_div=adc_div/2; 
      x=temp_adc_div/adc_div;
      adc_prescaler--; 
    } 
    while(!x); 
    /* Allow 30% maximum over the desired adc_clock else go to a higher adc_clock prescaler value */
    if( (f_cpu/adc_div)>=((adc_clock*13)/10) ) { adc_prescaler++; }
#endif

/*-------------------------------------------------------------------------------------------------------*/
   /* Make sure that all ADC work is ceased */
   adc_mode=mode;
   outp(0, ADC_CONTROL_REG);
   sbi(ADC_CONTROL_REG, ADIF);
#if   ADC_NOISE_REDUCTION == 1 && ADC_PORT_IS_BIDIRECTIONAL == 1
   outp(0xFF, ADC_PORT_DDR_REG);
   outp(0, ADC_PORT_OUT_REG);
#endif 
/*-------------------------------------------------------------------------------------------------------*/
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#if  ADC_NO_INT_MODE_NEEDED == 1
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

  if(mode==ADC_INT_DIS) { adc_convert(channel); }

/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#endif
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#if   ADC_INT_MODE_NEEDED == 1
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

#if    ADC_NO_INT_MODE_NEEDED == 1    
  else if(adc_mode==ADC_INT_EN) 
#elif  ADC_NO_INT_MODE_NEEDED == 0 
       if(adc_mode==ADC_INT_EN) 
#endif
        {
          /* Set varius switches and values */
          adc_data_valid=0;
          adc_auto_repeat=1;
          adc_channel=channel;

          /* Set the appropriate ADMUX bits */
          write_admux();

#if    ADC_NOISE_REDUCTION == 1 && ADC_PORT_IS_BIDIRECTIONAL == 1
          outp((~(1<<adc_channel)), ADC_PORT_DDR_REG);
          outp(0, ADC_PORT_OUT_REG);
#elif  ADC_NOISE_REDUCTION == 0 && ADC_PORT_IS_BIDIRECTIONAL == 1
          cbi(ADC_PORT_DDR_REG, adc_channel);   
          cbi(ADC_PORT_OUT_REG, adc_channel); 
#endif
          asm("sei");
       
         /* Start the first conversion  ADC ON, INTERRUPT ON    */
#if   ADC_DYNAMIC_CLOCK == 1
         outp((1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(adc_prescaler), ADC_CONTROL_REG);
#elif ADC_DYNAMIC_CLOCK == 0
         outp((1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(ADC_PRESCALER), ADC_CONTROL_REG);
#endif
         adc_init_flag=1;
       }
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#endif /* ADC_INT_MODE_NEEDED == 1   */
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#if  ADC_SCAN_MODE_NEEDED == 1 
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

#if    ADC_INT_MODE_NEEDED == 1 || ADC_NO_INT_MODE_NEEDED == 1   
  else if(adc_mode==ADC_INT_EN_SCAN)
#elif  ADC_INT_MODE_NEEDED == 0 && ADC_NO_INT_MODE_NEEDED == 0 
       if(adc_mode==ADC_INT_EN_SCAN)
#endif
        {
          adc_data_valid=0;
          adc_auto_repeat=1; 
          adc_scan_mask=channel;

           /* FIND THE FIRST CHANNELL TO SCAN. IF NONE REVERT TO ADC_INT_EN MODE */
           adc_channel=0;
           while( ((1<<adc_channel) & adc_scan_mask) == 0 ) 
               {
                  adc_channel++;
                  if(adc_channel>=NUMBER_OF_ADC_CHANNELS)
                   {
                      adc_channel=0;
                      adc_mode=ADC_INT_DIS;
                      return;
                   }
               }

           /* Set the appropriate ADMUX bits */
           write_admux();
 
#if    ADC_NOISE_REDUCTION == 1 && ADC_PORT_IS_BIDIRECTIONAL == 1
           outp((~(1<<adc_channel)), ADC_PORT_DDR_REG);
           outp(0, ADC_PORT_OUT_REG); 
#elif  ADC_NOISE_REDUCTION == 0 && ADC_PORT_IS_BIDIRECTIONAL == 1
           cbi(ADC_PORT_DDR_REG, adc_channel); 
           cbi(ADC_PORT_OUT_REG, adc_channel); 
#endif
           asm("sei");
           /* Start the first conversion  ADC ON, INTERRUPT ON    */
#if   ADC_DYNAMIC_CLOCK == 1
           outp((1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(adc_prescaler), ADC_CONTROL_REG);
#elif ADC_DYNAMIC_CLOCK == 0
           outp((1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(ADC_PRESCALER), ADC_CONTROL_REG);
#endif
           adc_init_flag=1;
         }
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#endif /*  #if  ADC_SCAN_MODE_NEEDED == 1  */
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#if  ADC_SLEEP_MODE_NEEDED == 1 
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

#if   ADC_SCAN_MODE_NEEDED == 1 || ADC_INT_MODE_NEEDED == 1 || ADC_NO_INT_MODE_NEEDED == 1   
  else if(adc_mode==ADC_INT_EN_SLEEP)
#elif ADC_SCAN_MODE_NEEDED == 0 && ADC_INT_MODE_NEEDED == 0 && ADC_NO_INT_MODE_NEEDED == 0 
       if(adc_mode==ADC_INT_EN_SLEEP)
#endif
        {
           adc_auto_repeat=0;
           asm("sei"); 
           adc_convert(channel);
        }

/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#endif /* #if  ADC_SLEEP_MODE_NEEDED == 1 */
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

return;
}
/*######################################################################################################*/

/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#if  ADC_SLEEP_MODE_NEEDED == 1 || ADC_NO_INT_MODE_NEEDED == 1
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

/* channel =  0 to 7 (ADC CHANNEL)  ADC uses NO interrupt   */
unsigned int adc_convert(unsigned char channel)
{
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#if ADC_NO_INT_MODE_NEEDED == 1         
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
         if(adc_mode==ADC_INT_DIS)
          {  
             /* Set the appropriate ADMUX bits */    
             adc_channel=channel;
             write_admux();

#if    ADC_NOISE_REDUCTION == 1 && ADC_PORT_IS_BIDIRECTIONAL == 1
             outp((~(1<<adc_channel)), ADC_PORT_DDR_REG); 
             outp(0, ADC_PORT_OUT_REG);
#elif  ADC_NOISE_REDUCTION == 0 && ADC_PORT_IS_BIDIRECTIONAL == 1
             cbi(ADC_PORT_DDR_REG, adc_channel);
             cbi(ADC_PORT_OUT_REG, adc_channel);   
#endif
             sbi(ADC_CONTROL_REG, ADIF);
#if   ADC_DYNAMIC_CLOCK == 1
             outp((1<<ADEN)|(1<<ADSC)|(adc_prescaler), ADC_CONTROL_REG);
#elif ADC_DYNAMIC_CLOCK == 0
             outp((1<<ADEN)|(1<<ADSC)|(ADC_PRESCALER), ADC_CONTROL_REG);
#endif
             loop_until_bit_is_set(ADC_CONTROL_REG, ADIF);
             adc.adc_byte[0]=inp(ADCL);
             adc.adc_byte[1]=inp(ADCH);
             adc_value[adc_channel]=adc.adc_value;

             return(adc.adc_value);
          }
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#endif
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#if  ADC_SLEEP_MODE_NEEDED == 1 
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

#if   ADC_SLEEP_MODE_NEEDED == 1 && ADC_NO_INT_MODE_NEEDED == 1
        else if(adc_mode==ADC_INT_EN_SLEEP)
#elif ADC_SLEEP_MODE_NEEDED == 1 && ADC_NO_INT_MODE_NEEDED == 0
             if(adc_mode==ADC_INT_EN_SLEEP)
#endif
              {
                 /* Set the appropriate ADMUX bits */
                 adc_channel=channel;
                 write_admux();

#if    ADC_NOISE_REDUCTION == 1 && ADC_PORT_IS_BIDIRECTIONAL == 1
                 outp((~(1<<adc_channel)), ADC_PORT_DDR_REG);
                 outp(0, ADC_PORT_OUT_REG);
#elif  ADC_NOISE_REDUCTION == 0 && ADC_PORT_IS_BIDIRECTIONAL == 1
                 cbi(ADC_PORT_DDR_REG, adc_channel);
                 cbi(ADC_PORT_OUT_REG, adc_channel);  
#endif

#if defined(SE)
#if ADC_SPECIFIC_SLEEP_MODE == 1
                sbi(MCUCR, SE);
                sbi(MCUCR, SM0);
#elif ADC_SPECIFIC_SLEEP_MODE == 0
                sbi(MCUCR, SE);
#endif
#endif
                 /* ADC ON, INTERRUPT ON    */
#if   ADC_DYNAMIC_CLOCK == 1
                outp((1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(adc_prescaler), ADC_CONTROL_REG);
#elif ADC_DYNAMIC_CLOCK == 0
                outp((1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(ADC_PRESCALER), ADC_CONTROL_REG);
#endif
                /* Go to sleep. The ADC interrupt ISR will wake up the AVR */
                asm("sleep");
                asm("nop");
                asm("nop");
                return(adc.adc_value);
              } 
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/	
#endif /* #if ADC_NO_INT_MODE_NEEDED == 1 */
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

return(0);                        
}
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#endif
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
/*######################################################################################################*/

unsigned int adc_get_channel(unsigned char channel)
{
#if  ADC_NO_INT_MODE_NEEDED == 1 
   if(adc_mode==ADC_INT_DIS) { return(adc_value[channel]); }
#endif
#if    ADC_INT_MODE_NEEDED == 1 || ADC_SCAN_MODE_NEEDED == 1
   while(!adc_data_valid);  
#endif

return(adc_value[channel]);  
}
/*######################################################################################################*/
#if  ADC_INT_MODE_NEEDED == 1 
void adc_set_channel(unsigned char channel)
{
  if(adc_mode==ADC_INT_EN)
   { 
      adc_channel=channel;
      adc_data_valid=0;
      adc_init_flag=1;
   }

return;
}
#endif

/*######################################################################################################*/
/*                                  INTERRUPT SERVICE ROUTINE                                           */
/*######################################################################################################*/

/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#if  ADC_INT_MODE_NEEDED == 1 || ADC_SLEEP_MODE_NEEDED == 1 || ADC_SCAN_MODE_NEEDED
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

/* INTERRUPT SERVICE ROUTINE */ 
SIGNAL(SIG_ADC)
{
#if defined(SE)
#if ADC_SPECIFIC_SLEEP_MODE == 1
   cbi(MCUCR, SE);
   cbi(MCUCR, SM0);
#elif ADC_SPECIFIC_SLEEP_MODE == 0
   cbi(MCUCR, SE);
#endif
#endif 
    adc.adc_byte[0]=inp(ADCL);
    adc.adc_byte[1]=inp(ADCH);
    adc_value[(inp(ADC_MUX_REG)&0x07)]=adc.adc_value; 

/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#if   ADC_SCAN_MODE_NEEDED == 1         
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

    if(adc_mode == ADC_INT_EN_SCAN)
     {
       do
        {
          adc_channel++;
          if(adc_channel>=NUMBER_OF_ADC_CHANNELS) { adc_channel=0; adc_data_valid=1; adc_init_flag=0;}
        }   
       while( ((1<<adc_channel) & adc_scan_mask) == 0 );  
       /* Set the appropriate ADMUX bits */
       write_admux();

#if    ADC_NOISE_REDUCTION == 1 && ADC_PORT_IS_BIDIRECTIONAL == 1
       outp((~(1<<adc_channel)), ADC_PORT_DDR_REG);  
       outp(0, ADC_PORT_OUT_REG); 
#elif  ADC_NOISE_REDUCTION == 0 && ADC_PORT_IS_BIDIRECTIONAL == 1
       cbi(ADC_PORT_DDR_REG, adc_channel);
       cbi(ADC_PORT_OUT_REG, adc_channel);  
#endif
     }

/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#endif /* #if   ADC_SCAN_MODE_NEEDED == 1 */
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

#if   ADC_INT_MODE_NEEDED == 1  && ADC_SCAN_MODE_NEEDED == 1
    else 
       {  
#endif          

/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#if   ADC_INT_MODE_NEEDED == 1 
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

          if(adc_channel != (inp(ADC_MUX_REG)&0x07))
           {
              /* Set the appropriate ADMUX bits */
              write_admux();
#if    ADC_NOISE_REDUCTION == 1 && ADC_PORT_IS_BIDIRECTIONAL == 1
              outp((~(1<<adc_channel)), ADC_PORT_DDR_REG);
              outp(0, ADC_PORT_OUT_REG);
#elif  ADC_NOISE_REDUCTION == 0 && ADC_PORT_IS_BIDIRECTIONAL == 1
              cbi(ADC_PORT_DDR_REG, adc_channel);
              cbi(ADC_PORT_OUT_REG, adc_channel);   
#endif 
           }
          else { adc_data_valid=1; adc_init_flag=0; }
#if   ADC_INT_MODE_NEEDED == 1 && ADC_SCAN_MODE_NEEDED == 1
       }
#endif    

/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#endif /* #if   ADC_INT_MODE_NEEDED == 1 */
/*222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

    if(adc_auto_repeat) { sbi(ADC_CONTROL_REG, ADSC); }

}
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
#endif
/*111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/


/*######################################################################################################*/
/*                                         T H E   E N D                                                */
/*######################################################################################################*/


