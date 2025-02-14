/************************************************************************************
 Title  :   C Test program for the SOFTWARE UART MINIMUM FUNCTIONS (testuart.c)
 Author:    Chris efstathiou hendrix@otenet.gr
 Date:      03/Oct/2002
 Software:  AVR-GCC with AVR-AS
 Target:    any AVR device
 Comments:  This software is FREE.

************************************************************************************/
   
#include <io.h>
#include "suart_min.h"


void main(void)
{
unsigned char string[]="Type a few chars + ENTER";
unsigned char string1[]="You typed: ";
unsigned char string2[]="Press ENTER to repeat...";
unsigned char string3[]="TIMEOUT";
unsigned char buffer[11];
unsigned char x;
unsigned char c=0;
asm("cli");        

while(1)
    {
      suart_putc('\n'); suart_putc('\r');      
      x=0; while((c=string[x++])) suart_putc(c);
      suart_putc('\n'); suart_putc('\r');

      for(x=0; x < 10; x++)
        {
           buffer[x]=(c=suart_getc());
           if(c == '\r' || suart_timeout_flag) { break; }
        }
      buffer[x]='\0';

      if(suart_timeout_flag) {x=0; while((c=string3[x++]))  { suart_putc(c); }  }

      suart_putc('\n'); suart_putc('\r');
      x=0; while((c=string1[x++])) { suart_putc(c); }
      x=0; while((c=buffer[x++]))  { suart_putc(c); }
      suart_putc('\n'); suart_putc('\r');
      x=0; while((c=string2[x++])) { suart_putc(c); }

      while((c=suart_getc())) { if(c=='\r' || suart_timeout_flag) break; }

      if(suart_timeout_flag) {x=0; while((c=string3[x++]))  { suart_putc(c); }  }

      suart_putc('\n'); suart_putc('\r');

    }
        
}
/*######################################################################################################*/
/*                                         T H E   E N D                                                */
/*######################################################################################################*/

