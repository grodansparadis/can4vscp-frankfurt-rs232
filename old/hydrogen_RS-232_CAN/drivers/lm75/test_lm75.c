
/*********************************************************************************************************
 Title  :   C  main file for the ................................. (_________.c)
 Author:    ........................
 E-mail:    ........................
 Homepage:  ........................
 Date:      18/Nov/2002
 Compiler:  AVR-GCC with AVR-AS
 MCU type:  any AVR MCU device
 Comments:  This software is FREE.
*********************************************************************************************************/

/*
    The temperature can be negative or positive and the accuracy is 1 degree Celsius
    The LM75 is sending the temperature over the I2C bus in 0,5 degrees steps 
    but there is no point to read the whole value from the temperature register
    since the LM75 sensor accuracy is 2~3 degrees Celsius.
*/



#include <io.h>
#include "lm75.h"
#include "lcd_io.h"





/********************************************************************************************************/
/*                                   MAIN FUNCTION                                                      */
/********************************************************************************************************/
void main(void)
{
signed char temperature=0;


lm75_select_unit(LM75_UNIT_0);

while(1)
    {
       lcd_gotoxy(0,0);
       lcd_puts_P("Temperature = ");
       temperature=lm75_get_temp();
       lcd_puti((signed int)temperature,0);
    }


return;
}

/*######################################################################################################*/
/*                                         T H E   E N D                                                */
/*######################################################################################################*/

