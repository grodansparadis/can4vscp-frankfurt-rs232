# Updating the firmware

New firmware is released from time to time for all our modules. We expect this to be true also for the Frankfurt RS-232 as it is an active project. The process of updating the firmware of the module is a simple one. Just follow the steps outlined here.

## The bootloader application

First you should download and install [this file](http://www.grodansparadis.com/downloads/Serial_Bootloader_AN1310_v1.05r.exe).  You can uninstall the file as soon as you are done uploading the firmware of the Frankfurt RS-232 module.

![](media/bootloader0.png ':size=300')

After install you can find the executable under the Microchip folder in the startup menu.

## Where is the firmware?

As with all our modules you can find the firmware for the Frankfurt RS-232 on [the product page](http://www.grodansparadis.com/frankfurt/rs232/frankfurt-rs232.html) under the firmware tab. If in doubt which version to download use the latest.

## Where is the source for the firmware?

You can find the latest source for the firmware [here](https://github.com/grodansparadis/can4vscp_frankfurt_rs232).

## Setting communication parameters

You should  set communication parameters to use before you start. You find them under the menu /program/start of the bootloader program. This will bring up this dialog

![](media/bootloader5.png ':size=600')

This is the recommended settings but you may need to lower the bootloader baudrated  if you have problem finding the module or problems with the loading of the code. For most uses 115200 should work fine. The Frankfurt RS-232 bootloader have an autobaudrate detector so any baudrate will do. Note however that the application baudrate is locked to 115200 baud and should not be changed.

## Setting the module into bootloader mode

You set the module into bootloader mode by restarting (turning of and turning on) the module while the <em>break/Reset application firmware</em> button of the bootloader application is pressed.  You can also do the same thing with the <em>BOOT command</em> in the verbose mode of the module.

![](media/bootloader2.png ':size=600')

while the break button is depressed then restart the board.

When the module is restarted you should press the <em>enter boot loader mode</em> to activate the bootloader connection.

![](media/bootloader4.png ':size=600')

If you have set the wrong port, a baudrate tat does not work or the module did not enter the bootloader firmware it wil show here. Change settings and try again.

If all goes well you will get a new screen

![](media/bootloader6.png ':size=600')

It just show an empty flash memory.

At this point you need the firmware code so load it by selecting the hex file under the /file/open menu.

![](media/bootloader7.png ':size=600')

After the hex file is loaded you should see code at offset 0x400 and forward.

![](media/bootloader8.png ':size=600')

To program the module click <em>Write Device</em>

![](media/bootloader9.png ':size=600')

and the firmware will be written to the device.

You may get a complaint like this

![](media/verify_error.png ':size=300')

after the code is loaded but this is no problem it s due to code protection.  Run the verify under the /program/verify menu to make sure the write of the firmware was OK.

Thats it!

![](media/bootloader11.png ':size=600')

Now select <em>run application firmware</em> and restart the board again and the Frankfurt RS-232 application will run.

![](media/bootloader12.png ':size=600')

<br>

---

![](media/grodan_logo.png ':size=100')
<br>

<center>

**Paradise of the Frog AB** Brattbergavägen 17 820 50 LOS SWEDEN<br>
**email:** [info@grodansparadis.com](mailto:info@grodansparadis.com) **phone:** +46 (0)8 40011835 <br>
**web:**[http://www.grodansparadis.com](http://www.grodansparadis.com) <br>

</center>
