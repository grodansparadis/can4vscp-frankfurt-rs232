
New firmware is released from time to time for all our modules. We
expect this to be true also for the module as it is an active
project. The process of updating the firmware of the module is a simple
one. Just follow the steps outlined in the [VSCP Works
documentation](https://grodansparadis.github.io/vscp-works-qt/#/bootload_window).



## Where is the firmware?

Firmware is available in the Github repository
<https://github.com/grodansparadis/can4vscp_paris/releases> and the [MDF of the module](https://github.com/grodansparadis/can4vscp_paris/tree/master/mdf) contains a list of available firmware files and a pointer to there location. This file is loaded by the firmware updating software of VSCP Works+ and you can select the file you need from there.

### Device firmware code
The standard registers of a module contains a register that holds the firmware code for the module. This is a 16-bit value that can be read by the VSCP firmware update software. The value is a code for the specific hardware the board uses. Typically hardware has been changed and processors have been updated to later versions as time goes. The VSCP firmware update software can use this code to find the correct firmware for the module. It is **VERY** important that the correct firmware is used for the module. If the wrong firmware is used the module will not work as expected and may even be damaged. So make sure this code is the same as the code in the firmware file you use.

Older firmware files may not have this code in the standard registers as it is a pretty late addition to the standard. In that case you should use the latest firmware file for the module for the type of processor you have.

## Where is the source for the firmware?

You can find the latest source for the firmware
[here](https://github.com/grodansparadis/can4vscp_paris).

## Replacing/updating the bootloader on the module

If you need to update the bootloader on this module you can find firmware files for it [here](https://github.com/grodansparadis/vscp-pic1-bootloader/releases). Select the hex files for your board and use the bootloader application to update the bootloader. When the bootloader is loaded you can use [VSCP Works+](https://grodansparadis.github.io/vscp-works-qt/#/bootload_window) to update the firmware of the module.

### Getting the software

First you should download and install [this
file](http://www.grodansparadis.com/downloads/Serial_Bootloader_AN1310_v1.05r.exe).
You can uninstall the file as soon as you are done uploading the
firmware of the module.

![](./images/bootloader0.png)

After install you can find the executable under the Microchip folder in
the startup menu.

### Setting communication parameters

You should set communication parameters to use before you start. You
find them under the menu /program/start of the bootloader program. This
will bring up this dialog

![](./images/bootloader5.png)

This is the recommended settings but you may need to lower the
bootloader baudrate if you have problem finding the module or problems
with the loading of the code. For most uses 115200 should work fine. The
Frankfurt RS-232 bootloader have an autobaudrate detector so any
baudrate will do. Note however that the application baudrate is locked
to 115200 baud and should not be changed.

### Setting the module into bootloader mode

You set the module into bootloader mode by restarting (turning of and
turning on) the module while the *break/Reset application firmware*
button of the bootloader application is pressed. You can also do the
same thing with the *BOOT command* in the verbose mode of the module.

![](./images/bootloader2.png)

while the break button is depressed then restart the board.

When the module is restarted you should press the *enter boot loader
mode* to activate the bootloader connection.

![](./images/bootloader4.png)

If you have set the wrong port, a baudrate tat does not work or the
module did not enter the bootloader firmware it wil show here. Change
settings and try again.

If all goes well you will get a new screen

![](./images/bootloader6.png)

It just show an empty flash memory.

At this point you need the firmware code so load it by selecting the hex
file under the /file/open menu.

![](./images/bootloader7.png)

After the hex file is loaded you should see code at offset 0x400 and
forward.

![](./images/bootloader8.png)

To program the module click *Write Device*

![](./images/bootloader9.png)

and the firmware will be written to the device.

You may get a complaint like this

![](./images/verify_error.png)

after the code is loaded but this is no problem it s due to code
protection. Run the verify under the /program/verify menu to make sure
the write of the firmware was OK.

Thats it\!

![](./images/bootloader11.png)

Now select *run application firmware* and restart the board again and
the firmware will run.

![](./images/bootloader12.png)

  
[filename](./bottom-copyright.md ':include')
