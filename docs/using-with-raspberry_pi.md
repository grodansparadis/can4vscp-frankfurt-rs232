# Using with Raspberry Pi

## Making sure it works

The USB serial adapter that is shipped with Frankfurt RS-232 s
identified by Raspberry Pi as ch341. **lsusb** (*sudo apt-get install
usbutils*) will show something like

     Bus 001 Device 004: ID 1a86:7523 QinHeng Electronics HL-340 USB-Serial adapter

If you issue **lsmod** you should have a line like

     usbserial              31088  1 ch341

then normally you have the serial port as **/dev/ttyUSB0**

and can open it using **minicom** (*sudo apt-get install minicom*) with

     sudo minicom --baudrate 115200 --device /dev/ttySUB0

You may want to change permissions for who can access the serial driver
with

     sudo chmod a+rw /dev/ttyUSB0

which let all users use the serial channel.

## Using with VSCP software

If using the non TTL version just see instructions [Using with the VSCP
Daemon](using_with_vscp_daemon) and [Using with VSCP
Works](using_with_vscp_works).

[filename](./bottom-copyright.md ':include')
