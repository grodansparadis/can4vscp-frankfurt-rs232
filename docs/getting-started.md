# Getting started

You need

-   RJ-45 patch cables or similar.
-   A stable +9V- +28V DC power source.
-   At least one additional CAN4VSCP module.
-   A host computer with a serial or USB interface.

The Frankfurt RS-232 is powered from the CAN4VSCP bus. [This
article](https://github.com/grodansparadis/vscp/wiki/connecting-can4vscp-devices-together)
have information on how the bus is powered and set up. Follow it for a reliable setup. Don\'t forget the 120 ohm terminators at each end of the bus. **They are important!** You can activate the on board terminator on the Frankfurt RS-232 if it is located at one end of your
bus.

When you power the module by inserting the Rj-45 cable the red LED on the board should light up to indicate it is alive. This is not just apower indicator LED. It used to indicate certain bus errors as well

Remember that there need to be [at least two]{.underline} CAN4VSCP (or other CAN device set to 125kbps) devices (one additional if you have the Frankfurt RS-232) on the bus to be able to communicate. A single device can\'t talk by itself on a bus. It\'s an error.

The Frankfurt RS-232 comes with a USB serial adapter. So just connect it to to your computer. The adapter is found automatically by most systems and if not you may need to use the supplied driver disc to install driver for your system.
install driver for your system.

If you want to connect the adapter to a device using RS-232 use a straight serial cable that at least have TX, RX and ground. RTS/CTS is not used at the moment but may be available in future firmware upgrades so it\'s good if the cable have them to. Most cables you buy in a store have. You will find a female DB9 connector on the device so your cable should have a DB9 male connector at one end and a suitable connector for the device you want to connect the Frankfurt RS-232 to on the other end.

You should use the [vscpl1-can4vscpdrv.dll/so](https://github.com/grodansparadis/vscpl1drv-can4vscp) with the module when connecting to it using VSCP Works or to the VSCP daemon. You can also us the [vscpl1-can232drv.dll/so](https://github.com/grodansparadis/vscpl1drv-can232) if the Frankfurt RS-232 is configured to use the SLCAN compatibility mode.

[filename](./bottom-copyright.md ':include')
