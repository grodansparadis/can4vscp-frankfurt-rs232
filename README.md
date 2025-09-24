# Frankfurt RS-232

<img src="http://www.grodansparadis.com/images/frankfurt_rs232_4_small.png">

## Abstract

Frankfurt RS-232 interfaces serial RS-232 to the CAN4VSCP bus and is powered from the CAN4VSCP bus. It can be used with a USB to serial adapter free of charge so it can also be connected to host computers that does not have a RS-232 port. The Frankfurt RS-232 is designed to be mounted on a DIN rail or screwed on to a wall or used in an other installed fashion. This module is as our other modules designed to do its work behind the scenes. It is a powerful and reliable workhorse that does not need to be in the centre of the spotlights to do the work it is designed for.

It is important to note that Frankfurt RS-232 is not a general CAN interface. The CAN speed of the device is fixed to 125 kbps and the serial speed is fixed at 115200 baud 8N1. This is because the Frankfurt RS-232 is made for the CAN4VSCP bus and VSCP and therefore does not need the variable speeds. If you need a general CAN adapter please look at [Rusoku Technologies TouCan](https://www.rusoku.com/products) or other CAN <-> USB interfaces.

Frankfurt RS-232 have three working modes:

## Verbose mode
In this mode easy to remember and use text based commands can be issued to control the device. It is possible to find VSCP nodes on the connected CAN4VSCP bus, read and write registers and many other of the common functions needed to diagnose a CAN4VSCP system. The mode is created for users that need to check up on a CAN4VSCP network and get diagnostic information etc. It is possible to go to this mode from any of the other modes at any time and therefore get the most out of each modes available functionality.

## CAN4VSCP mode
This mode uses the efficient and secure VSCP serial protocol to talk to a host computer. Drivers for Windows and Linux is readably available and it is a turn key solution to connect any CAN4VSCP bus to a host. The driver follows the CANAL standard so both the VSCP Daemon and the VSCP Works programs can be used.

## SLCAN mode
The Swedish company Lawicel AB and Lars Wictorsson created a serial protocol for CAN adapters that since been a standard for such devices. The SLCAN mode of Frankfurt RS-232 support a limited set of commands from this standard so that the module can be connected to the Socketcan system available for Linux. Se the manual for the commands that are implemented. Also in this case the CAN speed is fixed at 125 kbps and the serial speed is fixed at 115200 kbps.


It is possible to have hundreds of Frankfurt RS-232 modules connected to a host computer where each in turn is connected to hundreds of nodes, thereby building very large and complex systems. Together with cards like Raspberry Pi, Beaglebone and others it is possible to have distributed control systems controlled over Ethernet and TCP/IP. 

<hr>

## Project files

### User manual
  * [User Manual](https://grodansparadis.github.io/can4vscp_frankfurt_rs232/#)

### Schematic, PCB, 3D files etc
  * [Schematics reversion B](./eagle/paris_revb_sch.png)
 * Hardware design files is made in [KiCad](https://kicad.org) and can be found in the `kicad` directory. Valid from reversion B of the hardware.
   * Gerber files for PCB production can be found in the `gerber` directory (in the `kicad` folder).
 * Eagle schema and board files for reversion A and B can be found in the `eagle` directory. Only the KiCad version is actively updated.

 ### Firmware

 The firmware is developed in [MPLAB X IDE](https://www.microchip.com/mplab/mplab-x-ide) using the [XC8 compiler](https://www.microchip.com/mplab/compilers).

  * Binary release files is available [here](https://github.com/grodansparadis/can4vscp_paris/releases)

### MDF - Module Description File(s)
  * [MDF file version: 1.1.3 Release date: 2022-03-25](http://www.eurosource.se/paris_010.xml)
  * [MDF file version: 0.0.2 Release date: 2009-10-07](http://www.eurosource.se/paris_001.xml)

### Support
If you need support, please open an issue in the [GitHub repository](https://github.com/grodansparadis/can4vscp_kelvin_ntc10k/issues).

### Buy a ready made modules
You can buy a ready made module from [Grodans Paradis](http://www.grodansparadis.com).

### Project related links
  * [VSCP project](https://www.vscp.org)
  * [VSCP Documentation site](https://docs.vscp.org/)
  * [VSCP Wiki](https://github.com/grodansparadis/vscp/wiki)


<hr>

This project is part of the <a href="http://www.vscp.org">VSCP (Very Simple Control Protocol) project</a>. 

<hr>

This project is licensed under the 
<a href="http://creativecommons.org/licenses/by-nc-sa/3.0/">Creative Commons open source license Attribution-NonCommercial-ShareAlike 3.0 Unported</a>. 
Other license options are available by conacting <a href="malto:info@grodansparadis.com">Paradis of the Frog AB</a>

