# The CAN4VSCP mode

This mode export a [standard VSCP serial
interface](https://grodansparadis.github.io/vscp-doc-spec/#/./vscp_over_a_serial_channel_rs-232). The CAN4VSCP driver can be used to export this interface to a [CANAL interface](https://grodansparadis.github.io/vscp-doc-canal/#/) which in turn can be used by the [VSCP daemon](https://grodansparadis.github.io/vscp/#/), [VSCP Works](https://grodansparadis.github.io/vscp-works-qt/#/) and other software that can work with a CANAL interface.

An end user does not have to worry about any of this of course. He/she just use the [CAN4VSCP driver](https://github.com/grodansparadis/vscpl1drv-can4vscp) that is available both for Windows and Linux.


## Available commands for driver

  | Code | Command |
  |------|---------|
  | 0    | NOOP - No operation. |
  | 1    | Open connection to CAN4VSCP bus in normal mode. |
  | 2    | Open connection to CAN4VSCP bus in listen only mode. |
  | 3    | Open connection to CAN4VSCP bus in loopback mode. |
  | 4    | Close connection to CAN4VSCP bus. |
  | 5    | Set filter. One of fifteen. |
  | 6    | Set mask. One of two. |
  | 0    | NOOP - No operation. |
  | 1    | Open connection to CAN4VSCP bus in normal mode. |
  | 2    | Open connection to CAN4VSCP bus in listen only mode. |
  | 3    | Open connection to CAN4VSCP bus in loopback mode. |
  | 4    | Close connection to CAN4VSCP bus. |
  | 5    | Set filter. One of fifteen. |
  | 6    | Set mask. One of two. |

For all of the above a command reply frame (op=254) is replied from the node with a 0 (==OK) as payload if the operation was carried out.

### NOOP

The payload has the following content.

| pos | content |
|-----|---------|
| 0   | Command code = 0 |

### Open

The payload has the following content.

| pos | content |
|-----|---------|
| 0   | Command code = 1 |

### Listen

The payload has the following content.

| pos | content |
|-----|---------|
| 0   | Command code = 2 |

### Loopback

The payload has the following content.

| pos | content |
|-----|---------|
| 0   | Command code = 3 |

### Close

The payload has the following content.

| pos | content |
|-----|---------|
| 0   | Command code = 4 |

### Set filter

The payload has the following content.

| pos | content |
|-----|---------|
| 0   | Command code = 5 |
| 1   | Filter number to set 0-15 |
| 2-5 | 32-bit filter MSB first |
| 6   | Nonzero if filter should be stored persistently |

### Set mask

The payload has the following content.

| pos | content |
|-----|---------|
| 0   | Command code = 6 |
| 1   | Mask number to set 0-1 |
| 2-5 | 32-bit mask MSB first |
| 6   | Nonzero if mask should be stored persistently |

## Available configuration settings for driver

Settings have an op-code in byte zero of the payload and can use the rest of the payload, up to the size it needs for

  | Code | Payload size | Command
  ------|--------------|-----------------------------------------------------
  0     | x            | NOOP - No operation.
  1     | 2            | Set mode. The change is persistent.
  2     | 2            | Enable disable timestamp. The change is persistent.
  3     | 2            | Set baudrate. NOT persistent

### NOOP

No operation. Nothing happens other than that an ACK is returned.

### Set mode

Set the mode of the device. Second byte of the payload contains the
requested mode. The change is persistent.

| Mode | Description               |
|------|---------------------------|
| 0    | Verbose.                  |
| 1    | CAN4VSCP (this mode).     |
| 2    | SLCAN.                    |
| 3    | VSCP serial node.         |

### Enable/disable timestamp

Enable or disable timestamp. Set payload byte 2 to zero to disable or to non zero to enable. The change is persistent.

### Set baudrate

With this setting you can temporarily change the baudrate of the serial line. Restarting the module makes it go back to the default baudrate 115200 baud again. The following settings are available

| Baudrate | Code | Error | Windows | Linux | 
|----------|------|-------|---------|-------|
| 115200   | 0    | -1.36%| yes     | yes   |
| 128000   | 1    | -2.34%| yes     | no    |
| 230400   | 2    | -1.36%| no      | yes   |
| 256000   | 3    | -2.34%| yes     | no    |
| 460800   | 4    | 8.51% | no      | no    |
| 500000   | 5    | 0%    | yes     | yes   |
| 625000   | 6    | 0%    | bad     | no    |
| 921600   | 7    | -9.58%| no      | bad   |
| 1000000  | 8    | 16.67%| no      | bad   |
| 9600     | 9    | 0.16% | yes     | yes   |
| 19200    | 10   | 0,16% | yes     | yes   |
| 38400   | 11   | 0,16% | yes     | yes   |
| 57600   | 12   | 0.94% | yes     | yes   |

Tests on Windows and Linux has been done on a Windows 10 machine and on a Ubuntu machine with the USB serial adapter that ship with Frankfurt RS-232.

[filename](./bottom-copyright.md ':include')
