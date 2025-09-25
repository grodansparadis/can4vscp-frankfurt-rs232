# Using with the VSCP Daemon

Full info about the driver is
[here](https://github.com/grodansparadis/vscpl1drv-can4vscp)

Add the driver to the **vscpd.conf** file in the \<level1driver\> section.

## Windows

```xml
  <!-- The can4vscp driver  -->
  <driver enable="true" >
      <name>can4vscp</name>
      <config>com10</config>
      <path>c:\somewhere\vscpl1_can4vscpdrv.dll</path>
      <flags>0</flags>
      <guid>00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00</guid>
  </driver>
```    

## Unix

```xml
  <driver enable="true" >
      <name>can4vscp</name>
      <path>/home/akhe/vscp_software/src/vscp/drivers/level1/can4vscp/linux/vscpl1drv_can4vscp.so</path>
      <config>/dev/ttyUSB2</config>
      <guid>00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00</guid>
  </driver>
```

then restart the vscp daemon to load the driver.

## Explanations

#### driver enable

Set to **true** to activate the driver or **false** to disable it.

#### name

Set to anything you like. This is just an identifier for the driver.

#### path

The full path to the driver

#### config

The serial adapter the Frankfurt RS-232 module is connected to.

#### guid

The GUID for the driver. Will be used if not non-zero which is the
default. If zero or not given the GUID of the interface of the daemon
will be used.

[filename](./bottom-copyright.md ':include')
