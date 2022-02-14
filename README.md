DRT-301M-MODBusClient
=====================

modbusclient for the DRT-301M multi tarif energy meter from Forlong

http://www.cqbluejay.com/Support/Downloads/

The DRT-301M is a similar device to DRT-301C-II, DRS-202M, DRS-202C and DRT-301C.

They all are interfaced via RS-485 serial interface with MODBus RTU protocol.

The goal of this software is to have an easy possibility to interact with the meter, especially the energy and max demand counters.

The original documentation has some errors in the register definition. These have been annotated in the PDF.

modbusclient depends on libmodbus from libmodbus.org (libmodbus-3.1.6.tar.gz attached).

Add udev rules to /etc/udev/rules.d/10-modbus.rules (in my case a FT232 based adapter):

```ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}==".....", SYMLINK+="DRT-301"```

Get these values via ```udevadm info -a -p $( udevadm info -q path -n /dev/ttyUSB0 ) | less```

Test the configuration with ```udevadm test $( udevadm info -q path -n /dev/ttyUSB0 ) 2>&1 | less```

Force reload with ```udevadm control --reload-rules && udevadm trigger -attr-match=subsystem=tty```



Possible future features:
* Different output formats (csv, json)
* csv table headers optional
* csv record separator
* precede output with customisable timestamp
* additional/cascading reports 
* split reports
  * volt, current
  * power, cos phi

---
2022-02-13
* upgrade to libmodbus-3.1.6
* fix issues with parameter -i
* fix issues with parameter -v
* introduce parameter -V
* introduce parameter -sa
* other minor enhancements

2014-11-18
* Initial release
