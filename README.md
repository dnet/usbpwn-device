USBpwn AVR device
=================

The purpose of this device is to receive any (text or binary) file using the
LEDs on the keyboard (Num, Caps and Scroll lock). It uses the V-USB library to
act as a USB HID keyboard, and is the modified version of one of the projects
using it (c64keys, which in turn used HIDkeys).

Storing data
------------

First, it sends a text-based script from its Flash memory as keystrokes
(`SD2KEYS` mode), which can be used to act as a host helper in sending data
in the next step. After the last keystoke is sent, the device turns into
`KEYS2EEPROM` mode, and it monitors the LED updates mentioned above. The
protocol used is the following:

 - when the Scroll lock is turned on, the Num and Caps lock state is sampled
 - the Num and Caps lock state is the last two bits sent (in this order)
 - the device sends an Enter/Return keystroke

In other words, Scroll lock is used as a "clock signal", and at raising edge,
the two other LEDs can be sampled for data. The newline (that can be achieved
by the Enter/Return key) is the acknowledgement signal, making the protocol
fully synchronous. For example, sending the bits `1101` can be done in the
following way:

	            __________________________________
	   NUM ____/
	                               _______________
	  CAPS _______________________/
	                ______            ______
	SCROLL ________/      \__________/      \_____
	
	                  01                11

Bits are sent from LSB to MSB, n bytes are sent from 0 to n-1, stored at the
nth position in the EEPROM. The MCU I used is the ATmega328, which has 1 kbyte
of it, which limits the size of the largest file that can be received.

Retrieving data
---------------

If the 5th bit of PORT B (digital 13 on Arduino) is pulled to ground upon boot,
the device starts up in `EEPROM2KEYS` mode, in which it sends the contents of
the EEPROM bytes in hexadecimal encoding. It doesn't know which cells were used
last time, so it sends all bytes in ascending order in an endless loop. One
easy way of decoding it is using the `unhexlify` method of the `binascii`
Python module.

Dependencies
------------

 - GNU AVR C library
 - Objective Development V-USB library
 - Python with `serial` module (for Arduino upload only)

Compiling
---------

Install the dependencies and run `make hex`. The `main.hex` file contains the
compiled image, which can be uploaded to Arduinos using `upload.sh`. If your
serial port is not at `/dev/ttyUSB0`, you'll need to change it there.

License
-------

The whole project is under OBDEV license.
