#!/bin/sh
python pulsedtr.py /dev/ttyUSB0
avrdude -p m328p -F -b 57600 -c stk500v1 -P /dev/ttyUSB0 -U flash:w:main.hex
