/*********************************************************************
 * main.c - Main firmware                                            *
 *********************************************************************
 * USBpwn is Copyright (C) 2012 Andras Veres-Szentkiralyi            *
 * licensed the same as and based on C64key, see copyright below     *
 * c64key is Copyright (C) 2006-2007 Mikkel Holm Olsen               *
 * based on HID-Test by Christian Starkjohann, Objective Development *
 *********************************************************************
 * Spaceman Spiff's Commodire 64 USB Keyboard (c64key for short) is  *
 * is free software; you can redistribute it and/or modify it under  *
 * the terms of the OBDEV license, as found in the licence.txt file. *
 *                                                                   *
 * c64key is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of    *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the     *
 * OBDEV license for further details.                                *
 *********************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>

#include "usbdrv.h"
#include "oddebug.h"

/* The LED states */
#define LED_NUM     0x01
#define LED_CAPS    0x02
#define LED_SCROLL  0x04

/* USB report descriptor (length is defined in usbconfig.h)
   This has been changed to conform to the USB keyboard boot
   protocol */
const char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] 
  PROGMEM = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION  
};

/* The ReportBuffer contains the USB report sent to the PC */
static uchar reportBuffer[8];    /* buffer for HID reports */
static uchar idleRate;           /* in 4 ms units */
static uchar protocolVer=1;      /* 0 is the boot protocol, 1 is report protocol */

#define SD2KEYS 0
#define KEYS2EEPROM 1
#define EEPROM2KEYS 2
uint8_t mode;

static void hardwareInit(void) {
  PORTD = 0xfa;   /* 1111 1010 bin: activate pull-ups except on USB lines */
  DDRD  = 0x07;   /* 0000 0111 bin: all pins input except USB (-> USB reset) */

  /* USB Reset by device only required on Watchdog Reset */
  _delay_us(11);   /* delay >10ms for USB reset */ 

  DDRD = 0x02;    /* 0000 0010 bin: remove USB reset condition */
  PORTB |= (1 << PB5); /* pull-up on Arduino D13 */
  _delay_us(1);
  mode = ((PINB & (1 << PB5)) == (1 << PB5) ? SD2KEYS : EEPROM2KEYS);
}

uchar expectReport=0;
uchar usbFunctionSetup(uchar data[8]) {
  usbRequest_t *rq = (void *)data;
  usbMsgPtr = reportBuffer;
  if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
    if(rq->bRequest == USBRQ_HID_GET_REPORT){  
      /* wValue: ReportType (highbyte), ReportID (lowbyte) */
      /* we only have one report type, so don't look at wValue */
      return sizeof(reportBuffer);
    }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
      if (rq->wLength.word == 1) { /* We expect one byte reports */
        expectReport=1;
        return 0xFF; /* Call usbFunctionWrite with data */
      }  
    }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
      usbMsgPtr = &idleRate;
      return 1;
    }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
      idleRate = rq->wValue.bytes[1];
    }else if(rq->bRequest == USBRQ_HID_GET_PROTOCOL) {
      if (rq->wValue.bytes[1] < 1) {
        protocolVer = rq->wValue.bytes[1];
      }
    }else if(rq->bRequest == USBRQ_HID_SET_PROTOCOL) {
      usbMsgPtr = &protocolVer;
      return 1;
    }
  }
  return 0;
}

#define LEDstate data[0]
#define NUM_CAPS_MASK (LED_NUM | LED_CAPS)
#define RECV_WAIT 0
#define RECV_ACK 1

const char dropper[] PROGMEM = {'H', 'e', 'l', 'l', 'o', ',', ' ', 'w', 'o', 'r', 'l', 'd'};

uint8_t recv_byte = 0;
uint8_t recv_byte_pos = 0;
uint8_t recv_state = RECV_WAIT;
uint32_t offset = 0;

uchar usbFunctionWrite(uchar *data, uchar len) {
  if ((expectReport)&&(len==1)) {
    if (mode == KEYS2EEPROM && ((LEDstate & LED_SCROLL) == LED_SCROLL) && recv_state == RECV_WAIT) {
		recv_byte |= (LEDstate & NUM_CAPS_MASK) << recv_byte_pos;
		recv_byte_pos += 2;
		recv_state = RECV_ACK;
		if (recv_byte_pos == 8) {
			eeprom_write_byte((uint8_t *)offset, recv_byte);
			recv_byte = 0;
			recv_byte_pos = 0;
			offset++;
		}
    }
  }
  expectReport=0;
  return 0x01;
}

uint8_t type_buf, lastbuf;

static inline void buf2report() {
	if (type_buf >= 'a' && type_buf <= 'x') {
		reportBuffer[2] = type_buf - 'a' + 4;
	} else if (type_buf >= 'A' && type_buf <= 'X') {
		reportBuffer[0] = 2;
		reportBuffer[2] = type_buf - 'A' + 4;
	} else if (type_buf >= '1' && type_buf <= '9') {
		reportBuffer[2] = type_buf - '1' + 30;
	} else if (type_buf == '=')  {
		reportBuffer[0] = 2;
		reportBuffer[2] = 36;
	} else if (type_buf == '"')  {
		reportBuffer[0] = 2;
		reportBuffer[2] = 31;
	} else if (type_buf == '+')  {
		reportBuffer[0] = 2;
		reportBuffer[2] = 32;
	} else if (type_buf == '/')  {
		reportBuffer[0] = 2;
		reportBuffer[2] = 35;
	} else if (type_buf == '(')  {
		reportBuffer[0] = 2;
		reportBuffer[2] = 37;
	} else if (type_buf == ')')  {
		reportBuffer[0] = 2;
		reportBuffer[2] = 38;
	} else if (type_buf == 'Z')  {
		reportBuffer[0] = 2;
		reportBuffer[2] = 28;
	} else if (type_buf == 'Y')  {
		reportBuffer[0] = 2;
		reportBuffer[2] = 29;
	} else if (type_buf == 'z')  { reportBuffer[2] = 28;
	} else if (type_buf == 'y')  { reportBuffer[2] = 29;
	} else if (type_buf == ',')  { reportBuffer[2] = 54;
	} else if (type_buf == '0')  { reportBuffer[2] = 53;
	} else if (type_buf == '.')  { reportBuffer[2] = 55;
	} else if (type_buf == ' ')  { reportBuffer[2] = 44;
	} else if (type_buf == '\n') { reportBuffer[2] = 40;
	}
}

int main(void) {
  wdt_enable(WDTO_2S); /* Enable watchdog timer 2s */
  hardwareInit(); /* Initialize hardware (I/O) */
  
  odDebugInit();

  usbInit(); /* Initialize USB stack processing */
  sei(); /* Enable global interrupts */
  
  for(;;){  /* Main loop */
    wdt_reset(); /* Reset the watchdog */
    usbPoll(); /* Poll the USB stack */

    if(usbInterruptIsReady()){
		lastbuf = type_buf;
		switch (mode) {
			case SD2KEYS:
				type_buf = pgm_read_byte(&dropper[offset]);
				break;
			case KEYS2EEPROM:
				if (recv_state == RECV_ACK) {
					type_buf = '\n';
					recv_state = RECV_WAIT;
				} else {
					type_buf = 0;
				}
				break;
			case EEPROM2KEYS:
				type_buf = eeprom_read_byte((uint8_t *)(offset >> 1));
				if ((offset & 1) == 0) type_buf >>= 4;
				type_buf &= 0x0f;
				if (type_buf < 10) type_buf = '0' + type_buf; else type_buf = 'A' + type_buf - 10;
				break;
		}
		if (offset == sizeof(dropper) - 1 && mode == SD2KEYS) {
			mode++;
			offset = 0;
		} else if ((lastbuf & 0xdf) == (type_buf & 0xdf)) {
			lastbuf = 0;
			type_buf = 0;
		} else if (mode == SD2KEYS || mode == EEPROM2KEYS) {
			offset++;
		}
	memset(reportBuffer,0,sizeof(reportBuffer)); /* Clear report buffer */
	buf2report();
      usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
    }
  }
  return 0;
}
