/*********************************************************************
 * main.c - Main firmware (ATmega16 version)                         *
 * $Id: $
 * Version 1.98ß                                                     *
 *********************************************************************
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
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>

/* Now included from the makefile */
//#include "keycodes.h"

#include <sd-reader_config.h>
#include <sd_raw.h>
#include <sd_raw_config.h>

#include "usbdrv.h"
#define DEBUG_LEVEL 0
#include "oddebug.h"

#define TCCR0 TCCR0A
#define TIFR TIFR0

/* Hardware documentation:
 * ATmega-16 @12.000 MHz
 *
 * XT1..XT2: 12MHz X-tal
 * PB0..PB7: Keyboard matrix Row0..Row7 (pins 12,11,10,5,8,7,6,9 on C64 kbd)
 * PC0..PC7: Keyboard matrix Col0..Col7 (pins 13,19,18,17,16,15,14,20 on C64 kbd)
 * PD0     : D- USB negative (needs appropriate zener-diode and resistors)
 * PD1     : UART TX
 * PD2/INT0: D+ USB positive (needs appropriate zener-diode and resistors)
 * PD7     : Keyboard matrix Row8 (Restore key)
 *
 * USB Connector:
 * -------------
 *  1 (red)    +5V
 *  2 (white)  DATA-
 *  3 (green)  DATA+
 *  4 (black)  GND
 *    
 *
 *
 *                                     VCC
 *                  +--[4k7]--+--[2k2]--+
 *      USB        GND        |                     ATmega-16
 *                            |
 *      (D-)-------+----------+--------[82r]------- PD0
 *                 |
 *      (D+)-------|-----+-------------[82r]------- PD2/INT0
 *                 |     |
 *                 _     _
 *                 ^     ^  2 x 3.6V 
 *                 |     |  zener to GND
 *                 |     |
 *                GND   GND
 */

/* The LED states */
#define LED_NUM     0x01
#define LED_CAPS    0x02
#define LED_SCROLL  0x04
#define LED_COMPOSE 0x08
#define LED_KANA    0x10


/* Originally used as a mask for the modifier bits, but now also
   used for other x -> 2^x conversions (lookup table). */
const char modmask[8] PROGMEM = {
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80
  };


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

static void hardwareInit(void) {
  PORTB = 0xFF;   /* Port B are row drivers - enable pull-up */
  DDRB  = 0x00;   /* Port B is input */

  PORTC = 0xFF;   /* activate all pull-ups */
  DDRC  = 0x00;   /* all pins input */
  
  PORTD = 0xfa;   /* 1111 1010 bin: activate pull-ups except on USB lines */
  DDRD  = 0x07;   /* 0000 0111 bin: all pins input except USB (-> USB reset) */

  /* USB Reset by device only required on Watchdog Reset */
  _delay_us(11);   /* delay >10ms for USB reset */ 

  DDRD = 0x02;    /* 0000 0010 bin: remove USB reset condition */
  /* configure timer 0 for a rate of 12M/(1024 * 256) = 45.78 Hz (~22ms) */
  TCCR0 = 5;      /* timer 0 prescaler: 1024 */
  sd_raw_init();
}

uchar expectReport=0;
uchar LEDstate=0;

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

uchar usbFunctionWrite(uchar *data, uchar len) {
  if ((expectReport)&&(len==1)) {
    LEDstate=data[0]; /* Get the state of all 5 LEDs */
    if (LEDstate&LED_CAPS) { /* Check state of CAPS lock LED */
      //PORTD|=0x02; XXX
    } else {
      //PORTD&=~0x02; XXX
    }
    expectReport=0;
    return 1;
  }
  expectReport=0;
  return 0x01;
}

uint32_t offset = 0;
uint8_t sd_buf, lastbuf;
uchar written = 1;

static uchar scankeys(void) {
  uchar retval=0;
  
    memset(reportBuffer,0,sizeof(reportBuffer)); /* Clear report buffer */
	if (sd_buf >= 'a' && sd_buf <= 'z') {
		reportBuffer[2] = sd_buf - 'a' + 4;
	} else if (sd_buf >= 'A' && sd_buf <= 'Z') {
		reportBuffer[0] = 2;
		reportBuffer[2] = sd_buf - 'A' + 4;
	} else if (sd_buf == ',') {
		reportBuffer[2] = 54;
	} else if (sd_buf == ' ') {
		reportBuffer[2] = 44;
	} else if (sd_buf == '\n') {
		reportBuffer[2] = 40;
	}
	if (written) {
		lastbuf = sd_buf;
		sd_raw_read(offset, &sd_buf, 1);
		if (sd_buf == 0) {
			offset = 0;
			sd_buf = '\n';
		} else if (lastbuf == sd_buf) {
			lastbuf = 0;
			sd_buf = 0;
		} else {
			offset++;
		}
		written = 0;
	}
	retval|=1; /* Must have been a change at some point, since debounce is done */
  
  return retval;
}


int main(void) {
  uchar   updateNeeded = 0;
  uchar   idleCounter = 0;

  wdt_enable(WDTO_2S); /* Enable watchdog timer 2s */
  hardwareInit(); /* Initialize hardware (I/O) */
  
  odDebugInit();

  usbInit(); /* Initialize USB stack processing */
  sei(); /* Enable global interrupts */
  
  for(;;){  /* Main loop */
    wdt_reset(); /* Reset the watchdog */
    usbPoll(); /* Poll the USB stack */

    updateNeeded|=scankeys(); /* Scan the keyboard for changes */
    
    /* Check timer if we need periodic reports */
    if(TIFR & (1<<TOV0)){
      TIFR = 1<<TOV0; /* Reset flag */
      if(idleRate != 0){ /* Do we need periodic reports? */
        if(idleCounter > 4){ /* Yes, but not yet */
          idleCounter -= 5;   /* 22 ms in units of 4 ms */
        }else{ /* Yes, it is time now */
          updateNeeded = 1;
          idleCounter = idleRate;
        }
      }
    }
    
    /* If an update is needed, send the report */
    if(updateNeeded && usbInterruptIsReady()){
      updateNeeded = 0;
      usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
	  written = 1;
    }
  }
  return 0;
}
