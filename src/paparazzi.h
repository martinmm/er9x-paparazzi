/*
 * Author - Karl Szmutny <shadow@privy.de>
 *
 * Based on the Code from Peter "woggle" Mack, mac@denich.net
 * -> http://svn.mikrokopter.de/filedetails.php?repname=Projects&path=/Transportables_Koptertool/trunk/jeti.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef paparazzi_h
#define paparazzi_h


#include "er9x.h"

#define JETI_KEY_LEFT		0x70
#define JETI_KEY_RIGHT		0xe0
#define JETI_KEY_UP		0xd0
#define JETI_KEY_DOWN		0xb0
#define JETI_KEY_NOCHANGE	0xf0

extern uint16_t jeti_keys;
extern uint8_t JetiBuffer[32]; // 32 characters
extern uint8_t JetiBufferReady;

void JETI_DisableTXD (void);
void JETI_EnableTXD (void);
void JETI_DisableRXD (void);
void JETI_EnableRXD (void);

void JETI_putw (uint16_t c);
void JETI_putc (uint8_t c);
void JETI_puts (char *s);
void JETI_put_start (void);
void JETI_put_stop (void);

#define TX_BUF_SIZE         64

#define XBEE_START          0x7E
#define XBEE_TX_ID          0x01
#define XBEE_NO_FRAME_ID    0x00
#define XBEE_BCAST          0xFF
#define XBEE_TX_OPTIONS     0x00
#define XBEE_CMD_START      "+++"
#define XBEE_NORM_MODE      "ATAP0\r"
#define XBEE_API_MODE       "ATAP1\r"
#define XBEE_CMD_EXIT       "ATCN\r"

#define TRANSP_STX          0x99

#define LAIRD_START         0x81
#define LAIRD_RESVD         0x00
#define LAIRD_RETRIES       0x00
#define LAIRD_BCAST         0xFF

#define PPRZ_RC_3CH         -2
#define PPRZ_RC_4CH         -1
#define PPRZ_RC_8CH         0

#define PPRZ_RCTX_ID        0xF0

#define AC_ID               42

#define RC_3CH_ID           51
#define RC_4CH_ID           52
#define RC_8CH_ID           64


void PPRZ_Init(void);
void PPRZ_setupPulses(void);

#endif

