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

#include "paparazzi.h"
#include "er9x.h"

uint16_t jeti_keys = JETI_KEY_NOCHANGE;
uint8_t JetiBuffer[32]; // 32 characters
uint8_t JetiBufferReady;

static uint8_t           tx_head0; /* next free in buf */
static volatile uint8_t  tx_tail0; /* next char to send */
uint8_t                  tx_buf0[TX_BUF_SIZE];
uint8_t                  pprz_last_proto = PROT_MAX+1;

ISR (USART0_RX_vect)
{
        uint8_t stat;
        uint8_t rh;
        uint8_t rl;
        static uint8_t jbp;
        
        stat = UCSR0A;
        rh = UCSR0B;
        rl = UDR0;

        {       // data
                if (jbp < 32)
                {
                    if(rl==0xDF) JetiBuffer[jbp++] = '@'; //@ => °  Issue 163
                    else         JetiBuffer[jbp++] = rl;
                }
                else {
                    JetiBufferReady = 1;
                    jbp = 0;
                }
        }

}

ISR (USART0_TX_vect)
{
  if (tx_head0 == tx_tail0) {
    /* Nothing more to send, disable interrupt */
    UCSR0B &= ~(1 << TXCIE0);
  } else {
    UDR0 = tx_buf0[tx_tail0];
    tx_tail0 = (tx_tail0 + 1) % TX_BUF_SIZE;
  }
}

void PPRZ_Init (void) {

  jeti_keys = JETI_KEY_NOCHANGE;

  tx_head0 = 0;
  tx_tail0 = 0;

  /* RXD0 input, pull-up disabled */
  DDRE  &= ~(1 << DDE0);
  PORTE &= ~(1 << PORTE0);

#undef BAUD
#define BAUD 57600
#include <util/setbaud.h>
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;

  /* enable double speed operation */
  UCSR0A |= (1 << U2X0);

  /* set 8N1 */
  UCSR0B = 0|(0<<RXCIE0)|(0<<TXCIE0)|(0<<UDRIE0)|(0<<RXEN0)|(1<<TXEN0)|(0<<UCSZ02);
  UCSR0C = 0|(1<<UCSZ01)|(1<<UCSZ00);
 
  /* flush receive buffer */
  while ( UCSR0A & (1 << RXC0) ) UDR0;
}

void JETI_DisableTXD (void)
{
        UCSR0B &= ~(1 << TXEN0);        // disable TX
}


void JETI_EnableTXD (void)
{
        UCSR0B |=  (1 << TXEN0);        // enable TX
}


void JETI_DisableRXD (void)
{
        UCSR0B &= ~(1 << RXEN0);        // disable RX
        UCSR0B &= ~(1 << RXCIE0);       // disable Interrupt
}


void JETI_EnableRXD (void)
{
        UCSR0B |=  (1 << RXEN0);        // enable RX
        UCSR0B |=  (1 << RXCIE0);       // enable Interrupt
}


void JETI_putw (uint16_t c)
{
        loop_until_bit_is_set(UCSR0A, UDRE0);
        UCSR0B &= ~(1 << TXB80);
        if (c & 0x0100)
        {
                UCSR0B |= (1 << TXB80);
        }
        UDR0 = c;
}

void JETI_putc (uint8_t c)
{
        loop_until_bit_is_set(UCSR0A, UDRE0);
        //      UCSRB &= ~(1 << TXB8);
        UCSR0B |= (1 << TXB80);
        UDR0 = c;
}

void JETI_puts (char *s)
{
        while (*s)
        {
                JETI_putc (*s);
                s++;
        }
}

void JETI_put_start (void)
{
        loop_until_bit_is_set(UCSR0A, UDRE0);
        UCSR0B &= ~(1 << TXB80);
        UDR0 = 0xFE;
}

void JETI_put_stop (void)
{
        loop_until_bit_is_set(UCSR0A, UDRE0);
        UCSR0B &= ~(1 << TXB80);
        UDR0 = 0xFF;
}

void PPRZ_putc(uint8_t data) {
  if (UCSR0B & (1<<TXCIE0)) {
    /* we are waiting for the last char to be sent : buffering */
    if (tx_tail0 == ((tx_head0 + 1) % TX_BUF_SIZE)) return;
    tx_buf0[tx_head0] = data;
    tx_head0 = (tx_head0 + 1) % TX_BUF_SIZE;
  } else { /* Channel is free: just send */
    UDR0 = data;
    UCSR0B |= (1 << TXCIE0);
  }
}

void PPRZ_puts(const char *s) {
  while (*s) {
    PPRZ_putc (*s);
    s++;
  }
}

void PPRZ_putm(uint8_t *s, int16_t len) {
  while (len--) {
    PPRZ_putc(*s);
    s++;
  }
}

void PPRZ_block_putc(uint8_t data) {
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = data;
}

void PPRZ_block_puts(const char *s) {
  while (*s) {
    PPRZ_block_putc (*s);
    s++;
  }
}

uint8_t xbee_api_chk(uint8_t* dat, uint16_t len) {
  uint8_t i, ck=0;
  for (i=0;i<len;i++) ck+=dat[i];
  return(0xff - ck);
}

uint16_t transparent_chk(uint8_t* dat, uint16_t len) {
  return 42;
}

uint8_t laird_chk(uint8_t* dat, uint16_t len) {
  return 42;
}

void PPRZ_setupPulses(void)
{
  static uint8_t pprz_xbee_api_conf = 0;
  static uint16_t xbee_time;
  uint8_t msg[32], i, mode;
  int16_t thr, ail, ele, rud, ch;
  uint16_t lenpos, ck, j=0;
  
  /* run at 60Hz */
  pulses2MHz[j++]=300u*2;
  pulses2MHz[j++]=16366u*2;
  pulses2MHz[j++]=0;
  
  /* currently configuring the XBee API mode? */
  switch (pprz_xbee_api_conf) {
    case 1:
      if (get_tmr10ms() > (xbee_time + 110)) {
        PPRZ_puts(XBEE_CMD_START);
        pprz_xbee_api_conf = 3;
      }
      return;
    case 2:
      if (get_tmr10ms() > (xbee_time + 110)) {
        PPRZ_puts(XBEE_CMD_START);
        pprz_xbee_api_conf = 4;
      }
      return;
    case 3:
      if (get_tmr10ms() > (xbee_time + 220)) {
        PPRZ_puts(XBEE_API_MODE);
        PPRZ_puts(XBEE_CMD_EXIT);		  
        pprz_xbee_api_conf = 0;
      }
      return;
    case 4:
      if (get_tmr10ms() > (xbee_time + 220)) {
        PPRZ_puts(XBEE_NORM_MODE);
        PPRZ_puts(XBEE_CMD_EXIT);		  
        pprz_xbee_api_conf = 0;
      }
      return;
  }  

  /* did the protocol change to/from XBee API mode? */
  if (g_model.protocol != pprz_last_proto) {
    if (g_model.protocol == PROTO_PPRZ_XBEE_API) {
      PPRZ_putc(0x55);
      pprz_xbee_api_conf = 1;
      xbee_time = get_tmr10ms();
    } else
    if (pprz_last_proto == PROTO_PPRZ_XBEE_API) {
      PPRZ_putc(0x55);
      pprz_xbee_api_conf = 2;
      xbee_time = get_tmr10ms();
    }
    pprz_last_proto = g_model.protocol;
    /* no more serial data */
    return;
  }

  /* message header */
  j=0;
  switch (g_model.protocol) {
    case PROTO_PPRZ_XBEE_API:
      msg[j++] = XBEE_START;
      lenpos=j++; /* len msb */
      j++;        /* len lsb */
      msg[j++] = XBEE_TX_ID;
      msg[j++] = XBEE_NO_FRAME_ID;
      msg[j++] = XBEE_BCAST;
      msg[j++] = XBEE_BCAST;
      msg[j++] = XBEE_TX_OPTIONS;
      break;
    case PROTO_PPRZ_TRANSPARENT:
      msg[j++] = TRANSP_STX;
      lenpos=j++;
      break;
    case PROTO_PPRZ_LAIRD:
      msg[j++] = LAIRD_START;
      lenpos=j++;
      msg[j++] = LAIRD_RESVD;
      msg[j++] = LAIRD_RETRIES;
      msg[j++] = LAIRD_BCAST;
      msg[j++] = LAIRD_BCAST;
      msg[j++] = LAIRD_BCAST;
      break;
    default:
      return;
  }

  /* FIXME make switch configurable */
  mode = (keyState(SW_ID2) << 1) | keyState(SW_ID1);
  /* wtf? */
  thr = calibratedStick[THR_STICK];
  if (thr == 1024) thr = 1023;
  ele = calibratedStick[ELE_STICK];
  if (ele == 1024) ele = 1023;
  ail = calibratedStick[AIL_STICK];
  if (ail == 1024) ail = 1023;
  rud = calibratedStick[RUD_STICK];
  if (rud == 1024) rud = 1023;

  /* message payload */
  /* FIXME make transmitter ID configurable */
  msg[j++] = PPRZ_RCTX_ID;
  switch (g_model.ppmNCH) {
    case PPRZ_RC_3CH:
      msg[j++] = RC_3CH_ID;
      /* throttle is encoded in bit 7..2 */
      msg[j] = ((thr + 1024) / 8) & 0xFC;
      /* mode is encoded in bit 1..0 */
      msg[j++] |= mode;
      msg[j++] = ail / 8;
      msg[j++] = ele / 8;
      break;
    case PPRZ_RC_4CH:
      msg[j++] = RC_4CH_ID;
      /* FIXME switch to transmitter binding */
      msg[j++] = AC_ID;
      msg[j++] = mode;
      msg[j++] = (thr + 1024) / 8;
      msg[j++] = ail / 8;
      msg[j++] = ele / 8;
      msg[j++] = rud / 8;
      break;
    case PPRZ_RC_8CH:
      msg[j++] = RC_8CH_ID;
      /* roll lsb is encoded in bit 7..6 */
      msg[j]  = ((ail / 2) & 0x03) << 6;
      /* pitch lsb is encoded in bit 5..4 */
      msg[j] |= ((ele / 2) & 0x03) << 4;
      /* yaw lsb is encoded in bit 3..2 */
      msg[j] |= ((rud / 2) & 0x03) << 2;
      /* mode is encoded in bit 1..0 */
      msg[j++] |= mode;
      msg[j++] = (thr + 1024) / 8;
      msg[j++] = ail / 8;
      msg[j++] = ele / 8;
      msg[j++] = rud / 8;
      for (i=4;i<8;i++) {
        ch = calibratedStick[i];
        if (ch == 1024) ch = 1023;
        msg[j++] = ch / 8;
      }
      break;
    default:
      return;
  }

  /* message trailer */
  switch (g_model.protocol) {
    case PROTO_PPRZ_XBEE_API:
      msg[lenpos] = (j-3) / 256; /* starts with xbee payload */
      msg[lenpos+1] = (j-3) % 256;
      ck = xbee_api_chk(&msg[3], j-3);
      msg[j++] = ck;
      break;
    case PROTO_PPRZ_TRANSPARENT:
      ck = transparent_chk(msg, j);
      msg[j++] = ck >> 8;
      msg[j++] = ck & 0xFF;
      msg[lenpos] = j;
      break;
    case PROTO_PPRZ_LAIRD:
      ck = laird_chk(msg, j);
      msg[j++] = ck;
      msg[lenpos] = j-7; /* starts with pprz payload */
      break;
    default:
      return;
  }

 {
  static uint8_t plop;

  if (plop++ > 30) {
    /* queue it */
    PPRZ_putm(msg, j);
    plop = 0;
  }
 }

#if 0
  static uint8_t plop;
  int16_t ch0 = calibratedStick[THR_STICK];

  if (plop++ > 30) {
    if (ch0 > 0) {
      PPRZ_putc('+');
    } else {
      PPRZ_putc('-');
      ch0 = -ch0;
    }
    PPRZ_putc((ch0 / 1000) + '0');
    PPRZ_putc((ch0 / 100) % 10 + '0');
    PPRZ_putc((ch0 / 10) % 10 + '0');
    PPRZ_putc((ch0 % 10) + '0');
    PPRZ_putc('\r');
    PPRZ_putc('\n');
    plop = 0;
  }
#endif  

   /* XBee-message: ABCDxxxxxxxE
     A XBEE_START (0x7E)
     B LENGTH_MSB (D->D)
     C LENGTH_LSB
     D XBEE_PAYLOAD
       0 XBEE_TX16 (0x01) / XBEE_RX16 (0x81)
       1 FRAME_ID (0)     / SRC_ID_MSB
       2 DEST_ID_MSB      / SRC_ID_LSB
       3 DEST_ID_LSB      / XBEE_RSSI
       4 TX16_OPTIONS (0) / RX16_OPTIONS
       5 PPRZ_DATA
         0 SENDER_ID
         1 MSG_ID
         2 MSG_PAYLOAD
         . DATA (messages.xml)
     E XBEE_CHECKSUM (sum[A->D])

     ID is AC_ID for aircraft, 0x100 for ground station


     PPRZ-message: ABCxxxxxxxDE
     A PPRZ_STX (0x99)
     B LENGTH (A->E)
     C PPRZ_DATA
       0 SENDER_ID
       1 MSG_ID
       2 MSG_PAYLOAD
       . DATA (messages.xml)
     D PPRZ_CHECKSUM_A (sum[B->C])
     E PPRZ_CHECKSUM_B (sum[ck_a])


     Laird-message: ABCxxxxxxxxE
     A START_DELIMITER (0x81)
     B LENGTH 0x01-0x50 (C5->D)
     C LAIRD_PAYLOAD
       0 reserved                            / reserved
       1 transmit retries/broadcast attempts / received RSSI
       2 DEST_MAC_MSB                        / SRC_MAC_MSB
       3 DEST_MAC                            / SRC_MAC
       4 DEST_MAC_LSB                        / SRC_MAC_LSB
       5 PPRZ_DATA
         0 SENDER_ID
         1 MSG_ID
         2 MSG_PAYLOAD
         . DATA (messages.xml)
     D LAIRD_CHECKSUM (sum[A->D])

    ID is AC_ID for aircraft, 0x00 for ground station

      
  <message name="RC_3CH" id="51" link="broadcasted">
    <field name="throttle_mode" type="uint8" unit="byte_mask"/>
    <field name="roll"  type="int8"/>
    <field name="pitch" type="int8"/>
  </message>

  <message name="RC_4CH" id="52" link="broadcasted">
    <field name="ac_id"       type="uint8"/>
    <field name="mode"        type="uint8"/>
    <field name="throttle"    type="uint8"/>
    <field name="roll"        type="int8"/>
    <field name="pitch"       type="int8"/>
    <field name="yaw"         type="int8"/>
  </message>

  <message name="RC_8CH" id="64" link="broadcasted">
    <field name="mode_lsb"    type="uint8"/>
    <field name="throttle"    type="int8"/>
    <field name="roll"        type="int8"/>
    <field name="pitch"       type="int8"/>
    <field name="yaw"         type="int8"/>
    <field name="ch5"         type="int8"/>
    <field name="ch6"         type="int8"/>
    <field name="ch7"         type="int8"/>
    <field name="ch8"         type="int8"/>
  </message>
     
  */
}
