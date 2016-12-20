/*
 * slip_net.c
 *
 *  Created on: Jul 4, 2014
 *      Author: yushev
 */
/**
 * \file
 *         A brief description the file
 */

#include "emb6.h"
#include "bsp.h"
#include "uip.h"
#include "packetbuf.h"
#include "slip.h"
#include <stdio.h>

#define SLIP_END     0300
#define SLIP_ESC     0333
#define SLIP_ESC_END 0334
#define SLIP_ESC_ESC 0335

#define DEBUG DEBUG_NONE
#include "uip-debug.h"

static void* p_slipUart;

/*---------------------------------------------------------------------------*/
void
slipnet_init(s_ns_t* p_ns)
{
  /* get the according UART from the BSP */
  p_slipUart = bsp_uartInit( EN_HAL_UART_SLIP );
  EMB6_ASSERT( p_slipUart != NULL );
}
/*---------------------------------------------------------------------------*/
void
slip_send_packet(const uint8_t *ptr, int len)
{
  uint16_t i;
  uint8_t c;

  uint8_t SLIPEND_C = SLIP_END;
  uint8_t SLIPESC_C = SLIP_ESC;

  bsp_uartTx(p_slipUart, &SLIPEND_C, 1);
  for(i = 0; i < len; ++i) {
    c = *ptr++;
    if(c == SLIP_END) {
        bsp_uartTx(p_slipUart, &SLIPESC_C, 1);
        c = SLIP_ESC_END;
    } else if(c == SLIP_ESC) {
        bsp_uartTx(p_slipUart, &SLIPESC_C, 1);
        c = SLIP_ESC_ESC;
    }
    bsp_uartTx(p_slipUart, &c, 1);
  }
  bsp_uartTx(p_slipUart, &SLIPEND_C, 1);
}
/*---------------------------------------------------------------------------*/
void
slipnet_input(void)
{
  int i;
  /* radio should be configured for filtering so this should be simple */
  /* this should be sent over SLIP! */
  /* so just copy into uip-but and send!!! */
  /* Format: !R<data> ? */
  uip_len = packetbuf_datalen();
  i = packetbuf_copyto(uip_buf);

  if(DEBUG) {
    printf("Slipnet got input of len: %d, copied: %d\n",
       packetbuf_datalen(), i);

    for(i = 0; i < uip_len; i++) {
      printf("%02x", (unsigned char) uip_buf[i]);
      if((i & 15) == 15) printf("\n");
      else if((i & 7) == 7) printf(" ");
    }
    printf("\n");
  }

  /* printf("SUT: %u\n", uip_len); */
  slip_send_packet(uip_buf, uip_len);
}
/*---------------------------------------------------------------------------*/
const s_nsHeadComp_t hc_driver_slipnet = {
  "slipnet",
  slipnet_init,
  slipnet_input
};
/*---------------------------------------------------------------------------*/

