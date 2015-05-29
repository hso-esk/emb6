/*
 * emb6 is licensed under the 3-clause BSD license. This license gives everyone
 * the right to use and distribute the code, either in binary or source code
 * format, as long as the copyright license is retained in the source code.
 *
 * The emb6 is derived from the Contiki OS platform with the explicit approval
 * from Adam Dunkels. However, emb6 is made independent from the OS through the
 * removal of protothreads. In addition, APIs are made more flexible to gain
 * more adaptivity during run-time.
 *
 * The license text is:
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*============================================================================*/
/**
 * 	 \addtogroup embetter6
 * 	 @{
 * 	 \addtogroup demo
 * 	 @{
 * 	 \addtogroup demo_udp_alive
 * 	 @{
*/
/*! \file   demo_udp_alive.c

 \author Peter Lehmann, peter.lehmann@hs-offenburg.de

 \brief  UDP Client Source for DODAG visualization on Cetic 6LBR

 \version 0.0.1
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 =============================================================================*/

#include "emb6.h"
#include "emb6_conf.h"
#include "bsp.h"
#include "demo_udp_alive.h"
#include "etimer.h"
#include "evproc.h"
#include "tcpip.h"
#include "uip.h"
#include "rpl.h"
#include "uip-debug.h"
#include "uip-udp-packet.h"

/*==============================================================================
 	 	 	 	 	 	 	 	 MACROS
 =============================================================================*/
#define 	LOGGER_ENABLE		LOGGER_DEMO_EXUDP
#if			LOGGER_ENABLE 	== 	TRUE
#define		LOGGER_SUBSYSTEM	"dUDP"
#endif
#include	"logger.h"

/** set the send interval */
#define 	SEND_INTERVAL			60

/** set the payload length */
#define 	MAX_PAYLOAD_LEN			40

/** set the communication port (default for 6LBR: 3000) */
#define 	_PORT 					3000

/*==============================================================================
 	 	 	 	 	 LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

static struct uip_udp_conn *client_conn = NULL;

uip_ip6addr_t dest_addr;
uint8_t use_user_dest_addr = 0;
uip_ip6addr_t user_dest_addr;
uint16_t user_dest_port = _PORT ;
uint8_t udp_client_run = 0;

struct etimer st_et;

/*==============================================================================
 	 	 	 	 	 	 LOCAL FUNCTION PROTOTYPES
 =============================================================================*/

static 		void 		_6lbr_udp_sendMsg(void);
static		char *		add_ipaddr(char * buf, const uip_ipaddr_t *addr);

/*==============================================================================
 	 	 	 	 	 	 	 LOCAL FUNCTIONS
 =============================================================================*/

char * add_ipaddr(char * buf, const uip_ipaddr_t *addr)
{
  uint16_t a;
  unsigned int i;
  int f;
  char *p = buf;

  for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
    a = (addr->u8[i] << 8) + addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0) {
        p += sprintf(p, "::");
      }
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0) {
        p += sprintf(p, ":");
      }
      p += sprintf(p, "%04x", a);
    }
  }
  return p;
}

/*----------------------------------------------------------------------------*/
/** \brief  This function is sending message with a sequence number.
 *
 *  \param  none
 *  \param	none
 *
 *  \returns none
 */
/*----------------------------------------------------------------------------*/
static
void _6lbr_udp_sendMsg(void)
{
    LOG_INFO("Send message: \n\r");

	 static int seq_id;
	  char buf[MAX_PAYLOAD_LEN];
	  int i;
	  uip_ip6addr_t *globaladdr = NULL;
	  uint16_t dest_port = use_user_dest_addr ? user_dest_port : _PORT;
	  int has_dest=0;

	  if ( use_user_dest_addr ) {
	    uip_ipaddr_copy(&dest_addr, &user_dest_addr);
	    has_dest=1;
	  } else {
	    uip_ds6_addr_t * addr_desc = uip_ds6_get_global(ADDR_PREFERRED);
	    if(addr_desc != NULL) {
	      globaladdr = &addr_desc->ipaddr;
	#if UIP_CONF_IPV6_RPL
	      rpl_dag_t *dag = rpl_get_any_dag();
	      if(dag) {
	        uip_ipaddr_copy(&dest_addr, globaladdr);
	        memcpy(&dest_addr.u8[8], &dag->dag_id.u8[8], sizeof(uip_ipaddr_t) / 2);
	        has_dest = 1;
	      }
	#else
		  uip_ipaddr_t * defrt;
	      defrt = uip_ds6_defrt_choose();
	      if ( defrt != NULL ) {
	        uip_ipaddr_copy(&dest_addr, defrt);
	        uip_ipaddr_copy(&dest_addr, globaladdr);
	        memcpy(&dest_addr.u8[8], &defrt->u8[8], sizeof(uip_ipaddr_t) / 2);
	        has_dest=1;
	      }
	#endif
	    }
	  }

	  if (has_dest) {
	    if (client_conn == NULL) {
	      PRINTF("UDP-CLIENT: address destination: ");
	      PRINT6ADDR(&dest_addr);
	      PRINTF("\n");
	      client_conn = udp_new(&dest_addr, UIP_HTONS(dest_port), NULL);

	      if (client_conn != NULL) {
	        PRINTF("Created a connection with the server ");
	        PRINT6ADDR(&client_conn->ripaddr);
	        PRINTF(" local/remote port %u/%u\n",
	          UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));
	      } else {
	        PRINTF("Could not open connection\n");
	      }
	    } else {
	      if(memcmp(&client_conn->ripaddr, &dest_addr, sizeof(uip_ipaddr_t)) != 0) {
	        PRINTF("UDP-CLIENT: new address destination: ");
	        PRINT6ADDR(&dest_addr);
	        PRINTF("\n");
	        uip_udp_remove(client_conn);
	        client_conn = udp_new(&dest_addr, UIP_HTONS(dest_port), NULL);
	        if (client_conn != NULL) {
	          PRINTF("Created a connection with the server ");
	          PRINT6ADDR(&client_conn->ripaddr);
	          PRINTF(" local/remote port %u/%u\n",
	            UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));
	        } else {
	          PRINTF("Could not open connection\n");
	        }
	      }
	    }
	    if (client_conn != NULL) {
	      if ( udp_client_run ) {
	        PRINTF("Client sending to: ");
	        PRINT6ADDR(&client_conn->ripaddr);
	        i = sprintf(buf, "%d | ", ++seq_id);
	#if UIP_CONF_IPV6_RPL
	        rpl_dag_t *dag = rpl_get_any_dag();
	        if(dag && dag->instance->def_route) {
	          add_ipaddr(buf + i, &dag->instance->def_route->ipaddr);
	        } else {
	          sprintf(buf + i, "(null)");
	        }
	#else
	        if (defrt != NULL) {
	          add_ipaddr(buf + i, defrt);
	        } else {
	          sprintf(buf + i, "(null)");
	        }
	#endif
	        PRINTF(" (msg: %s)\n", buf);
	        #if SEND_TOO_LARGE_PACKET_TO_TEST_FRAGMENTATION
	        uip_udp_packet_send(client_conn, buf, UIP_APPDATA_SIZE);
	        #else /* SEND_TOO_LARGE_PACKET_TO_TEST_FRAGMENTATION */
	        uip_udp_packet_send(client_conn, buf, strlen(buf));
	        #endif /* SEND_TOO_LARGE_PACKET_TO_TEST_FRAGMENTATION */
	      }
	    } else {
	      PRINTF("No connection created\n");
	    }
	  } else {
	    PRINTF("No address configured\n");
	  }
} /* _demo_udp_sendMsg */

/*----------------------------------------------------------------------------*/
/** \brief  This function is called whenever the timer expired.
 *
 *  \param  event 	Event type
 *  \param	data	Pointer to data
 *
 *  \returns none
 */
/*----------------------------------------------------------------------------*/
static	void _demo_udp_callback(c_event_t c_event, p_data_t p_data) {
	if (etimer_expired(&st_et))
	{
		_6lbr_udp_sendMsg();
		etimer_restart(&st_et);
	}
}

/*==============================================================================
 	 	 	 	 	 	 	 	 API FUNCTIONS
 =============================================================================*/

uint8_t demo_udpAliveConf(s_ns_t* pst_netStack)
{
	uint8_t c_ret = 1;
	/*
	 * By default stack
	 */
    if (pst_netStack != NULL) {
    	if (!pst_netStack->c_configured) {
        	pst_netStack->hc     = &sicslowpan_driver;
        	pst_netStack->hmac   = &nullmac_driver;
        	pst_netStack->lmac   = &sicslowmac_driver;
        	pst_netStack->frame  = &framer_802154;
        	pst_netStack->c_configured = 1;
            /* Transceiver interface is defined by @ref board_conf function*/
            /* pst_netStack->inif   = $<some_transceiver>;*/
    	} else {
            if ((pst_netStack->hc == &sicslowpan_driver)   &&
            	(pst_netStack->hmac == &nullmac_driver)    &&
            	(pst_netStack->lmac == &sicslowmac_driver) &&
            	(pst_netStack->frame == &framer_802154)) {
            	/* right configuration */
            }
            else {
                c_ret = 0;
            }
    	}
    }
    return c_ret;
}

/*----------------------------------------------------------------------------*/
/*	demo_udp_init() 														  */
/*----------------------------------------------------------------------------*/

int8_t demo_udpAliveInit(void)
{
	/* set periodic timer */
	etimer_set(&st_et, SEND_INTERVAL * bsp_get(E_BSP_GET_TRES), _demo_udp_callback);
	/* set udp is running */
	udp_client_run = 1;
	return 1;
}/* demo_udpAliveInit()  */
/** @} */
/** @} */
/** @} */
