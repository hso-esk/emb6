/*
 * Copyright (c) 2012-2014, Thingsquare, http://www.thingsquare.com/.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "emb6.h"

#include "bsp.h"
#include "clist.h"
#include "etimer.h"
#include "tcpip.h"
#include "uip-udp-packet.h"
#include "udp-socket.h"


void _udp_sock_callback(c_event_t c_event, p_data_t p_data);

static uint8_t buf[UIP_BUFSIZE];

#define UIP_IP_BUF   ((struct uip_udpip_hdr *)&uip_buf[UIP_LLH_LEN])

#define     LOGGER_ENABLE        LOGGER_CORE
#if            LOGGER_ENABLE     ==     TRUE
#define        LOGGER_SUBSYSTEM    "core"
#endif
#include    "logger.h"

/*---------------------------------------------------------------------------*/
static void
init(void)
{

  evproc_unregCallback(EVENT_TYPE_TCPIP,_udp_sock_callback);
  evproc_regCallback(EVENT_TYPE_TCPIP,_udp_sock_callback);

}
/*---------------------------------------------------------------------------*/
int
udp_socket_register(struct udp_socket *c,
                    void *ptr,
                    udp_socket_input_callback_t input_callback)
{
  init();

  if(c == NULL) {
    return -1;
  }
  c->ptr = ptr;
  c->input_callback = input_callback;

  c->udp_conn = udp_new(NULL, 0, c);
  LOG_INFO("socket ptr = %p:%p", c, (void *)input_callback);

  if(c->udp_conn == NULL) {
    return -1;
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
int
udp_socket_close(struct udp_socket *c)
{
  if(c == NULL) {
    return -1;
  }
  if(c->udp_conn != NULL) {
    uip_udp_remove(c->udp_conn);
    return 1;
  }
  return -1;
}
/*---------------------------------------------------------------------------*/
int
udp_socket_bind(struct udp_socket *c,
                uint16_t local_port)
{
  if(c == NULL || c->udp_conn == NULL) {
    return -1;
  }
  udp_bind(c->udp_conn, UIP_HTONS(local_port));

  return 1;
}
/*---------------------------------------------------------------------------*/
int
udp_socket_connect(struct udp_socket *c,
                   uip_ipaddr_t *remote_addr,
                   uint16_t remote_port)
{
  if(c == NULL || c->udp_conn == NULL) {
    return -1;
  }

  if(remote_addr != NULL) {
    uip_ipaddr_copy(&c->udp_conn->ripaddr, remote_addr);
  }
  c->udp_conn->rport = UIP_HTONS(remote_port);
  return 1;
}
/*---------------------------------------------------------------------------*/
int
udp_socket_send(struct udp_socket *c,
                const void *data, uint16_t datalen)
{
  if(c == NULL || c->udp_conn == NULL) {
    return -1;
  }

  uip_udp_packet_send(c->udp_conn, data, datalen);
  return datalen;
}
/*---------------------------------------------------------------------------*/
int
udp_socket_sendto(struct udp_socket *c,
                  const void *data, uint16_t datalen,
                  const uip_ipaddr_t *to,
                  uint16_t port)
{
  if(c == NULL || c->udp_conn == NULL) {
    return -1;
  }

  if(c->udp_conn != NULL) {
    uip_udp_packet_sendto(c->udp_conn, data, datalen,
                          to, UIP_HTONS(port));
    return datalen;
  }
  return -1;
}
/*---------------------------------------------------------------------------*/
void _udp_sock_callback(c_event_t c_event, p_data_t p_data)
{
    struct udp_socket *c;

    if(c_event == EVENT_TYPE_TCPIP) {
        /* An appstate pointer is passed to use from the IP stack
           through the 'data' pointer. We registered this appstate when
           we did the udp_new() call in udp_socket_register() as the
           struct udp_socket pointer. So we extract this
           pointer and use it when calling the reception callback. */
        c = (struct udp_socket *)p_data;

        /* Defensive coding: although the appstate *should* be non-null
           here, we make sure to avoid the program crashing on us. */
        if(c != NULL) {
            LOG_INFO("socket ptr from callback = %p:%p", c, (void *)(c->input_callback));
            /* If we were called because of incoming data, we should call
               the reception callback. */
            if(uip_newdata()) {
                /* Copy the data from the uIP data buffer into our own
                   buffer to avoid the uIP buffer being messed with by the
                   callee. */
                memcpy(buf, uip_appdata, uip_datalen());

                /* Call the client process. We use the PROCESS_CONTEXT
                   mechanism to temporarily switch process context to the
                   client process. */
                if(c->input_callback != NULL) {
                    c->input_callback(c, c->ptr,
                                      &(UIP_IP_BUF->srcipaddr),
                                      UIP_HTONS(UIP_IP_BUF->srcport),
                                      &(UIP_IP_BUF->destipaddr),
                                      UIP_HTONS(UIP_IP_BUF->destport),
                                      buf, uip_datalen());
                }
            }
        }
    }
}
/*---------------------------------------------------------------------------*/
const s_nsSocket_t udp_socket_driver = {
        "udp_bsd_socket",
        NULL, /* udp_socket_register */
        NULL, /* udp_socket_connect */
        NULL, /* udp_socket_bind */
        NULL, /* udp_socket_send */
        NULL, /* udp_socket_sendto */
        NULL, /* udp_socket_close */
};
/*---------------------------------------------------------------------------*/
/** @} */
