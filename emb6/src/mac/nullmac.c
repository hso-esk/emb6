/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: nullmac.c,v 1.15 2010/06/14 19:19:17 adamdunkels Exp $
 */

/**
 * \file
 *         A MAC protocol that does not do anything.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "emb6_conf.h"
#include "emb6.h"

static s_ns_t*    p_ns = NULL;

/*---------------------------------------------------------------------------*/
static void send_packet(mac_callback_t sent, void *ptr)
{
    if ((p_ns != NULL) && (p_ns->lmac != NULL))
        p_ns->lmac->send(sent, ptr);
}
/*---------------------------------------------------------------------------*/
static void packet_input(void)
{
    if ((p_ns != NULL) && (p_ns->llsec != NULL))
        p_ns->llsec->input();
}
/*---------------------------------------------------------------------------*/
static int8_t on(void)
{
    if ((p_ns != NULL) && (p_ns->lmac != NULL))
        return p_ns->lmac->on();
    else
        return 0;
}
/*---------------------------------------------------------------------------*/
static int8_t off(int keep_radio_on)
{
    if ((p_ns != NULL) && (p_ns->lmac != NULL))
        return p_ns->lmac->on();
    else
        return 0;
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static void init(s_ns_t* p_netStatck)
{
    if (p_netStatck != NULL)
        p_ns = p_netStatck;
}
/*---------------------------------------------------------------------------*/
const s_nsHighMac_t nullmac_driver = {
  "nullmac",
  init,
  send_packet,
  packet_input,
  on,
  off,
  channel_check_interval,
};
/*---------------------------------------------------------------------------*/
