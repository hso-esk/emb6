/*
 * Copyright (c) 2011, Swedish Institute of Computer Science
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
 * Sets up some commands for the RF230 radio.
 */
/**
 * \file
 *         A brief description of what this file is
 */

#include "emb6.h"
#include "cmd.h"


#define DEBUG DEBUG_NONE
#if DEBUG
#include <stdio.h>
#define PRINTF(FORMAT,args...) printf_P(PSTR(FORMAT),##args)
#define PRINTSHORT(FORMAT,args...) printf_P(PSTR(FORMAT),##args)
#else
#define PRINTF(...)
#define PRINTSHORT(...)
#endif

static int chan_num = 26;

int
cmd_handler_rf(const uint8_t *data, int len)
{
  e_nsErr_t err;
  const s_ns_t* ps_stack = emb6_get();
  if( (ps_stack == NULL) || (ps_stack->rf == NULL) )
    return 0;

  if(data[0] == '!') {
    if(data[1] == 'C' && len == 3) {
      PRINTF("CMD: Setting channel: %d\n", data[2]);
      ps_stack->rf->ioctrl(NETSTK_CMD_RF_CHAN_NUM_SET, (void*)&data[2], &err );
      if( err == NETSTK_ERR_NONE )
      {
        chan_num = data[2];
        return 1;
      }
      else
        return 0;
    }
  } else if(data[0] == '?') {
    if(data[1] == 'C' && len == 2) {
      uint8_t buf[4];
      PRINTF("CMD: Getting channel: %d\n", data[2]);
      buf[0] = '!';
      buf[1] = 'C';
      buf[2] = chan_num;
      cmd_send(buf, 3);
      return 1;
    }
  }
  return 0;
}
