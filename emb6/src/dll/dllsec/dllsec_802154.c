/*
 * Copyright (c) 2013, Hasso-Plattner-Institut.
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
 */

/**
 * \file
 *         Insecure link layer security driver.
 * \author
 *         Konrad Krentz <konrad.krentz@gmail.com>
 */

/**
 * \addtogroup dllsec_802154
 * @{
 */

/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"
#include "dllsec_null.h"
#include "dllsec_802154.h"
#include "framer_802154.h"
#include "packetbuf.h"
#include "ccm-star.h"

#define     LOGGER_ENABLE        LOGGER_LLCSEC
#include    "logger.h"




/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/

static s_ns_t *pdllsec_netstk;
static mac_callback_t dllsec_txCbFnct;

#if LLSEC802154_ENABLED && LLSEC802154_USES_AUX_HEADER
static frame802154_frame_counter_t counter;
static uint8_t Key1[] =  {0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF};
//static uint8_t Key2[] = {0x90, 0x92, 0x9a, 0x4b, 0x0a, 0xc6, 0x5b, 0x35, 0x0a, 0xd1, 0x59, 0x16, 0x11, 0xfe, 0x48, 0x29};
#endif /* LLSEC802154_ENABLED && LLSEC802154_USES_AUX_HEADER */
/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
static void dllsec_cbTx(void *p_arg, e_nsErr_t *p_err);
static void dllsec_send(mac_callback_t sent, void *p_arg);
static int dllsec_onFrameCreated(void);
static void dllsec_input(void);
static uint8_t dllsec_getOverhead(void);
static void dllsec_init(s_ns_t *p_netstk);
#if LLSEC802154_ENABLED && LLSEC802154_USES_AUX_HEADER
static void dllsec_security(void);
/* storing the received frame counter value */
static uint32_t dllsec_receivedFrameCounter(void);
/* Comparing the received frame counter value to the expected frame counter value */
static _Bool dllsec_receivedFrameCounterCheck(uint32_t revCounter);
#endif /*LLSEC802154_ENABLED && LLSEC802154_USES_AUX_HEADER*/

/*---------------------------------------------------------------------------*/

/*
 * @brief   Transmission callback function handler
 *
 * @param   p_arg
 * @param   p_err
 */
static void dllsec_cbTx(void *p_arg, e_nsErr_t *p_err)
{
  int status;
  int retx = 0;

  switch (*p_err) {
    case NETSTK_ERR_NONE:
      status = MAC_TX_OK;
      retx = 1;
      break;

    case NETSTK_ERR_CHANNEL_ACESS_FAILURE:
      status = MAC_TX_COLLISION;
      retx = 0;
      break;

    case NETSTK_ERR_TX_NOACK:
      status = MAC_TX_NOACK;
      retx = 1;
      break;

    case NETSTK_ERR_BUSY:
      status = MAC_TX_DEFERRED;
      retx = 0;
      break;

    default:
      status = MAC_TX_ERR_FATAL;
      retx = 0;
      break;
  }

  if (dllsec_txCbFnct != NULL) {
    dllsec_txCbFnct(p_arg, status, retx);
    dllsec_txCbFnct = NULL;
  }
}


#if LLSEC802154_ENABLED && LLSEC802154_USES_AUX_HEADER
/*---------------------------------------------------------------------------*/
/* storing the received frame counter value
 */
static uint32_t dllsec_receivedFrameCounter(void)
{
	frame802154_frame_counter_t recCounter;
	recCounter.u8[3] = packetbuf_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_0_1) >> 8;
	recCounter.u8[2] = packetbuf_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_0_1) & 0xff;
	recCounter.u8[1] = packetbuf_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_2_3) >> 8;
	recCounter.u8[0] = packetbuf_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_2_3) & 0xff;

	return recCounter.u32;
}

/*-----------------------------------------+----------------------------------*/
/* Comparing the received frame counter value to the expected frame counter value. Expected frame counter
 * value is accessed from the Access Control List based on the source address.
 */
static _Bool dllsec_receivedFrameCounterCheck(uint32_t revCounter)
{
	static uint32_t lastFrameCounterVal;
	uint8_t source_addr[8];
	linkaddr_copy((linkaddr_t *)&source_addr, packetbuf_addr(PACKETBUF_ADDR_SENDER));
	#if LOGGER_ENABLE
		LOG_INFO("source Address");
		LOG2_HEXDUMP(&source_addr, 8);

		LOG_INFO("revCounter from dllsec_input function");
		LOG2_HEXDUMP(&revCounter, 4);
	#endif
	/* Searching for entry in Access Control List
	 * if entry found, fetching the last frame counter value from Access Control List
	 */
	for(uint8_t count = 0; count< MAX_ENTRY;count++)
	{
		_Bool equal = linkaddr_cmp((linkaddr_t *)&database[count].src_addr,(linkaddr_t *)&source_addr);
		if(equal)
		{
			lastFrameCounterVal = database[count].last_frame_counter_value.u32;
			#if LOGGER_ENABLE
				LOG_INFO("source address matched!");
				LOG_INFO("lastFrameCounterVal before comparing");
				LOG2_HEXDUMP(&lastFrameCounterVal, 4);
			#endif
		}
	}
		/*
		 * checking if the received frame counter value is less than the expected frame counter value
		 */
		if(revCounter <= lastFrameCounterVal)
		{
			#if LOGGER_ENABLE
				LOG_INFO("Received Counter value is invalid");
				LOG_INFO("lastFrameCounterVal");
				LOG2_HEXDUMP(&lastFrameCounterVal, 4);
			#endif
			return 1;
		}
		else
		{
			lastFrameCounterVal = revCounter;
			for(uint8_t count = 0; count< MAX_ENTRY;count++)
			{
				_Bool equal = linkaddr_cmp((linkaddr_t *)&database[count].src_addr,(linkaddr_t *)&source_addr);
				if(equal)
				{
					database[count].last_frame_counter_value.u32 = lastFrameCounterVal;
				}
			}
			#if LOGGER_ENABLE
				LOG_INFO("Received Counter value is valid");
		    	LOG_INFO("last Counter value updated");
		    	LOG2_HEXDUMP(&lastFrameCounterVal, 4);
			#endif
		}
	return 0;
}

/*---------------------------------------------------------------------------*/
/* setting of security level and frame counter values of Auxiliary Security Header in packetbuf attributes
 */
static void dllsec_security(void)
{
	/* Assigning the frame counter values in the corresponding packetbuf attributes */
    #if LLSEC802154_USES_FRAME_COUNTER
	packetbuf_set_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_0_1, counter.u16[0]);
	packetbuf_set_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_2_3, counter.u16[1]);
    #endif /* LLSEC802154_USES_FRAME_COUNTER */

    #if LLSEC802154_SECURITY_LEVEL
    /* assigning the security level in the corresponding packetbuf attributes */
    packetbuf_set_attr(PACKETBUF_ATTR_SECURITY_LEVEL, SEC_LVL);
    #endif  /* LLSEC802154_SECURITY_LEVEL*/
}
/*---------------------------------------------------------------------------*/
static void dllsec_input(void)
{
	uint32_t CounterValReceived;
	_Bool replayAttack;
	CounterValReceived = dllsec_receivedFrameCounter();
	replayAttack = dllsec_receivedFrameCounterCheck(CounterValReceived);
	/* if frame counter value is less than expected, discarding the frame */
	if(replayAttack == true)
	{
		#if LOGGER_ENABLE
		LOG_INFO("REPLAY ATTACK!!");
			LOG_INFO("Data not passed to the higher layer");
		#endif
	}
	else
	{
		/* if frame counter value is greater than the last frame counter value,
		 *  passing the frame to higher layer*/
		pdllsec_netstk->hc->input();
		#if LOGGER_ENABLE
			LOG_INFO("Data passed to the higher layer");
		#endif
	}
}

#endif /* LLSEC802154_ENABLED && LLSEC802154_USES_AUX_HEADER */

/*---------------------------------------------------------------------------*/
static void dllsec_send(mac_callback_t sent, void *p_arg)
{
  e_nsErr_t err = NETSTK_ERR_NONE;

  dllsec_txCbFnct = sent;
  packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, FRAME802154_DATAFRAME);

#if LLSEC802154_ENABLED && LLSEC802154_USES_AUX_HEADER
  if (SEC_LVL != FRAME802154_SECURITY_LEVEL_NONE)
  {
	  /* Assigning security key at the sender side */
	  frame802154_securityKey(Key1);
	  /* Setting of security level and frame counter values of Auxiliary Security Header */
	  dllsec_security();
	  /* Incrementing the frame counter values of Auxiliary Security Header */
	  counter.u32 = counter.u32+1;
  }
#endif /*LLSEC802154_ENABLED && LLSEC802154_USES_AUX_HEADER*/
  /*
   * Issue next lower layer to transmit the prepared packet
   */
  pdllsec_netstk->dllc->send(packetbuf_hdrptr(), packetbuf_totlen(), &err );
  if (err != NETSTK_ERR_NONE) {
    TRACE_LOG_ERR("<DLLS> e=-%d", err);
  }

  /* inform upper layer of the TX status */
  dllsec_cbTx(p_arg, &err);
}

/*---------------------------------------------------------------------------*/
static int dllsec_onFrameCreated(void)
{
  return 1;
}


/*---------------------------------------------------------------------------*/
static uint8_t dllsec_getOverhead(void)
{
  return 0;
}

/*---------------------------------------------------------------------------*/
static void dllsec_init(s_ns_t *p_netstk)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_netstk == NULL) {
    return;
  }
#endif
  e_nsErr_t err = NETSTK_ERR_NONE;

  pdllsec_netstk = p_netstk;
#if LLSEC802154_ENABLED && LLSEC802154_USES_AUX_HEADER
  pdllsec_netstk->dllc->ioctrl(NETSTK_CMD_RX_CBFNT_SET, (void *) dllsec_input, &err);
  /* Initialising the value of frame counter of Auxiliary Security Header */
  counter.u32 = 0;

  /*Initialising the ACL for Replay Protection*/
  frame802154_replayProtectionDatabase();
#endif /* LLSEC802154_ENABLED && LLSEC802154_USES_AUX_HEADER */
}

/*---------------------------------------------------------------------------*/
const s_nsDllsec_t dllsec_driver_802154 =
{
 "LLSEC 802154",
  dllsec_init,
  dllsec_send,
  dllsec_onFrameCreated,
  dllsec_input,
  dllsec_getOverhead
};
/*---------------------------------------------------------------------------*/

/** @} */
