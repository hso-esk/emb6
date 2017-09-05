/*
 * Copyright (c) 2015, SICS Swedish ICT.
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
 *         IEEE 802.15.4 TSCH MAC implementation.
 *         Does not use any RDC layer. Should be used with nordc.
 * \author
 *         Simon Duquennoy <simonduq@sics.se>
 *         Beshr Al Nahas <beshr@sics.se>
 *
 */

#include "emb6.h"
#include "packetbuf.h"
#include "queuebuf.h"
#include "nbr-table.h"
#include "tsch.h"
#include "tsch-slot-operation.h"
#include "tsch-queue.h"
#include "tsch-private.h"
#include "tsch-log.h"
#include "tsch-packet.h"
#include "tsch-security.h"
#include "mac-sequence.h"
#include "random.h"
#include "bsp.h"
#include "ctimer.h"
#include "evproc.h"

/***********************/

 s_ns_t  *pmac_netstk;


#define TISCH_ASSOCIATE_EVENT_POST()     evproc_putEvent(E_EVPROC_HEAD, EVENT_TYPE_TISCH_PROCESS, NULL)
#define TISCH_REG_PROCESS_HANDLER()      evproc_regCallback(EVENT_TYPE_TISCH_PROCESS, tsch_process)

#define TISCH_PENDING_TX_RX_EVENT_POST()     evproc_putEvent(E_EVPROC_HEAD, EVENT_TYPE_TISCH_TX_RX_PENDING, NULL)
#define TISCH_REG_PENDING_TX_RX_HANDLER()    evproc_regCallback(EVENT_TYPE_TISCH_TX_RX_PENDING, tsch_pending_events_process)

static void tsch_send_eb_process_start(void);
static void tsch_send_eb_process_stop(void);
static void tsch_send_eb_process (void *ptr);
static void tsch_pending_events_process(c_event_t c_event, p_data_t p_data);

/**********************/

#if FRAME802154_VERSION < FRAME802154_IEEE802154E_2012
#error TSCH: FRAME802154_VERSION must be at least FRAME802154_IEEE802154E_2012
#endif

#if TSCH_LOG_LEVEL >= 1
#define DEBUG DEBUG_PRINT
#else /* TSCH_LOG_LEVEL */
#define DEBUG DEBUG_NONE
#endif /* TSCH_LOG_LEVEL */
#include "net/net-debug.h"

/* Use to collect link statistics even on Keep-Alive, even though they were
 * not sent from an upper layer and don't have a valid packet_sent callback */
#ifndef TSCH_LINK_NEIGHBOR_CALLBACK
#if NETSTACK_CONF_WITH_IPV6
void uip_ds6_link_neighbor_callback(int status, int numtx);
#define TSCH_LINK_NEIGHBOR_CALLBACK(dest, status, num) uip_ds6_link_neighbor_callback(status, num)
#endif /* NETSTACK_CONF_WITH_IPV6 */
#endif /* TSCH_LINK_NEIGHBOR_CALLBACK */

/* Let TSCH select a time source with no help of an upper layer.
 * We do so using statistics from incoming EBs */
#if TSCH_AUTOSELECT_TIME_SOURCE
int best_neighbor_eb_count;
struct eb_stat {
  int rx_count;
  int jp;
};
NBR_TABLE(struct eb_stat, eb_stats);
#endif /* TSCH_AUTOSELECT_TIME_SOURCE */

/* TSCH channel hopping sequence */
uint8_t tsch_hopping_sequence[TSCH_HOPPING_SEQUENCE_MAX_LEN];
struct tsch_asn_divisor_t tsch_hopping_sequence_length;

/* Default TSCH timeslot timing (in micro-second) */
static const uint16_t tsch_default_timing_us[tsch_ts_elements_count] = {
  TSCH_DEFAULT_TS_CCA_OFFSET,
  TSCH_DEFAULT_TS_CCA,
  TSCH_DEFAULT_TS_TX_OFFSET,
  TSCH_DEFAULT_TS_RX_OFFSET,
  TSCH_DEFAULT_TS_RX_ACK_DELAY,
  TSCH_DEFAULT_TS_TX_ACK_DELAY,
  TSCH_DEFAULT_TS_RX_WAIT,
  TSCH_DEFAULT_TS_ACK_WAIT,
  TSCH_DEFAULT_TS_RX_TX,
  TSCH_DEFAULT_TS_MAX_ACK,
  TSCH_DEFAULT_TS_MAX_TX,
  TSCH_DEFAULT_TS_TIMESLOT_LENGTH,
};
/* TSCH timeslot timing (in rtimer ticks) */
rtimer_clock_t tsch_timing[tsch_ts_elements_count];

#if LINKADDR_SIZE == 8
/* 802.15.4 broadcast MAC address  */
const linkaddr_t tsch_broadcast_address = { { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff } };
/* Address used for the EB virtual neighbor queue */
const linkaddr_t tsch_eb_address = { { 0, 0, 0, 0, 0, 0, 0, 0 } };
#else /* LINKADDR_SIZE == 8 */
const linkaddr_t tsch_broadcast_address = { { 0xff, 0xff } };
const linkaddr_t tsch_eb_address = { { 0, 0 } };
#endif /* LINKADDR_SIZE == 8 */

/* Is TSCH started? */
int tsch_is_started = 0;
/* Has TSCH initialization failed? */
int tsch_is_initialized = 0;
/* Are we coordinator of the TSCH network? */
int tsch_is_coordinator = 0;
/* Are we associated to a TSCH network? */
int tsch_is_associated = 0;
/* Is the PAN running link-layer security? */
int tsch_is_pan_secured = LLSEC802154_ENABLED;
/* The current Absolute Slot Number (ASN) */
struct tsch_asn_t tsch_current_asn;
/* Device rank or join priority:
 * For PAN coordinator: 0 -- lower is better */
uint8_t tsch_join_priority;
/* The current TSCH sequence number, used for unicast data frames only */
static uint8_t tsch_packet_seqno = 0;
/* Current period for EB output */
static clock_time_t tsch_current_eb_period;
/* Current period for keepalive output */
static clock_time_t tsch_current_ka_timeout;

/* timer for sending keepalive messages */
static struct ctimer keepalive_timer;

/* Other function prototypes */
static void packet_input(void);

/* Getters and setters */
/*---------------------------------------------------------------------------*/
void
tsch_set_coordinator(int enable)
{
  tsch_is_coordinator = enable;
  tsch_set_eb_period(TSCH_EB_PERIOD);
}
/*---------------------------------------------------------------------------*/
void
tsch_set_pan_secured(int enable)
{
  tsch_is_pan_secured = LLSEC802154_ENABLED && enable;
}
/*---------------------------------------------------------------------------*/
void
tsch_set_join_priority(uint8_t jp)
{
  tsch_join_priority = jp;
}
/*---------------------------------------------------------------------------*/
void
tsch_set_ka_timeout(uint32_t timeout)
{
  tsch_current_ka_timeout = timeout;
}
/*---------------------------------------------------------------------------*/
void
tsch_set_eb_period(uint32_t period)
{
  tsch_current_eb_period = MIN(period, TSCH_MAX_EB_PERIOD);
}
/*---------------------------------------------------------------------------*/

 void mac_call_sent_callback(mac_callback_t sent, void *ptr, int status, int num_tx)
{
  PRINTF("mac_callback_t %p ptr %p status %d num_tx %d\n",
         (void *)sent, ptr, status, num_tx);
  switch(status) {
  case MAC_TX_COLLISION:
    PRINTF("mac: collision after %d tx\n", num_tx);
    break;
  case MAC_TX_NOACK:
    PRINTF("mac: noack after %d tx\n", num_tx);
    break;
  case MAC_TX_OK:
    PRINTF("mac: sent after %d tx\n", num_tx);
    break;
  default:
    PRINTF("mac: error %d after %d tx\n", status, num_tx);
  }

  if(sent) {
    sent(ptr, status, num_tx);
  }
}

/*----------------------------------------------------------------------------*/

static void
tsch_reset(void)
{
  int i;
  frame802154_set_pan_id(0xffff);
  /* First make sure pending packet callbacks are sent etc */
  tsch_pending_events_process_start_now();
  /* Reset neighbor queues */
  tsch_queue_reset();
  /* Remove unused neighbors */
  tsch_queue_free_unused_neighbors();
  tsch_queue_update_time_source(NULL);
  /* Initialize global variables */
  tsch_join_priority = 0xff;
  TSCH_ASN_INIT(tsch_current_asn, 0, 0);
  current_link = NULL;
  /* Reset timeslot timing to defaults */
  for(i = 0; i < tsch_ts_elements_count; i++) {
    tsch_timing[i] = (rtimer_clock_t) bsp_us_to_rtimerTiscks(tsch_default_timing_us[i]);
  }
#ifdef TSCH_CALLBACK_LEAVING_NETWORK
  TSCH_CALLBACK_LEAVING_NETWORK();
#endif
#if TSCH_AUTOSELECT_TIME_SOURCE
  best_neighbor_eb_count = 0;
  nbr_table_register(eb_stats, NULL);
  tsch_set_eb_period(TSCH_EB_PERIOD);
#endif
}

/* TSCH keep-alive functions */

/*---------------------------------------------------------------------------*/
/* Tx callback for keepalive messages */
static void
keepalive_packet_sent(void *ptr, int status, int transmissions)
{
#ifdef TSCH_LINK_NEIGHBOR_CALLBACK
  TSCH_LINK_NEIGHBOR_CALLBACK(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), status, transmissions);
#endif
  PRINTF("TSCH: KA sent to %u, st %d-%d\n",
         TSCH_LOG_ID_FROM_LINKADDR(packetbuf_addr(PACKETBUF_ADDR_RECEIVER)), status, transmissions);
  tsch_schedule_keepalive();
}
/*---------------------------------------------------------------------------*/
/* Prepare and send a keepalive message */
static void
keepalive_send(void *ptr)
{
  if(tsch_is_associated) {
    struct tsch_neighbor *n = tsch_queue_get_time_source();
    /* Simply send an empty packet */
    packetbuf_clear();
    packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &n->addr);
    pmac_netstk->dllsec->send(keepalive_packet_sent, NULL);
    PRINTF("TSCH: sending KA to %u\n",
           TSCH_LOG_ID_FROM_LINKADDR(&n->addr));
  }
}
/*---------------------------------------------------------------------------*/
/* Set ctimer to send a keepalive message after expiration of TSCH_KEEPALIVE_TIMEOUT */
void
tsch_schedule_keepalive()
{
  /* Pick a delay in the range [tsch_current_ka_timeout*0.9, tsch_current_ka_timeout[ */
  if(!tsch_is_coordinator && tsch_is_associated && tsch_current_ka_timeout > 0) {
    unsigned long delay = (tsch_current_ka_timeout - tsch_current_ka_timeout / 10)
      + random_rand() % (tsch_current_ka_timeout / 10);
    ctimer_set(&keepalive_timer, delay, keepalive_send, NULL);
  }
}
/*---------------------------------------------------------------------------*/
static void
eb_input(struct input_packet *current_input)
{
  /* PRINTF("TSCH: EB received\n"); */
  frame802154_t frame;
  /* Verify incoming EB (does its ASN match our Rx time?),
   * and update our join priority. */
  struct ieee802154_ies eb_ies;

  if(tsch_packet_parse_eb(current_input->payload, current_input->len,
                          &frame, &eb_ies, NULL, 1)) {
    /* PAN ID check and authentication done at rx time */

#if TSCH_AUTOSELECT_TIME_SOURCE
    if(!tsch_is_coordinator) {
      /* Maintain EB received counter for every neighbor */
      struct eb_stat *stat = (struct eb_stat *)nbr_table_get_from_lladdr(eb_stats, (linkaddr_t *)&frame.src_addr);
      if(stat == NULL) {
        stat = (struct eb_stat *)nbr_table_add_lladdr(eb_stats, (linkaddr_t *)&frame.src_addr, NBR_TABLE_REASON_MAC, NULL);
      }
      if(stat != NULL) {
        stat->rx_count++;
        stat->jp = eb_ies.ie_join_priority;
        best_neighbor_eb_count = MAX(best_neighbor_eb_count, stat->rx_count);
      }
      /* Select best time source */
      struct eb_stat *best_stat = NULL;
      stat = nbr_table_head(eb_stats);
      while(stat != NULL) {
        /* Is neighbor eligible as a time source? */
        if(stat->rx_count > best_neighbor_eb_count / 2) {
          if(best_stat == NULL ||
             stat->jp < best_stat->jp) {
            best_stat = stat;
          }
        }
        stat = nbr_table_next(eb_stats, stat);
      }
      /* Update time source */
      if(best_stat != NULL) {
        tsch_queue_update_time_source(nbr_table_get_lladdr(eb_stats, best_stat));
        tsch_join_priority = best_stat->jp + 1;
      }
    }
#endif

    struct tsch_neighbor *n = tsch_queue_get_time_source();
    /* Did the EB come from our time source? */
    if(n != NULL && linkaddr_cmp((linkaddr_t *)&frame.src_addr, &n->addr)) {
      /* Check for ASN drift */
      int32_t asn_diff = TSCH_ASN_DIFF(current_input->rx_asn, eb_ies.ie_asn);
      if(asn_diff != 0) {
        /* We disagree with our time source's ASN -- leave the network */
        PRINTF("TSCH:! ASN drifted by %ld, leaving the network\n", asn_diff);
        tsch_disassociate();
      }

      if(eb_ies.ie_join_priority >= TSCH_MAX_JOIN_PRIORITY) {
        /* Join priority unacceptable. Leave network. */
        PRINTF("TSCH:! EB JP too high %u, leaving the network\n",
               eb_ies.ie_join_priority);
        tsch_disassociate();
      } else {
#if TSCH_AUTOSELECT_TIME_SOURCE
        /* Update join priority */
        if(tsch_join_priority != eb_ies.ie_join_priority + 1) {
          PRINTF("TSCH: update JP from EB %u -> %u\n",
                 tsch_join_priority, eb_ies.ie_join_priority + 1);
          tsch_join_priority = eb_ies.ie_join_priority + 1;
        }
#endif /* TSCH_AUTOSELECT_TIME_SOURCE */
      }
    }
  }
}

/*---------------------------------------------------------------------------*/
/* Process pending input packet(s) */
static void
tsch_rx_process_pending()
{
  int16_t input_index;
  /* Loop on accessing (without removing) a pending input packet */
  while((input_index = ringbufindex_peek_get(&input_ringbuf)) != -1) {
    struct input_packet *current_input = &input_array[input_index];
    frame802154_t frame;
    uint8_t ret = frame802154_parse(current_input->payload, current_input->len, &frame);
    int is_data = ret && frame.fcf.frame_type == FRAME802154_DATAFRAME;
    int is_eb = ret
      && frame.fcf.frame_version == FRAME802154_IEEE802154E_2012
      && frame.fcf.frame_type == FRAME802154_BEACONFRAME;

    if(is_data) {
      /* Skip EBs and other control messages */
      /* Copy to packetbuf for processing */
      packetbuf_copyfrom(current_input->payload, current_input->len);
      packetbuf_set_attr(PACKETBUF_ATTR_RSSI, current_input->rssi);
      packetbuf_set_attr(PACKETBUF_ATTR_CHANNEL, current_input->channel);
    }

    /* Remove input from ringbuf */
    ringbufindex_get(&input_ringbuf);

    if(is_data) {
      /* Pass to upper layers */
      packet_input();
    } else if(is_eb) {
      eb_input(current_input);
    }
  }
}

/*---------------------------------------------------------------------------*/
/* Pass sent packets to upper layer */
static void
tsch_tx_process_pending()
{
  int16_t dequeued_index;
  /* Loop on accessing (without removing) a pending input packet */
  while((dequeued_index = ringbufindex_peek_get(&dequeued_ringbuf)) != -1) {
    struct tsch_packet *p = dequeued_array[dequeued_index];
    /* Put packet into packetbuf for packet_sent callback */
    queuebuf_to_packetbuf(p->qb);
    /* Call packet_sent callback */
    mac_call_sent_callback(p->sent, p->ptr, p->ret, p->transmissions);
    /* Free packet queuebuf */
    tsch_queue_free_packet(p);
    /* Free all unused neighbors */
    tsch_queue_free_unused_neighbors();
    /* Remove dequeued packet from ringbuf */
    ringbufindex_get(&dequeued_ringbuf);
  }
}
/*---------------------------------------------------------------------------*/
/* Setup TSCH as a coordinator */
static void
tsch_start_coordinator(void)
{
  frame802154_set_pan_id(IEEE802154_PANID);
  /* Initialize hopping sequence as default */
  memcpy(tsch_hopping_sequence, TSCH_DEFAULT_HOPPING_SEQUENCE, sizeof(TSCH_DEFAULT_HOPPING_SEQUENCE));
  TSCH_ASN_DIVISOR_INIT(tsch_hopping_sequence_length, sizeof(TSCH_DEFAULT_HOPPING_SEQUENCE));
#if TSCH_SCHEDULE_WITH_6TISCH_MINIMAL
  tsch_schedule_create_minimal();
#endif

  tsch_is_associated = 1;
  /* post TISCH process event */
  TISCH_ASSOCIATE_EVENT_POST();
  /* launch the tsch_send_eb_process  */
  tsch_send_eb_process_start();

  tsch_join_priority = 0;

  PRINTF("TSCH: starting as coordinator, PAN ID %x, asn-%x.%lx\n",
      frame802154_get_pan_id(), tsch_current_asn.ms1b, tsch_current_asn.ls4b);

  /* Start slot operation */
  tsch_slot_operation_sync(RTIMER_NOW(), &tsch_current_asn);
}
/*---------------------------------------------------------------------------*/
/* Leave the TSCH network */
void
tsch_disassociate(void)
{
  if(tsch_is_associated == 1) {
    tsch_is_associated = 0;
    /* post TISCH process event */
    TISCH_ASSOCIATE_EVENT_POST();
    /* stop the tsch_send_eb_process */
    tsch_send_eb_process_stop();
    PRINTF("TSCH: leaving the network\n");
  }
}
/*---------------------------------------------------------------------------*/
/* Attempt to associate to a network form an incoming EB */
static int
tsch_associate(const struct input_packet *input_eb, rtimer_clock_t timestamp)
{
  frame802154_t frame;
  struct ieee802154_ies ies;
  uint8_t hdrlen;
  int i;

  if(input_eb == NULL || tsch_packet_parse_eb(input_eb->payload, input_eb->len,
                                              &frame, &ies, &hdrlen, 0) == 0) {
    PRINTF("TSCH:! failed to parse EB (len %u)\n", input_eb->len);
    return 0;
  }

  tsch_current_asn = ies.ie_asn;
  tsch_join_priority = ies.ie_join_priority + 1;

#if TSCH_JOIN_SECURED_ONLY
  if(frame.fcf.security_enabled == 0) {
    PRINTF("TSCH:! parse_eb: EB is not secured\n");
    return 0;
  }
#endif /* TSCH_JOIN_SECURED_ONLY */
  
#if LLSEC802154_ENABLED
  if(!tsch_security_parse_frame(input_eb->payload, hdrlen,
      input_eb->len - hdrlen - tsch_security_mic_len(&frame),
      &frame, (linkaddr_t*)&frame.src_addr, &tsch_current_asn)) {
    PRINTF("TSCH:! parse_eb: failed to authenticate\n");
    return 0;
  }
#endif /* LLSEC802154_ENABLED */

#if !LLSEC802154_ENABLED
  if(frame.fcf.security_enabled == 1) {
    PRINTF("TSCH:! parse_eb: we do not support security, but EB is secured\n");
    return 0;
  }
#endif /* !LLSEC802154_ENABLED */

#if TSCH_JOIN_MY_PANID_ONLY
  /* Check if the EB comes from the PAN ID we expect */
  if(frame.src_pid != IEEE802154_PANID) {
    PRINTF("TSCH:! parse_eb: PAN ID %x != %x\n", frame.src_pid, IEEE802154_PANID);
    return 0;
  }
#endif /* TSCH_JOIN_MY_PANID_ONLY */

  /* There was no join priority (or 0xff) in the EB, do not join */
  if(ies.ie_join_priority == 0xff) {
    PRINTF("TSCH:! parse_eb: no join priority\n");
    return 0;
  }

  /* TSCH timeslot timing */
  for(i = 0; i < tsch_ts_elements_count; i++) {
    if(ies.ie_tsch_timeslot_id == 0) {
      tsch_timing[i] = (rtimer_clock_t) bsp_us_to_rtimerTiscks(tsch_default_timing_us[i]);
    } else {
      tsch_timing[i] = (rtimer_clock_t) bsp_us_to_rtimerTiscks(ies.ie_tsch_timeslot[i]);
    }
  }

  /* TSCH hopping sequence */
  if(ies.ie_channel_hopping_sequence_id == 0) {
    memcpy(tsch_hopping_sequence, TSCH_DEFAULT_HOPPING_SEQUENCE, sizeof(TSCH_DEFAULT_HOPPING_SEQUENCE));
    TSCH_ASN_DIVISOR_INIT(tsch_hopping_sequence_length, sizeof(TSCH_DEFAULT_HOPPING_SEQUENCE));
  } else {
    if(ies.ie_hopping_sequence_len <= sizeof(tsch_hopping_sequence)) {
      memcpy(tsch_hopping_sequence, ies.ie_hopping_sequence_list, ies.ie_hopping_sequence_len);
      TSCH_ASN_DIVISOR_INIT(tsch_hopping_sequence_length, ies.ie_hopping_sequence_len);
    } else {
      PRINTF("TSCH:! parse_eb: hopping sequence too long (%u)\n", ies.ie_hopping_sequence_len);
      return 0;
    }
  }

#if TSCH_CHECK_TIME_AT_ASSOCIATION > 0
  /* Divide by 4k and multiply again to avoid integer overflow */
  uint32_t expected_asn = 4096 * TSCH_CLOCK_TO_SLOTS(clock_time() / 4096, tsch_timing_timeslot_length); /* Expected ASN based on our current time*/
  int32_t asn_threshold = TSCH_CHECK_TIME_AT_ASSOCIATION * 60ul * TSCH_CLOCK_TO_SLOTS(bsp_getTRes(), tsch_timing_timeslot_length);
  int32_t asn_diff = (int32_t)tsch_current_asn.ls4b - expected_asn;
  if(asn_diff > asn_threshold) {
    PRINTF("TSCH:! EB ASN rejected %lx %lx %ld\n",
           tsch_current_asn.ls4b, expected_asn, asn_diff);
    return 0;
  }
#endif

#if TSCH_INIT_SCHEDULE_FROM_EB
  /* Create schedule */
  if(ies.ie_tsch_slotframe_and_link.num_slotframes == 0) {
#if TSCH_SCHEDULE_WITH_6TISCH_MINIMAL
    PRINTF("TSCH: parse_eb: no schedule, setting up minimal schedule\n");
    tsch_schedule_create_minimal();
#else
    PRINTF("TSCH: parse_eb: no schedule\n");
#endif
  } else {
    /* First, empty current schedule */
    tsch_schedule_remove_all_slotframes();
    /* We support only 0 or 1 slotframe in this IE */
    int num_links = ies.ie_tsch_slotframe_and_link.num_links;
    if(num_links <= FRAME802154E_IE_MAX_LINKS) {
      int i;
      struct tsch_slotframe *sf = tsch_schedule_add_slotframe(
          ies.ie_tsch_slotframe_and_link.slotframe_handle,
          ies.ie_tsch_slotframe_and_link.slotframe_size);
      for(i = 0; i < num_links; i++) {
        tsch_schedule_add_link(sf,
            ies.ie_tsch_slotframe_and_link.links[i].link_options,
            LINK_TYPE_ADVERTISING, &tsch_broadcast_address,
            ies.ie_tsch_slotframe_and_link.links[i].timeslot, ies.ie_tsch_slotframe_and_link.links[i].channel_offset);
      }
    } else {
      PRINTF("TSCH:! parse_eb: too many links in schedule (%u)\n", num_links);
      return 0;
    }
  }
#endif /* TSCH_INIT_SCHEDULE_FROM_EB */

  if(tsch_join_priority < TSCH_MAX_JOIN_PRIORITY) {
    struct tsch_neighbor *n;

    /* Add coordinator to list of neighbors, lock the entry */
    n = tsch_queue_add_nbr((linkaddr_t *)&frame.src_addr);

    if(n != NULL) {
      tsch_queue_update_time_source((linkaddr_t *)&frame.src_addr);

      /* Set PANID */
      frame802154_set_pan_id(frame.src_pid);

      /* Synchronize on EB */
      tsch_slot_operation_sync(timestamp - tsch_timing[tsch_ts_tx_offset], &tsch_current_asn);

      /* Update global flags */
      tsch_is_associated = 1;
      /* post TISCH process event */
      TISCH_ASSOCIATE_EVENT_POST();
      /* launch the tsch_send_eb_process  */
      tsch_send_eb_process_start();

      tsch_is_pan_secured = frame.fcf.security_enabled;

      /* Start sending keep-alives now that tsch_is_associated is set */
      tsch_schedule_keepalive();

#ifdef TSCH_CALLBACK_JOINING_NETWORK
      TSCH_CALLBACK_JOINING_NETWORK();
#endif

      PRINTF("TSCH: association done, sec %u, PAN ID %x, asn-%x.%lx, jp %u, timeslot id %u, hopping id %u, slotframe len %u with %u links, from ",
             tsch_is_pan_secured,
             frame.src_pid,
             tsch_current_asn.ms1b, tsch_current_asn.ls4b, tsch_join_priority,
             ies.ie_tsch_timeslot_id,
             ies.ie_channel_hopping_sequence_id,
             ies.ie_tsch_slotframe_and_link.slotframe_size,
             ies.ie_tsch_slotframe_and_link.num_links);
      PRINTLLADDR((const uip_lladdr_t *)&frame.src_addr);
      PRINTF("\n");

      return 1;
    }
  }
  PRINTF("TSCH:! did not associate.\n");
  return 0;
}

/* Processes and protothreads used by TSCH */

/*---------------------------------------------------------------------------*/
/* Scanning protothread, called by tsch_process:
 * Listen to different channels, and when receiving an EB,
 * attempt to associate.
 */
static struct ctimer scan_timer;
static uint8_t scan_active = 0;
static void tsch_scan(void *ptr)
{
  static struct input_packet input_eb;
  /* Time when we started scanning on current_channel */
  static clock_time_t current_channel_since;
  e_nsErr_t err = NETSTK_ERR_NONE;
  uint8_t max_pkt_len = TSCH_PACKET_MAX_LEN;

  if(!scan_active)
  {
  TSCH_ASN_INIT(tsch_current_asn, 0, 0);
  current_channel_since = bsp_getTick();
  scan_active = 1;
  }

  if(!tsch_is_associated && !tsch_is_coordinator) {
    /* Hop to any channel offset */
    static uint8_t current_channel = 0;

    /* We are not coordinator, try to associate */
    rtimer_clock_t t0;
    int is_packet_pending = 0;
    clock_time_t now_time = bsp_getTick();

    /* Switch to a (new) channel for scanning */
    if(current_channel == 0 || now_time - current_channel_since > TSCH_CHANNEL_SCAN_DURATION) {
      /* Pick a channel at random in TSCH_JOIN_HOPPING_SEQUENCE */
      uint8_t scan_channel = TSCH_JOIN_HOPPING_SEQUENCE[
          random_rand() % sizeof(TSCH_JOIN_HOPPING_SEQUENCE)];
      if(current_channel != scan_channel) {
        pmac_netstk->phy->ioctrl(NETSTK_CMD_RF_CHAN_NUM_SET, &scan_channel, &err);
        current_channel = scan_channel;
        PRINTF("TSCH: scanning on channel %u\n", scan_channel);
      }
      current_channel_since = now_time;
    }

    /* Turn radio on and wait for EB */
	pmac_netstk->phy->on(&err);
    is_packet_pending = tsch_pending_packet();

    t0 = RTIMER_NOW();
    BUSYWAIT_UNTIL_ABS((is_packet_pending = tsch_pending_packet()), t0, bsp_rtimer_arch_second());

    if(!is_packet_pending && tsch_receiving_packet()) {
      /* If we are currently receiving a packet, wait until end of reception */
      t0 = RTIMER_NOW();
      BUSYWAIT_UNTIL_ABS((is_packet_pending = tsch_pending_packet()), t0, bsp_rtimer_arch_second() / 100);
    }

    if(is_packet_pending) {
      /* Read packet */
      input_eb.len = (int) pmac_netstk->rf->read(input_eb.payload, (uint16_t) TSCH_PACKET_MAX_LEN);
      /* Save packet timestamp */
      pmac_netstk->phy->ioctrl(NETSTK_CMD_RF_TIMESTAMP_GET, &t0, &err);

      /* Parse EB and attempt to associate */
      PRINTF("TSCH: association: received packet (%u bytes) on channel %u\n", input_eb.len, current_channel);

      tsch_associate(&input_eb, t0);
    }

    if(tsch_is_associated) {
      /* End of association, turn the radio off */
      pmac_netstk->phy->off(&err);
      /* if the scan process is launched again we need to initialize some variable  */
      scan_active = 0 ;
    } else if(!tsch_is_coordinator) {
      /* Go back to scanning after the required time */
      ctimer_set(&scan_timer, bsp_getTRes() / TSCH_ASSOCIATION_POLL_FREQUENCY, tsch_scan, NULL);
    }
  }
}

/*---------------------------------------------------------------------------*/
/* The main TSCH process */
void  tsch_process(c_event_t c_event, p_data_t p_data)
{
static uint8_t was_associated = 0 ;

  if(c_event == EVENT_TYPE_TISCH_PROCESS )
  {
    if (!tsch_is_associated) /* initialization or dissociation event */
    {
      if(was_associated)
         /* Will need to re-synchronize */
         tsch_reset();
	  if(tsch_is_coordinator) {
		/* We are coordinator, start operating now */
		tsch_start_coordinator();
	  } else {
		/* Start scanning, will attempt to join when receiving an EB */
		 if(!scan_active)
		   tsch_scan(NULL);
	  }
	}
	else  /* association event */
	{
	/* We are part of a TSCH network, start slot operation */
	tsch_slot_operation_start();
    /* set the flag in order to reset the tisch in case of dissociation */
	was_associated = 1;
	}
  }
}


static struct ctimer send_eb_timer;

static void tsch_send_eb_process_start(void)
{
  /* Set an initial delay except for coordinator, which should send an EB asap */
  if(!tsch_is_coordinator)
  {
    ctimer_set(&send_eb_timer, random_rand() % TSCH_EB_PERIOD , tsch_send_eb_process, NULL);
  }
  else
  {
    ctimer_set(&send_eb_timer, bsp_getTRes() , tsch_send_eb_process, NULL);
  }
}

static void tsch_send_eb_process_stop(void)
{
  ctimer_stop(&send_eb_timer);
}
/*---------------------------------------------------------------------------*/
/* A periodic process to send TSCH Enhanced Beacons (EB) */
static void tsch_send_eb_process (void *ptr)
{
    unsigned long delay;

    if(tsch_is_associated && tsch_current_eb_period > 0) {
      /* Enqueue EB only if there isn't already one in queue */
      if(tsch_queue_packet_count(&tsch_eb_address) == 0) {
        int eb_len;
        uint8_t hdr_len = 0;
        uint8_t tsch_sync_ie_offset;
        /* Prepare the EB packet and schedule it to be sent */
        packetbuf_clear();
        packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, FRAME802154_BEACONFRAME);
#if LLSEC802154_ENABLED
        if(tsch_is_pan_secured) {
          /* Set security level, key id and index */
          packetbuf_set_attr(PACKETBUF_ATTR_SECURITY_LEVEL, TSCH_SECURITY_KEY_SEC_LEVEL_EB);
          packetbuf_set_attr(PACKETBUF_ATTR_KEY_ID_MODE, FRAME802154_1_BYTE_KEY_ID_MODE); /* Use 1-byte key index */
          packetbuf_set_attr(PACKETBUF_ATTR_KEY_INDEX, TSCH_SECURITY_KEY_INDEX_EB);
        }
#endif /* LLSEC802154_ENABLED */
        eb_len = tsch_packet_create_eb(packetbuf_dataptr(), PACKETBUF_SIZE,
            &hdr_len, &tsch_sync_ie_offset);
        if(eb_len > 0) {
          struct tsch_packet *p;
          packetbuf_set_datalen(eb_len);
          /* Enqueue EB packet */
          if(!(p = tsch_queue_add_packet(&tsch_eb_address, NULL, NULL))) {
            PRINTF("TSCH:! could not enqueue EB packet\n");
          } else {
            PRINTF("TSCH: enqueue EB packet %u %u\n", eb_len, hdr_len);
            p->tsch_sync_ie_offset = tsch_sync_ie_offset;
            p->header_len = hdr_len;
          }
        }
      }
    }
    if(tsch_current_eb_period > 0) {
      /* Next EB transmission with a random delay
       * within [tsch_current_eb_period*0.75, tsch_current_eb_period[ */
      delay = (tsch_current_eb_period - tsch_current_eb_period / 4)
        + random_rand() % (tsch_current_eb_period / 4);
    } else {
      delay = TSCH_EB_PERIOD;
    }
    ctimer_set(&send_eb_timer, delay , tsch_send_eb_process, NULL);
}


void tsch_pending_events_process_start_now(void)
{
  /* run tsch_pending_events_process */
  tsch_pending_events_process(EVENT_TYPE_TISCH_TX_RX_PENDING, NULL);
}

void tsch_pending_events_process_start_asap(void)
{
	/* Post the tx_rx_pending event */
	TISCH_PENDING_TX_RX_EVENT_POST();
}

/*---------------------------------------------------------------------------*/
/* A process that is polled from interrupt and calls tx/rx input
 * callbacks, outputs pending logs. */
static void  tsch_pending_events_process(c_event_t c_event, p_data_t p_data)
{
  if(c_event == EVENT_TYPE_TISCH_TX_RX_PENDING)
  {
    tsch_rx_process_pending();
    tsch_tx_process_pending();
    tsch_log_process_pending();
  }
}

/* Functions from the Contiki MAC layer driver interface */

/*---------------------------------------------------------------------------*/
static void
tsch_init(void *p_netstk, e_nsErr_t *p_err)
{

rtimer_clock_t t;

#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }

  if (p_netstk == NULL) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

  /* initialize local variables */
  pmac_netstk = (s_ns_t *) p_netstk;
  /* register TISCH process handler  */
  TISCH_REG_PROCESS_HANDLER();
  /* register TISCH TX RX pending handler  */
  TISCH_REG_PENDING_TX_RX_HANDLER();

  /* configure stack MAC address */
  memcpy(&uip_lladdr.addr, &mac_phy_config.mac_address, 8);
  linkaddr_set_node_addr((linkaddr_t *) mac_phy_config.mac_address);

  uint8_t radio_rx_mode = 0;
  /* Disable radio in frame filtering */
  //radio_rx_mode &= ~RADIO_RX_MODE_ADDRESS_FILTER;
  /* Unset autoack */
 // radio_rx_mode &= ~RADIO_RX_MODE_AUTOACK;
  /* Set radio in poll mode */
  radio_rx_mode |= 0x04; //RADIO_RX_MODE_POLL_MODE;
  pmac_netstk->phy->ioctrl(NETSTK_CMD_RF_OP_MODE_SET, &radio_rx_mode, p_err);
  if(*p_err != NETSTK_ERR_NONE) {
    printf("TSCH:! radio does not support setting required RADIO_PARAM_RX_MODE. Abort init.\n");
    return;
  }

  /* Test setting channel */
  pmac_netstk->phy->ioctrl(NETSTK_CMD_RF_CHAN_NUM_SET, &TSCH_DEFAULT_HOPPING_SEQUENCE[0], p_err);
  if(*p_err != NETSTK_ERR_NONE) {
    printf("TSCH:! radio does not support setting channel. Abort init.\n");
    return;
  }
  /* Test getting timestamp */
    pmac_netstk->phy->ioctrl(NETSTK_CMD_RF_TIMESTAMP_GET, &t, p_err);
    if(*p_err != NETSTK_ERR_NONE) {
    printf("TSCH:! radio does not support getting last packet timestamp. Abort init.\n");
    return;
  }
  /* Check max hopping sequence length vs default sequence length */
  if(TSCH_HOPPING_SEQUENCE_MAX_LEN < sizeof(TSCH_DEFAULT_HOPPING_SEQUENCE)) {
    printf("TSCH:! TSCH_HOPPING_SEQUENCE_MAX_LEN < sizeof(TSCH_DEFAULT_HOPPING_SEQUENCE). Abort init.\n");
  }

  /* Init TSCH sub-modules */
  tsch_reset();
  tsch_queue_init();
  tsch_schedule_init();
  tsch_log_init();
  ringbufindex_init(&input_ringbuf, TSCH_MAX_INCOMING_PACKETS);
  ringbufindex_init(&dequeued_ringbuf, TSCH_DEQUEUED_ARRAY_SIZE);

  tsch_is_initialized = 1;

#if TSCH_AUTOSTART
  /* Start TSCH operation.
   * If TSCH_AUTOSTART is not set, one needs to call tschmac_driver.on() to start TSCH. */
   tschmac_driver.on();
#endif /* TSCH_AUTOSTART */

}
/*---------------------------------------------------------------------------*/
/* Function send for TSCH-MAC, puts the packet in packetbuf in the MAC queue */
static void
send_packet(mac_callback_t sent, void *ptr)
{
  int ret = MAC_TX_DEFERRED;
  int packet_count_before;
  int hdr_len = 0;
  const linkaddr_t *addr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);

  if(!tsch_is_associated) {
    if(!tsch_is_initialized) {
      PRINTF("TSCH:! not initialized (see earlier logs), drop outgoing packet\n");
    } else {
      PRINTF("TSCH:! not associated, drop outgoing packet\n");
    }
    ret = MAC_TX_ERR;
    mac_call_sent_callback(sent, ptr, ret, 1);
    return;
  }

  /* Ask for ACK if we are sending anything other than broadcast */
  if(!linkaddr_cmp(addr, &linkaddr_null)) {
    /* PACKETBUF_ATTR_MAC_SEQNO cannot be zero, due to a pecuilarity
           in framer-802154.c. */
    if(++tsch_packet_seqno == 0) {
      tsch_packet_seqno++;
    }
    packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, tsch_packet_seqno);
    packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
  } else {
    /* Broadcast packets shall be added to broadcast queue
     * The broadcast address in Contiki is linkaddr_null which is equal
     * to tsch_eb_address */
    addr = &tsch_broadcast_address;
  }

  packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, FRAME802154_DATAFRAME);

#if LLSEC802154_ENABLED
  if(tsch_is_pan_secured) {
    /* Set security level, key id and index */
    packetbuf_set_attr(PACKETBUF_ATTR_SECURITY_LEVEL, TSCH_SECURITY_KEY_SEC_LEVEL_OTHER);
    packetbuf_set_attr(PACKETBUF_ATTR_KEY_ID_MODE, FRAME802154_1_BYTE_KEY_ID_MODE); /* Use 1-byte key index */
    packetbuf_set_attr(PACKETBUF_ATTR_KEY_INDEX, TSCH_SECURITY_KEY_INDEX_OTHER);
  }
#endif /* LLSEC802154_ENABLED */

  packet_count_before = tsch_queue_packet_count(addr);

#if !NETSTACK_CONF_BRIDGE_MODE
  /*
   * In the Contiki stack, the source address of a frame is set at the RDC
   * layer. Since TSCH doesn't use any RDC protocol and bypasses the layer to
   * transmit a frame, it should set the source address by itself.
   */
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);
#endif

  if((hdr_len = pmac_netstk->frame->create()) < 0) {
    PRINTF("TSCH:! can't send packet due to framer error\n");
    ret = MAC_TX_ERR;
  } else {
    struct tsch_packet *p;
    /* Enqueue packet */
    p = tsch_queue_add_packet(addr, sent, ptr);
    if(p == NULL) {
      PRINTF("TSCH:! can't send packet to %u with seqno %u, queue %u %u\n",
          TSCH_LOG_ID_FROM_LINKADDR(addr), tsch_packet_seqno,
          packet_count_before,
          tsch_queue_packet_count(addr));
      ret = MAC_TX_ERR;
    } else {
      p->header_len = hdr_len;
      PRINTF("TSCH: send packet to %u with seqno %u, queue %u %u, len %u %u\n",
             TSCH_LOG_ID_FROM_LINKADDR(addr), tsch_packet_seqno,
             packet_count_before,
             tsch_queue_packet_count(addr),
             p->header_len,
             queuebuf_datalen(p->qb));
      (void)packet_count_before; /* Discard "variable set but unused" warning in case of TSCH_LOG_LEVEL of 0 */
    }
  }
  if(ret != MAC_TX_DEFERRED) {
    mac_call_sent_callback(sent, ptr, ret, 1);
  }
}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
  int frame_parsed = 1;

  frame_parsed = pmac_netstk->frame->parse();

  if(frame_parsed < 0) {
    PRINTF("TSCH:! failed to parse %u\n", packetbuf_datalen());
  } else {
    int duplicate = 0;

    /* Seqno of 0xffff means no seqno */
    if(packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO) != 0xffff) {
      /* Check for duplicates */
      duplicate = mac_sequence_is_duplicate();
      if(duplicate) {
        /* Drop the packet. */
        PRINTF("TSCH:! drop dup ll from %u seqno %u\n",
               TSCH_LOG_ID_FROM_LINKADDR(packetbuf_addr(PACKETBUF_ADDR_SENDER)),
               packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO));
      } else {
        mac_sequence_register_seqno();
      }
    }

    if(!duplicate) {
      PRINTF("TSCH: received from %u with seqno %u\n",
             TSCH_LOG_ID_FROM_LINKADDR(packetbuf_addr(PACKETBUF_ADDR_SENDER)),
             packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO));
      pmac_netstk->dllsec->input();
    }
  }
}
/*---------------------------------------------------------------------------*/
static int
turn_on(void)
{
  if(tsch_is_initialized == 1 && tsch_is_started == 0) {
    tsch_is_started = 1;
    /* try to associate to a network or start one if setup as coordinator: post TISCH process event */
    TISCH_ASSOCIATE_EVENT_POST();
    PRINTF("TSCH: starting as %s\n", tsch_is_coordinator ? "coordinator" : "node");
    return 1;
  }
	  return 0;
}
/*---------------------------------------------------------------------------*/
static int
turn_off(int keep_radio_on, e_nsErr_t *p_err)
{
	  if(keep_radio_on) {
		  pmac_netstk->phy->on(p_err);
	  } else {
		  pmac_netstk->phy->off(p_err);
	  }
	  return 0;
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
const tsch_driver_t tschmac_driver = {
  "TSCH",
  tsch_init,
  send_packet,
  packet_input,
  turn_on,
  turn_off,
  channel_check_interval,
};
/*---------------------------------------------------------------------------*/
