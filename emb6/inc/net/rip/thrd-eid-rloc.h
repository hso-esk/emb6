/*
 * thrd-eid-rloc.h
 *
 *  Created on: 7 Jun 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  EID-to-RLOC Map Cache.
 */

#ifndef EMB6_INC_NET_RIP_THRD_EID_RLOC_H_
#define EMB6_INC_NET_RIP_THRD_EID_RLOC_H_

#include "emb6.h"
#include "clist.h"
#include "uip.h"

extern void thrd_eid_rloc_cache_init(void);

/**
 * EID-to-RLOC Map Cache.
 */
typedef struct thrd_eid_rloc_cache {
	struct thrd_eid_rloc_cache *next;

	uip_ipaddr_t EID;
	uip_ipaddr_t RLOC;
	clock_time_t Age;
} thrd_eid_rloc_cache_t;

uint8_t thrd_eid_rloc_cache_num();

thrd_eid_rloc_cache_t *thrd_eid_rloc_cache_head(void);

thrd_eid_rloc_cache_t *thrd_eid_rloc_cache_next(thrd_eid_rloc_cache_t *i);

thrd_eid_rloc_cache_t *thrd_eid_rloc_cache_lookup(uip_ipaddr_t eid);

void thrd_eid_rloc_cache_rm(thrd_eid_rloc_cache_t *entry);

void thrd_eid_rloc_cache_empty();

/**
 * Managing EID-to-RLOC Map Cache Entries.
 * @return
 */
thrd_eid_rloc_cache_t *thrd_eid_rloc_cache_update(uip_ipaddr_t eid, uip_ipaddr_t rloc);

/* --------------------------------------------------------------------------- */
/* --------------------------------- DEBUG ----------------------------------- */
/* --------------------------------------------------------------------------- */

void thrd_eid_rloc_cache_print();

#endif /* EMB6_INC_NET_RIP_THRD_EID_RLOC_H_ */
