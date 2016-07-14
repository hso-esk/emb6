/*
 * thrd-send-adv.h
 *
 *  Created on: 23 May 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 *  Sending Advertisements using Trickle.
 */

#ifndef EMB6_INC_NET_RIP_THRD_SEND_ADV_H_
#define EMB6_INC_NET_RIP_THRD_SEND_ADV_H_

/**
 * Initialize Trickle timer and start with initial intervals.
 */
extern void thrd_trickle_init(void);

/**
 * Reset the Trickle timer and restart with initial intervals.
 */
extern void thrd_trickle_reset(void);

/**
 * Stop the Trickle timer.
 */
extern void thrd_trickle_stop(void);

#endif /* EMB6_INC_NET_RIP_THRD_SEND_ADV_H_ */
