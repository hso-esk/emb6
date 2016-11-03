/**
 * \file thrd-send-adv.h
 * \author Institute for reliable Embedded Systems and Communication Electronics
 * \date 2016/04/29
 * \version 1.0
 *
 * \brief Sending MLE advertisements using Trickle algorithm
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
