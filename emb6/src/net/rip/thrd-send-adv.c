/*
 * thrd-send-adv.c
 *
 *  Created on: 29 Apr 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 *  Sending Advertisements using Trickle.
 */

#include "emb6.h"
#include "bsp.h"
#include "ctimer.h"
#include "thread_conf.h"
#include "thrd-send-adv.h"
#include "tlv.h"
#include "thrd-partition.h"
#include "thrd-adv.h"
#include "mle_management.h"

#include "thrd-iface.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"

#define     LOGGER_ENABLE                 LOGGER_THRD_NET
#include    "logger.h"

/* Trickle Timers */
struct trickle_param {
	clock_time_t i_min;			/* The minimum interval size. */
	uint8_t i_max;				/* Max number of doublings. */
	clock_time_t t_next;		/* Clock ticks, randomised in [I/2, I). */
	clock_time_t interval;		/* I. */
	uint8_t k;
};


/**
 * \brief Convert a timer to a sane clock_time_t value after d doublings
 * m is a value of Imin, d is a number of doublings
 * Careful of overflows
 */
#define TRICKLE_TIME(m, d) ((clock_time_t)((m) << (d)))

/**
 * \brief Convert Imax from number of doublings to clock_time_t units for
 * trickle_param t. Again, watch out for overflows */
#define TRICKLE_IMAX(t) ((uint32_t)((t)->i_min << (t)->i_max))

/**
 * \brief Init trickle_timer[m]
 */
#define TIMER_CONFIGURE() do { \
		t.i_min = ADVERTISEMENT_I_MIN * bsp_get(E_BSP_GET_TRES); \
		t.i_max = ADVERTISEMENT_I_MAX; \
		t.k = 0xFF; \
} while(0)

/*---------------------------------------------------------------------------*/
/* Internal Data Structures */
/*---------------------------------------------------------------------------*/
static struct trickle_param t;
static struct ctimer ci;		// Timer for interval I.
static struct ctimer ct;		// Timer for time t.
/*---------------------------------------------------------------------------*/
/* Local function prototypes */
/*---------------------------------------------------------------------------*/
static clock_time_t random_start_interval(clock_time_t min, uint8_t d);
static clock_time_t random_interval(void);
static void handle_send_timer(void *ptr);
static void double_interval(void *ptr);
static void trickle_start(void);
static void trickle_reset(void);
static void trickle_stop(void);
static void handle_send_timer(void *);

/*---------------------------------------------------------------------------*/
/* Return a random number in [min, d * min]. */
static clock_time_t
random_start_interval(clock_time_t min, uint8_t d)
{
	return ((bsp_getrand(0) % (TRICKLE_TIME(min, d) - min + 1)) + min);
}

/*---------------------------------------------------------------------------*/
/* Return a random number in [I/2, I). */
static clock_time_t
random_interval(void)
{
	clock_time_t min = (t.interval >> 1);		// [I/2.
	clock_time_t max = t.interval - 1;		// I).
	// Be aware of 0.
	if ( min == 0 )
		min = 1;
	if ( max == 0 )
		max = 1;
	return ((bsp_getrand(0) % (max - min + 1)) + min);
}

/*---------------------------------------------------------------------------*/
/*
 * Called after a random time in [I/2, I) has expired.
 */
static void
handle_send_timer(void *ptr)
{
	LOG_RAW("thrd_handle_send_timer: Sending MLE Advertisement!\n");

	// TODO Send MLE Advertisement.
	tlv_leader_t *leader_tlv = thrd_generate_leader_data_tlv();
	size_t route64_len = 0;
	tlv_route64_t *route64_tlv = thrd_generate_route64(&route64_len);
	printf("route64_len = %d\n", route64_len);
	send_mle_advertisement(route64_tlv, route64_len, leader_tlv);
	return;
}

/*---------------------------------------------------------------------------*/
/* Called at the end of the current interval for timer. */
static void
double_interval(void *ptr)
{
	LOG_RAW("thrd_double_interval: Doubling interval!\n");

	t.interval = t.interval << 1;		// Double the interval I.

	LOG_RAW("thrd_double_interval: New interval: %lu secs!\n", (unsigned long)t.interval / bsp_get(E_BSP_GET_TRES));

	if ( t.interval > TRICKLE_TIME(t.i_min, t.i_max) )
		t.interval = TRICKLE_TIME(t.i_min, t.i_max);

	ctimer_reset(&ct);

	t.t_next = random_interval();

	LOG_RAW("thrd_handle_timer: Timer will expire in %lu secs.\n",
			(unsigned long)t.t_next / bsp_get(E_BSP_GET_TRES));

	ctimer_set(&ci, t.interval, double_interval, (void *)&t);
	ctimer_set(&ct, t.t_next, handle_send_timer, (void *)&t);

	return;
}

/*---------------------------------------------------------------------------*/

static void
trickle_start(void)
{
	t.interval = random_start_interval(t.i_min, t.i_max);	// I.
	t.t_next = random_interval();	// t.

	LOG_RAW("trickle_start: Timer (I) will expire in %lu secs.\n",
			(unsigned long)t.interval / bsp_get(E_BSP_GET_TRES));
	LOG_RAW("trickle_start: Timer (t) will expire in %lu secs.\n",
			(unsigned long)t.t_next / bsp_get(E_BSP_GET_TRES));

	ctimer_set(&ci, t.interval, double_interval, (void *)&t);
	ctimer_set(&ct, t.t_next, handle_send_timer, (void *)&t);
}

/*---------------------------------------------------------------------------*/

static void
trickle_reset(void)
{
	// Reset old timers.
	ctimer_reset(&ci);
	ctimer_reset(&ct);

	t.interval = t.i_min;				// I.
	t.t_next = random_interval();	// t.

	LOG_RAW("trickle_reset: Timer (I) will expire in %lu secs.\n",
			(unsigned long)t.interval / bsp_get(E_BSP_GET_TRES));
	LOG_RAW("trickle_reset: Timer (t) will expire in %lu secs.\n",
			(unsigned long)t.t_next / bsp_get(E_BSP_GET_TRES));

	ctimer_set(&ci, t.interval, double_interval, (void *)&t);
	ctimer_set(&ct, t.t_next, handle_send_timer, (void *)&t);
}

/*---------------------------------------------------------------------------*/

static void
trickle_stop(void)
{
	ctimer_stop(&ci);
	ctimer_stop(&ct);
}

/*---------------------------------------------------------------------------*/
void
thrd_trickle_init(void)
{
	TIMER_CONFIGURE();

	trickle_start();
	return;
}
/*---------------------------------------------------------------------------*/

void
thrd_trickle_reset(void)
{
	trickle_reset();
}

/*---------------------------------------------------------------------------*/

void
thrd_trickle_stop(void)
{
	trickle_stop();
}


