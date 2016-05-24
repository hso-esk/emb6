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
#include "random.h"
#include "thread_conf.h"
#include "thrd-send-adv.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"

/* Trickle Timers */
struct trickle_param {
	clock_time_t i_min;			/* The minimum interval size. */
	uint8_t i_max;				/* Max number of doublings. */
	clock_time_t t_start;		/* Start of the interval (absolute clock_time). */
	clock_time_t t_end;			/* End of the interval (absolute clock_time). */
	clock_time_t t_next;		/* Clock ticks, randomised in [I/2, I). */
	clock_time_t interval;		/* I. */
	uint8_t i_current;			/* Current doublings from i_min */
	clock_time_t t_last_trigger;
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
static void thrd_reset_trickle_timer();
static void thrd_handle_send_timer(void *);

/*---------------------------------------------------------------------------*/
/* Return a random number in [min, d * min]. */
static clock_time_t
thrd_random_start_interval(clock_time_t min, uint8_t d)
{
	return ((random_rand() % (TRICKLE_TIME(min, d) - min + 1)) + min);
}

/*---------------------------------------------------------------------------*/
/* Return a random number in [I/2, I). */
static clock_time_t
thrd_random_interval()
{
	clock_time_t min = (t.interval >> 1);		// [I/2.
	clock_time_t max = t.interval - 1;		// I).
	// Be aware of 0.
	if ( min == 0 )
		min = 1;
	if ( max == 0 )
		max = 1;
	return ((random_rand() % (max - min + 1)) + min);
}


/*---------------------------------------------------------------------------*/
/*
 * Called after a random time in [I/2, I) has expired.
 */
static void
thrd_handle_send_timer(void *ptr)
{
	PRINTF("thrd_handle_send_timer: Sending MLE Advertisement!\n");

	// TODO Send MLE Advertisement.

	return;
}

/*---------------------------------------------------------------------------*/
/* Called at the end of the current interval for timer. */
static void
thrd_double_interval(void *ptr)
{
	PRINTF("thrd_double_interval: Doubling interval!\n");

	t.interval = t.interval << 1;		// Double the interval I.

	PRINTF("thrd_double_interval: New interval: %lu secs!\n", (unsigned long)t.interval / bsp_get(E_BSP_GET_TRES));

	if ( t.interval > TRICKLE_TIME(t.i_min, t.i_max) )
		t.interval = TRICKLE_TIME(t.i_min, t.i_max);

	ctimer_reset(&ct);

	t.t_next = thrd_random_interval();

	PRINTF
	("thrd_handle_timer: Timer will expire in %lu secs.\n",
			(unsigned long)t.t_next / bsp_get(E_BSP_GET_TRES));

	ctimer_set(&ci, t.interval, thrd_double_interval, (void *)&t);
	ctimer_set(&ct, t.t_next, thrd_handle_send_timer, (void *)&t);

	return;
}

/*---------------------------------------------------------------------------*/

static void
thrd_reset_trickle_timer()
{
	t.t_start = bsp_getTick();
	t.t_end = t.t_start + (t.i_min);
	t.interval = thrd_random_start_interval(t.i_min, t.i_max);	// I.
	t.t_next = thrd_random_interval();	// t.


	PRINTF
	("thrd_reset_trickle_timer: Timer (I) will expire in %lu secs.\n",
			(unsigned long)t.interval / bsp_get(E_BSP_GET_TRES));
	PRINTF
	("thrd_reset_trickle_timer: Timer (t) will expire in %lu secs.\n",
			(unsigned long)t.t_next / bsp_get(E_BSP_GET_TRES));

	ctimer_set(&ci, t.interval, thrd_double_interval, (void *)&t);
	ctimer_set(&ct, t.t_next, thrd_handle_send_timer, (void *)&t);
}

/*---------------------------------------------------------------------------*/
void
init(void)
{
	PRINTF("THRD_SEND_ADV: Sending Advertisements.\n");

	TIMER_CONFIGURE();

	PRINTF("THRD_SEND_ADV: imin = %lu, imax = %lu\n", t.i_min, t.i_max);

	thrd_reset_trickle_timer();
	return;
}
/*---------------------------------------------------------------------------*/


