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
#include "rip.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"

/* Trickle Timers */
struct trickle_param {
	clock_time_t i_min;
	clock_time_t i_max;			/* Max number of doublings. */
	clock_time_t t_start;         /* Start of the interval (absolute clock_time). */
	clock_time_t t_end;           /* End of the interval (absolute clock_time). */
	clock_time_t t_next;          /* Clock ticks, randomised in [I/2, I). */
	uint8_t i_current;            /* Current doublings from i_min */
	clock_time_t t_last_trigger;
	uint8_t k;
	struct ctimer ct;
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
		t.i_min = ADVERTISEMENT_I_MIN; \
		t.i_max = ADVERTISEMENT_I_MAX; \
		t.k = 0xFF; \
} while(0)

/*---------------------------------------------------------------------------*/
/* Internal Data Structures */
/*---------------------------------------------------------------------------*/
static struct trickle_param t;
/*---------------------------------------------------------------------------*/
/* Local function prototypes */
/*---------------------------------------------------------------------------*/
static void thrd_reset_trickle_timer();
static void thrd_handle_timer(void *);

/*---------------------------------------------------------------------------*/
/* Return a random number in [I/2, I), for a timer with Imin when the timer's
 * current number of doublings is d */
static clock_time_t
thrd_random_interval(clock_time_t i_min, uint8_t d)
{
	clock_time_t min = TRICKLE_TIME(i_min >> 1, d);

	PRINTF("SEND ADV: Random [%lu, %lu)\n", (unsigned long) min,
			(unsigned long)(TRICKLE_TIME(i_min, d)));

	return min + (random_rand() % (TRICKLE_TIME(i_min, d) - 1 - min));
}

/*---------------------------------------------------------------------------*/
/* Called at the end of the current interval for timer ptr */
static void
thrd_double_interval(void *ptr)
{
	struct trickle_param *param = (struct trickle_param *)ptr;
	int16_t offset;
	clock_time_t next;

	/*
	 * If we got called long past our expiration, store the offset and try to
	 * compensate this period
	 */
	offset = (int16_t)(bsp_getTick() - param->t_end);

	/* Calculate next interval */
	if(param->i_current < param->i_max) {
		param->i_current++;
	}

	param->t_start = param->t_end;
	param->t_end = param->t_start + (param->i_min << param->i_current);

	next = thrd_random_interval(param->i_min, param->i_current);
	if(next > offset) {
		next -= offset;
	} else {
		next = 0;
	}
	param->t_next = next;
	ctimer_set(&param->ct, param->t_next, thrd_handle_timer, (void *)param);

	PRINTF("SEND ADV: Doubling at %lu (offset %d), Start %lu, End %lu,"
			" Periodic in %lu\n", bsp_getTick(), offset,
			(unsigned long)param->t_start,
			(unsigned long)param->t_end, (unsigned long)param->t_next);
}

/*---------------------------------------------------------------------------*/
/*
 * Called at a random point in [I/2,I) of the current interval for ptr
 * PARAM is a pointer to the timer that triggered the callback (&t[index])
 */
static void
thrd_handle_timer(void *ptr)
{
	struct trickle_param *param;
	clock_time_t diff_last;       /* Time diff from last pass */
	clock_time_t diff_start;      /* Time diff from interval start */

	param = (struct trickle_param *)ptr;
	if ( param == &t) {

	} else {
		/* This is an ooops and a serious one too */
		return;
	}

	PRINTF("SEND ADV: Periodic at %lu, last=%lu\n",
			(unsigned long)bsp_getTick(),
			(unsigned long)param->t_last_trigger);

	/* Temporarily store 'now' in t_next and calculate diffs */
	param->t_next = bsp_getTick();
	diff_last = param->t_next - param->t_last_trigger;
	diff_start = param->t_next - param->t_start;
	param->t_last_trigger = param->t_next;

	PRINTF
	("SEND ADV: Periodic diff from last %lu, from start %lu\n",
			(unsigned long) diff_last, (unsigned long) diff_start);

	param->t_next = param->t_end - param->t_next;

	PRINTF
	("SEND ADV: Periodic at %lu, Interval End at %lu in %lu\n",
			(unsigned long)bsp_getTick(), (unsigned long)param->t_end,
			(unsigned long)param->t_next);

	ctimer_set(&param->ct, param->t_next, thrd_double_interval, (void *)param);

	return;
}

/*---------------------------------------------------------------------------*/

static void
thrd_reset_trickle_timer()
{
	t.t_start = bsp_getTick();
	t.t_end = t.t_start + (t.i_min);
	t.i_current = 0;
	t.t_next = thrd_random_interval(t.i_min, t.i_current);

	PRINTF
	("SEND ADV: Reset at %lu, Start %lu, End %lu, New Interval %lu\n",
			(unsigned long)t.t_start, (unsigned long)t.t_start,
			(unsigned long)t.t_end, (unsigned long)t.t_next);

	ctimer_set(&t.ct, t.t_next, thrd_handle_timer, (void *)&t);
}
