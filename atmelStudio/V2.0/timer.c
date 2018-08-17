/*
 * timer.c
 *
 * simple timer management library
 */

#include	<stdint.h>
#include    <util/atomic.h>

#include    "timer.h"

// global

volatile timer_t	system_timer;


timer_t timer_get()
{
	timer_t t;

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		t = system_timer;
	}
	return t;
}

timer_t timer_diff(timer_t t1, timer_t t0)
{
	// t1 - t0
	if (t1 >= t0)
		return t1 - t0;
	else {
		return UINT32_MAX - t1 + t0 + 1;
	}
}

uint32_t timer_ms(timer_t t)
{
	return (t*8+312)/625;
}

void stats_init(stats_t *stats)
{
	stats->sum2 = stats->sum = stats->n = stats->max = 0;
}

void stats_add(stats_t *stats, timer_t val)
{
	stats->n++;
	stats->sum += val;
	stats->sum2 += val*val;
	if (val > stats->max)
		stats->max = val;
}

timer_t stats_mean(stats_t *stats)
{
	return stats->sum / stats->n;
}

timer_t stats_var(stats_t *stats)
{
    timer_t mean = stats->sum / stats->n;

	return stats->sum2 / stats->n - mean*mean;
}
