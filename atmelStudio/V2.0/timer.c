/*
 * timer.c
 *
 * simple timer management library
 */

#include <math.h>
#include	<stdint.h>
#include    <util/atomic.h>

#include    "timer.h"
#include    "global.h"

// global


void timer_get(timer_t *t)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
        t->clock = TCNT1;   // read the 20MHz counter
        t->ms = milliseconds_counter;   // read ms from the ISR
        t->sec = seconds_counter;        // read sec from the ISR
	}
}

uint32_t timer_diff_us(timer_t *t1, timer_t *t0)
{
    uint32_t    t;

	// computer t1 - t0
    t = (t1->sec - t0->sec) * 1000000ul;

    if (t1->ms > t0->ms)
        t += (t1->ms - t0->ms) * 1000ul;
    else
        t -= (t0->ms - t1->ms) * 1000ul;

    if (t1->clock >= t0->clock)
        t += (t1->clock - t0->clock)/20u;
    else {
        if (t > 0)
            t -=  (t0->clock - t1->clock)/20u;
        else
            t =  1000ul - (t0->clock - t1->clock)/20u;  // ISR still pending
    }
    return t;
}

void stats_init(stats_t *stats)
{
	stats->sum2 = stats->sum = stats->n = stats->max = 0;
}

void stats_add(stats_t *stats, uint32_t val)
{
	stats->n++;
	stats->sum += val;
	stats->sum2 += val*val;
	if (val > stats->max)
		stats->max = val;
}

uint32_t stats_mean(stats_t *stats)
{
	return stats->sum / stats->n;
}

uint32_t stats_var(stats_t *stats)
{
    uint32_t  mean = stats->sum / stats->n;

	return stats->sum2 / stats->n - mean*mean;
}

uint32_t stats_std(stats_t *stats)
{
    /*
    float  n = stats->n;
    float  sum = stats->sum;
    float  sum2 = stats->sum2;
    float  mean = sum / n;

	return (uint32_t) sqrtf(sum2/n - mean*mean);
    */
    uint32_t  mean = stats->sum / stats->n;
	return (uint32_t) sqrtf((float) (stats->sum2/stats->n - mean*mean));
}
