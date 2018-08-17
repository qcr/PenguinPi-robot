/*
 * timer.c
 *
 * simple timer management library
 */

#ifndef __timer_h__
#define __timer_h__

typedef uint32_t timer_t;

// global

volatile extern timer_t	system_timer;

timer_t timer_get();
timer_t timer_diff(timer_t t1, timer_t t0);
uint32_t timer_ms(timer_t t);

typedef struct _stats {
    uint32_t    sum;
    uint32_t    sum2;
    uint32_t    n;
    uint32_t    max;
    } stats_t;

void stats_init(stats_t *stats);
void stats_add(stats_t *stats, timer_t val);
timer_t stats_mean(stats_t *stats);
timer_t stats_var(stats_t *stats);

#endif
