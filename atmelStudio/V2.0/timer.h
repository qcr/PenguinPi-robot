/*
 * timer.c
 *
 * simple timer management library
 */

#ifndef __timer_h__
#define __timer_h__

typedef struct {
    uint32_t sec;   // seconds
    uint16_t ms;    // milliseconds
    uint16_t clock; // 20MHz counter
} timer_t;


// global
void timer_get(timer_t *t);
uint32_t timer_diff_us(timer_t *t1, timer_t *t0);

typedef struct {
    uint32_t    sum;
    uint32_t    sum2;
    uint32_t    n;
    uint32_t    max;
} stats_t;

void stats_init(stats_t *stats);
void stats_add(stats_t *stats, uint32_t val);
uint32_t stats_mean(stats_t *stats);
uint32_t stats_var(stats_t *stats);
uint32_t stats_std(stats_t *stats);

#endif
