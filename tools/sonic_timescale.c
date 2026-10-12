/* Stretch the SONIC deploy binary's wall-clock pacing, for SIMULATION ONLY.
 *
 * WHY. The deploy binary paces itself on the wall clock: policy every 20 ms,
 * command republish every 2 ms. A physics backend that runs slower than real
 * time therefore receives a controller whose motions execute FASTER than the
 * world responds -- measured 2026-08-12 on Isaac at RTF 0.18: at IDLE the
 * legs churned at 7.65 rad/s and knee targets swept 4.6 rad. The routing was
 * proven correct by then (open-loop signature test); the timebase was not.
 *
 * WHAT. Every recurrent thread in the binary is paced by exactly one
 * timerfd_settime call made at thread start (unitree_sdk2
 * RecurrentThread::ThreadFunc). Disassembly of the shipped
 * libunitree_sdk2.a shows flags=0 -- a RELATIVE periodic timer whose
 * it_value and it_interval are the same interval -- so multiplying both by a
 * constant stretches the period and nothing else. Internal math is
 * untouched: control_dt_ stays 0.02, so the observation history stride, the
 * reference-motion advance and the INIT ramp all remain correct in SIM time,
 * which is the timebase that matters here.
 *
 * SAFETY. Opt-in twice over: it does nothing unless LD_PRELOADed AND
 * SONIC_TIME_SCALE is set above 1.0. It refuses absolute timers
 * (TFD_TIMER_ABSTIME), where scaling a deadline would be nonsense rather
 * than a slowdown. THIS MUST NEVER BE USED WITH A PHYSICAL ROBOT: real
 * hardware runs at real time, and a controller deliberately slowed against
 * it is a falling robot. It belongs with --disable-crc-check and
 * SONIC_FALL_RESET_M on the sim-only list.
 *
 *   cc -shared -fPIC -O2 -o sonic_timescale.so sonic_timescale.c -ldl
 *   SONIC_TIME_SCALE=2.5 LD_PRELOAD=/abs/path/sonic_timescale.so ./g1_deploy...
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/timerfd.h>
#include <time.h>

typedef int (*settime_fn)(int, int, const struct itimerspec *,
                          struct itimerspec *);

static settime_fn real_settime = NULL;
static double scale = 1.0;
static int initialised = 0;

static void init_once(void)
{
    const char *s;
    real_settime = (settime_fn)dlsym(RTLD_NEXT, "timerfd_settime");
    s = getenv("SONIC_TIME_SCALE");
    if (s != NULL) {
        double v = atof(s);
        /* Bounded on purpose: a typo must not stall the controller for an
         * hour, and scale < 1 (speeding the controller UP) is never what a
         * slow backend needs. */
        if (v > 1.0 && v <= 50.0)
            scale = v;
        else if (v != 0.0 && v != 1.0)
            fprintf(stderr, "[sonic-timescale] ignoring SONIC_TIME_SCALE=%s "
                            "(allowed range is 1.0 < scale <= 50.0)\n", s);
    }
    initialised = 1;
}

static void scale_timespec(struct timespec *t)
{
    double ns = (double)t->tv_sec * 1e9 + (double)t->tv_nsec;
    ns *= scale;
    t->tv_sec = (time_t)(ns / 1e9);
    t->tv_nsec = (long)(ns - (double)t->tv_sec * 1e9);
}

int timerfd_settime(int fd, int flags, const struct itimerspec *new_value,
                    struct itimerspec *old_value)
{
    if (!initialised)
        init_once();

    if (scale > 1.0 && new_value != NULL && !(flags & TFD_TIMER_ABSTIME)) {
        struct itimerspec scaled = *new_value;
        long before_us = (long)(new_value->it_interval.tv_sec * 1000000L
                                + new_value->it_interval.tv_nsec / 1000L);
        scale_timespec(&scaled.it_interval);
        scale_timespec(&scaled.it_value);
        fprintf(stderr, "[sonic-timescale] fd=%d period %ld us -> %ld us "
                        "(x%.2f)\n", fd, before_us,
                (long)(scaled.it_interval.tv_sec * 1000000L
                       + scaled.it_interval.tv_nsec / 1000L), scale);
        fflush(stderr);
        return real_settime(fd, flags, &scaled, old_value);
    }
    return real_settime(fd, flags, new_value, old_value);
}
