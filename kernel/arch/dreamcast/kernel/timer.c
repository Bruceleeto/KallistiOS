/* KallistiOS ##version##

   timer.c
   Copyright (C) 2000, 2001, 2002 Megan Potter
   Copyright (C) 2023 Falco Girgis
   Copyright (C) 2023, 2024 Paul Cercueil <paul@crapouillou.net>
*/

#include <assert.h>
#include <stdio.h>

#include <arch/arch.h>
#include <arch/timer.h>
#include <kos/irq.h>
#include <kos/regfield.h>

/* Register access macros */
#define TIMER8(o)   ( *((volatile uint8_t  *)(TIMER_BASE + (o))) )
#define TIMER16(o)  ( *((volatile uint16_t *)(TIMER_BASE + (o))) )
#define TIMER32(o)  ( *((volatile uint32_t *)(TIMER_BASE + (o))) )

/* Register base address */
#define TIMER_BASE 0xffd80000

/* Register offsets */
#define TOCR    0x00    /* Timer Output Control Register */
#define TSTR    0x04    /* Timer Start Register */
#define TCOR0   0x08    /* Timer Constant Register 0 */
#define TCNT0   0x0c    /* Timer Counter Register 0 */
#define TCR0    0x10    /* Timer Control Register 0 */
#define TCOR1   0x14    /* Timer Constant Register 1 */
#define TCNT1   0x18    /* Timer Counter Register 1 */
#define TCR1    0x1c    /* Timer Control Register 1 */
#define TCOR2   0x20    /* Timer Constant Register 2 */
#define TCNT2   0x24    /* Timer Counter Register 2 */
#define TCR2    0x28    /* Timer Control Register 2 */
#define TCPR2   0x2c    /* Timer Input Capture */

/* Timer Start Register fields */
#define STR2    2   /* TCNT2 Counter Start */
#define STR1    1   /* TCNT1 Counter Start */
#define STR0    0   /* TCNT0 Counter Start */

/* Timer Control Register fields */
#define ICPF    BIT(9)          /* Input Capture Interrupt Flag (TMU2 only) */
#define UNF     BIT(8)          /* Underflow Flag */
#define ICPE    GENMASK(7, 6)   /* Input Capture Control (TMU2 only) */
#define UNIE    BIT(5)          /* Underflow Interrupt Control */
#define CKEG    GENMASK(4, 3)   /* Clock Edge */
#define TPSC    GENMASK(2, 0)   /* Timer Prescalar */

/* Clock divisor value for each TPSC value. */
#define TDIV(div)   (4 << (2 * div))

/* Timer Prescalar TPSC values (Peripheral clock divided by N) */
typedef enum PCK_DIV {
    PCK_DIV_4,      /* Pck/4    => 80ns */
    PCK_DIV_16,     /* Pck/16   => 320ns*/
    PCK_DIV_64,     /* Pck/64   => 1280ns*/
    PCK_DIV_256,    /* Pck/256  => 5120ns*/
    PCK_DIV_1024    /* Pck/1024 => 20480ns*/
} PCK_DIV;

/* Timer TPSC values (4 for highest resolution timings) */
#define TIMER_TPSC      PCK_DIV_4
/* Timer IRQ priority levels (0-15) */
#define TIMER_PRIO      15

/* Peripheral clock rate (~49.9 Mhz).
 * The main clock is not exactly 200 MHz, and has been measured at
 * 199499520 Hz. The peripheral clock is a quarter of that. */
#define TIMER_PCK       (199499520 / 4)

/* Timer registers, indexed by Timer ID. */
static const unsigned tcors[] = { TCOR0, TCOR1, TCOR2 };
static const unsigned tcnts[] = { TCNT0, TCNT1, TCNT2 };
static const unsigned tcrs[] = { TCR0, TCR1, TCR2 };

/* Apply timer configuration to registers. */
static int timer_prime_apply(int which, uint32_t count, int interrupts) {
    assert(which <= TMU2);

    TIMER32(tcnts[which]) = count;
    TIMER32(tcors[which]) = count;

    TIMER16(tcrs[which]) = TIMER_TPSC;

    /* Enable IRQ generation plus unmask and set priority */
    if(interrupts) {
        TIMER16(tcrs[which]) |= UNIE;
        timer_enable_ints(which);
    }

    return 0;
}

/* Pre-initialize a timer; set values but don't start it.
   "speed" is the number of desired ticks per second. */
int timer_prime(int which, uint32_t speed, int interrupts) {
    /* Initialize counters; formula is P0/(tps*div) */
    const uint32_t cd = TIMER_PCK / (speed * TDIV(TIMER_TPSC));

    return timer_prime_apply(which, cd, interrupts);
}

/* Works like timer_prime, but takes an interval in milliseconds
   instead of a rate. Used by the primary timer stuff. */
static int timer_prime_wait(int which, uint32_t millis, int interrupts) {
    /* Calculate the countdown, formula is P0 * millis/div*1000. We
       rearrange the math a bit here to avoid integer overflows. */
    const uint32_t cd = (TIMER_PCK / TDIV(TIMER_TPSC)) * millis / 1000;

    return timer_prime_apply(which, cd, interrupts);
}

/* Start a timer -- starts it running (and interrupts if applicable) */
int timer_start(int which) {
    assert(which <= TMU2);

    TIMER8(TSTR) |= BIT(which);
    return 0;
}

/* Stop a timer -- and disables its interrupt */
int timer_stop(int which) {
    assert(which <= TMU2);

    timer_disable_ints(which);

    /* Stop timer */
    TIMER8(TSTR) &= ~BIT(which);

    return 0;
}

int timer_running(int which) {
    assert(which <= TMU2);

    return !!(TIMER8(TSTR) & BIT(which));
}

/* Returns the count value of a timer */
uint32_t timer_count(int which) {
    assert(which <= TMU2);

    return TIMER32(tcnts[which]);
}

/* Clears the timer underflow bit and returns what its value was */
int timer_clear(int which) {
    uint16_t value;

    assert(which <= TMU2);
    value = TIMER16(tcrs[which]);

    TIMER16(tcrs[which]) &= ~UNF;
    return !!(value & UNF);
}

/* Enable timer interrupts; needs to move to irq.c sometime. */
void timer_enable_ints(int which) {
    irq_set_priority(IRQ_SRC_TMU0 - which, TIMER_PRIO);
}

/* Disable timer interrupts; needs to move to irq.c sometime. */
void timer_disable_ints(int which) {
    irq_set_priority(IRQ_SRC_TMU0 - which, IRQ_PRIO_MASKED);
}

/* Check whether ints are enabled */
int timer_ints_enabled(int which) {
    return irq_get_priority(IRQ_SRC_TMU0 - which) > 0;
}

/* Tick rate of a timer channel running at TIMER_TPSC, in Hz. This is the
   unit of the "ticks" field of timer_val_t, and is what the documented
   80ns-per-tick conversion in <arch/timer.h> is derived from. */
#define TIMER_TICK_RATE (TIMER_PCK / TDIV(TIMER_TPSC))

/* Largest gap between two consecutive __dreamcast_get_ticks() calls that we
   are willing to believe. Anything larger is treated as a discontinuity in
   TMU1 (i.e. something reloaded TCNT1 behind our back) rather than as real
   elapsed time, and is discarded instead of being accumulated. Without this,
   a single stray write to TCNT1 would inject up to a full wrap period
   (~344s) of bogus uptime, which would wreck every timeout in the kernel.

   Must stay under the wrap period, and the resulting tick count must stay
   inside 32 bits: 300 * 12468720 = 3740616000, plus a sub-second
   uptime_ticks, still fits. Note the "u" -- that product overflows a signed
   int, so the multiplication below has to be unsigned. */
#define TIMER_MAX_GAP_SECS  300u

/* Uptime since KOS startup, accumulated from TMU1 by __dreamcast_get_ticks().
   uptime_ticks is always kept below TIMER_TICK_RATE. tmu1_last is the TCNT1
   value at the time of the last accumulation. */
static uint32_t uptime_secs;
static uint32_t uptime_ticks;
static uint32_t tmu1_last;

/* Set up the uptime clock, which reads TMU1.

   TMU1 is meant to be owned by the application here: a free-running 32-bit
   down-counter at TIMER_TICK_RATE, started once and never reloaded, with no
   interrupts. KOS only ever reads TCNT1; it does not prime, reload, stop or
   take interrupts from this channel. TMU2 is left completely idle and is
   free for the application to use.

   timer_init() runs before main(), though, so at this point the application
   has not had a chance to configure TMU1 yet, and a stopped TMU1 would mean
   a frozen uptime clock for everything that runs during KOS init (the
   scheduler, and the GD-ROM driver via thd_poll()/cdrom_poll()). So we start
   it here *only if it is not already running*. An application that wants to
   own TMU1 can simply leave this configuration alone -- it is exactly the
   free-running setup described above. */
static void timer_uptime_enable(void) {
    if(!timer_running(TMU1)) {
        /* TCOR1 must be the full 32-bit range: on underflow the hardware
           reloads TCNT1 from TCOR1, and that reload is what makes the
           counter wrap cleanly at 2^32 rather than stalling at zero. */
        TIMER32(tcnts[TMU1]) = 0xffffffff;
        TIMER32(tcors[TMU1]) = 0xffffffff;

        /* No UNIE: the underflow flag will set every wrap and stay set,
           which is harmless because we never look at it and the interrupt
           stays masked. */
        TIMER16(tcrs[TMU1]) = TIMER_TPSC;
        timer_disable_ints(TMU1);

        timer_start(TMU1);
    }

    uptime_secs = 0;
    uptime_ticks = 0;
    tmu1_last = TIMER32(tcnts[TMU1]);
}

/* Generic function for retrieving the current time maintained by TMU1.
   Returns the total amount of time that has elapsed since KOS has been
   initialized, in seconds + ticks. */
timer_val_t __dreamcast_get_ticks(void) {
    uint32_t current, delta;

    /* Unlike the old TMU2 implementation, this one is a read-modify-write on
       shared state, and it is called both from thread context and from IRQ
       context (thd_poll()), so it has to be atomic with respect to IRQs. */
    irq_disable_scoped();

    current = TIMER32(tcnts[TMU1]);

    /* TMU1 counts down and is never reloaded, so the ticks elapsed since the
       previous call are just (previous - current). Evaluating that in 32-bit
       unsigned arithmetic makes the counter's wrap fall out for free: it is
       the same subtraction whether or not the counter passed through zero in
       between, so no software wrap counter or underflow-flag polling is
       needed here.

       This does assume __dreamcast_get_ticks() is called more often than once
       per wrap period -- 2^32 / 12468720 Hz, about 344 seconds -- because a
       longer gap is indistinguishable from a shorter one. That assumption
       holds comfortably in practice: the scheduler and cdrom_poll() both sit
       on this clock and run on millisecond timescales. */
    delta = tmu1_last - current;
    tmu1_last = current;

    /* Reject implausible jumps; see TIMER_MAX_GAP_SECS. Keeping delta bounded
       this way also keeps the accumulation below in 32-bit arithmetic. */
    if(__predict_false(delta > TIMER_MAX_GAP_SECS * TIMER_TICK_RATE))
        delta = 0;

    uptime_ticks += delta;

    while(__predict_false(uptime_ticks >= TIMER_TICK_RATE)) {
        uptime_ticks -= TIMER_TICK_RATE;
        uptime_secs++;
    }

    return (timer_val_t){ .secs = uptime_secs, .ticks = uptime_ticks, };
}

/* Primary kernel timer. What we'll do here is handle actual timer IRQs
   internally, and call the callback only after the appropriate number of
   millis has passed. For the DC you can't have timers spaced out more
   than about one second, so we emulate longer waits with a counter. */
static timer_primary_callback_t tp_callback;
static uint32_t tp_ms_remaining;

/* IRQ handler for the primary timer interrupt. */
static void tp_handler(irq_t src, irq_context_t *cxt, void *data) {
    (void)src;
    (void)data;

    /* Are we at zero? */
    if(tp_ms_remaining == 0) {
        /* Disable any further timer events. The callback may
           re-enable them of course. */
        timer_stop(TMU0);
        timer_disable_ints(TMU0);

        /* Call the callback, if any */
        if(tp_callback)
            tp_callback(cxt);
    }
    /* Do we have less than a second remaining? */
    else if(tp_ms_remaining < 1000) {
        /* Schedule a "last leg" timer. */
        timer_stop(TMU0);
        timer_prime_wait(TMU0, tp_ms_remaining, 1);
        timer_clear(TMU0);
        timer_start(TMU0);
        tp_ms_remaining = 0;
    }
    /* Otherwise, we're just counting down. */
    else {
        tp_ms_remaining -= 1000;
    }
}

/* Enable / Disable primary kernel timer */
static void timer_primary_init(void) {
    /* Clear out our vars */
    tp_callback = NULL;

    /* Clear out TMU0 and get ready for wakeups */
    irq_set_handler(EXC_TMU0_TUNI0, tp_handler, NULL);
    timer_clear(TMU0);
}

static void timer_primary_shutdown(void) {
    timer_stop(TMU0);
    timer_disable_ints(TMU0);
    irq_set_handler(EXC_TMU0_TUNI0, NULL, NULL);
}

timer_primary_callback_t timer_primary_set_callback(timer_primary_callback_t cb) {
    timer_primary_callback_t cbold = tp_callback;
    tp_callback = cb;
    return cbold;
}

void timer_primary_wakeup(uint32_t millis) {
    /* Don't allow zero */
    if(millis == 0) {
        assert_msg(millis != 0, "Received invalid wakeup delay");
        millis++;
    }

    /* Make sure we stop any previous wakeup */
    timer_stop(TMU0);

    /* If we have less than a second to wait, then just schedule the
       timeout event directly. Otherwise schedule a periodic second
       timer. We'll replace this on the last leg in the IRQ. */
    if(millis >= 1000) {
        timer_prime_wait(TMU0, 1000, 1);
        timer_clear(TMU0);
        timer_start(TMU0);
        tp_ms_remaining = millis - 1000;
    }
    else {
        timer_prime_wait(TMU0, millis, 1);
        timer_clear(TMU0);
        timer_start(TMU0);
        tp_ms_remaining = 0;
    }
}

/* Init */
int timer_init(void) {
    /* Stop TMU0 and TMU2, but deliberately *not* TMU1: it backs the uptime
       clock as a free-running counter, and if the application already
       configured and started it we must not disturb it. */
    TIMER8(TSTR) &= ~(BIT(TMU0) | BIT(TMU2));

    /* Set to internal clock source */
    TIMER8(TOCR) = 0;

    /* Leave TMU2 fully idle -- stopped above, no prime, no handler, its
       interrupt masked -- so the application is free to own it. */
    timer_disable_ints(TMU2);

    /* Setup the primary timer stuff */
    timer_primary_init();

    /* Setup the uptime clock (TMU1) */
    timer_uptime_enable();

    return 0;
}

/* Shutdown */
void timer_shutdown(void) {
    /* Shutdown primary timer stuff */
    timer_primary_shutdown();

    /* Disable all timers */
    TIMER8(TSTR) = 0;
    timer_disable_ints(TMU0);
    timer_disable_ints(TMU1);
    timer_disable_ints(TMU2);
}
