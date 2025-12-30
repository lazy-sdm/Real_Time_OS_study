/* When the execution is complete, 
a deadline miss of each of the three tasks appears. 
*/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems.h>
#include <rtems/bspIo.h>
#include <stdint.h>
#include <stdbool.h>

#define ASSERT_SUCCESS(sc) \
  do { \
    if ((sc) != RTEMS_SUCCESSFUL) { \
      rtems_fatal(RTEMS_FATAL_SOURCE_EXIT, __LINE__); \
    } \
  } while (0)

#define MAX_TLS_SIZE RTEMS_ALIGN_UP(64, RTEMS_TASK_STORAGE_ALIGNMENT)
#define TASK_ATTRIBUTES RTEMS_DEFAULT_ATTRIBUTES

#define TASK_STORAGE_SIZE \
  RTEMS_TASK_STORAGE_SIZE( \
    MAX_TLS_SIZE + RTEMS_MINIMUM_STACK_SIZE, \
    TASK_ATTRIBUTES \
  )

/* ---------- output via msg_queue + rtems_putc (no printf) ---------- */
#define OUT_MAX_PENDING 128
#define OUT_MSG_SIZE sizeof(char)

static rtems_id putc_done;
static rtems_id out_q;

/* ---------- Experiment: Tick-based periods ---------- */
/* Periods are in ticks (NOT seconds). Make them small to induce misses easily. */
#define T1_PERIOD_TICKS  5
#define T2_PERIOD_TICKS  10
#define T3_PERIOD_TICKS  20

/* RMS priorities (smaller number => higher priority) */
#define PRIO_T1  3
#define PRIO_T2  4
#define PRIO_T3  5

/* Stop after total activations across all tasks */
#define TOTAL_ACTIVATIONS 1500

/* Busy loops: tune up/down.
 * If miss is still 0, increase BUSY_T2/BUSY_T3.
 */
#define BUSY_T1  600000u
#define BUSY_T2  1200000u
#define BUSY_T3  2400000u

/* Optional: progress print (every N activations). 0 = off */
#define PROGRESS_EVERY 0

/* ---------- Shared experiment state ---------- */
static rtems_id lock_id;
static volatile bool done;

static uint32_t total_activations;

typedef struct {
  uint32_t activations;
  uint32_t miss_count;
  rtems_interval c_max_ticks;
  rtems_interval c_sum_ticks;
} task_stats_t;

static task_stats_t s1, s2, s3;

/* ticks per second (info only) */
static rtems_interval tps;

/* ---------- Task storage ---------- */
RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT)
static char out_task_storage[TASK_STORAGE_SIZE];

RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT)
static char t1_storage[TASK_STORAGE_SIZE];

RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT)
static char t2_storage[TASK_STORAGE_SIZE];

RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT)
static char t3_storage[TASK_STORAGE_SIZE];

/* ---------- Task configs ---------- */
static const rtems_task_config out_task_config = {
  .name = rtems_build_name('O','U','T',' '),
  .initial_priority = 2,
  .storage_area = out_task_storage,
  .storage_size = sizeof(out_task_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};

static const rtems_task_config t1_config = {
  .name = rtems_build_name('T','1',' ',' '),
  .initial_priority = PRIO_T1,
  .storage_area = t1_storage,
  .storage_size = sizeof(t1_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};

static const rtems_task_config t2_config = {
  .name = rtems_build_name('T','2',' ',' '),
  .initial_priority = PRIO_T2,
  .storage_area = t2_storage,
  .storage_size = sizeof(t2_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};

static const rtems_task_config t3_config = {
  .name = rtems_build_name('T','3',' ',' '),
  .initial_priority = PRIO_T3,
  .storage_area = t3_storage,
  .storage_size = sizeof(t3_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};

/* ---------- output queue buffers/config ---------- */
static RTEMS_MESSAGE_QUEUE_BUFFER(OUT_MSG_SIZE) out_buffers[OUT_MAX_PENDING];

static const rtems_message_queue_config out_q_config = {
  .name = rtems_build_name('Q','O','U','T'),
  .maximum_pending_messages = RTEMS_ARRAY_SIZE(out_buffers),
  .maximum_message_size = OUT_MSG_SIZE,
  .storage_area = out_buffers,
  .storage_size = sizeof(out_buffers),
  .attributes = RTEMS_DEFAULT_ATTRIBUTES
};

/* ---------- Minimal printing helpers (no libc) ---------- */
static void wait_putc_done(void)
{
  rtems_status_code sc =
    rtems_semaphore_obtain(putc_done, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  ASSERT_SUCCESS(sc);
}

static void send_char_out(char c, size_t *pending)
{
  rtems_status_code sc;

  if (*pending >= OUT_MAX_PENDING) {
    wait_putc_done();
    --(*pending);
  }

  sc = rtems_message_queue_send(out_q, &c, sizeof(c));
  ASSERT_SUCCESS(sc);

  ++(*pending);
}

static void flush_out(size_t *pending)
{
  while (*pending > 0) {
    wait_putc_done();
    --(*pending);
  }
}

static void out_string(const char *s)
{
  size_t pending = 0;
  while (*s) {
    send_char_out(*s++, &pending);
  }
  flush_out(&pending);
}

static void out_int32(int32_t v)
{
  char buf[12];
  int i = 0;
  size_t pending = 0;

  if (v == 0) {
    send_char_out('0', &pending);
    flush_out(&pending);
    return;
  }

  if (v < 0) {
    send_char_out('-', &pending);
    if (v == INT32_MIN) {
      const char *min_str = "2147483648";
      while (*min_str) send_char_out(*min_str++, &pending);
      flush_out(&pending);
      return;
    }
    v = -v;
  }

  while (v > 0 && i < (int)sizeof(buf)) {
    buf[i++] = (char)('0' + (v % 10));
    v /= 10;
  }

  while (i-- > 0) {
    send_char_out(buf[i], &pending);
  }

  flush_out(&pending);
}

static void out_uint32(uint32_t v)
{
  out_int32((int32_t)v);
}

/* ---------- Output task ---------- */
static void out_task(rtems_task_argument arg)
{
  (void)arg;

  while (true) {
    rtems_status_code sc;
    char c;
    size_t n = 0;

    sc = rtems_message_queue_receive(out_q, &c, &n, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    ASSERT_SUCCESS(sc);

    rtems_putc(c);

    sc = rtems_semaphore_release(putc_done);
    ASSERT_SUCCESS(sc);
  }
}

/* ---------- helpers ---------- */
static void busy_work(uint32_t loops)
{
  volatile uint32_t x = 0;
  for (uint32_t i = 0; i < loops; ++i) {
    x ^= (i * 1664525u) + 1013904223u;
  }
}

static rtems_interval tick_delta(rtems_interval start, rtems_interval end)
{
  return (rtems_interval)(end - start);
}

static uint32_t ceil_div_u32(uint32_t a, uint32_t b)
{
  return (a + b - 1u) / b;
}

/* ---------- RTA computation (ticks) ----------
 * R_i = C_i + sum_{hp} ceil(R_i / T_hp) * C_hp
 */
static uint32_t rta_compute(uint32_t Ci, uint32_t Ti,
                           const uint32_t *C_hp, const uint32_t *T_hp, uint32_t n_hp)
{
  uint32_t R = Ci;
  for (uint32_t iter = 0; iter < 100; ++iter) {
    uint32_t I = 0;
    for (uint32_t j = 0; j < n_hp; ++j) {
      I += ceil_div_u32(R, T_hp[j]) * C_hp[j];
    }
    uint32_t R_next = Ci + I;
    if (R_next == R) return R;
    R = R_next;
    if (R > Ti) return R; /* already misses */
  }
  return R;
}

/* ---------- stats update ---------- */
static void stats_update(task_stats_t *st, rtems_interval c_ticks, bool missed_deadline)
{
  rtems_status_code sc =
    rtems_semaphore_obtain(lock_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  ASSERT_SUCCESS(sc);

  st->activations++;
  st->c_sum_ticks += c_ticks;
  if (c_ticks > st->c_max_ticks) st->c_max_ticks = c_ticks;
  if (missed_deadline) st->miss_count++;

  total_activations++;
  uint32_t now = total_activations;

  if (total_activations >= TOTAL_ACTIVATIONS) {
    done = true;
  }

  sc = rtems_semaphore_release(lock_id);
  ASSERT_SUCCESS(sc);

#if PROGRESS_EVERY > 0
  if ((now % PROGRESS_EVERY) == 0u || now == TOTAL_ACTIVATIONS) {
    out_string("progress=");
    out_uint32(now);
    out_string("\n");
  }
#else
  (void)now;
#endif
}

/* ---------- periodic task body ---------- */
static void periodic_task_body(task_stats_t *st,
                               rtems_name period_name,
                               rtems_interval period_ticks,
                               uint32_t busy_loops)
{
  rtems_id period_id;
  rtems_status_code sc;

  sc = rtems_rate_monotonic_create(period_name, &period_id);
  ASSERT_SUCCESS(sc);

  for (;;) {
    if (done) {
      rtems_task_suspend(RTEMS_SELF);
    }

    rtems_interval start = rtems_clock_get_ticks_since_boot();
    busy_work(busy_loops);
    rtems_interval end = rtems_clock_get_ticks_since_boot();

    rtems_interval c_ticks = tick_delta(start, end);

    sc = rtems_rate_monotonic_period(period_id, period_ticks);
    bool miss = (sc != RTEMS_SUCCESSFUL);

    stats_update(st, c_ticks, miss);
  }
}

static void t1_task(rtems_task_argument arg)
{
  (void)arg;
  periodic_task_body(&s1, rtems_build_name('P','1',' ',' '), (rtems_interval)T1_PERIOD_TICKS, BUSY_T1);
}

static void t2_task(rtems_task_argument arg)
{
  (void)arg;
  periodic_task_body(&s2, rtems_build_name('P','2',' ',' '), (rtems_interval)T2_PERIOD_TICKS, BUSY_T2);
}

static void t3_task(rtems_task_argument arg)
{
  (void)arg;
  periodic_task_body(&s3, rtems_build_name('P','3',' ',' '), (rtems_interval)T3_PERIOD_TICKS, BUSY_T3);
}

/* ---------- print summary ---------- */
static void print_summary_and_exit(void)
{
  uint32_t C1 = (uint32_t)s1.c_max_ticks;
  uint32_t C2 = (uint32_t)s2.c_max_ticks;
  uint32_t C3 = (uint32_t)s3.c_max_ticks;

  uint32_t T1 = (uint32_t)T1_PERIOD_TICKS;
  uint32_t T2 = (uint32_t)T2_PERIOD_TICKS;
  uint32_t T3 = (uint32_t)T3_PERIOD_TICKS;

  /* RMS order: T1 > T2 > T3 */
  uint32_t R1 = rta_compute(C1, T1, NULL, NULL, 0);

  uint32_t C_hp2[] = { C1 };
  uint32_t T_hp2[] = { T1 };
  uint32_t R2 = rta_compute(C2, T2, C_hp2, T_hp2, 1);

  uint32_t C_hp3[] = { C1, C2 };
  uint32_t T_hp3[] = { T1, T2 };
  uint32_t R3 = rta_compute(C3, T3, C_hp3, T_hp3, 2);

  out_string("\n=== Deadline Miss & RTA (Tick-Period) Summary ===\n");
  out_string("ticks_per_second="); out_uint32((uint32_t)tps); out_string("\n");
  out_string("Periods(ticks): T1="); out_uint32(T1);
  out_string(" T2="); out_uint32(T2);
  out_string(" T3="); out_uint32(T3);
  out_string("\n");
  out_string("TOTAL_ACTIVATIONS="); out_uint32(total_activations); out_string("\n\n");

  out_string("[T1] Cmax_ticks="); out_uint32(C1);
  out_string(" RTA_R="); out_uint32(R1);
  out_string(" miss="); out_uint32(s1.miss_count);
  out_string(" act="); out_uint32(s1.activations);
  out_string("\n");

  out_string("[T2] Cmax_ticks="); out_uint32(C2);
  out_string(" RTA_R="); out_uint32(R2);
  out_string(" miss="); out_uint32(s2.miss_count);
  out_string(" act="); out_uint32(s2.activations);
  out_string("\n");

  out_string("[T3] Cmax_ticks="); out_uint32(C3);
  out_string(" RTA_R="); out_uint32(R3);
  out_string(" miss="); out_uint32(s3.miss_count);
  out_string(" act="); out_uint32(s3.activations);
  out_string("\n\n");

  out_string("RTA check (R <= P means schedulable under theory):\n");
  out_string("T1: "); out_string((R1 <= T1) ? "OK\n" : "FAIL\n");
  out_string("T2: "); out_string((R2 <= T2) ? "OK\n" : "FAIL\n");
  out_string("T3: "); out_string((R3 <= T3) ? "OK\n" : "FAIL\n");

  out_string("\nIf miss is still 0: increase BUSY_T2/BUSY_T3.\n");
  out_string("If too slow/hangs: reduce BUSY_* or TOTAL_ACTIVATIONS.\n");

  rtems_fatal(RTEMS_FATAL_SOURCE_EXIT, 0);
}

/* ---------- Init ---------- */
static void Init(rtems_task_argument arg)
{
  (void)arg;

  rtems_status_code sc;
  rtems_id out_id, id1, id2, id3;

  done = false;
  total_activations = 0;
  s1 = (task_stats_t){0};
  s2 = (task_stats_t){0};
  s3 = (task_stats_t){0};

  tps = rtems_clock_get_ticks_per_second();
  if (tps == 0) tps = 1;

  /* output semaphore + queue */
  sc = rtems_semaphore_create(
    rtems_build_name('P','U','T','C'),
    0,
    RTEMS_COUNTING_SEMAPHORE,
    0,
    &putc_done
  );
  ASSERT_SUCCESS(sc);

  sc = rtems_message_queue_construct(&out_q_config, &out_q);
  ASSERT_SUCCESS(sc);

  /* lock for stats */
  sc = rtems_semaphore_create(
    rtems_build_name('L','O','C','K'),
    1,
    RTEMS_BINARY_SEMAPHORE,
    0,
    &lock_id
  );
  ASSERT_SUCCESS(sc);

  /* start output task */
  sc = rtems_task_construct(&out_task_config, &out_id);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(out_id, out_task, 0);
  ASSERT_SUCCESS(sc);

  out_string("\n[RTEMS] Deadline miss + RTA (tick periods)\n");
  out_string("Try increasing BUSY_* to force misses.\n\n");

  /* start periodic tasks */
  sc = rtems_task_construct(&t1_config, &id1);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(id1, t1_task, 0);
  ASSERT_SUCCESS(sc);

  sc = rtems_task_construct(&t2_config, &id2);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(id2, t2_task, 0);
  ASSERT_SUCCESS(sc);

  sc = rtems_task_construct(&t3_config, &id3);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(id3, t3_task, 0);
  ASSERT_SUCCESS(sc);

  /* wait until done */
  while (!done) {
    rtems_task_wake_after(tps);
  }

  print_summary_and_exit();
}

/* ---------- RTEMS configuration ---------- */
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_MAXIMUM_PROCESSORS 2

#define CONFIGURE_MAXIMUM_TASKS 5              /* Init + out + T1 + T2 + T3 */
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES 1     /* out_q */
#define CONFIGURE_MAXIMUM_SEMAPHORES 2         /* putc_done + lock */
#define CONFIGURE_MAXIMUM_PERIODS 3            /* P1,P2,P3 */

#define CONFIGURE_MINIMUM_TASKS_WITH_USER_PROVIDED_STORAGE \
  CONFIGURE_MAXIMUM_TASKS

#define CONFIGURE_MICROSECONDS_PER_TICK 1000

/* keep qual-only style */
#define CONFIGURE_MAXIMUM_FILE_DESCRIPTORS 0
#define CONFIGURE_DISABLE_NEWLIB_REENTRANCY
#define CONFIGURE_APPLICATION_DISABLE_FILESYSTEM

#define CONFIGURE_MAXIMUM_THREAD_LOCAL_STORAGE_SIZE MAX_TLS_SIZE

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_INIT_TASK_ATTRIBUTES TASK_ATTRIBUTES
#define CONFIGURE_INIT_TASK_INITIAL_MODES RTEMS_DEFAULT_MODES
#define CONFIGURE_INIT_TASK_CONSTRUCT_STORAGE_SIZE TASK_STORAGE_SIZE

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
