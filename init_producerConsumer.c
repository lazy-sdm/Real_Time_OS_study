/*
 * Producer-Consumer (Bounded Buffer) Demo - minimized logging
 *
 * Visible points:
 *  - Producer blocks when buffer is FULL (empty_sem unsatisfied)
 *  - Consumer blocks when buffer is EMPTY (full_sem unsatisfied)
 *
 * Logging policy:
 *  - Print ONLY when a block happens (key event)
 *  - Print ONLY when buffer hits 0 or BUFFER_SIZE (edge states)
 *  - Init prints progress every 10 items
 *
 * qual-only friendly: no printf/snprintf, output via rtems_putc
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

/* ---- minimal output ---- */
static void out_char(char c) { rtems_putc(c); }

static void out_string(const char *s)
{
  while (*s) out_char(*s++);
}

static void out_u32(uint32_t v)
{
  char buf[11];
  int i = 0;
  if (v == 0) { out_char('0'); return; }
  while (v > 0 && i < (int)sizeof(buf)) {
    buf[i++] = (char)('0' + (v % 10u));
    v /= 10u;
  }
  while (i-- > 0) out_char(buf[i]);
}

static void out_ln(const char *s)
{
  out_string(s);
  out_string("\n");
}

static void out_kv_ln(const char *k, uint32_t v)
{
  out_string(k);
  out_u32(v);
  out_string("\n");
}

/* ---------- demo parameters ---------- */
#define BUFFER_SIZE     8u
#define TOTAL_ITEMS     60u

/* 20x slower periods */
#define P1_PERIOD_SEC   20u
#define P2_PERIOD_SEC   40u
#define C_PERIOD_SEC    60u

/* ---------- bounded buffer ---------- */
typedef struct {
  uint32_t value;
  uint32_t producer_id;
  uint32_t seq;
} item_t;

static item_t ring[BUFFER_SIZE];
static uint32_t head = 0;
static uint32_t tail = 0;
static uint32_t count_in_buffer = 0;

/* semaphores */
static rtems_id sem_empty; /* slots */
static rtems_id sem_full;  /* items */
static rtems_id mtx;       /* mutex */

/* statistics */
static volatile uint32_t produced_total = 0;
static volatile uint32_t consumed_total = 0;
static volatile uint32_t p_blocked_empty = 0;
static volatile uint32_t c_blocked_full  = 0;

/* ---------- time helper ---------- */
static rtems_interval seconds_to_ticks(rtems_interval tps, rtems_interval sec)
{
  if (tps == 0) tps = 1;
  rtems_interval ticks = tps * sec;
  if (sec != 0 && ticks / sec != tps) ticks = (rtems_interval)-1;
  if (ticks < tps) ticks = tps;
  return ticks;
}

/* ---------- ring ops (must hold mutex) ---------- */
static void ring_put(item_t it)
{
  ring[tail] = it;
  tail = (tail + 1u) % BUFFER_SIZE;
  count_in_buffer++;
}

static item_t ring_get(void)
{
  item_t it = ring[head];
  head = (head + 1u) % BUFFER_SIZE;
  count_in_buffer--;
  return it;
}

/* ---------- static task storage/config ---------- */
#define MAX_TLS_SIZE RTEMS_ALIGN_UP(64, RTEMS_TASK_STORAGE_ALIGNMENT)
#define TASK_ATTRIBUTES RTEMS_DEFAULT_ATTRIBUTES
#define TASK_STORAGE_SIZE \
  RTEMS_TASK_STORAGE_SIZE( \
    MAX_TLS_SIZE + RTEMS_MINIMUM_STACK_SIZE, \
    TASK_ATTRIBUTES \
  )

RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT) static char prod1_storage[TASK_STORAGE_SIZE];
RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT) static char prod2_storage[TASK_STORAGE_SIZE];
RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT) static char cons_storage[TASK_STORAGE_SIZE];

static const rtems_task_config prod1_cfg = {
  .name = rtems_build_name('P','1',' ',' '),
  .initial_priority = 3,
  .storage_area = prod1_storage,
  .storage_size = sizeof(prod1_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};
static const rtems_task_config prod2_cfg = {
  .name = rtems_build_name('P','2',' ',' '),
  .initial_priority = 4,
  .storage_area = prod2_storage,
  .storage_size = sizeof(prod2_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};
static const rtems_task_config cons_cfg = {
  .name = rtems_build_name('C','N','S',' '),
  .initial_priority = 2,
  .storage_area = cons_storage,
  .storage_size = sizeof(cons_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};

/* ---------- producer ---------- */
static rtems_task producer_task(rtems_task_argument arg)
{
  uint32_t pid = (uint32_t)arg; /* 1 or 2 */
  uint32_t local_seq = 0;

  rtems_interval tps = rtems_clock_get_ticks_per_second();
  if (tps == 0) tps = 1;

  rtems_interval period = seconds_to_ticks(
    tps, (pid == 1) ? P1_PERIOD_SEC : P2_PERIOD_SEC
  );

  rtems_id rm_id;
  rtems_status_code sc = rtems_rate_monotonic_create(
    (pid == 1) ? rtems_build_name('R','P','1',' ') : rtems_build_name('R','P','2',' '),
    &rm_id
  );
  ASSERT_SUCCESS(sc);

  out_string("[P"); out_u32(pid); out_string("] start period_sec=");
  out_u32((pid == 1) ? P1_PERIOD_SEC : P2_PERIOD_SEC);
  out_string("\n");

  while (true) {
    if (produced_total >= TOTAL_ITEMS) {
      out_string("[P"); out_u32(pid); out_string("] stop\n");
      rtems_task_exit();
    }

    /* try empty no-wait to detect block */
    sc = rtems_semaphore_obtain(sem_empty, RTEMS_NO_WAIT, 0);
    if (sc == RTEMS_UNSATISFIED) {
      p_blocked_empty++;
      out_string("[P"); out_u32(pid); out_string("] BLOCK on EMPTY (buffer full) at produced=");
      out_u32(produced_total);
      out_string("\n");
      sc = rtems_semaphore_obtain(sem_empty, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    }
    ASSERT_SUCCESS(sc);

    sc = rtems_semaphore_obtain(mtx, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    ASSERT_SUCCESS(sc);

    if (produced_total >= TOTAL_ITEMS) {
      sc = rtems_semaphore_release(mtx); ASSERT_SUCCESS(sc);
      sc = rtems_semaphore_release(sem_empty); ASSERT_SUCCESS(sc);
      rtems_task_exit();
    }

    uint32_t global_seq = produced_total++;
    item_t it;
    it.value = (pid * 1000u) + local_seq;
    it.producer_id = pid;
    it.seq = global_seq;
    local_seq++;

    ring_put(it);

    /* edge-state logging only */
    if (count_in_buffer == BUFFER_SIZE) {
      out_string("[BUF] FULL by P"); out_u32(pid);
      out_string(" seq="); out_u32(global_seq);
      out_string(" buf="); out_u32(count_in_buffer);
      out_string("\n");
    }

    sc = rtems_semaphore_release(mtx);
    ASSERT_SUCCESS(sc);

    sc = rtems_semaphore_release(sem_full);
    ASSERT_SUCCESS(sc);

    (void) rtems_rate_monotonic_period(rm_id, period);
  }
}

/* ---------- consumer ---------- */
static rtems_task consumer_task(rtems_task_argument arg)
{
  (void)arg;

  rtems_interval tps = rtems_clock_get_ticks_per_second();
  if (tps == 0) tps = 1;
  rtems_interval period = seconds_to_ticks(tps, C_PERIOD_SEC);

  rtems_id rm_id;
  rtems_status_code sc = rtems_rate_monotonic_create(
    rtems_build_name('R','C','N','S'),
    &rm_id
  );
  ASSERT_SUCCESS(sc);

  out_string("[C] start period_sec="); out_u32(C_PERIOD_SEC); out_string("\n");

  while (true) {
    if (consumed_total >= TOTAL_ITEMS) {
      out_ln("[C] done");
      rtems_task_exit();
    }

    sc = rtems_semaphore_obtain(sem_full, RTEMS_NO_WAIT, 0);
    if (sc == RTEMS_UNSATISFIED) {
      c_blocked_full++;
      out_string("[C] BLOCK on FULL (buffer empty) at consumed=");
      out_u32(consumed_total);
      out_string("\n");
      sc = rtems_semaphore_obtain(sem_full, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    }
    ASSERT_SUCCESS(sc);

    sc = rtems_semaphore_obtain(mtx, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    ASSERT_SUCCESS(sc);

    item_t it = ring_get();
    consumed_total++;

    /* edge-state logging only */
    if (count_in_buffer == 0) {
      out_string("[BUF] EMPTY after C consumed seq=");
      out_u32(it.seq);
      out_string(" from P"); out_u32(it.producer_id);
      out_string("\n");
    }

    sc = rtems_semaphore_release(mtx);
    ASSERT_SUCCESS(sc);

    sc = rtems_semaphore_release(sem_empty);
    ASSERT_SUCCESS(sc);

    (void) rtems_rate_monotonic_period(rm_id, period);
  }
}

/* ---------- Init ---------- */
static void Init(rtems_task_argument arg)
{
  (void)arg;

  out_ln("\n=== Producer-Consumer (Bounded Buffer) Demo (minimal logs) ===");
  out_kv_ln("BUFFER_SIZE=", BUFFER_SIZE);
  out_kv_ln("TOTAL_ITEMS=", TOTAL_ITEMS);
  out_ln("Logs only on BLOCK events and BUFFER edge (FULL/EMPTY).");
  out_ln("");

  rtems_status_code sc;

  sc = rtems_semaphore_create(
    rtems_build_name('E','M','P','T'),
    BUFFER_SIZE,
    RTEMS_COUNTING_SEMAPHORE,
    0,
    &sem_empty
  );
  ASSERT_SUCCESS(sc);

  sc = rtems_semaphore_create(
    rtems_build_name('F','U','L','L'),
    0,
    RTEMS_COUNTING_SEMAPHORE,
    0,
    &sem_full
  );
  ASSERT_SUCCESS(sc);

  sc = rtems_semaphore_create(
    rtems_build_name('M','U','T','X'),
    1,
    RTEMS_BINARY_SEMAPHORE,
    0,
    &mtx
  );
  ASSERT_SUCCESS(sc);

  rtems_id tid;

  sc = rtems_task_construct(&cons_cfg, &tid);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(tid, consumer_task, 0);
  ASSERT_SUCCESS(sc);

  sc = rtems_task_construct(&prod1_cfg, &tid);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(tid, producer_task, 1);
  ASSERT_SUCCESS(sc);

  sc = rtems_task_construct(&prod2_cfg, &tid);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(tid, producer_task, 2);
  ASSERT_SUCCESS(sc);

  rtems_interval tps = rtems_clock_get_ticks_per_second();
  if (tps == 0) tps = 1;

  /* progress: every 10 items only */
  uint32_t last_print = 0;

  while (consumed_total < TOTAL_ITEMS) {
    if (consumed_total / 10u != last_print / 10u) {
      last_print = consumed_total;
      out_string("[Init] progress produced=");
      out_u32(produced_total);
      out_string(" consumed=");
      out_u32(consumed_total);
      out_string(" buf=");
      out_u32(count_in_buffer);
      out_string("\n");
    }
    (void) rtems_task_wake_after(tps); /* 1 sec */
  }

  (void) rtems_task_wake_after(tps);

  out_ln("\n=== Summary ===");
  out_kv_ln("produced_total=", produced_total);
  out_kv_ln("consumed_total=", consumed_total);
  out_kv_ln("producer_blocked_on_empty=", p_blocked_empty);
  out_kv_ln("consumer_blocked_on_full=", c_blocked_full);
  out_ln("If producer_blocked_on_empty > 0 => buffer FULL happened (producer blocked).");
  out_ln("If consumer_blocked_on_full  > 0 => buffer EMPTY happened (consumer blocked).");
  out_ln("\n(END) exiting via rtems_fatal(EXIT,0)");

  rtems_fatal(RTEMS_FATAL_SOURCE_EXIT, 0);
}

/* ---------- RTEMS configuration ---------- */
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_MAXIMUM_PROCESSORS 2

/* Init + 2 producers + 1 consumer */
#define CONFIGURE_MAXIMUM_TASKS 4
#define CONFIGURE_MINIMUM_TASKS_WITH_USER_PROVIDED_STORAGE \
  CONFIGURE_MAXIMUM_TASKS

/* empty, full, mutex */
#define CONFIGURE_MAXIMUM_SEMAPHORES 3

/* rate-monotonic: P1, P2, C */
#define CONFIGURE_MAXIMUM_PERIODS 3

#define CONFIGURE_MICROSECONDS_PER_TICK 1000

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
