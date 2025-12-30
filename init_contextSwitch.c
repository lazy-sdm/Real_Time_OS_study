/*
 * Task State & Context Switch observation (qual-only friendly, no printf/snprintf)
 *
 * Tasks:
 *  - OUT : prints chars via rtems_putc()
 *  - H   : high priority periodic-ish task (Running -> Blocked by wake_after)
 *  - W   : waits on semaphore (Blocked -> Ready -> Running when released)
 *  - R   : low priority runner (runs when others are blocked; can be Suspended/Resumed)
 *
 * Controller: Init task periodically releases semaphore and suspends/resumes R
 *
 * Expected observations (by timestamps and interleaving):
 *  - When H wakes up, it preempts R (context switch to higher prio)
 *  - When Init releases sem_work, W unblocks and preempts R
 *  - When R is suspended, you won't see [R] logs until resumed
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems.h>
#include <rtems/bspIo.h>
#include <stdint.h>
#include <stdbool.h>

/* ---------- minimal output (no libc) ---------- */
#define OUT_MAX_PENDING 256
#define OUT_MSG_SIZE sizeof(char)

static rtems_id putc_done;
static rtems_id out_q;

static RTEMS_MESSAGE_QUEUE_BUFFER( OUT_MSG_SIZE ) out_buffers[ OUT_MAX_PENDING ];

static const rtems_message_queue_config out_q_config = {
  .name = rtems_build_name('Q','O','U','T'),
  .maximum_pending_messages = RTEMS_ARRAY_SIZE(out_buffers),
  .maximum_message_size = OUT_MSG_SIZE,
  .storage_area = out_buffers,
  .storage_size = sizeof(out_buffers),
  .attributes = RTEMS_DEFAULT_ATTRIBUTES
};

#define ASSERT_SUCCESS(sc) \
  do { \
    if ((sc) != RTEMS_SUCCESSFUL) { \
      rtems_fatal(RTEMS_FATAL_SOURCE_EXIT, __LINE__); \
    } \
  } while (0)

#define MAX_TLS_SIZE RTEMS_ALIGN_UP( 64, RTEMS_TASK_STORAGE_ALIGNMENT )
#define TASK_ATTRIBUTES RTEMS_DEFAULT_ATTRIBUTES

#define TASK_STORAGE_SIZE \
  RTEMS_TASK_STORAGE_SIZE( \
    MAX_TLS_SIZE + RTEMS_MINIMUM_STACK_SIZE, \
    TASK_ATTRIBUTES \
  )

/* ---------- priorities (smaller number = higher priority) ---------- */
#define PRIO_OUT   2
#define PRIO_H     3
#define PRIO_W     5
#define PRIO_R     10

/* ---------- timing (ticks) ---------- */
#define H_PERIOD_TICKS        10000   /* H runs then sleeps */
#define R_PRINT_EVERY_TICKS   4000  /* R prints progress */
#define INIT_RELEASE_EVERY    24000  /* Init releases semaphore periodically */
#define INIT_SUSPEND_AT       80000  /* at this time, suspend R */
#define INIT_RESUME_AT        140000  /* at this time, resume R */

/* ---------- shared objects ---------- */
static rtems_id sem_work;   /* W blocks on this */
static rtems_id r_task_id;  /* so Init can suspend/resume R */

/* ---------- task storage ---------- */
RTEMS_ALIGNED( RTEMS_TASK_STORAGE_ALIGNMENT )
static char out_task_storage[ TASK_STORAGE_SIZE ];

RTEMS_ALIGNED( RTEMS_TASK_STORAGE_ALIGNMENT )
static char h_task_storage[ TASK_STORAGE_SIZE ];

RTEMS_ALIGNED( RTEMS_TASK_STORAGE_ALIGNMENT )
static char w_task_storage[ TASK_STORAGE_SIZE ];

RTEMS_ALIGNED( RTEMS_TASK_STORAGE_ALIGNMENT )
static char r_task_storage[ TASK_STORAGE_SIZE ];

/* ---------- task configs ---------- */
static const rtems_task_config out_task_config = {
  .name = rtems_build_name('O','U','T',' '),
  .initial_priority = PRIO_OUT,
  .storage_area = out_task_storage,
  .storage_size = sizeof(out_task_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};

static const rtems_task_config h_task_config = {
  .name = rtems_build_name('H','I','G','H'),
  .initial_priority = PRIO_H,
  .storage_area = h_task_storage,
  .storage_size = sizeof(h_task_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};

static const rtems_task_config w_task_config = {
  .name = rtems_build_name('W','A','I','T'),
  .initial_priority = PRIO_W,
  .storage_area = w_task_storage,
  .storage_size = sizeof(w_task_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};

static const rtems_task_config r_task_config = {
  .name = rtems_build_name('R','U','N',' '),
  .initial_priority = PRIO_R,
  .storage_area = r_task_storage,
  .storage_size = sizeof(r_task_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};

/* ---------- output helpers ---------- */
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

static void out_uint32(uint32_t v)
{
  char buf[11];
  int i = 0;
  size_t pending = 0;

  if (v == 0) {
    send_char_out('0', &pending);
    flush_out(&pending);
    return;
  }

  while (v > 0 && i < (int)sizeof(buf)) {
    buf[i++] = (char)('0' + (v % 10u));
    v /= 10u;
  }

  while (i-- > 0) {
    send_char_out(buf[i], &pending);
  }

  flush_out(&pending);
}

static void out_ts_prefix(const char *tag)
{
  out_string("[t=");
  out_uint32((uint32_t) rtems_clock_get_ticks_since_boot());
  out_string("] ");
  out_string(tag);
}

/* ---------- OUT task ---------- */
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

/* ---------- H task: Running -> Blocked(sleep) 반복 ---------- */
static void h_task(rtems_task_argument arg)
{
  (void)arg;

  for (uint32_t k = 0;; ++k) {
    out_ts_prefix("[H] running, will sleep\n");

    /* 짧은 계산(눈에 띄는 “실행”) */
    volatile uint32_t x = 0;
    for (uint32_t i = 0; i < 50000u; ++i) {
      x ^= (i + k);
    }
    (void)x;

    out_ts_prefix("[H] -> Blocked (wake_after)\n");
    rtems_status_code sc = rtems_task_wake_after(H_PERIOD_TICKS);
    ASSERT_SUCCESS(sc);
    /* 깨어나면 Ready->Running으로 다시 선점할 수 있음 */
  }
}

/* ---------- W task: Blocked(semaphore) -> Ready -> Running ---------- */
static void w_task(rtems_task_argument arg)
{
  (void)arg;

  for (uint32_t k = 0;; ++k) {
    out_ts_prefix("[W] -> Blocked (semaphore_obtain)\n");

    rtems_status_code sc =
      rtems_semaphore_obtain(sem_work, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    ASSERT_SUCCESS(sc);

    out_ts_prefix("[W] acquired sem, running\n");

    /* 짧은 처리 후 다시 loop (다시 Blocked로 들어감) */
    volatile uint32_t y = 1;
    for (uint32_t i = 0; i < 30000u; ++i) {
      y = (y * 1664525u) + 1013904223u;
    }
    (void)y;

    out_ts_prefix("[W] done, will wait again\n");
  }
}

/* ---------- R task: 낮은 우선순위로 “틈새 CPU” 사용 ---------- */
static void r_task(rtems_task_argument arg)
{
  (void)arg;

  uint32_t last_print = rtems_clock_get_ticks_since_boot();

  for (uint32_t k = 0;; ++k) {
    /* 아주 약간 실행(계속 Ready/Running일 수 있음) */
    volatile uint32_t z = k;
    for (uint32_t i = 0; i < 10000u; ++i) {
      z ^= (i * 3u);
    }
    (void)z;

    uint32_t now = (uint32_t) rtems_clock_get_ticks_since_boot();
    if ((uint32_t)(now - last_print) >= R_PRINT_EVERY_TICKS) {
      last_print = now;
      out_ts_prefix("[R] running (low prio)\n");
    }

    /* 너무 독점하지 않게 아주 짧게 yield 느낌 */
    (void) rtems_task_wake_after(1);
  }
}

/* ---------- Init task (controller) ---------- */
static void Init(rtems_task_argument arg)
{
  (void)arg;

  rtems_status_code sc;
  rtems_id out_id, h_id, w_id;

  /* output semaphore */
  sc = rtems_semaphore_create(
    rtems_build_name('P','U','T','C'),
    0,
    RTEMS_COUNTING_SEMAPHORE,
    0,
    &putc_done
  );
  ASSERT_SUCCESS(sc);

  /* work semaphore (W blocks here) */
  sc = rtems_semaphore_create(
    rtems_build_name('W','O','R','K'),
    0,
    RTEMS_COUNTING_SEMAPHORE,
    0,
    &sem_work
  );
  ASSERT_SUCCESS(sc);

  /* output queue */
  sc = rtems_message_queue_construct(&out_q_config, &out_q);
  ASSERT_SUCCESS(sc);

  /* start OUT */
  sc = rtems_task_construct(&out_task_config, &out_id);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(out_id, out_task, 0);
  ASSERT_SUCCESS(sc);

  out_string("\n=== Task State & Context Switch Demo ===\n");
  out_string("States to observe:\n");
  out_string(" - H: Running -> Blocked(wake_after)\n");
  out_string(" - W: Blocked(semaphore) -> Ready -> Running (when Init releases)\n");
  out_string(" - R: Low prio runs when others blocked; can be Suspended/Resumed\n\n");

  /* start H, W, R */
  sc = rtems_task_construct(&h_task_config, &h_id);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(h_id, h_task, 0);
  ASSERT_SUCCESS(sc);

  sc = rtems_task_construct(&w_task_config, &w_id);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(w_id, w_task, 0);
  ASSERT_SUCCESS(sc);

  sc = rtems_task_construct(&r_task_config, &r_task_id);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(r_task_id, r_task, 0);
  ASSERT_SUCCESS(sc);

  /* controller loop */
  uint32_t last_release = (uint32_t) rtems_clock_get_ticks_since_boot();
  bool suspended = false;

  for (;;) {
    uint32_t now = (uint32_t) rtems_clock_get_ticks_since_boot();

    /* periodically unblock W */
    if ((uint32_t)(now - last_release) >= INIT_RELEASE_EVERY) {
      last_release = now;
      out_ts_prefix("[Init] releasing sem_work -> W becomes Ready\n");
      sc = rtems_semaphore_release(sem_work);
      ASSERT_SUCCESS(sc);
    }

    /* show Suspended/Resumed */
    if (!suspended && now >= INIT_SUSPEND_AT) {
      suspended = true;
      out_ts_prefix("[Init] suspending R -> R becomes Suspended\n");
      sc = rtems_task_suspend(r_task_id);
      ASSERT_SUCCESS(sc);
    }

    if (suspended && now >= INIT_RESUME_AT) {
      suspended = false;
      out_ts_prefix("[Init] resuming R -> R becomes Ready\n");
      sc = rtems_task_resume(r_task_id);
      ASSERT_SUCCESS(sc);
    }

    /* keep Init from busy looping */
    (void) rtems_task_wake_after(5);
  }
}

/* ---------- RTEMS Configuration ---------- */
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_MAXIMUM_PROCESSORS 2

/* tasks: Init + OUT + H + W + R = 5 */
#define CONFIGURE_MAXIMUM_TASKS 5
#define CONFIGURE_MINIMUM_TASKS_WITH_USER_PROVIDED_STORAGE \
  CONFIGURE_MAXIMUM_TASKS

/* semaphores: putc_done + sem_work = 2 */
#define CONFIGURE_MAXIMUM_SEMAPHORES 2

/* message queues: out_q = 1 */
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES 1

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
