/*
 * Deadlock Demo (Intentional)
 *
 * Two tasks acquire two mutexes in opposite order:
 *   - TaskA: M1 -> wait -> M2
 *   - TaskB: M2 -> wait -> M1
 *
 * This creates a circular wait => deadlock.
 *
 * No printf/snprintf. Output via rtems_putc.
 * Uses static task storage with rtems_task_construct().
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

/* ---- timing ---- */
static rtems_interval seconds_to_ticks(rtems_interval tps, rtems_interval sec)
{
  if (tps == 0) tps = 1;
  rtems_interval ticks = tps * sec;
  if (sec != 0 && ticks / sec != tps) ticks = (rtems_interval)-1;
  if (ticks < tps) ticks = tps;
  return ticks;
}

/* ---- mutexes ---- */
static rtems_id m1;
static rtems_id m2;

/* ---- task storage/config ---- */
#define MAX_TLS_SIZE RTEMS_ALIGN_UP(64, RTEMS_TASK_STORAGE_ALIGNMENT)
#define TASK_ATTRIBUTES RTEMS_DEFAULT_ATTRIBUTES
#define TASK_STORAGE_SIZE \
  RTEMS_TASK_STORAGE_SIZE( \
    MAX_TLS_SIZE + RTEMS_MINIMUM_STACK_SIZE, \
    TASK_ATTRIBUTES \
  )

RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT) static char taskA_storage[TASK_STORAGE_SIZE];
RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT) static char taskB_storage[TASK_STORAGE_SIZE];

static const rtems_task_config taskA_cfg = {
  .name = rtems_build_name('T','A',' ',' '),
  .initial_priority = 2, /* higher */
  .storage_area = taskA_storage,
  .storage_size = sizeof(taskA_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};

static const rtems_task_config taskB_cfg = {
  .name = rtems_build_name('T','B',' ',' '),
  .initial_priority = 3,
  .storage_area = taskB_storage,
  .storage_size = sizeof(taskB_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};

/* ---- Task A: lock M1 then M2 ---- */
static rtems_task taskA(rtems_task_argument arg)
{
  (void)arg;
  rtems_status_code sc;

  rtems_interval tps = rtems_clock_get_ticks_per_second();
  if (tps == 0) tps = 1;
  rtems_interval wait_ticks = seconds_to_ticks(tps, 2u); /* 2 seconds */

  out_ln("[A] start");

  out_ln("[A] obtain M1");
  sc = rtems_semaphore_obtain(m1, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  ASSERT_SUCCESS(sc);
  out_ln("[A] got M1");

  out_ln("[A] sleep before trying M2");
  sc = rtems_task_wake_after(wait_ticks);
  ASSERT_SUCCESS(sc);

  out_ln("[A] obtain M2 (this will deadlock)");
  sc = rtems_semaphore_obtain(m2, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  /* NEVER reaches here if deadlocked */
  ASSERT_SUCCESS(sc);

  out_ln("[A] got M2 (unexpected)");
  rtems_task_exit();
}

/* ---- Task B: lock M2 then M1 ---- */
static rtems_task taskB(rtems_task_argument arg)
{
  (void)arg;
  rtems_status_code sc;

  rtems_interval tps = rtems_clock_get_ticks_per_second();
  if (tps == 0) tps = 1;
  rtems_interval wait_ticks = seconds_to_ticks(tps, 2u); /* 2 seconds */

  out_ln("[B] start");

  out_ln("[B] obtain M2");
  sc = rtems_semaphore_obtain(m2, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  ASSERT_SUCCESS(sc);
  out_ln("[B] got M2");

  out_ln("[B] sleep before trying M1");
  sc = rtems_task_wake_after(wait_ticks);
  ASSERT_SUCCESS(sc);

  out_ln("[B] obtain M1 (this will deadlock)");
  sc = rtems_semaphore_obtain(m1, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  /* NEVER reaches here if deadlocked */
  ASSERT_SUCCESS(sc);

  out_ln("[B] got M1 (unexpected)");
  rtems_task_exit();
}

/* ---- Init ---- */
static void Init(rtems_task_argument arg)
{
  (void)arg;
  rtems_status_code sc;

  out_ln("\n=== Deadlock Demo (Intentional) ===");
  out_ln("TaskA: M1 -> wait -> M2");
  out_ln("TaskB: M2 -> wait -> M1");
  out_ln("Expected: Both block forever (deadlock).");
  out_ln("");

  /* Use binary semaphores as mutexes */
  sc = rtems_semaphore_create(
    rtems_build_name('M','1',' ',' '),
    1,
    RTEMS_BINARY_SEMAPHORE,
    0,
    &m1
  );
  ASSERT_SUCCESS(sc);

  sc = rtems_semaphore_create(
    rtems_build_name('M','2',' ',' '),
    1,
    RTEMS_BINARY_SEMAPHORE,
    0,
    &m2
  );
  ASSERT_SUCCESS(sc);

  rtems_id tid;

  sc = rtems_task_construct(&taskA_cfg, &tid);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(tid, taskA, 0);
  ASSERT_SUCCESS(sc);

  sc = rtems_task_construct(&taskB_cfg, &tid);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(tid, taskB, 0);
  ASSERT_SUCCESS(sc);

  /* Init prints a heartbeat so you can see system is alive */
  rtems_interval tps = rtems_clock_get_ticks_per_second();
  if (tps == 0) tps = 1;

  for (uint32_t hb = 0; ; ++hb) {
    out_string("[Init] heartbeat=");
    out_u32(hb);
    out_string(" (if A/B stop printing => deadlock happened)\n");
    (void) rtems_task_wake_after(tps); /* 1 second */
  }
}

/* ---- RTEMS Configuration ---- */
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_MAXIMUM_PROCESSORS 2

/* Init + A + B */
#define CONFIGURE_MAXIMUM_TASKS 3
#define CONFIGURE_MINIMUM_TASKS_WITH_USER_PROVIDED_STORAGE \
  CONFIGURE_MAXIMUM_TASKS

/* m1, m2 */
#define CONFIGURE_MAXIMUM_SEMAPHORES 2

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
