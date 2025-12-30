/*
When the execution is complete, 
it indicates how many times each of the three tasks was preempted.
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

/* Output queue (char-by-char) */
#define MSG_MAX_PENDING 32
#define MSG_SIZE sizeof(char)

/* ===== Periods (seconds): scaled down by /10 ===== */
#define T1_PERIOD_SEC  15
#define T2_PERIOD_SEC  30
#define T3_PERIOD_SEC  45

/* ===== Priorities (RTEMS: smaller number => higher priority) =====
 * RMS rule: shorter period => higher priority
 */
#define PRIO_T1  3
#define PRIO_T2  4
#define PRIO_T3  5

/* ===== Stop condition: total events ===== */
#define TOTAL_EVENTS 1000

static rtems_id putc_done;
static rtems_id msg_queue;

/* mutex-like semaphore to protect shared counters */
static rtems_id cnt_lock;

/* Shared counters (protected by cnt_lock) */
static uint32_t total_count;
static uint32_t cnt_t1, cnt_t2, cnt_t3;
static volatile bool done;

/* --- task storage --- */
RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT)
static char msg_task_storage[TASK_STORAGE_SIZE];

RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT)
static char t1_storage[TASK_STORAGE_SIZE];

RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT)
static char t2_storage[TASK_STORAGE_SIZE];

RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT)
static char t3_storage[TASK_STORAGE_SIZE];

/* --- task configs --- */
static const rtems_task_config msg_task_config = {
  .name = rtems_build_name('M','S','G',' '),
  .initial_priority = 2, /* keep output responsive */
  .storage_area = msg_task_storage,
  .storage_size = sizeof(msg_task_storage),
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

/* --- message queue buffers/config --- */
static RTEMS_MESSAGE_QUEUE_BUFFER(MSG_SIZE) msg_buffers[MSG_MAX_PENDING];

static const rtems_message_queue_config msg_queue_config = {
  .name = rtems_build_name('Q','O','U','T'),
  .maximum_pending_messages = RTEMS_ARRAY_SIZE(msg_buffers),
  .maximum_message_size = MSG_SIZE,
  .storage_area = msg_buffers,
  .storage_size = sizeof(msg_buffers),
  .attributes = RTEMS_DEFAULT_ATTRIBUTES
};

/* ===== output helpers (no libc) ===== */
static void msg_task(rtems_task_argument arg)
{
  (void) arg;

  while (true) {
    rtems_status_code sc;
    char c;
    size_t n = 0;

    sc = rtems_message_queue_receive(msg_queue, &c, &n, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    ASSERT_SUCCESS(sc);

    if (n != sizeof(char)) {
      rtems_fatal(RTEMS_FATAL_SOURCE_EXIT, __LINE__);
    }

    rtems_putc(c);

    sc = rtems_semaphore_release(putc_done);
    ASSERT_SUCCESS(sc);
  }
}

static void wait_putc_done(void)
{
  rtems_status_code sc =
    rtems_semaphore_obtain(putc_done, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  ASSERT_SUCCESS(sc);
}

static void send_char(char c, size_t *pending)
{
  rtems_status_code sc;

  if (*pending >= MSG_MAX_PENDING) {
    wait_putc_done();
    --(*pending);
  }

  sc = rtems_message_queue_send(msg_queue, &c, sizeof(c));
  ASSERT_SUCCESS(sc);

  ++(*pending);
}

static void flush_pending(size_t *pending)
{
  while (*pending > 0) {
    wait_putc_done();
    --(*pending);
  }
}

static void send_string(const char *s)
{
  size_t pending = 0;
  while (*s != '\0') {
    send_char(*s++, &pending);
  }
  flush_pending(&pending);
}

static void send_int32(int32_t v)
{
  char buf[12];
  int i = 0;
  size_t pending = 0;

  if (v == 0) {
    send_char('0', &pending);
    flush_pending(&pending);
    return;
  }

  if (v < 0) {
    send_char('-', &pending);
    if (v == INT32_MIN) {
      const char *min_str = "2147483648";
      while (*min_str) send_char(*min_str++, &pending);
      flush_pending(&pending);
      return;
    }
    v = -v;
  }

  while (v > 0 && i < (int)sizeof(buf)) {
    buf[i++] = (char)('0' + (v % 10));
    v /= 10;
  }

  while (i-- > 0) {
    send_char(buf[i], &pending);
  }

  flush_pending(&pending);
}

static rtems_interval sec_to_ticks(rtems_interval tps, rtems_interval sec)
{
  if (tps == 0) tps = 1;
  rtems_interval ticks = tps * sec;
  if (ticks < tps) ticks = tps;
  return ticks;
}

/* ===== busy work to make scheduling more visible (optional) ===== */
static void busy_work(uint32_t loops)
{
  volatile uint32_t x = 0;
  for (uint32_t i = 0; i < loops; ++i) {
    x ^= (i * 1664525u) + 1013904223u;
  }
}

/* ===== record one event (protected) ===== */
static void record_event(uint8_t owner_id)
{
  rtems_status_code sc =
    rtems_semaphore_obtain(cnt_lock, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  ASSERT_SUCCESS(sc);

  if (!done && total_count < TOTAL_EVENTS) {
    if (owner_id == 1) ++cnt_t1;
    else if (owner_id == 2) ++cnt_t2;
    else if (owner_id == 3) ++cnt_t3;

    ++total_count;

    if (total_count >= TOTAL_EVENTS) {
      done = true;
    }
  }

  sc = rtems_semaphore_release(cnt_lock);
  ASSERT_SUCCESS(sc);
}

/* ===== periodic task template ===== */
static void periodic_body(uint8_t owner_id,
                          uint32_t prio,
                          rtems_interval period_ticks,
                          uint32_t busy_loops,
                          rtems_name period_name)
{
  rtems_id period_id;
  rtems_status_code sc;

  sc = rtems_rate_monotonic_create(period_name, &period_id);
  ASSERT_SUCCESS(sc);

  /* One-time header */
  send_string("\n--- T");
  send_int32((int32_t)owner_id);
  send_string(" started --- prio=");
  send_int32((int32_t)prio);
  send_string(" period_ticks=");
  send_int32((int32_t)period_ticks);
  send_string("\n");

  for (;;) {
    if (done) {
      rtems_task_suspend(RTEMS_SELF);
    }

    busy_work(busy_loops);
    record_event(owner_id);

    sc = rtems_rate_monotonic_period(period_id, period_ticks);
    if (sc != RTEMS_SUCCESSFUL) {
      send_string("*** T");
      send_int32((int32_t)owner_id);
      send_string(" period status=");
      send_int32((int32_t)sc);
      send_string("\n");
    }
  }
}

static void t1_task(rtems_task_argument arg)
{
  (void)arg;
  rtems_interval tps = rtems_clock_get_ticks_per_second();
  periodic_body(
    1, PRIO_T1,
    sec_to_ticks(tps, T1_PERIOD_SEC),
    300000u,
    rtems_build_name('P','1',' ',' ')
  );
}

static void t2_task(rtems_task_argument arg)
{
  (void)arg;
  rtems_interval tps = rtems_clock_get_ticks_per_second();
  periodic_body(
    2, PRIO_T2,
    sec_to_ticks(tps, T2_PERIOD_SEC),
    450000u,
    rtems_build_name('P','2',' ',' ')
  );
}

static void t3_task(rtems_task_argument arg)
{
  (void)arg;
  rtems_interval tps = rtems_clock_get_ticks_per_second();
  periodic_body(
    3, PRIO_T3,
    sec_to_ticks(tps, T3_PERIOD_SEC),
    600000u,
    rtems_build_name('P','3',' ',' ')
  );
}

/* ===== summary ===== */
static void print_summary_and_exit(void)
{
  send_string("\n\n=== DONE ===\nTotal count = ");
  send_int32((int32_t)total_count);
  send_string("\n\n");

  send_string("T1 total = ");
  send_int32((int32_t)cnt_t1);
  send_string("\n");

  send_string("T2 total = ");
  send_int32((int32_t)cnt_t2);
  send_string("\n");

  send_string("T3 total = ");
  send_int32((int32_t)cnt_t3);
  send_string("\n");

  /* Optional: expected ratio for 15/30/45 -> 6:3:2 */
  send_string("\nExpected frequency ratio for periods 15/30/45 is ~ 6:3:2\n");

  rtems_fatal(RTEMS_FATAL_SOURCE_EXIT, 0);
}

/* ===== Init ===== */
static void Init(rtems_task_argument arg)
{
  (void)arg;

  rtems_status_code sc;
  rtems_id msg_id, id1, id2, id3;

  total_count = 0;
  cnt_t1 = cnt_t2 = cnt_t3 = 0;
  done = false;

  sc = rtems_semaphore_create(
    rtems_build_name('P','U','T','C'),
    0,
    RTEMS_COUNTING_SEMAPHORE,
    0,
    &putc_done
  );
  ASSERT_SUCCESS(sc);

  /* lock for counters */
  sc = rtems_semaphore_create(
    rtems_build_name('C','N','T','L'),
    1,
    RTEMS_BINARY_SEMAPHORE,
    0,
    &cnt_lock
  );
  ASSERT_SUCCESS(sc);

  sc = rtems_message_queue_construct(&msg_queue_config, &msg_queue);
  ASSERT_SUCCESS(sc);

  sc = rtems_task_construct(&msg_task_config, &msg_id);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(msg_id, msg_task, 0);
  ASSERT_SUCCESS(sc);

  send_string("\n=== RMS counters-only demo (stop at 1000) ===\n");
  send_string("T1: prio="); send_int32(PRIO_T1); send_string(" period="); send_int32(T1_PERIOD_SEC); send_string("s\n");
  send_string("T2: prio="); send_int32(PRIO_T2); send_string(" period="); send_int32(T2_PERIOD_SEC); send_string("s\n");
  send_string("T3: prio="); send_int32(PRIO_T3); send_string(" period="); send_int32(T3_PERIOD_SEC); send_string("s\n\n");

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

  while (!done) {
    rtems_task_wake_after(rtems_clock_get_ticks_per_second());
  }

  print_summary_and_exit();
}

/* ===== RTEMS Config ===== */
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_MAXIMUM_PROCESSORS 2

#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES 1
#define CONFIGURE_MAXIMUM_SEMAPHORES 2   /* putc_done + cnt_lock */
#define CONFIGURE_MAXIMUM_PERIODS 3      /* P1, P2, P3 */
#define CONFIGURE_MAXIMUM_TASKS 5        /* Init + msg + t1 + t2 + t3 */
#define CONFIGURE_MINIMUM_TASKS_WITH_USER_PROVIDED_STORAGE CONFIGURE_MAXIMUM_TASKS

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
