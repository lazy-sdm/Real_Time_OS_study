/* Performance per protocol is shown when execution is complete. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems.h>
#include <rtems/bspIo.h>
#include <stdint.h>
#include <stdbool.h>

/* 0: none, 1: inherit, 2: ceiling */
#define PROTOCOL 1

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

/* -------- output queue (no libc) -------- */
#define OUT_MAX_PENDING 128
#define OUT_MSG_SIZE sizeof(char)

static rtems_id putc_done;
static rtems_id out_q;

static RTEMS_MESSAGE_QUEUE_BUFFER(OUT_MSG_SIZE) out_buffers[OUT_MAX_PENDING];

static const rtems_message_queue_config out_q_config = {
  .name = rtems_build_name('Q','O','U','T'),
  .maximum_pending_messages = RTEMS_ARRAY_SIZE(out_buffers),
  .maximum_message_size = OUT_MSG_SIZE,
  .storage_area = out_buffers,
  .storage_size = sizeof(out_buffers),
  .attributes = RTEMS_DEFAULT_ATTRIBUTES
};

/* -------- priorities (smaller number => higher priority) -------- */
#define PRIO_H  3
#define PRIO_M  5
#define PRIO_L  10

/* Work amounts (tune if needed) */
#define BUSY_L  800000u
#define BUSY_M  9000000u

/* Sequence control:
 * L locks mutex first.
 * Then we start M (to try to preempt/delay L).
 * Then after a short delay we start H to request the mutex.
 */
#define START_H_DELAY_TICKS  5

/* -------- shared objects -------- */
static rtems_id mutex_id;
static rtems_id sem_L_locked;
static rtems_id sem_L_done;
static rtems_id sem_M_done;
static rtems_id sem_H_done;

static rtems_interval tick_H_request;
static rtems_interval tick_H_acquire;

/* -------- task storage -------- */
RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT)
static char out_task_storage[TASK_STORAGE_SIZE];
RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT)
static char h_storage[TASK_STORAGE_SIZE];
RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT)
static char m_storage[TASK_STORAGE_SIZE];
RTEMS_ALIGNED(RTEMS_TASK_STORAGE_ALIGNMENT)
static char l_storage[TASK_STORAGE_SIZE];

/* -------- task configs -------- */
static const rtems_task_config out_task_config = {
  .name = rtems_build_name('O','U','T',' '),
  .initial_priority = 2,
  .storage_area = out_task_storage,
  .storage_size = sizeof(out_task_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};

static const rtems_task_config h_config = {
  .name = rtems_build_name('H','I','G','H'),
  .initial_priority = PRIO_H,
  .storage_area = h_storage,
  .storage_size = sizeof(h_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};

static const rtems_task_config m_config = {
  .name = rtems_build_name('M','E','D',' '),
  .initial_priority = PRIO_M,
  .storage_area = m_storage,
  .storage_size = sizeof(m_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};

static const rtems_task_config l_config = {
  .name = rtems_build_name('L','O','W',' '),
  .initial_priority = PRIO_L,
  .storage_area = l_storage,
  .storage_size = sizeof(l_storage),
  .maximum_thread_local_storage_size = MAX_TLS_SIZE,
  .initial_modes = RTEMS_DEFAULT_MODES,
  .attributes = TASK_ATTRIBUTES
};

/* ===== output helpers (no libc) ===== */
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

static void out_uint32(uint32_t v) { out_int32((int32_t)v); }

/* -------- out task -------- */
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

/* -------- busy loop -------- */
static void busy_work(uint32_t loops)
{
  volatile uint32_t x = 0;
  for (uint32_t i = 0; i < loops; ++i) {
    x ^= (i * 1664525u) + 1013904223u;
  }
}

static void sem_signal(rtems_id sem)
{
  rtems_status_code sc = rtems_semaphore_release(sem);
  ASSERT_SUCCESS(sc);
}

static void sem_wait(rtems_id sem)
{
  rtems_status_code sc = rtems_semaphore_obtain(sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  ASSERT_SUCCESS(sc);
}

/* ===== L task ===== */
static void l_task(rtems_task_argument arg)
{
  (void)arg;

  rtems_status_code sc;

  sc = rtems_semaphore_obtain(mutex_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  ASSERT_SUCCESS(sc);

  out_string("[L] locked mutex tick=");
  out_uint32((uint32_t)rtems_clock_get_ticks_since_boot());
  out_string("\n");

  sem_signal(sem_L_locked);

  /* critical section: if PROTOCOL=CEILING, M should NOT be able to preempt here */
  busy_work(BUSY_L);

  out_string("[L] releasing mutex tick=");
  out_uint32((uint32_t)rtems_clock_get_ticks_since_boot());
  out_string("\n");

  sc = rtems_semaphore_release(mutex_id);
  ASSERT_SUCCESS(sc);

  sem_signal(sem_L_done);
  rtems_task_suspend(RTEMS_SELF);
}

/* ===== H task ===== */
static void h_task(rtems_task_argument arg)
{
  (void)arg;

  out_string("[H] request mutex tick=");
  tick_H_request = rtems_clock_get_ticks_since_boot();
  out_uint32((uint32_t)tick_H_request);
  out_string("\n");

  rtems_status_code sc =
    rtems_semaphore_obtain(mutex_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  ASSERT_SUCCESS(sc);

  tick_H_acquire = rtems_clock_get_ticks_since_boot();

  out_string("[H] acquired mutex tick=");
  out_uint32((uint32_t)tick_H_acquire);
  out_string(" wait_ticks=");
  out_uint32((uint32_t)(tick_H_acquire - tick_H_request));
  out_string("\n");

  sc = rtems_semaphore_release(mutex_id);
  ASSERT_SUCCESS(sc);

  sem_signal(sem_H_done);
  rtems_task_suspend(RTEMS_SELF);
}

/* ===== M task ===== */
static void m_task(rtems_task_argument arg)
{
  (void)arg;

  out_string("[M] start hog tick=");
  out_uint32((uint32_t)rtems_clock_get_ticks_since_boot());
  out_string("\n");

  /* If L is not boosted (NONE / late INHERIT), M can steal CPU and delay L */
  busy_work(BUSY_M);

  out_string("[M] end hog tick=");
  out_uint32((uint32_t)rtems_clock_get_ticks_since_boot());
  out_string("\n");

  sem_signal(sem_M_done);
  rtems_task_suspend(RTEMS_SELF);
}

/* ===== Init ===== */
static void Init(rtems_task_argument arg)
{
  (void)arg;

  rtems_status_code sc;
  rtems_id out_id, hid, mid, lid;

  /* putc_done + output queue */
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

  /* start output task */
  sc = rtems_task_construct(&out_task_config, &out_id);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(out_id, out_task, 0);
  ASSERT_SUCCESS(sc);

  out_string("\n=== Priority Ceiling Demo ===\n");
  out_string("PROTOCOL=");
  out_uint32((uint32_t)PROTOCOL);
  out_string(" (0=NONE, 1=INHERIT, 2=CEILING)\n");
  out_string("PRIO(H/M/L)=");
  out_uint32(PRIO_H); out_string("/");
  out_uint32(PRIO_M); out_string("/");
  out_uint32(PRIO_L); out_string("\n");
  out_string("BUSY_L="); out_uint32(BUSY_L);
  out_string(" BUSY_M="); out_uint32(BUSY_M);
  out_string(" START_H_DELAY_TICKS="); out_uint32(START_H_DELAY_TICKS);
  out_string("\n\n");

  /* sync semaphores */
  sc = rtems_semaphore_create(rtems_build_name('L','L','O','K'), 0,
                             RTEMS_COUNTING_SEMAPHORE, 0, &sem_L_locked);
  ASSERT_SUCCESS(sc);

  sc = rtems_semaphore_create(rtems_build_name('L','D','O','N'), 0,
                             RTEMS_COUNTING_SEMAPHORE, 0, &sem_L_done);
  ASSERT_SUCCESS(sc);

  sc = rtems_semaphore_create(rtems_build_name('M','D','O','N'), 0,
                             RTEMS_COUNTING_SEMAPHORE, 0, &sem_M_done);
  ASSERT_SUCCESS(sc);

  sc = rtems_semaphore_create(rtems_build_name('H','D','O','N'), 0,
                             RTEMS_COUNTING_SEMAPHORE, 0, &sem_H_done);
  ASSERT_SUCCESS(sc);

  /* create mutex with selected protocol */
  rtems_attribute attr = RTEMS_BINARY_SEMAPHORE | RTEMS_PRIORITY;

  rtems_task_priority ceiling_prio = 0;

#if PROTOCOL == 1
  attr |= RTEMS_INHERIT_PRIORITY;
#elif PROTOCOL == 2
  attr |= RTEMS_PRIORITY_CEILING;
  ceiling_prio = PRIO_H; /* ceiling must be as high as the highest user (H) */
#endif

  sc = rtems_semaphore_create(
    rtems_build_name('M','U','T','X'),
    1,
    attr,
    ceiling_prio,
    &mutex_id
  );
  ASSERT_SUCCESS(sc);

  /* Start L first -> it locks mutex */
  sc = rtems_task_construct(&l_config, &lid);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(lid, l_task, 0);
  ASSERT_SUCCESS(sc);

  sem_wait(sem_L_locked);

  /* Start M next -> try to hog CPU and delay L */
  sc = rtems_task_construct(&m_config, &mid);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(mid, m_task, 0);
  ASSERT_SUCCESS(sc);

  /* Delay a bit, then start H -> H requests mutex later */
  (void) rtems_task_wake_after(START_H_DELAY_TICKS);

  sc = rtems_task_construct(&h_config, &hid);
  ASSERT_SUCCESS(sc);
  sc = rtems_task_start(hid, h_task, 0);
  ASSERT_SUCCESS(sc);

  /* wait completion */
  sem_wait(sem_H_done);
  sem_wait(sem_M_done);
  sem_wait(sem_L_done);

  out_string("\n=== Summary ===\n");
  out_string("H_wait_ticks=");
  out_uint32((uint32_t)(tick_H_acquire - tick_H_request));
  out_string("\n");

#if PROTOCOL == 0
  out_string("Expected: largest wait (M can delay L a lot)\n");
#elif PROTOCOL == 1
  out_string("Expected: reduced wait AFTER H blocks (but M may delay before H requests)\n");
#else
  out_string("Expected: smallest wait (L boosted immediately when locking mutex)\n");
#endif

  rtems_fatal(RTEMS_FATAL_SOURCE_EXIT, 0);
}

/* ===== RTEMS config ===== */
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_MAXIMUM_PROCESSORS 2

/* Tasks: Init + OUT + H + M + L = 5 */
#define CONFIGURE_MAXIMUM_TASKS 5
#define CONFIGURE_MINIMUM_TASKS_WITH_USER_PROVIDED_STORAGE CONFIGURE_MAXIMUM_TASKS

/* Semaphores: putc_done + mutex + (L_locked, L_done, M_done, H_done) = 6 */
#define CONFIGURE_MAXIMUM_SEMAPHORES 6

/* Queue: out_q */
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES 1

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
