// The machine-side half of CoreMark for THIS directory's three-core harness.

#include "coremark.h"
#include "core_portme.h"
#include <stddef.h>

#ifndef COREMARK_FLAGS
#error "COREMARK_FLAGS must be defined with the exact compiler flags this was built with"
#endif
#ifndef ITERATIONS
#error "ITERATIONS must be defined -- the number of CoreMark iterations to run"
#endif

// Compiled into the image so a reader of the ELF can recover the flags without trusting a
// second copy of the string, the same reason dhry_port.c's dhry_flags[] exists.
const char coremark_flags[] __attribute__((used)) = COREMARK_FLAGS;

volatile ee_s32 seed1_volatile = 0;
volatile ee_s32 seed2_volatile = 0;
volatile ee_s32 seed3_volatile = 0x66;
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;

ee_u32 default_num_contexts = 1;

volatile unsigned coremark_ctl[2]
    __attribute__((section(".coremarkctl"), used));

static void publish(unsigned index, unsigned value) {
  // The barriers are the measurement.
  __asm__ volatile("" ::: "memory");
  coremark_ctl[index] = value;
  __asm__ volatile("" ::: "memory");
}

static unsigned marks;

void start_time(void) {
  publish(0, marks);
  marks++;
}

void stop_time(void) {
  publish(0, marks);
  marks++;
}

CORE_TICKS get_time(void) { return 0; }

secs_ret time_in_secs(CORE_TICKS ticks) {
  (void)ticks;
  return 0;
}

void portable_init(core_portable *p, int *argc, char *argv[]) {
  (void)argc;
  (void)argv;
  p->portable_id = 1;
}

int ee_printf(const char *fmt, ...) {
  (void)fmt;
  return 0;
}

enum coremark_verdict {
  COREMARK_PASS = 1,
  COREMARK_FAIL = 3,
  COREMARK_UNVALIDATED = 5,
};

#define COREMARK_2K_SEED1 0
#define COREMARK_2K_SEED2 0
#define COREMARK_2K_SEED3 0x66
#define COREMARK_2K_SIZE 666
#define COREMARK_2K_CRCLIST 0xe714u
#define COREMARK_2K_CRCMATRIX 0x1fd7u
#define COREMARK_2K_CRCSTATE 0x8e3au

void portable_fini(core_portable *p) {
  core_results *res =
      (core_results *)((char *)p - offsetof(core_results, port));
  int ran_2k_performance = res->execs == ALL_ALGORITHMS_MASK &&
                            res->seed1 == COREMARK_2K_SEED1 &&
                            res->seed2 == COREMARK_2K_SEED2 &&
                            res->seed3 == COREMARK_2K_SEED3 &&
                            res->size == COREMARK_2K_SIZE;
  enum coremark_verdict verdict = COREMARK_UNVALIDATED;
  if (ran_2k_performance) {
    int crcs_ok = res->crclist == COREMARK_2K_CRCLIST &&
                  res->crcmatrix == COREMARK_2K_CRCMATRIX &&
                  res->crcstate == COREMARK_2K_CRCSTATE;
    verdict = (crcs_ok && check_data_types() == 0 && res->err == 0)
                  ? COREMARK_PASS
                  : COREMARK_FAIL;
  }
  publish(1, (unsigned)verdict);
  for (;;) {
  }
}
