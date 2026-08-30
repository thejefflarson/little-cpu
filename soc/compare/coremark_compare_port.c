/*
 * The machine-side half of CoreMark for THIS directory's three-core harness.
 *
 * It implements test/bench/core_portme.h against machines that cannot all
 * time themselves: the VexRiscv configuration in the pinned riscv-formal
 * clone has no CSR file at all, and Hazard3's iCE40 configuration sets
 * `CSR_COUNTER=0` (docs/adr/0139), so `csrr mcycle` -- which is how
 * test/bench/coremark_port.c times a run -- is not an instruction either can
 * execute. The clock is therefore counted outside every core, the same way
 * soc/compare/dhry_port.c already does for Dhrystone: `start_time()` and
 * `stop_time()` each store a marker to the word at the base of RAM, and
 * soc/compare/coremark_tb.v counts cycles between the two markers on each
 * core's own bus. littlecpu is timed this way too, for the same reason
 * dhry_port.c gives -- one measured window has to mean the same thing on all
 * three cores.
 *
 * test/bench/coremark/ is the same vendored tree `make coremark` compiles,
 * unedited, and test/bench/core_portme.h -- the interface CoreMark's own
 * documentation asks every target to write -- is reused unmodified: it
 * declares no core-specific behaviour, only what a freestanding RV32 target
 * with no float and no libc needs. Only this port differs from
 * test/bench/coremark_port.c.
 *
 * ee_printf is a no-op here, not a working formatter. core_main.c's own
 * printed report -- sizes, per-algorithm CRCs, the pass/fail line -- is not
 * this measurement's business; soc/compare/coremark_tb.v's PASS/FAIL verdict,
 * published through the marker window below, is. A no-op that never
 * dereferences its format-string argument also means none of core_main.c's
 * own literal strings ever have to be read back out of memory at run time,
 * which matters because soc/compare/coremark.lds keeps `.rodata` out of ROM
 * for a different, load-bearing reason: core_state.c's own `intpat`/
 * `floatpat`/`scipat`/`errpat` string tables are read ALGORITHMICALLY by the
 * state-machine benchmark, not only handed to a printf, and
 * soc/compare/bench_vexriscv.v gives its core no data path to the ROM at all.
 */

#include "coremark.h"
#include "core_portme.h"
#include <stddef.h>

#ifndef COREMARK_FLAGS
#error "COREMARK_FLAGS must be defined with the exact compiler flags this was built with"
#endif
#ifndef ITERATIONS
#error "ITERATIONS must be defined -- the number of CoreMark iterations to run"
#endif

/* Compiled into the image so a reader of the ELF can recover the flags
 * without trusting a second copy of the string, the same reason
 * dhry_port.c's dhry_flags[] exists. */
const char coremark_flags[] __attribute__((used)) = COREMARK_FLAGS;

/* SEED_VOLATILE reads these -- the 2K performance run's own values, the same
 * ones test/bench/coremark_port.c and core_main.c's own default-value fixup
 * already recognise. seed4_volatile is the iteration count, ITERATIONS
 * arriving from the build the same way DHRY_RUNS does for Dhrystone. */
volatile ee_s32 seed1_volatile = 0;
volatile ee_s32 seed2_volatile = 0;
volatile ee_s32 seed3_volatile = 0x66;
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;

ee_u32 default_num_contexts = 1;

/* The two-word control window soc/compare/coremark.lds places at the base of
 * RAM, the address soc/compare/coremark_tb.v watches on every core's own
 * write bus. Its address is the linker script's to state and no literal here
 * repeats it. */
volatile unsigned coremark_ctl[2]
    __attribute__((section(".coremarkctl"), used));

static void publish(unsigned index, unsigned value) {
  /* The barriers are the measurement. Without them the compiler may move
   * work across the marker store, and the cycles counted would not be the
   * cycles the run took. */
  __asm__ volatile("" ::: "memory");
  coremark_ctl[index] = value;
  __asm__ volatile("" ::: "memory");
}

/* core_main.c calls this pair exactly once when `results[0].iterations` is
 * already nonzero (ITERATIONS above), which skips its own auto-calibration
 * loop entirely -- see its `if (results[0].iterations == 0)` guard. So marks
 * 0 and 1 below are the whole of the measured window, the same shape
 * dhry_port.c's begin/end pair uses. */
static unsigned marks;

void start_time(void) {
  publish(0, marks);
  marks++;
}

void stop_time(void) {
  publish(0, marks);
  marks++;
}

/* No counter behind either: the interval that matters is the one
 * soc/compare/coremark_tb.v takes on the bus, not anything computed here.
 * core_main.c only ever compares this against 0 or prints it, both harmless
 * with ITERATIONS nonzero. */
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

/* Ignores its arguments outright rather than formatting them -- see the file
 * header for why that is load-bearing rather than a shortcut. */
int ee_printf(const char *fmt, ...) {
  (void)fmt;
  return 0;
}

/* The report is the testbench's, not the program's: there is no console here,
 * and what this machine has to say is whether it validated -- 1 for pass, 3
 * for fail, 5 for "ran a configuration this port cannot check" -- in the
 * second control word, the riscv_test.h encoding every other program in this
 * repo uses for the odd two. */
enum coremark_verdict {
  COREMARK_PASS = 1,
  COREMARK_FAIL = 3,
  COREMARK_UNVALIDATED = 5,
};

/* The 2K performance run's seeds, size and EEMBC-published CRCs -- the same
 * literals test/bench/coremark_port.c states, copied rather than shared so
 * this port's verdict cannot silently move if that file's ever does. */
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
