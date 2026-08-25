/*
 * The porting layer CoreMark's own documentation asks for: seeds it cannot
 * read from argv, a timer, a memory block, and a report. Nothing in
 * test/bench/coremark/ is touched -- every function CoreMark calls out to is
 * defined here instead, the same split test/bench/dhry_port.c uses for
 * Dhrystone.
 *
 * THE REPORT IS THIS FILE'S OWN OUTPUT, not core_main.c's. core_main.c prints
 * its own text (sizes, per-algorithm CRCs, a pass/fail verdict) and that stays
 * exactly as EEMBC wrote it. What decides PASS/FAIL for this simulation and
 * prints the CoreMark/MHz figure is coremark_report() below, called from
 * portable_fini() after core_main.c's own report has already gone out. It
 * reads a live core_results back through the core_portable pointer
 * portable_fini() is handed, via the struct layout coremark.h declares --
 * `port` is that struct's last member, so this is offsetof arithmetic on a
 * type this file never redefines, not a guess at one it does.
 *
 * portable_fini() DOES NOT TRUST core_main.c's `err` FIELD ALONE. `err` stays
 * zero whenever the seed/size CRC does not match one of EEMBC's five known
 * configurations -- core_main.c sets its own local `known_id` to -1, skips the
 * whole CRC comparison, prints "Cannot validate operation for these seed
 * values", and never touches `err`. `TOTAL_DATA_SIZE` is an overridable build
 * flag, so a run built with a different data size would sail through that gap
 * with a PASS neither this file nor core_main.c ever checked. So this file
 * re-derives "the checked configuration ran" itself, against the same
 * literals core_main.c's switch on `seedcrc` case 0xe9f5 encodes for the 2K
 * performance run, and against EEMBC's own published CRCs for it rather than
 * core_main.c's copy of them -- and folds in check_data_types(), which
 * core_main.c calls but never folds into `err` either.
 *
 * COREMARK_FLAGS must be defined with the exact compiler flags this was built
 * with, the same rule test/bench/dhry_port.c enforces for Dhrystone: a
 * CoreMark/MHz figure whose flags are unknown is not a measurement, and EEMBC's
 * own run rules require disclosing them.
 */

#include "coremark.h"
#include "core_portme.h"
#include <stdarg.h>
#include <stddef.h>

#ifndef COREMARK_FLAGS
#error "COREMARK_FLAGS must be defined with the exact compiler flags this was built with"
#endif

/* The riscv-tests HTIF window, exactly as test/bench/dhry_port.c uses it. */
volatile unsigned tohost[2] __attribute__((section(".tohost"), aligned(8), used));

/* Read out of RAM by the runner's `--console`. */
char coremark_console[2048] __attribute__((used));
static unsigned console_len;

/* Seeds : SEED_VOLATILE reads these, and the values below are the ones
 * core_main.c's own default-value fixup already recognises as the 2K
 * performance run (seed1=seed2=0, seed3=0x66) once TOTAL_DATA_SIZE is
 * coremark.h's own default of 2000 -- so this file sets no size and no seed
 * that core_main.c does not already treat as a documented configuration.
 * seed4_volatile is the iteration count, the same role DHRY_RUNS plays for
 * Dhrystone; ITERATIONS arrives from the build the same way. */
#ifndef ITERATIONS
#error "ITERATIONS must be defined -- the number of CoreMark iterations to run"
#endif
volatile ee_s32 seed1_volatile = 0;
volatile ee_s32 seed2_volatile = 0;
volatile ee_s32 seed3_volatile = 0x66;
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;

ee_u32 default_num_contexts = 1;

/* Nominal only: it is what turns a cycle count into the "seconds" core_main.c
 * checks against its own ">=10 secs" rule, and it is NOT what the CoreMark/MHz
 * figure below is computed from -- that figure is iterations * 1e6 / cycles,
 * frequency-independent by construction, the same way DMIPS/MHz is. Picked to
 * match the board's target clock so the printed "seconds" line means the
 * runtime this design would have at that clock, not a runtime nothing here
 * will ever see. */
#define NOMINAL_HZ 12000000u

static CORE_TICKS start_ticks, stop_ticks;
static unsigned start_instret, stop_instret;

static inline unsigned read_mcycle(void) {
  unsigned value;
  __asm__ volatile("csrr %0, mcycle" : "=r"(value) : : "memory");
  return value;
}

static inline unsigned read_minstret(void) {
  unsigned value;
  __asm__ volatile("csrr %0, minstret" : "=r"(value) : : "memory");
  return value;
}

void start_time(void) {
  start_instret = read_minstret();
  start_ticks = (CORE_TICKS)read_mcycle();
}

void stop_time(void) {
  stop_ticks = (CORE_TICKS)read_mcycle();
  stop_instret = read_minstret();
}

CORE_TICKS get_time(void) { return stop_ticks - start_ticks; }

secs_ret time_in_secs(CORE_TICKS ticks) { return ticks / NOMINAL_HZ; }

void portable_init(core_portable *p, int *argc, char *argv[]) {
  (void)argc;
  (void)argv;
  p->portable_id = 1;
}

#ifdef COREMARK_UART
static void uart_putc(char c) {
  volatile unsigned *uart = (volatile unsigned *)(unsigned long)COREMARK_UART;
  while ((uart[1] & 1u) != 0u) {
  }
  uart[0] = (unsigned char)c;
}
#else
static void uart_putc(char c) { (void)c; }
#endif

static void put_c(char c) {
  uart_putc(c);
  if (console_len + 1 < sizeof(coremark_console)) {
    coremark_console[console_len++] = c;
  }
}

static void put_str(const char *s) {
  while (*s != '\0') {
    put_c(*s++);
  }
}

static void put_udec(unsigned long value, unsigned width, int zero_pad) {
  char digits[20];
  unsigned n = 0;
  do {
    digits[n++] = (char)('0' + value % 10u);
    value /= 10u;
  } while (value != 0u);
  for (unsigned pad = n; pad < width; pad++) {
    put_c(zero_pad ? '0' : ' ');
  }
  while (n-- > 0) {
    put_c(digits[n]);
  }
}

static void put_hex(unsigned long value, unsigned width, int zero_pad) {
  static const char digits[] = "0123456789abcdef";
  char out[16];
  unsigned n = 0;
  do {
    out[n++] = digits[value % 16u];
    value /= 16u;
  } while (value != 0u);
  for (unsigned pad = n; pad < width; pad++) {
    put_c(zero_pad ? '0' : ' ');
  }
  while (n-- > 0) {
    put_c(out[n]);
  }
}

/* Only the specifiers the vendored CoreMark sources actually call with:
 * %d, %u, %lu, %04x-style widths on %x, %s, and %%. core_main.c's %f arm is
 * unreachable with HAS_FLOAT 0, so no float formatting is implemented. */
int ee_printf(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  while (*fmt != '\0') {
    if (*fmt != '%') {
      put_c(*fmt++);
      continue;
    }
    fmt++;
    int zero_pad = 0;
    if (*fmt == '0') {
      zero_pad = 1;
      fmt++;
    }
    unsigned width = 0;
    while (*fmt >= '0' && *fmt <= '9') {
      width = width * 10u + (unsigned)(*fmt - '0');
      fmt++;
    }
    int is_long = 0;
    if (*fmt == 'l') {
      is_long = 1;
      fmt++;
    }
    switch (*fmt) {
    case 'd': {
      long v = is_long ? va_arg(ap, long) : va_arg(ap, int);
      if (v < 0) {
        put_c('-');
        v = -v;
      }
      put_udec((unsigned long)v, width, zero_pad);
      break;
    }
    case 'u':
      put_udec(is_long ? va_arg(ap, unsigned long) : va_arg(ap, unsigned),
                width, zero_pad);
      break;
    case 'x':
      put_hex(is_long ? va_arg(ap, unsigned long) : va_arg(ap, unsigned),
              width, zero_pad);
      break;
    case 's':
      put_str(va_arg(ap, const char *));
      break;
    case '%':
      put_c('%');
      break;
    default:
      put_c('%');
      if (*fmt != '\0') {
        put_c(*fmt);
      }
      break;
    }
    if (*fmt != '\0') {
      fmt++;
    }
  }
  va_end(ap);
  return 0;
}

/* 32x32 -> 64 and 64/64 for the fixed-point CoreMark/MHz print, the same
 * reason test/bench/dhry_port.c writes its own: -nostdlib links no libgcc. */
static unsigned long long umul64(unsigned a, unsigned b) {
  return (unsigned long long)a * (unsigned long long)b;
}

static unsigned long long udiv64(unsigned long long n, unsigned long long d) {
  unsigned long long quotient = 0;
  unsigned long long remainder = 0;
  if (d == 0) {
    return 0;
  }
  for (int i = 0; i < 64; i++) {
    remainder = (remainder << 1) | (n >> 63);
    n <<= 1;
    quotient <<= 1;
    if (remainder >= d) {
      remainder -= d;
      quotient |= 1;
    }
  }
  return quotient;
}

/* `value` is already reduced to 32-bit range by the caller's udiv64, scaled by
 * 10^places -- so this does only native 32-bit division, the same way
 * test/bench/dhry_port.c's put_fixed does and for the same reason: a 64-bit
 * `/` or `%` here would call libgcc's __udivdi3, which -nostdlib does not
 * link. */
static void put_fixed(unsigned value, unsigned places) {
  unsigned scale = 1u;
  for (unsigned i = 0; i < places; i++) {
    scale *= 10u;
  }
  put_udec(value / scale, 0, 0);
  put_str(".");
  for (unsigned digit = scale / 10u; digit > 0u; digit /= 10u) {
    put_udec((value / digit) % 10u, 0, 0);
  }
}

static void put_line(const char *label, const char *text) {
  put_str(label);
  put_str(text);
  put_str("\n");
}

/* One of three outcomes, each with its own tohost code so a run that could
 * not be validated cannot exit the way a PASS does. Codes are riscv_test.h's
 * shape (odd, decoded as `testnum = tohost >> 1`), and 1 is reserved for a
 * true PASS by the same convention every other program in this repo uses. */
enum coremark_verdict {
  COREMARK_PASS = 1,
  COREMARK_FAIL = 3,
  COREMARK_UNVALIDATED = 5,
};

/* Read the flags with the number, the same way test/bench/dhry_port.c's
 * report does: separate compilation and -O2 mean the optimiser cannot inline
 * across the three units the way core_list_join.c, core_matrix.c and
 * core_state.c would let it if built together. */
static void coremark_report(unsigned iterations, unsigned cycles,
                             unsigned instructions,
                             enum coremark_verdict verdict) {
  put_str("\n== this core's own CoreMark/MHz, not core_main.c's own report "
          "above ==\n");
  put_line("Compiler       : ", __VERSION__);
  put_line("Compiler flags : ", COREMARK_FLAGS);
  put_str("Memory config  : MEM_STACK, TOTAL_DATA_SIZE 2000 bytes, "
          "SEED_VOLATILE performance run\n");
  put_str("SIMULATED AT 16 KB OF ROM -- double this part's 8 KB. This "
          "figure describes a machine that cannot be built until the ROM "
          "grows; see the caveat this run prints below.\n\n");
  put_str("Iterations     : ");
  put_udec(iterations, 0, 0);
  put_str("\nCycles         : ");
  put_udec(cycles, 0, 0);
  put_str("\nInstructions   : ");
  put_udec(instructions, 0, 0);
  put_str("\nCPI            : ");
  put_fixed((unsigned)udiv64(umul64(cycles, 100u), instructions), 2);
  put_str("\nCoreMark/MHz   : ");
  put_fixed((unsigned)udiv64(umul64(iterations, 1000000000u), cycles), 3);
  put_str("\nSelf-check     : ");
  switch (verdict) {
  case COREMARK_PASS:
    put_str("PASS (2K performance seeds and size, list/matrix/state CRCs "
            "matched against EEMBC's published values, check_data_types "
            "clean)\n");
    break;
  case COREMARK_FAIL:
    put_str("FAIL (ran the 2K performance configuration; see the CRC lines "
            "core_main.c printed above, or a check_data_types error)\n");
    break;
  default:
    put_str("COULD NOT BE VALIDATED (seeds, size or the executed-algorithm "
            "mask did not match EEMBC's 2K performance run -- this figure "
            "is not a scored CoreMark result)\n");
    break;
  }
  put_str("\n"
          "READ THE FLAGS AND THE ROM SIZE WITH THE NUMBER. CoreMark is\n"
          "less string-dominated than Dhrystone and harder for the optimiser\n"
          "to delete, but it is still a compiled figure: the compiler, the\n"
          "flags and the iteration count travel with it because EEMBC's own\n"
          "run rules require disclosing all three. This core is stall-only\n"
          "with no bitmanip extension, and CoreMark leans on both -- a figure\n"
          "well under a core built with forwarding and Zba/Zbb/Zbs is the\n"
          "price of this core's four goals, not a defect in the port.\n");

  tohost[1] = 0;
  tohost[0] = (unsigned)verdict;

#ifdef COREMARK_UART
  for (;;) {
    for (volatile unsigned d = 0; d < 2000000u; d++) {
    }
    for (unsigned i = 0; i < console_len; i++) {
      uart_putc(coremark_console[i]);
    }
  }
#else
  for (;;) {
  }
#endif
}

/* The 2K performance run's seeds, size and EEMBC-published CRCs -- the same
 * literals core_main.c's own `switch (seedcrc)` encodes as case 0xe9f5 and
 * the same `list_known_crc`/`matrix_known_crc`/`state_known_crc[3]` it never
 * exposes past its own stack frame, so this file states them once more
 * rather than trusting a comparison it cannot see ran. */
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
  coremark_report((unsigned)res->iterations, stop_ticks - start_ticks,
                   stop_instret - start_instret, verdict);
}
