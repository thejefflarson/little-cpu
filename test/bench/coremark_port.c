/* CoreMark's porting layer. Undefined this is the 2K performance run; with
 * COREMARK_VALIDATION the 2K validation run, which prints no score. portable_fini()
 * re-derives "the checked configuration ran" rather than trusting core_main.c's `err`,
 * which stays zero when the seed/size CRC matches no known configuration. */

#include "coremark.h"
#include "core_portme.h"
#include <stdarg.h>
#include <stddef.h>

#ifndef COREMARK_FLAGS
#error "COREMARK_FLAGS must be defined with the exact compiler flags this was built with"
#endif

volatile unsigned tohost[2] __attribute__((section(".tohost"), aligned(8), used));

char coremark_console[2048] __attribute__((used));
static unsigned console_len;

// Both sets unconditionally, so either can be checked against the pin.
#define COREMARK_2K_SEED3 0x66
#define COREMARK_2K_SIZE 666

#define COREMARK_2K_PERF_SEED1 0
#define COREMARK_2K_PERF_SEED2 0
#define COREMARK_2K_PERF_CRCLIST 0xe714u
#define COREMARK_2K_PERF_CRCMATRIX 0x1fd7u
#define COREMARK_2K_PERF_CRCSTATE 0x8e3au

#define COREMARK_2K_VALIDATION_SEED1 0x3415
#define COREMARK_2K_VALIDATION_SEED2 0x3415
#define COREMARK_2K_VALIDATION_CRCLIST 0xe3c1u
#define COREMARK_2K_VALIDATION_CRCMATRIX 0x0747u
#define COREMARK_2K_VALIDATION_CRCSTATE 0x8d84u

#ifdef COREMARK_VALIDATION
#define COREMARK_2K_SEED1 COREMARK_2K_VALIDATION_SEED1
#define COREMARK_2K_SEED2 COREMARK_2K_VALIDATION_SEED2
#define COREMARK_2K_CRCLIST COREMARK_2K_VALIDATION_CRCLIST
#define COREMARK_2K_CRCMATRIX COREMARK_2K_VALIDATION_CRCMATRIX
#define COREMARK_2K_CRCSTATE COREMARK_2K_VALIDATION_CRCSTATE
#else
#define COREMARK_2K_SEED1 COREMARK_2K_PERF_SEED1
#define COREMARK_2K_SEED2 COREMARK_2K_PERF_SEED2
#define COREMARK_2K_CRCLIST COREMARK_2K_PERF_CRCLIST
#define COREMARK_2K_CRCMATRIX COREMARK_2K_PERF_CRCMATRIX
#define COREMARK_2K_CRCSTATE COREMARK_2K_PERF_CRCSTATE
#endif

// seed4_volatile is the iteration count; iterate() latches the CRCs on the first only.
#ifndef ITERATIONS
#error "ITERATIONS must be defined -- the number of CoreMark iterations to run"
#endif
volatile ee_s32 seed1_volatile = COREMARK_2K_SEED1;
volatile ee_s32 seed2_volatile = COREMARK_2K_SEED2;
volatile ee_s32 seed3_volatile = COREMARK_2K_SEED3;
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;

ee_u32 default_num_contexts = 1;

// Reaches only core_main.c's ">=10 secs" rule; the score is frequency-independent.
#ifndef COREMARK_HZ
#define COREMARK_HZ 12000000u
#endif

static CORE_TICKS start_ticks, stop_ticks;

static inline unsigned read_mcycle(void) {
  unsigned value;
  __asm__ volatile("csrr %0, mcycle" : "=r"(value) : : "memory");
  return value;
}

void start_time(void) { start_ticks = (CORE_TICKS)read_mcycle(); }

void stop_time(void) { stop_ticks = (CORE_TICKS)read_mcycle(); }

CORE_TICKS get_time(void) { return stop_ticks - start_ticks; }

secs_ret time_in_secs(CORE_TICKS ticks) { return ticks / COREMARK_HZ; }

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

// Only the specifiers the vendored sources call with: %d, %u, %lu, %0Nx, %s, %%.
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

// riscv_test.h's shape: an unvalidated run cannot exit the way a PASS does.
enum coremark_verdict {
  COREMARK_PASS = 1,
  COREMARK_FAIL = 3,
  COREMARK_UNVALIDATED = 5,
};

#ifndef COREMARK_VALIDATION
// Written out because -nostdlib links no libgcc.
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

// Both linker scripts give `rom` origin 0, so `.data`'s load address plus its size is
// the ROM image's byte count -- the same one run_coremark.sh reads back with objcopy.
extern char __data_load_start[];
extern char __data_start[];
extern char __data_end[];

static unsigned long rom_bytes(void) {
  return (unsigned long)__data_load_start +
         ((unsigned long)__data_end - (unsigned long)__data_start);
}
#endif

// EEMBC's one-line report syntax, performance build only. The caveats that travel with
// the number are printed host-side by run_coremark.sh, where they cost the ROM nothing.
static void coremark_report(unsigned iterations, unsigned cycles,
                             enum coremark_verdict verdict) {
#ifndef COREMARK_VALIDATION
  put_str("CoreMark 1.0 : ");
  put_fixed((unsigned)udiv64(umul64(iterations, 1000000000u), cycles), 3);
  put_str(" / GCC ");
  put_str(__VERSION__);
  put_str(" ");
  put_str(COREMARK_FLAGS);
  put_str(" / STACK, ROM ");
  put_udec(rom_bytes(), 0, 0);
  put_str(", RAM 64K / 1\n");
#endif
  put_str("Cycles         : ");
  put_udec(cycles, 0, 0);
  put_str("\nIterations     : ");
  put_udec(iterations, 0, 0);
  put_str("\nSelf-check     : ");
  switch (verdict) {
  case COREMARK_PASS:
    put_str("PASS\n");
    break;
  case COREMARK_FAIL:
    put_str("FAIL\n");
    break;
  default:
    put_str("UNVALIDATED\n");
    break;
  }

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

void portable_fini(core_portable *p) {
  core_results *res =
      (core_results *)((char *)p - offsetof(core_results, port));
  int ran_2k_config = res->execs == ALL_ALGORITHMS_MASK &&
                       res->seed1 == COREMARK_2K_SEED1 &&
                       res->seed2 == COREMARK_2K_SEED2 &&
                       res->seed3 == COREMARK_2K_SEED3 &&
                       res->size == COREMARK_2K_SIZE;
  enum coremark_verdict verdict = COREMARK_UNVALIDATED;
  if (ran_2k_config) {
    int crcs_ok = res->crclist == COREMARK_2K_CRCLIST &&
                  res->crcmatrix == COREMARK_2K_CRCMATRIX &&
                  res->crcstate == COREMARK_2K_CRCSTATE;
    verdict = (crcs_ok && check_data_types() == 0 && res->err == 0)
                  ? COREMARK_PASS
                  : COREMARK_FAIL;
  }
  coremark_report((unsigned)res->iterations, stop_ticks - start_ticks, verdict);
}
