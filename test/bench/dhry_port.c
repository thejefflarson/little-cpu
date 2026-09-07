// The machine-side half of the Dhrystone port: the four string/memory routines the
// benchmark calls, the two counters it is measured with, storage in place of malloc, and
// the report.

#include "dhry.h"
#include "dhry_port.h"

#ifndef DHRY_FLAGS
#error "DHRY_FLAGS must be defined with the exact compiler flags this was built with"
#endif

// The riscv-tests HTIF window at the base of RAM. Two 32-bit stores rather than one
// 64-bit store so the verdict is the write that stops the run, matching
// test/asm/riscv_test.h's RVTEST_PASS.
volatile unsigned tohost[2] __attribute__((section(".tohost"), aligned(8), used));

// Read out of RAM by the runner's `--console`, which is given this symbol's address.
char dhry_console[2048] __attribute__((used));

static unsigned console_len;

char *strcpy(char *dst, const char *src) {
  char *out = dst;
  while ((*out++ = *src++) != '\0') {
  }
  return dst;
}

int strcmp(const char *a, const char *b) {
  while (*a != '\0' && *a == *b) {
    a++;
    b++;
  }
  return (int)(unsigned char)*a - (int)(unsigned char)*b;
}

void *memcpy(void *dst, const void *src, size_t n) {
  char *d = dst;
  const char *s = src;
  while (n-- > 0) {
    *d++ = *s++;
  }
  return dst;
}

void *memset(void *dst, int c, size_t n) {
  char *d = dst;
  while (n-- > 0) {
    *d++ = (char)c;
  }
  return dst;
}

// The `memory` clobber is what keeps these at the edges of the measured loop.
unsigned dhry_mcycle(void) {
  unsigned value;
  __asm__ volatile("csrr %0, mcycle" : "=r"(value) : : "memory");
  return value;
}

unsigned dhry_minstret(void) {
  unsigned value;
  __asm__ volatile("csrr %0, minstret" : "=r"(value) : : "memory");
  return value;
}

static Rec_Type record_pool[2];
static unsigned records_taken;

Rec_Pointer dhry_alloc_record(void) {
  if (records_taken >= sizeof(record_pool) / sizeof(record_pool[0])) {
    return Null;
  }
  return &record_pool[records_taken++];
}

// THE SAME BYTES, ALSO DOWN THE WIRE, when this is built for a board.
#ifdef DHRY_UART
static void uart_putc(char c) {
  volatile unsigned *uart = (volatile unsigned *)(unsigned long)DHRY_UART;
  while ((uart[1] & 1u) != 0u) {
  }
  uart[0] = (unsigned char)c;
}
#else
static void uart_putc(char c) { (void)c; }
#endif

static void put_str(const char *s) {
  while (*s != '\0') {
    uart_putc(*s);
    if (console_len + 1 < sizeof(dhry_console)) {
      dhry_console[console_len++] = *s;
    }
    s++;
  }
}

static void put_u32(unsigned value) {
  char digits[10];
  int n = 0;
  do {
    digits[n++] = (char)('0' + value % 10u);
    value /= 10u;
  } while (value != 0u);
  while (n-- > 0) {
    uart_putc(digits[n]);
    if (console_len + 1 < sizeof(dhry_console)) {
      dhry_console[console_len++] = digits[n];
    }
  }
}

static void put_fixed(unsigned value, unsigned places) {
  unsigned scale = 1u;
  for (unsigned i = 0; i < places; i++) {
    scale *= 10u;
  }
  put_u32(value / scale);
  put_str(".");
  for (unsigned digit = scale / 10u; digit > 0u; digit /= 10u) {
    put_u32((value / digit) % 10u);
  }
}

static void put_line(const char *label, const char *text) {
  put_str(label);
  put_str(text);
  put_str("\n");
}

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

#define VAX_DHRYSTONES_PER_SEC 1757u

void dhry_report(int runs, unsigned cycles, unsigned instructions, int ok) {
  unsigned n = (unsigned)runs;

  put_str("Dhrystone Benchmark, Version 2.1 (Language: C)\n"
          "Program compiled without 'register' attribute\n\n");
  put_line("Compiler       : ", __VERSION__);
  put_line("Compiler flags : ", DHRY_FLAGS);
  put_line("String routines: ", "this port's own byte loops, not a libc's");
  put_line("Timer          : ", "mcycle, read by the program itself");
  put_str("\n");

  put_str("Runs           : ");
  put_u32(n);
  put_str("\nCycles         : ");
  put_u32(cycles);
  put_str("\nInstructions   : ");
  put_u32(instructions);
  put_str("\nCPI            : ");
  put_fixed((unsigned)udiv64(umul64(cycles, 100u), instructions), 2);
  put_str("\nCycles/Dhrystone: ");
  put_u32(cycles / (n == 0u ? 1u : n));
  put_str("\nDhrystones/s/MHz: ");
  put_fixed((unsigned)udiv64(umul64(n, 10000000u), cycles), 1);
  put_str("\nDMIPS/MHz      : ");
  put_fixed((unsigned)udiv64(umul64(n, 1000000000u),
                             umul64(cycles, VAX_DHRYSTONES_PER_SEC)),
            3);
  put_str("\nSelf-check     : ");
  put_str(ok ? "PASS\n" : "FAIL\n");
  put_str("\n");

  put_str(
      "DMIPS/MHz is Dhrystones/s/MHz divided by 1757, the VAX 11/780 rate.\n"
      "\n"
      "READ THE FLAGS WITH THE NUMBER. Dhrystone is string-dominated, small\n"
      "enough to fit any cache, and notoriously sensitive to the compiler: a\n"
      "higher -O level deletes part of the work outright, and the string\n"
      "routines above are worth tens of percent by themselves. A DMIPS/MHz\n"
      "figure quoted without its flags, its compiler and its string library is\n"
      "not a measurement. This benchmark is in this tree for one reason -- it\n"
      "is what the cores worth comparing against publish.\n");

  tohost[1] = 0;
  tohost[0] = ok ? 1u : 3u;

#ifdef DHRY_UART
  for (;;) {
    for (volatile unsigned d = 0; d < 2000000u; d++) {
    }
    for (unsigned i = 0; i < console_len; i++) {
      uart_putc(dhry_console[i]);
    }
  }
#else
  for (;;) {
  }
#endif
}
