/*
 * The machine-side half of Dhrystone for THIS directory's two-core harness.
 *
 * It implements test/bench/dhry_port.h against a machine with no CSRs, because
 * one of the two cores here has none: the VexRiscv configuration in the pinned
 * riscv-formal clone has no CSR file at all, so `csrr mcycle` -- which is how
 * test/bench/dhry_port.c times the run -- is not an instruction it can execute.
 * The clock therefore has to be counted outside the machine, and it is:
 * `dhry_mcycle` stores a marker to the word at the base of RAM and
 * soc/compare/dhry_tb.v counts cycles between the two markers, on each core's
 * own data bus, in the one simulation that runs both.
 *
 * dhry_1.c and dhry_2.c are the same files `make dhrystone` compiles, unedited.
 * The measured loop is therefore byte-identical between the two measurements
 * and only this file differs; every other difference between the numbers is in
 * the flags, in the geometry, or in the core.
 *
 * The four string and memory routines are copied from test/bench/dhry_port.c
 * rather than shared, so that a change to that file cannot silently move this
 * number: Dhrystone is string-dominated and these loops are worth tens of
 * percent. If you change one, change both and re-measure both.
 */

#include "dhry.h"
#include "dhry_port.h"

#ifndef DHRY_FLAGS
#error "DHRY_FLAGS must be defined with the exact compiler flags this was built with"
#endif

/* Compiled into the image and read back out of it by the runner, so the flags
 * beside the number are the flags the image was built with rather than a
 * second copy of the string that is free to drift. */
const char dhry_flags[] __attribute__((used)) = DHRY_FLAGS;

/* The two-word control window soc/compare/dhry.lds places at the base of RAM,
 * which is also the address soc/compare/dhry_tb.v watches on both data buses.
 * Its address is the linker script's to state and no literal here repeats it. */
volatile unsigned dhry_ctl[2] __attribute__((section(".dhryctl"), used));

static void publish(unsigned index, unsigned value) {
  /* The barriers are the measurement. Without them the compiler may move work
   * across the marker store, and the cycles counted would not be the cycles the
   * loop took. */
  __asm__ volatile("" ::: "memory");
  dhry_ctl[index] = value;
  __asm__ volatile("" ::: "memory");
}

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

/* Called immediately before and immediately after the measured loop. The value
 * it returns is unused -- dhry_1.c subtracts the two and hands the difference to
 * dhry_report below, which discards it -- because the count that matters is the
 * one the testbench takes on the bus. */
static unsigned marks;

unsigned dhry_mcycle(void) {
  marks++;
  publish(0, marks);
  return 0;
}

/* No counter, and no marker either: dhry_1.c reads this one just outside the
 * pair of dhry_mcycle() calls, so a store here would land inside nobody's
 * window and only add work to the measurement. */
unsigned dhry_minstret(void) {
  __asm__ volatile("" ::: "memory");
  return 0;
}

static Rec_Type record_pool[2];
static unsigned records_taken;

Rec_Pointer dhry_alloc_record(void) {
  if (records_taken >= sizeof(record_pool) / sizeof(record_pool[0])) {
    return Null;
  }
  return &record_pool[records_taken++];
}

/* The report is the testbench's, not the program's: there is no console here and
 * no 64-bit arithmetic to format one with on a machine whose multiplier this
 * image cannot use. What the program has to say is whether it computed the right
 * answers, and it says that in the second control word -- 1 for pass, 3 for
 * fail, the riscv-tests encoding test/asm/riscv_test.h already uses. */
void dhry_report(int runs, unsigned cycles, unsigned instructions, int ok) {
  (void)runs;
  (void)cycles;
  (void)instructions;
  publish(1, ok ? 1u : 3u);
  for (;;) {
  }
}
