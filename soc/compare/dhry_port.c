// The machine-side half of Dhrystone for the two cores this directory runs it on,
// littlecpu and VexRiscv (Hazard3's iCE40 build has no mcycle and its cycle factor is not
// built).

#include "dhry.h"
#include "dhry_port.h"

#ifndef DHRY_FLAGS
#error "DHRY_FLAGS must be defined with the exact compiler flags this was built with"
#endif

const char dhry_flags[] __attribute__((used)) = DHRY_FLAGS;

volatile unsigned dhry_ctl[2] __attribute__((section(".dhryctl"), used));

static void publish(unsigned index, unsigned value) {
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

static unsigned marks;

unsigned dhry_mcycle(void) {
  marks++;
  publish(0, marks);
  return 0;
}

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

void dhry_report(int runs, unsigned cycles, unsigned instructions, int ok) {
  (void)runs;
  (void)cycles;
  (void)instructions;
  publish(1, ok ? 1u : 3u);
  for (;;) {
  }
}
