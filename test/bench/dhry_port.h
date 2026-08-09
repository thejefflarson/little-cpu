/*
 * What Dhrystone needs from a machine with no libc, no allocator, no host
 * clock and no console.
 *
 * `strcpy`, `strcmp` and `memcpy` are declared here under their published names
 * because dhry_1.c and dhry_2.c call them under those names, and are DEFINED in
 * dhry_port.c as plain byte loops. That is a fact about the number: Dhrystone
 * is string-dominated, so a libc with a word-at-a-time `strcmp` scores higher
 * on the same hardware. dhry_report() prints it next to the result for that
 * reason.
 */

#ifndef DHRY_PORT_H
#define DHRY_PORT_H

#include <stddef.h>

/* For Rec_Pointer below, so this header stands on its own rather than trapping
 * whoever includes it in the wrong order. */
#include "dhry.h"

char *strcpy(char *dst, const char *src);
int   strcmp(const char *a, const char *b);
void *memcpy(void *dst, const void *src, size_t n);
void *memset(void *dst, int c, size_t n);

/* The two counters the run is measured with. `mcycle` is the clock; nothing
 * here reads a host clock, which would measure the simulator instead. */
unsigned dhry_mcycle(void);
unsigned dhry_minstret(void);

/* Storage for Ptr_Glob and Next_Ptr_Glob, in place of the published malloc. */
Rec_Pointer dhry_alloc_record(void);

/* Formats the run into the buffer the runner prints, then stops the machine
 * through the `tohost` window with the riscv-tests verdict encoding. */
void dhry_report(int runs, unsigned cycles, unsigned instructions, int ok);

#endif /* DHRY_PORT_H */
