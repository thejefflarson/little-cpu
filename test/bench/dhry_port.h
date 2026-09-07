
#ifndef DHRY_PORT_H
#define DHRY_PORT_H

#include <stddef.h>

#include "dhry.h"

char *strcpy(char *dst, const char *src);
int   strcmp(const char *a, const char *b);
void *memcpy(void *dst, const void *src, size_t n);
void *memset(void *dst, int c, size_t n);

unsigned dhry_mcycle(void);
unsigned dhry_minstret(void);

Rec_Pointer dhry_alloc_record(void);

void dhry_report(int runs, unsigned cycles, unsigned instructions, int ok);

#endif /* DHRY_PORT_H */
