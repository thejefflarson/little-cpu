/*
 * The porting header CoreMark's own coremark.h requires by that exact name --
 * this is EEMBC's documented customization point, not an edit to the vendored
 * benchmark. It stays outside test/bench/coremark/ on purpose: everything in
 * that directory is checked against PINNED.sha256 and this is not.
 *
 * HAS_FLOAT is 0: this target has no F extension, so a float build would run
 * the CRT's soft-float routines instead and the CoreMark/MHz figure would be
 * measuring those. That also drops the whole %f-printing arm of core_main.c,
 * so coremark_port.c's ee_printf never has to format one.
 */

#ifndef CORE_PORTME_H
#define CORE_PORTME_H

#include <stddef.h>

#define HAS_FLOAT 0
#define HAS_STDIO 0
#define HAS_PRINTF 0

#ifndef COMPILER_VERSION
#define COMPILER_VERSION "GCC" __VERSION__
#endif
#ifndef COMPILER_FLAGS
#define COMPILER_FLAGS COREMARK_FLAGS
#endif
#define MEM_LOCATION "STACK"

/* ee_ptr_int must hold a pointer or CoreMark's own doc comment says it may
 * fail; ilp32 makes that the same width as ee_u32. */
typedef signed short   ee_s16;
typedef unsigned short ee_u16;
typedef signed int     ee_s32;
typedef double         ee_f32;
typedef unsigned char  ee_u8;
typedef unsigned int   ee_u32;
typedef ee_u32         ee_ptr_int;
typedef size_t         ee_size_t;

#define align_mem(x) (void *)(4 + (((ee_ptr_int)(x)-1) & ~3))

#define CORETIMETYPE ee_u32
typedef ee_u32 CORE_TICKS;

/* No argv on this platform and no system function to call, so this is the
 * same choice test/bench/coremark/barebones ports make: read fixed volatiles
 * coremark_port.c defines, with the iteration count coming in as a compiled-in
 * constant the way DHRY_RUNS does for Dhrystone. */
#define SEED_METHOD SEED_VOLATILE

/* No malloc here, and MEM_STACK is what core_main.c's own MEM_STACK arm
 * already does with no porting work: a stack array sized TOTAL_DATA_SIZE. */
#define MEM_METHOD MEM_STACK

#define MULTITHREAD 1
#define USE_PTHREAD 0
#define USE_FORK    0
#define USE_SOCKET  0

/* test/crt0.S calls `main` with nothing in a0/a1, so main(argc, argv) would
 * read whatever those registers last held. */
#define MAIN_HAS_NOARGC 1
#define MAIN_HAS_NORETURN 0

extern ee_u32 default_num_contexts;

typedef struct CORE_PORTABLE_S
{
  ee_u8 portable_id;
} core_portable;

void portable_init(core_portable *p, int *argc, char *argv[]);
void portable_fini(core_portable *p);

int ee_printf(const char *fmt, ...);

#endif /* CORE_PORTME_H */
