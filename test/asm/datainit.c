// The first C program in this suite, and the thing the SoC could not run: it
// reads globals whose values exist only in the ROM image until test/crt0.S
// copies them into RAM, and globals that are only correct because that same
// startup zeroed them.
//
// Four numbered checks, in the riscv-tests verdict encoding the assembly
// programs use, so a failure names which one:
//
//   1  .data holds its initialisers after the reset path ran
//   2  .bss reads zero after the reset path ran
//   3  .data is restored when the copy runs over memory holding something else
//   4  .bss is cleared when the zeroing runs over memory holding something else
//
// Checks 3 and 4 exist because the simulated RAM comes up zeroed. Check 2 alone
// passes whether or not test/crt0.S ever cleared anything, and a check that
// cannot fail is not a check; scribbling first and calling `runtime_init` again
// runs the same code the reset path ran, over memory that is demonstrably not
// zero. Deleting the zeroing loop from test/crt0.S fails check 4, and deleting
// the copy loop fails check 1.

// The riscv-tests HTIF window, at the base of RAM. Two 32-bit halves rather
// than one 64-bit store so the order is this file's to choose: the upper word
// first and the verdict last, because Sail's HTIF fires on whichever half-write
// completes the pair and the verdict has to be the store that stops it. Same
// reasoning as riscv_test.h's RVTEST_PASS.
volatile unsigned tohost[2] __attribute__((section(".tohost"), aligned(8), used));

// test/crt0.S's copy-and-zero routine, which it also runs before main.
void runtime_init(void);

#define TABLE_WORDS 8

// A distinct nonzero value per slot, checked slot by slot, so a copy that ran
// with the wrong length, the wrong source or the wrong stride fails on the
// element it got wrong rather than on an aggregate that could coincide.
#define ENTRY(i) ((unsigned)(0x9e3779b9u * ((i) + 1u)))

unsigned initialised[TABLE_WORDS] = {
  ENTRY(0), ENTRY(1), ENTRY(2), ENTRY(3),
  ENTRY(4), ENTRY(5), ENTRY(6), ENTRY(7),
};

unsigned uninitialised[TABLE_WORDS];

__attribute__((noreturn)) static void finish(unsigned tohost_word) {
  tohost[1] = 0;
  tohost[0] = tohost_word;
  for (;;) {
  }
}

// The encoding riscv_test.h's RVTEST_FAIL writes: test/cxxrtl.cc reports the
// test number as `tohost >> 1`.
__attribute__((noreturn)) static void fail(unsigned testnum) {
  finish((testnum << 1) | 1u);
}

// `testnum` grades `.data`, `testnum + 1` grades `.bss`.
static void check(unsigned testnum) {
  for (unsigned i = 0; i < TABLE_WORDS; i++) {
    if (initialised[i] != ENTRY(i)) {
      fail(testnum);
    }
    if (uninitialised[i] != 0) {
      fail(testnum + 1);
    }
  }
}

static void scribble(void) {
  for (unsigned i = 0; i < TABLE_WORDS; i++) {
    initialised[i] = 0xdeadbeefu;
    uninitialised[i] = 0xdeadbeefu;
  }
}

int main(void) {
  check(1);
  scribble();
  runtime_init();
  check(3);
  finish(1);
}
