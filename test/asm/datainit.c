
volatile unsigned tohost[2] __attribute__((section(".tohost"), aligned(8), used));

// test/crt0.S's copy-and-zero routine, which it also runs before main.
void runtime_init(void);

#define TABLE_WORDS 8

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
