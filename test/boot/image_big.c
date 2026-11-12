// A boot image for test/boot_loader_test.sh's SPRAM-text target: a table that alone is larger than the default
// board's 8 KB ROM, summed on the part and compared with the closed form of its contents.
#define UART_DATA   (*(volatile unsigned *)0x00020020u)
#define UART_STATUS (*(volatile unsigned *)0x00020024u)

#define E1(b)  (b), (b) + 1
#define E2(b)  E1(b), E1((b) + 2)
#define E3(b)  E2(b), E2((b) + 4)
#define E4(b)  E3(b), E3((b) + 8)
#define E5(b)  E4(b), E4((b) + 16)
#define E6(b)  E5(b), E5((b) + 32)
#define E7(b)  E6(b), E6((b) + 64)
#define E8(b)  E7(b), E7((b) + 128)
#define E9(b)  E8(b), E8((b) + 256)
#define E10(b) E9(b), E9((b) + 512)
#define E11(b) E10(b), E10((b) + 1024)
#define ENTRIES 2048u

static const unsigned table[ENTRIES] = {E11(1u)};

static void putc_(char c) {
  while (UART_STATUS & 1u) {
  }
  UART_DATA = (unsigned char)c;
}

int main(void) {
  const volatile unsigned *t = table;
  unsigned sum = 0;
  for (unsigned i = 0; i < ENTRIES; i++) sum += t[i];
  const char *msg = sum == ENTRIES * (ENTRIES + 1u) / 2u ? "IMAGE BIG ok\n" : "IMAGE BIG bad\n";
  for (;;) {
    for (const char *s = msg; *s; s++) putc_(*s);
    unsigned load, start, now;
    __asm__ volatile("csrr %0, mscratch" : "=r"(load));
    for (const char *s = "LOAD cycles="; *s; s++) putc_(*s);
    char buf[10];
    int n = 0;
    do {
      buf[n++] = (char)('0' + load % 10u);
      load /= 10u;
    } while (load);
    while (n) putc_(buf[--n]);
    putc_('\n');
    __asm__ volatile("csrr %0, mcycle" : "=r"(start));
    do {
      __asm__ volatile("csrr %0, mcycle" : "=r"(now));
    } while (now - start < 3000000u);
  }
}
