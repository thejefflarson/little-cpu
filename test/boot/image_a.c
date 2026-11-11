// A boot image for test/boot_loader_test.sh: reports over the UART from .data, .bss and .rodata.
#define UART_DATA   (*(volatile unsigned *)0x00020020u)
#define UART_STATUS (*(volatile unsigned *)0x00020024u)

static unsigned table[4] = {1, 2, 3, 4};
static unsigned scratch[4];
static const char banner[] = "IMAGE A sum=";

static void putc_(char c) {
  while (UART_STATUS & 1u) {
  }
  UART_DATA = (unsigned char)c;
}

static void put_dec(unsigned v) {
  char buf[10];
  int n = 0;
  do {
    buf[n++] = (char)('0' + v % 10u);
    v /= 10u;
  } while (v);
  while (n) putc_(buf[--n]);
}

static void report_load(void) {
  unsigned load, start, now;
  __asm__ volatile("csrr %0, mscratch" : "=r"(load));
  for (const char *s = "LOAD cycles="; *s; s++) putc_(*s);
  put_dec(load);
  putc_('\n');
  __asm__ volatile("csrr %0, mcycle" : "=r"(start));
  do {
    __asm__ volatile("csrr %0, mcycle" : "=r"(now));
  } while (now - start < 3000000u);
}

int main(void) {
  unsigned sum = scratch[0];
  for (int i = 0; i < 4; i++) sum += table[i];
  for (;;) {
    for (const char *s = banner; *s; s++) putc_(*s);
    putc_((char)('0' + sum / 10u));
    putc_((char)('0' + sum % 10u));
    putc_('\n');
    report_load();
  }
}
