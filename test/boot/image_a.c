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

int main(void) {
  unsigned sum = scratch[0];
  for (int i = 0; i < 4; i++) sum += table[i];
  for (const char *s = banner; *s; s++) putc_(*s);
  putc_((char)('0' + sum / 10u));
  putc_((char)('0' + sum % 10u));
  putc_('\n');
  for (;;) {
  }
}
