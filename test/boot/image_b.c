// A second boot image, different in size and in what it prints, so two flashes can be told apart.
#define UART_DATA   (*(volatile unsigned *)0x00020020u)
#define UART_STATUS (*(volatile unsigned *)0x00020024u)

static unsigned table[8] = {3, 1, 4, 1, 5, 9, 2, 6};
static const char banner[] = "IMAGE B product=";

static void putc_(char c) {
  while (UART_STATUS & 1u) {
  }
  UART_DATA = (unsigned char)c;
}

int main(void) {
  unsigned product = 1;
  for (int i = 0; i < 8; i++) product *= table[i];
  for (const char *s = banner; *s; s++) putc_(*s);
  for (unsigned d = 100000u; d; d /= 10u) putc_((char)('0' + product / d % 10u));
  putc_('\n');
  for (;;) {
  }
}
