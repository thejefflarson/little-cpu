// ROM loader: copies a program image from the configuration flash into instruction memory and runs it.
// Read-only: the only flash command ever shifted out is the read, 0x03.
#include "boot_layout.h"

#define UART_DATA    (*(volatile unsigned *)0x00020020u)
#define UART_STATUS  (*(volatile unsigned *)0x00020024u)
#define SPI_DATA     (*(volatile unsigned *)0x00020028u)
#define SPI_CONTROL  (*(volatile unsigned *)0x0002002cu)
#define SPI_BUSY     0x100u
#define FLASH_READ   0x03u
#define IMAGE_MAX    ((unsigned)(BOOT_ROM_BYTES - BOOT_IMAGE_BASE))

static unsigned cycles(void) {
  unsigned c;
  __asm__ volatile("csrr %0, mcycle" : "=r"(c));
  return c;
}

static void uart_drain(void) {
  while (UART_STATUS & 1u) {
  }
}

static void putc_(char c) {
  uart_drain();
  UART_DATA = (unsigned char)c;
}

static void puts_(const char *s) {
  while (*s) putc_(*s++);
}

static void put_hex(unsigned v) {
  for (int i = 28; i >= 0; i -= 4) {
    unsigned n = (v >> i) & 15u;
    putc_((char)(n < 10 ? '0' + n : 'A' + n - 10));
  }
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

static unsigned byte_in(unsigned out) {
  SPI_DATA = out;
  while (SPI_DATA & SPI_BUSY) {
  }
  return SPI_DATA & 0xffu;
}

static unsigned word_in(void) {
  unsigned w = byte_in(0);
  w |= byte_in(0) << 8;
  w |= byte_in(0) << 16;
  return w | byte_in(0) << 24;
}

static unsigned fold(unsigned sum, unsigned w) {
  return ((sum << 1) | (sum >> 31)) ^ w;
}

static void fail(const char *why, unsigned a, unsigned b) {
  puts_("BOOT ");
  puts_(why);
  putc_(' ');
  put_hex(a);
  putc_(' ');
  put_hex(b);
  putc_('\n');
  for (;;) {
  }
}

int main(void) {
  puts_("BOOT\n");
  uart_drain();

  // Pin 14 carries the flash's data input while the select is held, so nothing is printed until it is released.
  unsigned start = cycles();
  SPI_CONTROL = 1;
  byte_in(FLASH_READ);
  byte_in((BOOT_FLASH_OFFSET >> 16) & 0xffu);
  byte_in((BOOT_FLASH_OFFSET >> 8) & 0xffu);
  byte_in(BOOT_FLASH_OFFSET & 0xffu);

  unsigned magic = word_in();
  unsigned len = word_in();
  unsigned want = word_in();
  unsigned sum = len;
  int sane = magic == BOOT_MAGIC && len != 0 && len <= IMAGE_MAX && !(len & 3u);
  if (sane) {
    volatile unsigned *dst = (volatile unsigned *)BOOT_IMAGE_BASE;
    __asm__("" : "+r"(dst));
    for (unsigned i = 0; i < len / 4; i++) {
      unsigned w = word_in();
      dst[i] = w;
      sum = fold(sum, w);
    }
  }
  SPI_CONTROL = 0;
  unsigned elapsed = cycles() - start;

  if (magic != BOOT_MAGIC) fail("NOIMAGE", magic, 0);
  if (!sane) fail("BADLEN", len, IMAGE_MAX);
  if (sum != want) fail("BADSUM", want, sum);

  puts_("BOOT OK len=");
  put_dec(len);
  puts_(" cycles=");
  put_dec(elapsed);
  putc_('\n');
  uart_drain();
  __asm__ volatile("fence.i");
  ((void (*)(void))BOOT_IMAGE_BASE)();
  return 0;
}
