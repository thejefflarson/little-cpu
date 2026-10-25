// UPduino board program: reads the flash's JEDEC id (0x9F) through the on-chip controller on the
// shared pins and prints what the FPGA side can see. Read-only: only 0x9F is ever shifted out.
// Lines are "FS <tag> ..." so a host can grade them; make bitstream SOC_PROG=soc/flash_share_probe.c.
#define UART_DATA    (*(volatile unsigned *)0x00020020u)
#define UART_STATUS  (*(volatile unsigned *)0x00020024u)
#define SPI_DATA     (*(volatile unsigned *)0x00020028u)
#define SPI_CONTROL  (*(volatile unsigned *)0x0002002cu)
#define SPI_BUSY     0x100u
#define JEDEC_ID     0x9f

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

static void put_hex(unsigned v, int digits) {
  for (int i = digits - 1; i >= 0; i--) {
    unsigned n = (v >> (4 * i)) & 15u;
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

static void wait_cycles(unsigned n) {
  unsigned t = cycles();
  while (cycles() - t < n) {
  }
}

static unsigned poll_seen;
static unsigned gap_seen;

// One byte. `gap` is a delay loop run after the byte completes, to stretch the time the clock sits low.
static unsigned xfer(unsigned out, unsigned gap) {
  unsigned t = cycles();
  if (poll_seen) gap_seen = t - poll_seen;
  SPI_DATA = out;
  while (SPI_DATA & SPI_BUSY) {
  }
  poll_seen = cycles();
  unsigned in = SPI_DATA & 0xffu;
  for (volatile unsigned i = 0; i < gap; i++) {
  }
  return in;
}

// Select, 0x9F, three bytes back, deselect. `hold` idles that many cycles between the select and the first byte.
static unsigned read_id(unsigned gap, unsigned hold, unsigned *span) {
  unsigned t0 = cycles();
  poll_seen = 0;
  SPI_CONTROL = 1;
  if (hold) wait_cycles(hold);
  xfer(JEDEC_ID, gap);
  unsigned a = xfer(0, gap);
  unsigned b = xfer(0, gap);
  unsigned c = xfer(0, 0);
  SPI_CONTROL = 0;
  *span = cycles() - t0;
  return a << 16 | b << 8 | c;
}

static void report(const char *tag, unsigned param, unsigned id, unsigned span) {
  puts_("FS ");
  puts_(tag);
  puts_(" n=");
  put_dec(param);
  puts_(" gap=");
  put_dec(gap_seen);
  puts_(" span=");
  put_dec(span);
  puts_(" id=");
  put_hex(id, 6);
  putc_('\n');
}

int main(void) {
  static const unsigned gaps[] = {0, 1, 2, 3, 4, 6, 8};
  static const unsigned holds[] = {4, 10, 14, 15, 16, 20, 40};
  for (;;) {
    unsigned span, id;

    uart_drain();
    id = read_id(0, 0, &span);
    report("A", 0, id, span);

    uart_drain();
    UART_DATA = 0u;  // all-low character left transmitting when the select arrives
    id = read_id(0, 0, &span);
    uart_drain();
    puts_("FS B");
    puts_(" id=");
    put_hex(id, 6);
    putc_('\n');

    for (unsigned i = 0; i < sizeof gaps / sizeof gaps[0]; i++) {
      uart_drain();
      id = read_id(gaps[i], 0, &span);
      report("G", gaps[i], id, span);
    }
    for (unsigned i = 0; i < sizeof holds / sizeof holds[0]; i++) {
      uart_drain();
      id = read_id(0, holds[i], &span);
      report("H", holds[i], id, span);
    }
    puts_("FS END\n");
    wait_cycles(3000000u);
  }
}
