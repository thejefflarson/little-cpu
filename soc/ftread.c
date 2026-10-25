// Reads the board's FT232H as a UART through libftdi, WITHOUT a /dev node.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <ftdi.h>
#include "uart_soft_rx.h"

// Quiet mode (docs/flashing-the-upduino.md): every ADBUS pin an input, UART decoded in software from ADBUS1.
#define TX_BIT 0x02
static const int clock_bauds[] = {57600, 28800, 14400};

static double now_ms(void) {
  struct timeval t;
  gettimeofday(&t, NULL);
  return t.tv_sec * 1000.0 + t.tv_usec / 1000.0;
}

static double sample_for(struct ftdi_context *f, int ms, void (*sink)(unsigned char)) {
  static unsigned char buf[65536], zeros[1024];
  double start = now_ms(), first = 0, last = 0;
  long n = 0;
  int feed = 0;
  while (now_ms() - start < ms) {
    int got = ftdi_read_data(f, buf, sizeof buf);
    if (got < 0) break;
    if (got == 0) {
      if (n == 0 && now_ms() - start > 300) feed = 1;
      if (feed) ftdi_write_data(f, zeros, sizeof zeros);
      usleep(200);
      continue;
    }
    if (!n) first = now_ms();
    for (int i = 0; i < got; i++) sink(buf[i]);
    n += got;
    last = now_ms();
  }
  return last > first ? n * 1000.0 / (last - first) : 0;
}

static soft_rx rx;
static long high[8], total_samples, bytes, printable, digits, eol;

static void emit(unsigned char c) {
  bytes++;
  if (c >= 32 && c < 127) { printable++; putchar(c); }
  else if (c == '\r' || c == '\n') { eol++; putchar(c == '\n' ? '\n' : '\r'); }
  else printf("<%02x>", c);
  if (c >= '0' && c <= '9') digits++;
}

static void decode_sample(unsigned char s) {
  total_samples++;
  for (int b = 0; b < 8; b++) high[b] += (s >> b) & 1;
  int c = soft_rx_feed(&rx, s & TX_BIT);
  if (c >= 0) emit((unsigned char)c);
}

static void drop_sample(unsigned char s) { (void)s; }

static int read_quiet(struct ftdi_context *f, int baud, int ms) {
  ftdi_set_bitmode(f, 0x00, BITMODE_BITBANG);
  ftdi_set_latency_timer(f, 1);
  ftdi_read_data_set_chunksize(f, 65536);
  int chosen = 0;
  double rate = 0;
  for (unsigned i = 0; i < sizeof clock_bauds / sizeof clock_bauds[0]; i++) {
    ftdi_set_baudrate(f, clock_bauds[i]);
    ftdi_tcioflush(f);
    rate = sample_for(f, 400, drop_sample);
    chosen = clock_bauds[i];
    if (rate > 0.97 * 16.0 * clock_bauds[i]) break;
  }
  if (rate <= 0) {
    fprintf(stderr, "quiet: the FT232H returned no samples\n");
    return 1;
  }
  soft_rx_init(&rx, rate / baud);
  ftdi_tcioflush(f);
  double got = sample_for(f, ms, decode_sample);
  fflush(stdout);
  fprintf(stderr, "\n[quiet %d baud] clock=%d nominal=%.0f/s measured=%.0f/s oversample=%.2f samples=%ld framing=%ld glitches=%ld\n",
          baud, chosen, 16.0 * chosen, got, rate / baud, total_samples, rx.framing, rx.glitches);
  fprintf(stderr, "[quiet] adbus high %%:");
  for (int b = 0; b < 8; b++) fprintf(stderr, " %d=%.1f", b, total_samples ? 100.0 * high[b] / total_samples : 0.0);
  fprintf(stderr, "\n[baud %d] bytes=%ld printable=%ld digits=%ld eol=%ld\n", baud, bytes, printable, digits, eol);
  ftdi_set_bitmode(f, 0x00, BITMODE_RESET);
  return bytes == 0 ? 1 : 0;
}

int main(int argc, char **argv) {
  int baud = argc > 1 ? atoi(argv[1]) : 115200;
  int ms   = argc > 2 ? atoi(argv[2]) : 3000;
  struct ftdi_context *f = ftdi_new();
  if (!f) { fprintf(stderr, "ftdi_new failed\n"); return 2; }
  if (ftdi_usb_open(f, 0x0403, 0x6014) < 0) {
    fprintf(stderr, "open: %s\n", ftdi_get_error_string(f));
    return 2;
  }
  if (argc > 3 && !strcmp(argv[3], "quiet")) {
    int rc = read_quiet(f, baud, ms);
    ftdi_usb_close(f);
    ftdi_free(f);
    return rc;
  }
  ftdi_set_bitmode(f, 0x00, BITMODE_RESET);   // plain UART, not MPSSE
  if (ftdi_set_baudrate(f, baud) < 0)
    fprintf(stderr, "baud: %s\n", ftdi_get_error_string(f));
  ftdi_set_line_property(f, BITS_8, STOP_BIT_1, NONE);
  ftdi_tcioflush(f);

  unsigned char buf[4096];
  long total = 0, printable = 0, digits = 0, eol = 0;
  // A TIGHT POLL, because the board streams faster than a sleepy reader drains.
  struct timeval t_start, t_now;
  gettimeofday(&t_start, NULL);
  for (;;) {
    gettimeofday(&t_now, NULL);
    long elapsed = (t_now.tv_sec - t_start.tv_sec) * 1000L
                 + (t_now.tv_usec - t_start.tv_usec) / 1000L;
    if (elapsed >= ms) break;
    int n = ftdi_read_data(f, buf, sizeof buf);
    for (int j = 0; j < n; j++) {
      unsigned char c = buf[j];
      total++;
      if (c >= 32 && c < 127) { printable++; putchar(c); }
      else if (c == '\r' || c == '\n') { eol++; putchar(c == '\n' ? '\n' : '\r'); }
      else printf("<%02x>", c);
      if (c >= '0' && c <= '9') digits++;
    }
    fflush(stdout);
    usleep(1000);
  }
  fprintf(stderr, "\n[baud %d] bytes=%ld printable=%ld digits=%ld eol=%ld\n",
          baud, total, printable, digits, eol);
  ftdi_usb_close(f);
  ftdi_free(f);
  return total == 0 ? 1 : 0;
}
