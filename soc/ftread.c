// Reads the board's FT232H as a UART through libftdi, WITHOUT a /dev node.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <ftdi.h>
#include "uart_soft_rx.h"

// Quiet mode (docs/flashing-the-upduino.md): every ADBUS pin an input, UART decoded in software from ADBUS1.
// The chip's sample rate is not what the baud argument says and samples can be dropped, so the bit width
// is taken from the line's own run lengths and the line is rebuilt at four samples to the bit.
#define TX_BIT 0x02
#define MAX_RUNS (16L << 20)

static double now_ms(void) {
  struct timeval t;
  gettimeofday(&t, NULL);
  return t.tv_sec * 1000.0 + t.tv_usec / 1000.0;
}

// Samples per second delivered over `ms`, not counting the first read, which holds the chip's backlog.
static double sample_for(struct ftdi_context *f, int ms, void (*sink)(unsigned char)) {
  static unsigned char buf[65536], zeros[1024];
  double start = now_ms(), first = 0, last = 0;
  long n = 0, after = 0;
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
    for (int i = 0; i < got; i++) sink(buf[i]);
    last = now_ms();
    if (!n) first = last; else after += got;
    n += got;
  }
  return last > first ? after * 1000.0 / (last - first) : 0;
}

static unsigned *runs;
static long run_count, total_samples, bytes, printable, digits, eol, high[8];
static int run_level = -1, first_level;
static unsigned run_len;

static void drop_sample(unsigned char s) { (void)s; }

static void store_sample(unsigned char s) {
  int level = (s & TX_BIT) != 0;
  total_samples++;
  for (int b = 0; b < 8; b++) high[b] += (s >> b) & 1;
  if (level == run_level) { run_len++; return; }
  if (run_level >= 0 && run_count < MAX_RUNS) runs[run_count++] = run_len;
  if (run_level < 0) first_level = level;
  run_level = level;
  run_len = 1;
}

static void emit(unsigned char c) {
  bytes++;
  if (c >= 32 && c < 127) { printable++; putchar(c); }
  else if (c == '\r' || c == '\n') { eol++; putchar(c == '\n' ? '\n' : '\r'); }
  else printf("<%02x>", c);
  if (c >= '0' && c <= '9') digits++;
}

static int read_quiet(struct ftdi_context *f, int baud, int ms) {
  ftdi_set_bitmode(f, 0x00, BITMODE_BITBANG);
  ftdi_set_latency_timer(f, 1);
  ftdi_read_data_set_chunksize(f, 65536);
  ftdi_set_baudrate(f, 28800);
  ftdi_tcioflush(f);
  double slow = sample_for(f, 400, drop_sample);
  ftdi_set_baudrate(f, 57600);
  ftdi_tcioflush(f);
  double fast = sample_for(f, 400, drop_sample);
  int clock = slow > 0 && fast >= 1.9 * slow ? 57600 : 28800;
  ftdi_set_baudrate(f, clock);
  ftdi_tcioflush(f);
  runs = malloc(sizeof *runs * MAX_RUNS);
  if (!runs) return 2;
  double got = sample_for(f, ms, store_sample);
  if (run_level >= 0 && run_count < MAX_RUNS) runs[run_count++] = run_len;
  double width = soft_rx_bit_width(runs, run_count, (int)(0.75 * 15 * clock / baud));
  soft_rx rx;
  soft_rx_init(&rx, 4);
  if (width > 0) soft_rx_decode_runs(&rx, runs, run_count, first_level, width, emit);
  fflush(stdout);
  fprintf(stderr, "\n[quiet %d baud] trial 28800=%.0f/s 57600=%.0f/s clock=%d delivered=%.0f/s bit=%.2f samples rate=%.0f/s samples=%ld runs=%ld framing=%ld glitches=%ld\n",
          baud, slow, fast, clock, got, width, width * baud, total_samples, run_count, rx.framing, rx.glitches);
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
