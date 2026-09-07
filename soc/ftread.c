// Reads the board's FT232H as a UART through libftdi, WITHOUT a /dev node.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <ftdi.h>

int main(int argc, char **argv) {
  int baud = argc > 1 ? atoi(argv[1]) : 115200;
  int ms   = argc > 2 ? atoi(argv[2]) : 3000;
  struct ftdi_context *f = ftdi_new();
  if (!f) { fprintf(stderr, "ftdi_new failed\n"); return 2; }
  if (ftdi_usb_open(f, 0x0403, 0x6014) < 0) {
    fprintf(stderr, "open: %s\n", ftdi_get_error_string(f));
    return 2;
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
