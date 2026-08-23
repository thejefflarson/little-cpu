// Reads the board's FT232H as a UART through libftdi, WITHOUT a /dev node.
//
// WHY NOT `screen`. On macOS the FT232H has one USB interface and it is both
// the serial port and the MPSSE engine iceprog drives. Apple's DriverKit
// extension claims it, so unprivileged every libftdi tool reports zero devices
// while ioreg shows the board plainly; and once iceprog has run, the chip is
// left in MPSSE mode and no `/dev/cu.usbserial-*` exists at all until it
// re-enumerates. Root can open it through libusb regardless, which is what this
// does -- so it works in the state a just-flashed board is actually in.
//
// It also prints a score, because the wire rate moves with the FPGA's clock:
// rtl/uart.v divides by a fixed constant, so a board running at f transmits at
// 115200 * f / 12e6. Sweeping the host rate and looking for the plateau where
// 8N1 still decodes is how this tree measured SB_HFOSC without a scope.
//
// Build: `make ftread`. Run: `sudo ./ftread 115200 8000`.
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
  /* A TIGHT POLL, because the board streams faster than a sleepy reader drains.
   * The first version slept 50ms between reads while a 2KB report arrived every
   * 0.8s; the FTDI's FIFO overran and successive reports came back interleaved,
   * which reads exactly like a crashing program and is not one. 1ms costs
   * nothing here and keeps the buffer empty. */
  /* A WALL-CLOCK DEADLINE, not an iteration count. Counting iterations assumes
   * each costs the sleep and nothing else; a board transmitting at line rate
   * makes every read take real time, and a run asked for 8 seconds took 113.
   * The caller means seconds, so measure seconds. */
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
