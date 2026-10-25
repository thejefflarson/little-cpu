// soc/uart_soft_rx.h against waveforms whose bytes are known: exact and fractional oversampling, a baud
// error of a few percent, a one-sample glitch on an idle line, and a frame whose stop bit is low.
#include <stdio.h>
#include <string.h>
#include "../soc/uart_soft_rx.h"

static int fails;
static void bad(const char *what) {
  printf("*** uart-soft-rx: %s\n", what);
  fails = 1;
}

// Appends one frame, `os` samples to the bit, and returns the new length.
static int frame(unsigned char *line, int len, int byte, double os, int stop) {
  for (int bit = 0; bit < 10; bit++) {
    int level = bit == 0 ? 0 : bit == 9 ? stop : (byte >> (bit - 1)) & 1;
    int count = (int)(os * (bit + 1) + 0.5) - (int)(os * bit + 0.5);
    for (int i = 0; i < count; i++) line[len++] = (unsigned char)level;
  }
  return len;
}

static int decode(const unsigned char *line, int len, double os, unsigned char *out, soft_rx *r) {
  int got = 0;
  soft_rx_init(r, os);
  for (int i = 0; i < len; i++) {
    int b = soft_rx_feed(r, line[i]);
    if (b >= 0) out[got++] = (unsigned char)b;
  }
  return got;
}

static void expect_text(const char *what, double line_os, double rx_os, int idle) {
  static unsigned char line[20000], out[64];
  static const unsigned char text[] = {'F', 'S', ' ', 0x00, 0xff, 0x55, 0xaa, '\n'};
  int n = (int)sizeof text, len = 0;
  for (int i = 0; i < n; i++) {
    for (int k = 0; k < idle; k++) line[len++] = 1;
    len = frame(line, len, text[i], line_os, 1);
  }
  for (int k = 0; k < 40; k++) line[len++] = 1;
  soft_rx r;
  int got = decode(line, len, rx_os, out, &r);
  if (got != n || memcmp(out, text, (size_t)n)) {
    char msg[160];
    snprintf(msg, sizeof msg, "%s: decoded %d of %d bytes correctly", what, got, n);
    bad(msg);
  }
}

int main(void) {
  expect_text("eight samples a bit, frames back to back", 8, 8, 0);
  expect_text("eight samples a bit, idle between frames", 8, 8, 5);
  expect_text("a fractional rate", 7.6, 7.6, 3);
  expect_text("a line 3% slow", 8.24, 8, 2);
  expect_text("a line 3% fast", 7.76, 8, 2);
  expect_text("sixteen samples a bit", 16, 16, 0);

  static unsigned char line[400], out[8];
  soft_rx r;
  int len = 0;
  for (int i = 0; i < 20; i++) line[len++] = 1;
  line[len++] = 0;
  for (int i = 0; i < 20; i++) line[len++] = 1;
  len = frame(line, len, 0x41, 8, 1);
  for (int i = 0; i < 20; i++) line[len++] = 1;
  int got = decode(line, len, 8, out, &r);
  if (got != 1 || out[0] != 0x41) bad("a one-sample glitch on an idle line was taken for a start or hid the next byte");
  if (r.glitches != 1) bad("the glitch was not counted");

  len = 0;
  for (int i = 0; i < 20; i++) line[len++] = 1;
  len = frame(line, len, 0x41, 8, 0);
  for (int i = 0; i < 40; i++) line[len++] = 1;
  got = decode(line, len, 8, out, &r);
  if (got != 0 || r.framing != 1) bad("a frame with a low stop bit was delivered or not counted");

  if (!fails)
    printf("uart-soft-rx: 8N1 decoded at integer and fractional oversampling and 3%% baud error; glitches and framing errors counted, not delivered.\n");
  return fails;
}
