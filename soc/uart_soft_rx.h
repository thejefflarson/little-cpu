// An 8N1 receiver over a stream of line samples taken `os` times per bit (os need not be an integer).
// It finds the start edge, checks the start bit again at its middle, then samples each bit at its middle.
#ifndef UART_SOFT_RX_H
#define UART_SOFT_RX_H

typedef struct {
  double os, next;
  long n, framing, glitches;
  unsigned acc;
  int state, bit, prev;
} soft_rx;

static void soft_rx_init(soft_rx *r, double os) {
  r->os = os;
  r->next = 0;
  r->n = r->framing = r->glitches = 0;
  r->acc = 0;
  r->state = 0;
  r->bit = 0;
  r->prev = 1;
}

// One sample; returns the byte it completed, or -1.
static int soft_rx_feed(soft_rx *r, int level) {
  int out = -1;
  level = level != 0;
  if (r->state == 0) {
    if (r->prev && !level) {
      r->state = 1;
      r->next = r->n + r->os * 0.5;
    }
  } else if (r->n >= r->next) {
    if (r->state == 1) {
      if (level) {
        r->glitches++;
        r->state = 0;
      } else {
        r->state = 2;
        r->bit = 0;
        r->acc = 0;
        r->next += r->os;
      }
    } else if (r->bit < 8) {
      r->acc |= (unsigned)level << r->bit++;
      r->next += r->os;
    } else {
      if (level) out = (int)r->acc;
      else r->framing++;
      r->state = 0;
    }
  }
  r->prev = level;
  r->n++;
  return out;
}

#endif
