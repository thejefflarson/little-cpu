// An 8N1 receiver over line samples taken `os` (not necessarily whole) times per bit, sampling mid-bit.
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

#define SOFT_RX_MAX_RUN 4096

// Alternating run lengths in samples: a bit is as wide as the shortest common run of at least `min_len`,
// refined against the longer runs; 0 when no length is common.
static double soft_rx_bit_width(const unsigned *runs, long count, int min_len) {
  static long hist[SOFT_RX_MAX_RUN + 1];
  long counted = 0;
  for (int l = 0; l <= SOFT_RX_MAX_RUN; l++) hist[l] = 0;
  for (long i = 0; i < count; i++) {
    unsigned l = runs[i] > SOFT_RX_MAX_RUN ? SOFT_RX_MAX_RUN : runs[i];
    if ((int)l >= min_len) { hist[l]++; counted++; }
  }
  if (counted < 20) return 0;
  long floor_count = counted / 50 > 10 ? counted / 50 : 10;
  int shortest = 0;
  for (int l = min_len; l <= SOFT_RX_MAX_RUN; l++)
    if (hist[l] >= floor_count) { shortest = l; break; }
  if (!shortest) return 0;
  double sum = 0, weight = 0;
  for (int l = shortest; l <= shortest * 13 / 10 + 1 && l <= SOFT_RX_MAX_RUN; l++) {
    sum += (double)l * hist[l];
    weight += (double)hist[l];
  }
  double w = sum / weight;
  for (int pass = 0; pass < 3; pass++) {
    double total = 0, bits = 0;
    for (long i = 0; i < count; i++) {
      if ((int)runs[i] < min_len || runs[i] > 6.5 * w) continue;
      total += runs[i];
      bits += (long)(runs[i] / w + 0.5);
    }
    if (bits > 0) w = total / bits;
  }
  return w;
}

// Rebuilds the line at four samples to the bit, each run rounded to whole bits alone, and feeds `rx` (os 4).
static void soft_rx_decode_runs(soft_rx *rx, const unsigned *runs, long count, int first_level, double width,
                                void (*emit)(unsigned char)) {
  for (long i = 0; i < count; i++) {
    long bits = (long)(runs[i] / width + 0.5);
    int level = first_level ^ (int)(i & 1);
    for (long k = 0; k < bits * 4; k++) {
      int c = soft_rx_feed(rx, level);
      if (c >= 0) emit((unsigned char)c);
    }
  }
}

#endif
