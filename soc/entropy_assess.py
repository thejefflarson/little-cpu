#!/usr/bin/env python3
"""Capture and assess the raw edge intervals of an on-chip oscillator.

  entropy_assess.py read   --out run.txt [--frames N] [--port NODE]   read N frames over the UART
  entropy_assess.py assess run.txt [run2.txt ...]                      assess captured intervals

A frame is 0xA5 0x5A, 4,096 little-endian 16-bit intervals, in core-clock cycles between rising
edges of the synchronised oscillator, and the 16-bit little-endian sum of those intervals
(soc/entropy_capture_icesugar_pro.v). A frame whose sum does not match is dropped and counted.
The same method serves any board whose capture streams that frame: the UPduino's SB_LFOSC owes
the same measurement.

The assessment follows NIST SP 800-90B section 6.3 for a non-IID source, on the one-bit sample
rtl/trng.v actually uses, ticks[1] ^ ticks[0] of the interval (ticks is interval - 1 at the edge):
the most common value, collision, Markov and compression estimates, with the minimum taken as the
claim. It then asks the questions the continuous tests cannot: whether intervals repeat with a
period (autocorrelation of the intervals and of the sample bits), and what rtl/trng.v's adaptive
proportion test and corrector would do with the stream.
"""

import argparse
import collections
import glob
import math
import os
import sys
import time

SAMPLES = 4096
HEADER = b"\xa5\x5a"
FRAME_BYTES = len(HEADER) + 2 * SAMPLES + 2
APT_WINDOW, APT_CUTOFF = 512, 410


def sample_bit(interval):
    ticks = (interval - 1) & 3
    return (ticks >> 1 & 1) ^ (ticks & 1)


def extract_frames(buf):
    """Frames whose trailer matches, removed from buf; the count of markers followed by a bad frame.

    A bad frame is skipped one marker at a time, so a marker found inside data or after a capture
    started mid-stream cannot swallow the real frame behind it.
    """
    frames, dropped = [], 0
    while True:
        at = buf.find(HEADER)
        if at < 0 or len(buf) < at + FRAME_BYTES:
            del buf[: at if at >= 0 else max(len(buf) - 1, 0)]
            return frames, dropped
        body = buf[at + 2 : at + 2 + 2 * SAMPLES]
        words = [body[i] | body[i + 1] << 8 for i in range(0, len(body), 2)]
        trailer = buf[at + 2 + 2 * SAMPLES] | buf[at + 3 + 2 * SAMPLES] << 8
        if sum(words) & 0xFFFF == trailer:
            frames.append(words)
            del buf[: at + FRAME_BYTES]
        else:
            dropped += 1
            del buf[: at + 2]


def read_frames(port, frames, seconds):
    import termios

    import board_port

    nodes = sorted(glob.glob(port)) if port else board_port.find_ports()
    if len(nodes) != 1:
        sys.exit("*** %d serial nodes match %s%s; name the one to read with --port"
                 % (len(nodes), board_port.show(port) if port else "USB %04x:%04x" % (board_port.VID, board_port.PID),
                    " (%s)" % ", ".join(map(board_port.show, nodes)) if nodes else ""))
    print("entropy_assess: reading %s" % board_port.show(nodes[0]), flush=True)
    fd = os.open(nodes[0], os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = termios.B115200
    attrs[2] = (attrs[2] | termios.CS8 | termios.CREAD | termios.CLOCAL) & ~(
        termios.PARENB | termios.CSTOPB
    )
    attrs[0] = attrs[1] = attrs[3] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    buf = bytearray()
    out, dropped = [], 0
    deadline = time.time() + seconds
    while time.time() < deadline and len(out) < frames:
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            time.sleep(0.01)
            continue
        if chunk:
            buf.extend(chunk)
        good, bad = extract_frames(buf)
        out += good
        dropped += bad
    os.close(fd)
    if dropped:
        print("entropy_assess: dropped %d frame(s) whose trailer did not match" % dropped)
    return out[:frames]


def load(paths):
    runs = []
    for path in paths:
        for line in open(path):
            if line.strip():
                runs.append([int(x) for x in line.split()])
    return runs


def mcv_min_entropy(bits):
    n = len(bits)
    p_hat = max(bits.count(0), bits.count(1)) / n
    p_u = min(1.0, p_hat + 2.576 * math.sqrt(p_hat * (1 - p_hat) / (n - 1)))
    return -math.log2(p_u)


def markov_min_entropy(bits):
    n = len(bits)
    p1 = sum(bits) / n
    p = [1 - p1, p1]
    trans = [[0, 0], [0, 0]]
    for a, b in zip(bits, bits[1:]):
        trans[a][b] += 1
    t = []
    for a in (0, 1):
        total = sum(trans[a]) or 1
        t.append([trans[a][0] / total, trans[a][1] / total])
    # Most probable length-128 sequence: dynamic programming over the two states.
    best = [math.log2(max(p[0], 1e-12)), math.log2(max(p[1], 1e-12))]
    for _ in range(127):
        best = [
            max(best[a] + math.log2(max(t[a][b], 1e-12)) for a in (0, 1)) for b in (0, 1)
        ]
    return min(1.0, -max(best) / 128)


def collision_min_entropy(bits):
    # Mean time to the first repeat in a binary stream is 2 + p*q, so p*q recovers the bias.
    times, i = [], 0
    while i + 1 < len(bits):
        if bits[i] == bits[i + 1]:
            times.append(2)
            i += 2
        elif i + 2 < len(bits):
            times.append(3)
            i += 3
        else:
            break
    if len(times) < 2:
        return 0.0
    mean = sum(times) / len(times)
    sd = math.sqrt(sum((x - mean) ** 2 for x in times) / (len(times) - 1))
    low = mean - 2.576 * sd / math.sqrt(len(times))
    pq = max(0.0, low - 2)
    p = 0.5 + math.sqrt(max(0.0, 0.25 - pq)) if pq <= 0.25 else 0.5
    return -math.log2(min(1.0, p))


def compression_min_entropy(bits):
    # Maurer-style estimate on 6-bit blocks; reports the mean log-distance entropy per bit.
    block, n = 6, len(bits) // 6
    if n < 200:
        return 1.0
    words = [int("".join(map(str, bits[i * block : (i + 1) * block])), 2) for i in range(n)]
    d, last, total, count = 1, {}, 0.0, 0
    init = 64 * 2
    for i, w in enumerate(words):
        if i >= init and w in last:
            total += math.log2(i - last[w])
            count += 1
        last[w] = i
    return min(1.0, total / count / block) if count else 1.0


def autocorr(xs, max_lag):
    n = len(xs)
    mean = sum(xs) / n
    var = sum((x - mean) ** 2 for x in xs) or 1.0
    return [
        sum((xs[i] - mean) * (xs[i + lag] - mean) for i in range(n - lag)) / var
        for lag in range(1, max_lag + 1)
    ]


def best_period(bits, max_period=64):
    """The shift at which the bit stream best matches itself, and the match fraction."""
    best = (0, 0.5)
    for lag in range(1, max_period + 1):
        match = sum(a == b for a, b in zip(bits, bits[lag:])) / (len(bits) - lag)
        if abs(match - 0.5) > abs(best[1] - 0.5):
            best = (lag, match)
    return best


def trng_model(intervals):
    """What rtl/trng.v does with this stream: raw samples, corrected bits, folded bits, APT."""
    samples = [sample_bit(v) for v in intervals]
    apt_trips, windows = 0, 0
    for start in range(0, len(samples) - APT_WINDOW + 1, APT_WINDOW):
        window = samples[start : start + APT_WINDOW]
        windows += 1
        if max(window.count(0), window.count(1)) >= APT_CUTOFF:
            apt_trips += 1
    corrected = []
    for a, b in zip(samples[0::2], samples[1::2]):
        if a != b:
            corrected.append(b)
    folded = []
    for i in range(0, len(corrected) - 7, 8):
        folded.append(sum(corrected[i : i + 8]) & 1)
    starved = 0
    blocks = 0
    for start in range(0, len(samples) - 63, 64):
        blocks += 1
        block = samples[start : start + 64]
        if all(a == b for a, b in zip(block[0::2], block[1::2])):
            starved += 1
    longest = run = 0
    last = None
    for bit in folded:
        run = run + 1 if bit == last else 1
        last = bit
        longest = max(longest, run)
    return samples, corrected, folded, apt_trips, windows, starved, blocks, longest


def assess(runs):
    print("runs: %d, intervals per run: %d" % (len(runs), len(runs[0]) if runs else 0))
    flat = [v for run in runs for v in run]
    count = collections.Counter(flat)
    mean = sum(flat) / len(flat)
    sd = math.sqrt(sum((v - mean) ** 2 for v in flat) / len(flat))
    print("interval (core cycles): mean %.2f  sd %.2f  min %d  max %d  saturated %d"
          % (mean, sd, min(flat), max(flat), count.get(0xFFFF, 0)))
    top = count.most_common(6)
    print("  most common: " + ", ".join("%d x%d" % kv for kv in top))
    worst = 1.0
    for index, run in enumerate(runs):
        samples, corrected, folded, apt_trips, windows, starved, blocks, longest = trng_model(run)
        mcv = mcv_min_entropy(samples)
        col = collision_min_entropy(samples)
        mkv = markov_min_entropy(samples)
        comp = compression_min_entropy(samples)
        est = min(mcv, col, mkv, comp)
        worst = min(worst, est)
        lag, match = best_period(samples)
        ac = autocorr(run, 16)
        peak = max(range(16), key=lambda i: abs(ac[i]))
        print("run %d: sample bits %d, ones %.3f" % (index, len(samples), sum(samples) / len(samples)))
        print("  min-entropy/sample  MCV %.3f  collision %.3f  Markov %.3f  compression %.3f  -> %.3f"
              % (mcv, col, mkv, comp, est))
        print("  interval autocorr lags 1-16: peak %+.3f at lag %d" % (ac[peak], peak + 1))
        print("  sample-bit self-match: strongest %.3f at shift %d (0.500 is none)" % (match, lag))
        print("  trng model: APT trips %d of %d windows, starved blocks %d of %d, corrected bits %d,"
              " folded bits %d, longest folded run %d (dead at 32)"
              % (apt_trips, windows, starved, blocks, len(corrected), len(folded), longest))
    print("minimum over runs: %.3f bit/sample (rtl/trng.v assumes 0.5)" % worst)
    pooled = [bit for run in runs for bit in trng_model(run)[2]]
    if len(pooled) > 64:
        ones = sum(pooled) / len(pooled)
        match = [
            sum(a == b for a, b in zip(pooled, pooled[lag:])) / (len(pooled) - lag)
            for lag in range(1, 9)
        ]
        sigma = 0.5 / math.sqrt(len(pooled))
        print("pooled folded bits %d: ones %.3f, min-entropy MCV %.3f, self-match at shifts 1-8: %s"
              " (one sigma %.3f)" % (len(pooled), ones, mcv_min_entropy(pooled),
                                   " ".join("%.3f" % m for m in match), sigma))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    rd = sub.add_parser("read")
    rd.add_argument("--out", required=True)
    rd.add_argument("--frames", type=int, default=4)
    rd.add_argument("--seconds", type=float, default=20.0)
    rd.add_argument("--port", help="a serial node or glob, instead of the iCELink found by VID/PID")
    asp = sub.add_parser("assess")
    asp.add_argument("files", nargs="+")
    args = parser.parse_args()
    if args.cmd == "read":
        import board_port

        board_port.eject_icelink()
        runs = read_frames(args.port, args.frames, args.seconds)
        with open(args.out, "w") as handle:
            for run in runs:
                handle.write(" ".join(map(str, run)) + "\n")
        print("read %d of %d frames into %s" % (len(runs), args.frames, args.out))
        return 0 if len(runs) == args.frames else 1
    runs = load(args.files)
    if not runs:
        sys.exit("*** no frames in %s" % ", ".join(args.files))
    assess(runs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
