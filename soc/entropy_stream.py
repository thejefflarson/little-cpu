#!/usr/bin/env python3
"""Capture consecutive raw samples from the iCESugar-Pro's seed source, for NIST's tools.

  entropy_stream.py capture --samples N --out FILE [--port NODE]
  entropy_stream.py restart --bit BITSTREAM --out FILE [--rows 1000] [--samples 1000]

A frame is 0xA5 0x5A, a little-endian 16-bit sequence number, 64 sample bits (the first in bit 0
of the first byte) and a little-endian 16-bit sum of the ten bytes after the marker
(soc/entropy_bit_framer.v). The board streams the sample bit of every oscillator edge with no gap,
so a run is consecutive exactly when the sequence numbers advance by one: a gap, which a frame
that fails its sum also makes, discards what was collected and starts the run again. Output is one byte per sample,
0 or 1, the layout ea_non_iid and ea_restart read with bits_per_symbol 1. `restart` loads the
bitstream into SRAM before every row and keeps the first N samples after the load.
"""

import argparse
import contextlib
import os
import subprocess
import sys
import time

import board_read

HEADER = b"\xa5\x5a"
DATA_BYTES = 8
FRAME_BYTES = len(HEADER) + 2 + DATA_BYTES + 2
FRAME_BITS = 8 * DATA_BYTES
SEQ_MOD = 1 << 16
ICELINK_MOUNT = "/Volumes/iCELink"


def parse_frames(buf):
    """(sequence, bits) of each frame whose sum matches, removed from buf; the count of bad ones.

    A bad frame is skipped one byte at a time, so a marker inside data cannot swallow the real
    frame behind it.
    """
    frames, bad = [], 0
    while True:
        at = buf.find(HEADER)
        if at < 0 or len(buf) < at + FRAME_BYTES:
            del buf[: at if at >= 0 else max(len(buf) - 1, 0)]
            return frames, bad
        body = buf[at + 2 : at + 2 + 2 + DATA_BYTES]
        want = buf[at + FRAME_BYTES - 2] | buf[at + FRAME_BYTES - 1] << 8
        if sum(body) & 0xFFFF == want:
            seq = body[0] | body[1] << 8
            bits = [body[2 + i // 8] >> (i % 8) & 1 for i in range(FRAME_BITS)]
            frames.append((seq, bits))
            del buf[: at + FRAME_BYTES]
        else:
            bad += 1
            del buf[: at + 1]


def gaps(sequences):
    """How many places the sequence numbers skip: 0 means every frame followed the last."""
    return sum((b - a) % SEQ_MOD != 1 for a, b in zip(sequences, sequences[1:]))


def frames_to_samples(frames, count):
    return [bit for _, bits in frames for bit in bits][:count]


class Run:
    """Frames fed in as bytes arrive, kept while they are consecutive.

    A gap in the sequence numbers throws away the run so far and counts a restart of it; a frame that
    fails its sum never joins the run, so the frame after it shows as the gap. With first_zero the run
    must begin at sequence 0, the first frame a board sends after it starts.
    """

    def __init__(self, samples, first_zero=False):
        self.samples, self.first_zero = samples, first_zero
        self.buf, self.frames, self.restarts = bytearray(), [], 0

    def feed(self, chunk):
        self.buf.extend(chunk)
        good, _ = parse_frames(self.buf)
        for seq, bits in good:
            if self.frames and (seq - self.frames[-1][0]) % SEQ_MOD != 1:
                self.frames, self.restarts = [], self.restarts + 1
            if not self.frames and self.first_zero and seq != 0:
                continue
            self.frames.append((seq, bits))

    @property
    def complete(self):
        return len(self.frames) * FRAME_BITS >= self.samples

    def result(self):
        return frames_to_samples(self.frames, self.samples) if self.complete else None


def pump(fd, run, seconds, until=lambda: False):
    """Read the port into run until it is complete, `until` says stop, or the time is up."""
    deadline = time.monotonic() + seconds
    while not run.complete and not until() and time.monotonic() < deadline:
        try:
            chunk = os.read(fd, 65536)
        except BlockingIOError:
            chunk = b""
        if chunk:
            run.feed(chunk)
        else:
            time.sleep(0.005)


@contextlib.contextmanager
def locked(path):
    """Hold the directory `path` for the body, waiting while another holder has it; no lock when empty."""
    if path:
        while True:
            try:
                os.mkdir(path)
                break
            except FileExistsError:
                time.sleep(1)
    try:
        yield
    finally:
        if path:
            os.rmdir(path)


def board_fd(port, seconds=30.0):
    """The board's port, waiting for it to appear: the debugger re-enumerates now and then."""
    deadline = time.monotonic() + seconds
    while True:
        nodes = board_read.candidates(port)
        if len(nodes) > 1:
            sys.exit("*** " + board_read.refusal(nodes, port))
        fd = board_read.open_port(nodes[0]) if nodes else None
        if fd is not None:
            return fd
        if time.monotonic() > deadline:
            sys.exit("*** %s" % (board_read.refusal(nodes, port) or "cannot open %s" % nodes[0]))
        time.sleep(0.5)


def capture(args):
    started = time.monotonic()
    with locked(args.lock):
        fd = board_fd(args.port)
        run = Run(args.samples)
        pump(fd, run, args.seconds)
        os.close(fd)
    samples, restarts = run.result(), run.restarts
    if samples is None:
        print("entropy_stream: no consecutive run of %d samples in %gs (%d restarts)"
              % (args.samples, args.seconds, restarts))
        return 1
    with open(args.out, "wb") as handle:
        handle.write(bytes(samples))
    print("entropy_stream: %d consecutive samples in %.0fs, %d restarts of the run, ones %.4f"
          % (len(samples), time.monotonic() - started, restarts, sum(samples) / len(samples)))
    return 0


def eject_icelink():
    if os.path.ismount(ICELINK_MOUNT):
        subprocess.run(["diskutil", "eject", ICELINK_MOUNT], capture_output=True, timeout=60)


def start_load(bit, loader_seconds, loader):
    argv = [sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "bounded.py"),
            str(loader_seconds), loader, "-c", "cmsisdap", "--vid", "0x1d50", "--pid", "0x602b",
            "-m", bit]
    return subprocess.Popen(argv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def load(bit, loader_seconds, loader, fd=None, run=None):
    """Load the bitstream into SRAM; with fd and run, keep reading the port meanwhile.

    The board's last design keeps streaming until the load erases it, and the new one starts before the
    loader exits, so a reader that waited for the loader could find its first frames dropped from a full
    buffer.
    """
    proc = start_load(bit, loader_seconds, loader)
    try:
        if fd is not None:
            pump(fd, run, loader_seconds + 30, until=lambda: proc.poll() is not None)
    finally:
        status = proc.wait()
    return status == 0


def restart(args):
    done, failures = 0, 0
    if os.path.exists(args.out):
        size = os.path.getsize(args.out)
        if size % args.samples:
            sys.exit("*** %s holds %d bytes, not a whole number of %d-sample rows" % (args.out, size, args.samples))
        done = size // args.samples
    if done < args.rows:
        with locked(args.lock):
            load(args.bit, args.loader_seconds, args.loader)
            eject_icelink()
    with open(args.out, "ab") as handle:
        while done < args.rows:
            with locked(args.lock):
                fd = board_fd(args.port)
                run = Run(args.samples, first_zero=True)
                try:
                    ok = load(args.bit, args.loader_seconds, args.loader, fd, run)
                    eject_icelink()
                    if ok:
                        pump(fd, run, args.seconds)
                except OSError:
                    ok = False
                finally:
                    os.close(fd)
                samples = run.result()
            if samples is None:
                failures += 1
                print("entropy_stream: row %d failed (load %s); retrying" % (done, "ok" if ok else "FAILED"), flush=True)
                if failures > args.max_failures:
                    sys.exit("*** %d failed rows; stopping" % failures)
                continue
            handle.write(bytes(samples))
            handle.flush()
            done += 1
            if done % 25 == 0 or done == args.rows:
                print("entropy_stream: %d of %d restarts, %d failed loads" % (done, args.rows, failures), flush=True)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    for name in ("capture", "restart"):
        p = sub.add_parser(name)
        p.add_argument("--out", required=True)
        p.add_argument("--lock", help="a directory made for the length of each board access and removed after")
        p.add_argument("--port", help="a serial node or glob, instead of the iCELink found by VID/PID")
        p.add_argument("--samples", type=int, default=1_000_000 if name == "capture" else 1000)
        p.add_argument("--seconds", type=float, default=900.0 if name == "capture" else 30.0)
    rp = sub.choices["restart"]
    rp.add_argument("--bit", required=True)
    rp.add_argument("--rows", type=int, default=1000)
    rp.add_argument("--loader", default="openFPGALoader")
    rp.add_argument("--loader-seconds", type=float, default=90.0)
    rp.add_argument("--max-failures", type=int, default=20)
    args = parser.parse_args()
    return capture(args) if args.cmd == "capture" else restart(args)


if __name__ == "__main__":
    sys.exit(main())
