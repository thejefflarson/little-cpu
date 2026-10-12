#!/usr/bin/env python3
"""Grades the gapless capture behind the NIST assessment of the iCESugar-Pro's seed source.

soc/entropy_bit_framer.v is simulated with edges at the board's rate and its UART bytes are read by the
same parser the capture uses; the bits must be the sample bits of the intervals, frame after frame, with
the sequence numbers advancing by one. Each way the capture could lose "consecutive" is then forced and
must be seen: edges faster than the UART (a dropped frame), a flipped byte, a skipped sequence number and
a wrong sample bit.

Usage: entropy_stream_test.py [repo-root]     # defaults to this script's parent
"""

import argparse
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))


def load(root, name):
    sys.path.insert(0, os.path.join(root, "soc"))
    spec = importlib.util.spec_from_file_location(name, os.path.join(root, "soc", name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def simulate(root, scratch, framer, defines=()):
    vvp = os.path.join(scratch, "bench.vvp")
    cmd = ["iverilog", "-g2012", "-Wall", "-o", vvp] + ["-D" + d for d in defines] + [
        os.path.join(root, "test", "entropy_stream_bench.v"), framer]
    done = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    if done.returncode or done.stderr.strip():
        raise RuntimeError("iverilog: " + done.stderr + done.stdout)
    out = subprocess.run(["vvp", "-n", vvp], capture_output=True, text=True, timeout=600).stdout
    wanted, raw = [], bytearray()
    for line in out.splitlines():
        kind, _, value = line.partition(" ")
        if kind == "S":
            wanted.append(int(value))
        elif kind == "B":
            raw.append(int(value, 16))
    return wanted, raw


def check(root):
    es = load(root, "entropy_stream")
    errors = []

    def expect(what, ok):
        if not ok:
            errors.append(what)

    def frame(seq):
        body = bytes([seq & 255, seq >> 8]) + bytes(range(seq % 7, seq % 7 + es.DATA_BYTES))
        total = sum(body)
        return es.HEADER + body + bytes([total & 255, total >> 8 & 255])

    def stream(seqs):
        return b"".join(frame(seq) for seq in seqs)

    if os.environ.get("ENTROPY_STREAM_SKIP_SIM"):
        raw = stream(range(20))
        frames = es.parse_frames(bytearray(raw))[0]
    else:
        with tempfile.TemporaryDirectory() as scratch:
            framer = os.path.join(root, "soc", "entropy_bit_framer.v")
            wanted, raw = simulate(root, scratch, framer)
            frames, bad = es.parse_frames(bytearray(raw))
            sequences = [seq for seq, _ in frames]
            got = [bit for _, bits in frames for bit in bits]
            expect("the framer sends at least twenty frames in the run (%d)" % len(frames), len(frames) >= 20)
            expect("no frame fails its sum (%d bad)" % bad, bad == 0)
            expect("the first frame is sequence 0 and each follows by one (%s)" % sequences[:5],
                   sequences[:1] == [0] and es.gaps(sequences) == 0 and sequences == list(range(len(frames))))
            expect("every bit sent is the sample bit of the interval at its place",
                   len(got) > 0 and got == wanted[: len(got)])
            expect("the bits sent keep up with the edges, a frame behind at most",
                   len(wanted) - len(got) < 2 * es.FRAME_BITS)
            expect("both sample bits occur", 0 < sum(got) < len(got))

            fast_wanted, fast_raw = simulate(root, scratch, framer, ["FAST_EDGES"])
            fast_frames, _ = es.parse_frames(fast_raw)
            fast_seq = [seq for seq, _ in fast_frames]
            expect("edges faster than the UART drop frames, and the sequence numbers show it (gaps %d)"
                   % es.gaps(fast_seq), es.gaps(fast_seq) > 0)

            wrong = os.path.join(scratch, "wrong.v")
            text = open(framer).read()
            flipped = text.replace("ticks[1] ^ ticks[0]", "ticks[1] ^ ticks[0] ^ 1'b1")
            expect("the polarity mutation changed the framer", flipped != text)
            with open(wrong, "w") as handle:
                handle.write(flipped)
            mut_wanted, mut_raw = simulate(root, scratch, wrong)
            mut_frames, _ = es.parse_frames(mut_raw)
            mut_got = [bit for _, bits in mut_frames for bit in bits]
            expect("a framer sending the wrong sample bit is told apart from the intervals",
                   mut_got != mut_wanted[: len(mut_got)])

    damaged = bytearray(raw)
    damaged[20] ^= 0x10
    good, bad = es.parse_frames(damaged)
    expect("a flipped byte drops its frame and no other", bad >= 1 and len(good) == len(frames) - 1)
    expect("a dropped frame is a gap in the sequence", es.gaps([seq for seq, _ in good]) == 1)
    expect("sequence numbers wrap from 65535 to 0 without a gap", es.gaps([65534, 65535, 0, 1]) == 0)
    expect("a repeated sequence number is a gap", es.gaps([4, 5, 5]) == 1)

    gated = es.Run(3 * es.FRAME_BITS, first_zero=True, gated=True)
    gated.arm(0.0)
    gated.feed(stream([65534, 65535, 0, 1, 2]), 0.01)
    gated.idle(0.05)
    expect("the previous design's frames, a wrapped sequence 0 among them, are not a run before the port is silent",
           not gated.frames and not gated.complete)
    gated.idle(0.2)
    gated.feed(stream([0, 1, 2]), 0.3)
    expect("after the silence the run starts at the new design's sequence 0",
           gated.complete and [seq for seq, _ in gated.frames] == [0, 1, 2])
    still = es.Run(es.FRAME_BITS, first_zero=True, gated=True)
    still.arm(0.0)
    for tick in range(1, 30):
        still.feed(stream([tick]), tick * 0.05)
        still.idle(tick * 0.05 + 0.04)
    expect("a stream that never goes silent never becomes a run", not still.frames)

    late = es.Run(3 * es.FRAME_BITS, first_zero=True)
    late.feed(stream([5, 6, 7]), 0.0)
    expect("a run that does not begin at sequence 0 is not accepted", not late.frames)
    broken = es.Run(4 * es.FRAME_BITS, first_zero=True)
    broken.feed(stream([0, 1, 3]), 0.0)
    broken.feed(stream([0, 1, 2, 3]), 0.1)
    expect("a gap ends a from-boot run for good, and a later sequence 0 does not revive it",
           broken.broken and not broken.complete)
    free = es.Run(2 * es.FRAME_BITS)
    free.feed(stream([0, 1, 3, 4]), 0.0)
    expect("an unanchored run restarts after a gap and completes", free.complete and free.restarts == 1)

    with tempfile.TemporaryDirectory() as lockdir:
        lock = os.path.join(lockdir, "board.lock")
        dead = subprocess.Popen([sys.executable, "-c", "pass"])
        dead.wait()
        shutil.rmtree(lock, ignore_errors=True)
        os.mkdir(lock)
        with open(os.path.join(lock, "pid"), "w") as handle:
            handle.write("%d\n" % dead.pid)
        try:
            with es.locked(lock, timeout=1.0):
                taken = True
        except TimeoutError:
            taken = False
        expect("a lock whose holder is dead is reclaimed", taken and not os.path.exists(lock))
        shutil.rmtree(lock, ignore_errors=True)
        os.mkdir(lock)
        with open(os.path.join(lock, "pid"), "w") as handle:
            handle.write("%d\n" % os.getpid())
        try:
            with es.locked(lock, timeout=0.3):
                held = False
        except TimeoutError:
            held = True
        expect("a lock whose holder is alive is waited for, not taken", held and os.path.exists(lock))
        shutil.rmtree(lock, ignore_errors=True)
        with es.locked(lock):
            pid = open(os.path.join(lock, "pid")).read().strip()
        expect("a held lock records the holder's PID and is removed after", pid == str(os.getpid()) and not os.path.exists(lock))

        meta = os.path.join(lockdir, "run.meta.json")
        es.bind(meta, {"bitstream_sha256": "aa", "row_samples": 1000})
        es.bind(meta, {"bitstream_sha256": "aa", "row_samples": 1000})
        try:
            es.bind(meta, {"bitstream_sha256": "bb", "row_samples": 1000})
            refused = False
        except SystemExit:
            refused = True
        expect("data bound to one bitstream is refused for another", refused)
        expect("the sidecar holds what it was bound to", json.load(open(meta)) == {"bitstream_sha256": "aa", "row_samples": 1000})

        nist = load(root, "entropy_nist")
        meta = {"bitstream_sha256": "aa", "core_pad": 0, "samples": 192, "restarts": 2}
        data, matrix, side = (os.path.join(lockdir, name) for name in ("s.bin", "m.bin", "meta.json"))

        def resume(sidecar, samples_len, matrix_len, expected):
            for path in (data, matrix, side):
                if os.path.exists(path):
                    os.remove(path)
            if sidecar is not None:
                with open(side, "w") as handle:
                    json.dump(sidecar, handle)
            for path, length in ((data, samples_len), (matrix, matrix_len)):
                if length is not None:
                    with open(path, "wb") as handle:
                        handle.write(bytes(length))
            try:
                nist.bind_resume(side, expected, data, matrix)
                return True
            except SystemExit:
                return False

        expect("a resume of data written for this run is accepted", resume(meta, 192, 1000, meta))
        expect("a resume with data but no sidecar is refused", not resume(None, 192, None, meta))
        expect("a resume of data from another bitstream is refused",
               not resume(dict(meta, bitstream_sha256="bb"), 192, None, meta))
        expect("a resume of data from another core clock is refused", not resume(dict(meta, core_pad=1), 192, None, meta))
        expect("a resume of data for another restart count is refused", not resume(dict(meta, restarts=3), 192, None, meta))
        expect("a captured file of the wrong size is refused", not resume(meta, 100, None, meta))
        expect("a matrix longer than the restarts asked for is refused", not resume(meta, 192, 2001, meta))
        expect("a first run with no data writes its sidecar", resume(None, None, None, meta) and os.path.exists(side))

        ns = argparse.Namespace(samples=3 * es.FRAME_BITS, out=os.path.join(lockdir, "cap.bin"), lock=None,
                                bit=None, port=None, seconds=1.0)
        real_fd, real_pump = es.board_fd, es.pump
        es.board_fd = lambda port: os.open(os.devnull, os.O_RDONLY)
        es.pump = lambda fd, run, seconds, until=None: run.feed(stream([0, 1, 2]))
        try:
            status = es.capture(ns)
            expect("a finished capture is moved into place whole", status == 0 and os.path.getsize(ns.out) == ns.samples
                   and not os.path.exists(ns.out + ".part"))
            os.remove(ns.out)
            es.pump = lambda fd, run, seconds, until=None: run.feed(stream([0, 1]))
            status = es.capture(ns)
            expect("a capture that falls short leaves no file behind",
                   status == 1 and not os.path.exists(ns.out) and not os.path.exists(ns.out + ".part"))
        finally:
            es.board_fd, es.pump = real_fd, real_pump

    marker = bytearray(es.HEADER + bytes(2) + es.HEADER + bytes(4))
    expect("a marker inside the data of a short buffer leaves no frame", es.parse_frames(marker)[0] == [])

    if errors:
        for message in errors:
            sys.stderr.write("error: %s\n" % message)
        return 1
    print("entropy-stream: the framer's bits, sequence numbers and sums agree with the intervals, and each loss is seen.")
    return 0


if __name__ == "__main__":
    sys.exit(check(sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(HERE)))
