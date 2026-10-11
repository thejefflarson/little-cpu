#!/usr/bin/env python3
"""Grades the gapless capture behind the NIST assessment of the iCESugar-Pro's seed source.

soc/entropy_bit_framer.v is simulated with edges at the board's rate and its UART bytes are read by the
same parser the capture uses; the bits must be the sample bits of the intervals, frame after frame, with
the sequence numbers advancing by one. Each way the capture could lose "consecutive" is then forced and
must be seen: edges faster than the UART (a dropped frame), a flipped byte, a skipped sequence number and
a wrong sample bit.

Usage: entropy_stream_test.py [repo-root]     # defaults to this script's parent
"""

import importlib.util
import os
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
