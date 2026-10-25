#!/usr/bin/env python3
"""Grades the UPduino's entropy capture path without a board.

soc/entropy_upduino.py reads the UART through ftread, which prints a byte outside 0x20-0x7e as `<xx>`;
every byte value must come back, and a stream of frames must survive the round trip. The periodicity
scan must find a repeating pattern of seven intervals and must not find one in noise. The loader must
write the configuration flash and never SRAM (`iceprog -S` leaves CDONE low on this board), must reboot
with `iceprog -t` (a read), and must not call a flash write verified without `VERIFY OK`. The capture
top must elaborate for both clocks and both captures.

Usage: entropy_upduino_test.py [repo-root]     # defaults to this script's parent
"""

import argparse
import importlib.util
import math
import os
import random
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


def ftread_text(data):
    """What soc/ftread.c prints for these bytes."""
    out = bytearray()
    for c in data:
        if 32 <= c < 127 or c in (10, 13):
            out.append(c)
        else:
            out += b"<%02x>" % c
    return bytes(out)


def elaborate(root, osc, stream, fab):
    with tempfile.TemporaryDirectory() as scratch:
        sources = ["entropy_capture_upduino.v", "entropy_interval_capture.v", "entropy_bit_framer.v",
                   "miso_share_enable.v"]
        script = ("read_verilog -sv -lib +/ice40/cells_sim.v; read_verilog -sv %s; "
                  "chparam -set INTERNAL_OSC %d -set STREAM %d -set FAB_LOG2 %d entropy_capture_upduino; "
                  "hierarchy -top entropy_capture_upduino -check; proc; opt_clean; check -assert"
                  % (" ".join(os.path.join(root, "soc", s) for s in sources), osc, stream, fab))
        done = subprocess.run(["yosys", "-p", script], capture_output=True, text=True, cwd=scratch, timeout=300)
        return done.returncode == 0 and "Warning:" not in done.stdout


def check(root):
    eu = load(root, "entropy_upduino")
    es = load(root, "entropy_stream")
    errors = []

    def expect(what, ok):
        if not ok:
            errors.append(what)

    everything = bytes(range(256)) * 2
    expect("every byte value survives ftread's text form", eu.unescape(ftread_text(everything)) == everything)
    expect("a literal less-than sign is left alone", eu.unescape(b"a<b") == b"a<b")
    expect("an escape is decoded where it stands", eu.unescape(b"A<a5>Z<00>") == b"A\xa5Z\x00")

    rng = random.Random(7)
    frames = b""
    for seq in range(200):
        body = bytes([seq & 255, seq >> 8, 0]) + bytes(rng.randrange(256) for _ in range(es.DATA_BYTES))
        crc = es.crc16(body)
        frames += es.HEADER + body + bytes([crc & 255, crc >> 8])
    parsed, bad = es.parse_frames(bytearray(eu.unescape(ftread_text(frames))))
    expect("frames survive the round trip with no CRC failure (%d bad of %d)" % (bad, len(parsed)),
           bad == 0 and [s for s, _, _ in parsed] == list(range(200)))

    noise = [rng.randrange(2) for _ in range(50_000)]
    expect("a stream of coin flips has no prominent lag", abs(eu.periodic_peak(noise, 256)[1]) < eu.PROMINENCE_LIMIT)
    beat = [(0, 0, 1, 0, 1, 0, 1)[i % 7] for i in range(50_000)]
    lag, prominence = eu.periodic_peak(beat, 256)
    expect("a pattern repeating every seven samples stands out (lag %d, %+.3f)" % (lag, prominence),
           abs(prominence) >= eu.PROMINENCE_LIMIT)
    expect("and matches itself whole at lag seven", dict(eu.self_matches(beat, 8))[7] > 0.999)
    alternate = [i % 2 for i in range(50_000)]
    expect("a stream that alternates stands out", abs(eu.periodic_peak(alternate, 256)[1]) >= eu.PROMINENCE_LIMIT)
    drift = [int(rng.random() < 0.5 + 0.45 * math.sin(i / 3000)) for i in range(120_000)]
    expect("a bias that drifts slowly is not a repeating pattern (%+.3f)" % eu.periodic_peak(drift, 256)[1],
           abs(eu.periodic_peak(drift, 256)[1]) < eu.PROMINENCE_LIMIT)

    calls = []
    answers = []

    def fake(argv, seconds):
        calls.append(argv)
        return answers.pop(0)

    eu.sudo = fake
    eu.time.sleep = lambda seconds: None
    answers[:] = [(0, b"", b"VERIFY OK\ncdone: high\n")]
    expect("a flash write that verifies is a load", eu.load("b.bin", "/x/iceprog") is True)
    expect("the load writes the flash: iceprog with the file and no -S", calls[-1] == ["/x/iceprog", "b.bin"])
    answers[:] = [(0, b"", b"cdone: low\n")] * eu.LOAD_TRIES
    expect("a flash write that never says VERIFY OK is not a load", eu.load("b.bin", "/x/iceprog") is False)
    answers[:] = [(None, b"", b"")] * eu.LOAD_TRIES
    expect("a flash write that times out is not a load", eu.load("b.bin", "/x/iceprog") is False)
    answers[:] = [(0, b"", b"Bye.\n")]
    expect("a reboot is a status-zero iceprog -t", eu.reboot("/x/iceprog") is True and calls[-1] == ["/x/iceprog", "-t"])
    answers[:] = [(0, b"ab", b"")]
    expect("a UART read hands the wrapper milliseconds alone", eu.read("/x/lc-uart", 4.5) == b"ab" and calls[-1] == ["/x/lc-uart", "4500"])
    expect("the tools are the two wrappers, keyed by role",
           eu.tools_for(argparse.Namespace(no_root_check=True)) == {"flash": os.path.join(eu.TOOLS, "lc-flash"),
                                                                    "uart": os.path.join(eu.TOOLS, "lc-uart")})

    if not os.environ.get("ENTROPY_UPDUINO_SKIP_YOSYS"):
        for osc, stream, fab in ((0, 0, 0), (1, 1, 0), (1, 0, 3), (0, 1, 2)):
            expect("the capture top elaborates warning-free (internal %d, stream %d, divide 2^%d)" % (osc, stream, fab),
                   elaborate(root, osc, stream, fab))

    for error in errors:
        print("FAIL: " + error)
    return 1 if errors else 0


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(HERE)
    status = check(root)
    if not status:
        print("entropy-upduino: ftread decoding, the periodicity scan, the loader and the capture top agree")
    return status


if __name__ == "__main__":
    sys.exit(main())
