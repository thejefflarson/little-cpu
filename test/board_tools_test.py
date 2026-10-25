#!/usr/bin/env python3
"""Grades the host-side board tooling in soc/ against inputs whose answer is known: the port the
reader opens, what reaches the terminal, the loader's time limit, the clock grader, the capture
frame check and the capture script's arguments.

Usage: board_tools_test.py [repo-root]     # defaults to this script's parent
"""

import importlib
import os
import random
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))


def usb(vid, pid, *callouts):
    return {
        "idVendor": vid,
        "idProduct": pid,
        "IORegistryEntryChildren": [
            {"IORegistryEntryChildren": [{"IOCalloutDevice": c}]} for c in callouts
        ],
    }


def frame(words, trailer=None):
    total = sum(words) & 0xFFFF if trailer is None else trailer
    body = b"".join(w.to_bytes(2, "little") for w in words)
    return b"\xa5\x5a" + body + total.to_bytes(2, "little")


def check(root):
    sys.path.insert(0, os.path.join(root, "soc"))
    bp, br, bd, cg, ea = (
        importlib.import_module(m)
        for m in ("board_port", "board_read", "bounded", "clock_grade", "entropy_assess")
    )
    errors = []

    def expect(what, ok):
        if not ok:
            errors.append(what)

    board = usb(bp.VID, bp.PID, "/dev/cu.usbmodem102")
    other = usb(0x2341, 0x0043, "/dev/cu.usbmodem9")
    expect("the board's node is found by its VID/PID",
           bp.find_ports(tree=[other, board]) == ["/dev/cu.usbmodem102"])
    expect("a CDC device with another VID is not the board",
           bp.find_ports(tree=[other]) == [])
    expect("a device with the board's VID and another PID is not the board",
           bp.find_ports(tree=[usb(bp.VID, bp.PID + 1, "/dev/cu.usbmodem5")]) == [])
    expect("two boards are two nodes, left for the caller to refuse",
           len(bp.find_ports(tree=[board, usb(bp.VID, bp.PID, "/dev/cu.usbmodem103")])) == 2)

    hostile = ["/dev/cu.usbmodem\x1b]0;pwned\x07", "/dev/cu.usbmodem\udcff2"]
    said = (br.refusal(hostile, None) or "") + bp.show(hostile[0]) + bp.show(hostile[1])
    expect("an escape byte in a node name does not reach the refusal message",
           "\x1b" not in said and "\x07" not in said and "\\x1b" in said)
    expect("a node name that is not valid UTF-8 is shown, not an error", "\\xff" in bp.show(hostile[1]))
    expect("one node is accepted", br.refusal(["/dev/cu.usbmodem102"], None) is None)
    expect("no node is refused", br.refusal([], None) is not None)
    expect("several nodes are refused", br.refusal(["/dev/a", "/dev/b"], None) is not None)

    shown = br.escape_display(b"ok\n\tA\x1b]52;c;ZXZpbA==\x07\x1b[2J\r\x00\x7f\xff")
    expect("an escape sequence does not reach the terminal as an escape byte",
           "\x1b" not in shown and "\x07" not in shown)
    expect("newline and tab survive and printable text is unchanged", shown.startswith("ok\n\tA\\x1b]52;c;"))
    expect("carriage return, NUL, DEL and high bytes are escaped",
           all(t in shown for t in ("\\x0d", "\\x00", "\\x7f", "\\xff")))
    lines, tail = br.stamped_lines(bytearray(b"x\r\nT\npart"), 0, 1.5)
    expect("each completed line is stamped and the partial one waits",
           lines == ["1.5000\tx", "1.5000\tT"] and tail == 5)

    expect("a command that outlives its limit exits 124", bd.run(0.3, ["sleep", "30"]) == 124)
    with tempfile.TemporaryDirectory() as scratch:
        marker = os.path.join(scratch, "term")
        stubborn = 'trap "touch %s" TERM; while :; do sleep 0.05; done' % marker
        expect("a command that ignores its limit is sent SIGTERM first and then killed",
               bd.run(0.3, ["sh", "-c", stubborn], grace=0.5) == 124 and os.path.exists(marker))
    expect("a command inside its limit keeps its own status", bd.run(5, ["sh", "-c", "exit 3"]) == 3)

    period = cg.PERIOD
    def stamps(hz, n=8):
        return ["%.4f\tT\n" % (i * period / hz) for i in range(n)] + ["0.0000\tchatter\n"]
    ok, _ = cg.grade(stamps(30e6), 30e6, 1000, 4)
    expect("a clock at the expected frequency is graded ok", ok)
    ok, _ = cg.grade(stamps(25e6), 30e6, 1000, 4)
    expect("a clock at 25 MHz against an expected 30 is out of tolerance", not ok)
    ok, _ = cg.grade(stamps(30e6 * 1.002), 30e6, 1000, 4)
    expect("a clock 2000 ppm off is out of a 1000 ppm tolerance", not ok)
    ok, _ = cg.grade(stamps(30e6 * 1.0005), 30e6, 1000, 4)
    expect("a clock 500 ppm off is inside a 1000 ppm tolerance", ok)
    ok, _ = cg.grade(stamps(30e6, 3), 30e6, 1000, 4)
    expect("too few T lines are refused rather than graded", not ok)
    ok, _ = cg.grade(["0.0000\tnoise\n"], 30e6, 1000, 4)
    expect("a capture with no T is refused", not ok)

    rng = random.Random(7)
    words = [rng.randrange(3000, 3200) for _ in range(ea.SAMPLES)]
    good = frame(words)
    buf = bytearray(good + good)
    got, dropped = ea.extract_frames(buf)
    expect("two intact frames are both taken", len(got) == 2 and dropped == 0 and got[0] == words)
    buf = bytearray(frame(words, trailer=(sum(words) + 1) & 0xFFFF))
    got, dropped = ea.extract_frames(buf)
    expect("a frame with a wrong trailer is dropped and counted", got == [] and dropped == 1)
    garbage = bytes(rng.randrange(256) for _ in range(1000)) + b"\xa5\x5a" + bytes(rng.randrange(256) for _ in range(3000))
    buf = bytearray(garbage + good)
    got, dropped = ea.extract_frames(buf)
    expect("a capture that starts mid-stream still finds the real frame behind a false marker",
           len(got) == 1 and got[0] == words)
    buf = bytearray(good[:-100])
    got, dropped = ea.extract_frames(buf)
    expect("a partial frame waits for its remaining bytes", got == [] and dropped == 0 and len(buf) == len(good) - 100)
    random_bytes = bytearray(rng.randrange(256) for _ in range(5 * ea.FRAME_BYTES))
    got, _ = ea.extract_frames(random_bytes)
    expect("random bytes are not assessed as frames", got == [])

    saved = bp.find_ports
    try:
        bp.find_ports = lambda *a, **k: ["/dev/cu.usbmodem1\x1b[2J", "/dev/cu.usbmodem2"]
        try:
            ea.read_frames(None, 1, 0.1)
            expect("the entropy reader refuses when several nodes match", False)
        except SystemExit as stop:
            expect("the entropy reader refuses when several nodes match", "2 serial nodes" in str(stop) and "\x1b" not in str(stop))
        except Exception:
            expect("the entropy reader refuses when several nodes match", False)
    finally:
        bp.find_ports = saved

    with open(os.path.join(root, "Makefile")) as handle:
        make = handle.read()
    with open(os.path.join(root, "soc", "run_suite_icesugar.sh")) as handle:
        suite = handle.read()
    for name, text, pattern in (("Makefile", make, r"\$\(ICESUGAR_LOADER\)"),
                                ("run_suite_icesugar.sh", suite, r'"\$LOADER"')):
        bare = [l for l in text.splitlines() if re.search(pattern, l) and "bounded.py" not in l]
        expect("%s runs the loader only under soc/bounded.py (unbounded: %s)" % (name, bare[:1]),
               not bare and re.search(pattern, text))

    script = os.path.join(root, "soc", "entropy_capture.sh")
    with tempfile.TemporaryDirectory() as scratch:
        os.makedirs(os.path.join(scratch, "soc"))
        local = os.path.join(scratch, "soc", "entropy_capture.sh")
        with open(script) as src, open(local, "w") as dst:
            dst.write(src.read())
        os.chmod(local, 0o755)
        for args in (["t", "1;x", "8", "0"], ["t", "1", "8!", "0"], ["t", "1", "8", ""],
                     ["t", "1", "8", "-1"], ["t;x", "1", "8", "0"], ["t/x", "1", "8", "0"], ["", "1", "8", "0"]):
            done = subprocess.run([local] + args, capture_output=True, text=True, timeout=60)
            expect("entropy_capture.sh refuses %r with status 2 before any tool runs" % (args,),
                   done.returncode == 2 and "error:" in done.stderr and not os.path.exists(os.path.join(scratch, "build")))

    if errors:
        for message in errors:
            sys.stderr.write("error: %s\n" % message)
        return 1
    print("board-tools: port choice, terminal escaping, loader limit, clock grade, frame check and capture arguments agree.")
    return 0


if __name__ == "__main__":
    sys.exit(check(sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(HERE)))
