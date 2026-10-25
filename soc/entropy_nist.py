#!/usr/bin/env python3
"""Run NIST's SP 800-90B assessment of a board's seed source, from bitstream to verdict.

  entropy_nist.py --tag pll --core-pad 0 --out build/nist --bin DIR [--restarts 1000] [--samples 1000000]
  entropy_nist.py --board upduino --tag t --osc internal --fab-log2 0 --out build/nist --bin DIR

One setting per run. On the iCESugar-Pro core-pad 0 clocks the capture from the 30 MHz PLL, 1 from the
25 MHz pad; on the UPduino --osc names the clock (crystal or internal SB_HFOSC) and --fab-log2 the fabric
divide of SB_LFOSC. It builds the gapless capture bitstream, loads it and captures consecutive samples from
the board's sequence 0 under one hold of the board lock, runs ea_non_iid on them, restarts the board for
every row of the restart matrix, runs ea_restart on it and grades both reports (soc/nist90b_grade.py).
The iCESugar-Pro restarts by a JTAG reconfiguration: the fabric, the PLL and the capture start over, a
power cycle is not made. The UPduino reboots from its flash. With --resume a capture already taken and a
partly filled restart matrix are kept, but only if the sidecar `<out>/<tag>_meta.json` says they came
from this bitstream, core clock, sample count and restart count.
"""

import argparse
import hashlib
import os
import subprocess
import sys

import entropy_stream
import nist90b_grade

SOC = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SOC)
BUILD_S, CAPTURE_S, TOOL_S, RESTART_S = 1800, 1500, 3600, 4 * 3600
ROW_SAMPLES = 1000
READ_S = 900
HOLDOFF_LOG2 = 25


def run(argv, out=None, timeout=TOOL_S):
    """The command's exit status; a command that outlives `timeout` is stopped and exits the script."""
    try:
        if out:
            with open(out, "w") as handle:
                return subprocess.run(argv, cwd=ROOT, stdout=handle, text=True, timeout=timeout).returncode
        return subprocess.run(argv, cwd=ROOT, text=True, timeout=timeout).returncode
    except subprocess.TimeoutExpired:
        sys.exit("*** %s still running after %ds; stopped" % (os.path.basename(argv[0]), timeout))


def bind_resume(meta_file, meta, samples_file, matrix_file):
    """Refuse data that is not this run's: no sidecar, a sidecar for another run, or a wrong size."""
    if not os.path.exists(meta_file) and (os.path.exists(samples_file) or os.path.exists(matrix_file)):
        sys.exit("*** data sits beside %s, which is missing, so nothing says where it came from; remove it" % meta_file)
    entropy_stream.bind(meta_file, meta)
    if os.path.exists(samples_file) and os.path.getsize(samples_file) != meta["samples"]:
        sys.exit("*** %s holds %d samples, not %d; remove it" % (samples_file, os.path.getsize(samples_file), meta["samples"]))
    if os.path.exists(matrix_file) and os.path.getsize(matrix_file) > meta["restarts"] * ROW_SAMPLES:
        sys.exit("*** %s holds more than %d restarts; remove it" % (matrix_file, meta["restarts"]))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--tag", required=True)
    parser.add_argument("--board", choices=("icesugar", "upduino"), default="icesugar")
    parser.add_argument("--core-pad", type=int, choices=(0, 1))
    parser.add_argument("--osc", choices=("crystal", "internal"))
    parser.add_argument("--fab-log2", type=int, default=0)
    parser.add_argument("--no-root-check", action="store_true")
    parser.add_argument("--out", required=True)
    parser.add_argument("--bin", required=True, help="the directory holding ea_non_iid and ea_restart")
    parser.add_argument("--samples", type=int, default=1_000_000)
    parser.add_argument("--restarts", type=int, default=1000)
    parser.add_argument("--floor", type=float, default=0.5)
    parser.add_argument("--lock")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    upduino = args.board == "upduino"
    if upduino == (args.core_pad is not None) or upduino != (args.osc is not None):
        sys.exit("*** the iCESugar-Pro takes --core-pad and the UPduino takes --osc, and neither takes the other's")
    os.makedirs(args.out, exist_ok=True)
    stem = os.path.join(args.out, args.tag)
    build_tag = "nist_" + args.tag
    bit = os.path.join(ROOT, "build", "entropy_%s.%s" % (build_tag, "bin" if upduino else "bit"))
    samples_file, matrix_file = stem + "_samples.bin", stem + "_restart.bin"
    meta_file = stem + "_meta.json"
    non_iid_txt, restart_txt, report = stem + "_non_iid.txt", stem + "_restart.txt", stem + "_report.txt"
    leftovers = [samples_file, samples_file + ".part", matrix_file, matrix_file + ".meta.json", meta_file]
    if not args.resume:
        for path in leftovers:
            if os.path.exists(path):
                os.remove(path)

    if upduino:
        build = [os.path.join(SOC, "entropy_capture_upduino.sh"), build_tag, args.osc, "stream",
                 str(HOLDOFF_LOG2), str(args.fab_log2), "1"]
        reader = "entropy_upduino.py"
        extra = ["--no-root-check"] if args.no_root_check else []
    else:
        build = [os.path.join(SOC, "entropy_capture.sh"), build_tag, "128", "9", str(args.core_pad), "stream"]
        reader, extra = "entropy_stream.py", []
    if run(build, timeout=BUILD_S):
        sys.exit("*** the capture bitstream did not build")
    digest = entropy_stream.file_sha256(bit)
    meta = {"bitstream_sha256": digest, "core_pad": args.core_pad, "samples": args.samples,
            "restarts": args.restarts}
    if upduino:
        meta.update(board="upduino", osc=args.osc, fab_log2=args.fab_log2)
    bind_resume(meta_file, meta, samples_file, matrix_file)

    if not os.path.exists(samples_file):
        argv = [sys.executable, os.path.join(SOC, reader), "capture", "--samples", str(args.samples),
                "--out", samples_file, "--seconds", str(READ_S), "--bit", bit] + extra
        capture_s = CAPTURE_S
        if upduino:
            import entropy_upduino
            capture_s = entropy_upduino.LOCK_WAIT_S + entropy_upduino.capture_budget(READ_S)
            argv += ["--budget", str(capture_s - entropy_upduino.LOCK_WAIT_S)]
        if args.lock:
            argv += ["--lock", args.lock]
        if run(argv, timeout=capture_s):
            sys.exit("*** no consecutive capture of %d samples from the board's start" % args.samples)
    if run([os.path.join(args.bin, "ea_non_iid"), "-v", samples_file, "1"], out=non_iid_txt):
        sys.exit("*** ea_non_iid failed; read %s" % non_iid_txt)
    with open(non_iid_txt) as handle:
        non_iid = nist90b_grade.parse_non_iid(handle.read())
    if non_iid["h_original"] is None:
        sys.exit("*** ea_non_iid printed no readable H_original; read %s" % non_iid_txt)

    argv = [sys.executable, os.path.join(SOC, reader), "restart", "--bit", bit, "--out", matrix_file,
            "--rows", str(args.restarts), "--samples", str(ROW_SAMPLES)] + extra
    if args.lock:
        argv += ["--lock", args.lock]
    if run(argv, timeout=RESTART_S):
        sys.exit("*** the restart matrix did not fill")
    if os.path.getsize(matrix_file) != args.restarts * ROW_SAMPLES:
        sys.exit("*** %s holds %d samples, not %d restarts of %d" % (
            matrix_file, os.path.getsize(matrix_file), args.restarts, ROW_SAMPLES))
    h_i = min(non_iid["h_original"], 1.0)
    status = run([os.path.join(args.bin, "ea_restart"), "-n", "-v", matrix_file, "1", "%f" % h_i], out=restart_txt)
    with open(restart_txt) as handle:
        restart = nist90b_grade.parse_restart(handle.read())
    if status:
        restart["passed"], restart["reason"] = False, "ea_restart exited %d" % status

    ok, lines = nist90b_grade.grade(non_iid, restart, args.floor)
    clock = ("%s clock, SB_LFOSC divided by 2^%d" % (args.osc, args.fab_log2) if upduino
             else "core clock from the %s" % ("25 MHz pad" if args.core_pad else "30 MHz PLL"))
    header = ["SP 800-90B assessment, setting %s (%s)" % (args.tag, clock),
              "bitstream %s sha256 %s" % (os.path.relpath(bit, ROOT), digest),
              "%d consecutive samples, %d restarts of %d samples" % (args.samples, args.restarts, ROW_SAMPLES)]
    text = "\n".join(header + lines + ["%s: min-entropy per sample %s %g" % (
        "PASS" if ok else "FAIL", ">=" if ok else "under", args.floor)]) + "\n"
    with open(report, "w") as handle:
        handle.write(text)
    print(text, end="")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
