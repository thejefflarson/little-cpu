#!/usr/bin/env python3
"""Run NIST's SP 800-90B assessment of the iCESugar-Pro's seed source, from bitstream to verdict.

  entropy_nist.py --tag pll --core-pad 0 --out build/nist --bin DIR [--restarts 1000] [--samples 1000000]

One setting per run (core-pad 0 clocks the capture from the 30 MHz PLL, 1 from the 25 MHz pad):
builds the gapless capture bitstream, loads it into SRAM and captures consecutive samples from the
board's sequence 0 under one hold of the board lock, runs ea_non_iid on them, restarts the board by
loading the bitstream again for every row of the restart matrix, runs ea_restart on it and grades both
reports (soc/nist90b_grade.py). The restart is a JTAG reconfiguration: the fabric, the PLL and the
capture start over, a power cycle is not made. With --resume a capture already taken and a partly
filled restart matrix are kept, but only if the sidecar `<out>/<tag>_meta.json` says they came from
this bitstream, core clock, sample count and restart count.
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
    parser.add_argument("--core-pad", type=int, choices=(0, 1), required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--bin", required=True, help="the directory holding ea_non_iid and ea_restart")
    parser.add_argument("--samples", type=int, default=1_000_000)
    parser.add_argument("--restarts", type=int, default=1000)
    parser.add_argument("--floor", type=float, default=0.5)
    parser.add_argument("--lock")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    stem = os.path.join(args.out, args.tag)
    build_tag = "nist_" + args.tag
    bit = os.path.join(ROOT, "build", "entropy_%s.bit" % build_tag)
    samples_file, matrix_file = stem + "_samples.bin", stem + "_restart.bin"
    meta_file = stem + "_meta.json"
    non_iid_txt, restart_txt, report = stem + "_non_iid.txt", stem + "_restart.txt", stem + "_report.txt"
    leftovers = [samples_file, samples_file + ".part", matrix_file, matrix_file + ".meta.json", meta_file]
    if not args.resume:
        for path in leftovers:
            if os.path.exists(path):
                os.remove(path)

    if run([os.path.join(SOC, "entropy_capture.sh"), build_tag, "128", "9", str(args.core_pad), "stream"],
           timeout=BUILD_S):
        sys.exit("*** the capture bitstream did not build")
    digest = entropy_stream.file_sha256(bit)
    meta = {"bitstream_sha256": digest, "core_pad": args.core_pad, "samples": args.samples,
            "restarts": args.restarts}
    bind_resume(meta_file, meta, samples_file, matrix_file)

    if not os.path.exists(samples_file):
        argv = [sys.executable, os.path.join(SOC, "entropy_stream.py"), "capture", "--samples", str(args.samples),
                "--out", samples_file, "--seconds", "900", "--bit", bit]
        if args.lock:
            argv += ["--lock", args.lock]
        if run(argv, timeout=CAPTURE_S):
            sys.exit("*** no consecutive capture of %d samples from the board's start" % args.samples)
    if run([os.path.join(args.bin, "ea_non_iid"), "-v", samples_file, "1"], out=non_iid_txt):
        sys.exit("*** ea_non_iid failed; read %s" % non_iid_txt)
    with open(non_iid_txt) as handle:
        non_iid = nist90b_grade.parse_non_iid(handle.read())
    if non_iid["h_original"] is None:
        sys.exit("*** ea_non_iid printed no readable H_original; read %s" % non_iid_txt)

    argv = [sys.executable, os.path.join(SOC, "entropy_stream.py"), "restart", "--bit", bit, "--out", matrix_file,
            "--rows", str(args.restarts), "--samples", str(ROW_SAMPLES)]
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
    header = ["SP 800-90B assessment, setting %s (core clock from the %s)" % (
                  args.tag, "25 MHz pad" if args.core_pad else "30 MHz PLL"),
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
