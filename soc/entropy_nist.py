#!/usr/bin/env python3
"""Run NIST's SP 800-90B assessment of the iCESugar-Pro's seed source, from bitstream to verdict.

  entropy_nist.py --tag pll --core-pad 0 --out build/nist --bin DIR [--restarts 1000] [--samples 1000000]

One setting per run (core-pad 0 clocks the capture from the 30 MHz PLL, 1 from the 25 MHz pad):
builds the gapless capture bitstream, loads it into SRAM, captures consecutive samples, runs
ea_non_iid on them, restarts the board by loading the bitstream again for every row of the restart
matrix, runs ea_restart on it and grades both reports (soc/nist90b_grade.py). The restart is a JTAG
reconfiguration: the fabric, the PLL and the capture start over, a power cycle is not made. With
--resume a stage whose output file exists is kept, and a partly filled restart matrix is continued.
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


def run(argv, out=None):
    done = subprocess.run(argv, cwd=ROOT, stdout=open(out, "w") if out else None, text=True)
    return done.returncode


def load_bitstream(bit, lock):
    with entropy_stream.locked(lock):
        ok = entropy_stream.load(bit, 90.0, "openFPGALoader")
        entropy_stream.eject_icelink()
    return ok


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
    non_iid_txt, restart_txt, report = stem + "_non_iid.txt", stem + "_restart.txt", stem + "_report.txt"
    if not args.resume:
        for path in (samples_file, matrix_file):
            if os.path.exists(path):
                os.remove(path)

    if run([os.path.join(SOC, "entropy_capture.sh"), build_tag, "128", "8", str(args.core_pad), "stream"]):
        sys.exit("*** the capture bitstream did not build")
    digest = hashlib.sha256(open(bit, "rb").read()).hexdigest()

    if not os.path.exists(samples_file):
        if not load_bitstream(bit, args.lock):
            sys.exit("*** the loader failed")
        argv = [sys.executable, os.path.join(SOC, "entropy_stream.py"), "capture", "--samples", str(args.samples),
                "--out", samples_file, "--seconds", "900"]
        if args.lock:
            argv += ["--lock", args.lock]
        if run(argv):
            sys.exit("*** no consecutive capture of %d samples" % args.samples)
    if run([os.path.join(args.bin, "ea_non_iid"), "-v", samples_file, "1"], out=non_iid_txt):
        sys.exit("*** ea_non_iid failed; read %s" % non_iid_txt)
    non_iid = nist90b_grade.parse_non_iid(open(non_iid_txt).read())
    if non_iid["h_original"] is None:
        sys.exit("*** ea_non_iid printed no H_original; read %s" % non_iid_txt)

    argv = [sys.executable, os.path.join(SOC, "entropy_stream.py"), "restart", "--bit", bit, "--out", matrix_file,
            "--rows", str(args.restarts), "--samples", "1000"]
    if args.lock:
        argv += ["--lock", args.lock]
    if run(argv):
        sys.exit("*** the restart matrix did not fill")
    h_i = min(non_iid["h_original"], 1.0)
    run([os.path.join(args.bin, "ea_restart"), "-n", "-v", matrix_file, "1", "%f" % h_i], out=restart_txt)
    restart = nist90b_grade.parse_restart(open(restart_txt).read())

    ok, lines = nist90b_grade.grade(non_iid, restart, args.floor)
    header = ["SP 800-90B assessment, setting %s (core clock from the %s)" % (
                  args.tag, "25 MHz pad" if args.core_pad else "30 MHz PLL"),
              "bitstream %s sha256 %s" % (os.path.relpath(bit, ROOT), digest),
              "%d consecutive samples, %d restarts of 1000 samples" % (args.samples, args.restarts)]
    text = "\n".join(header + lines + ["%s: min-entropy per sample %s %g" % (
        "PASS" if ok else "FAIL", ">=" if ok else "under", args.floor)]) + "\n"
    open(report, "w").write(text)
    print(text, end="")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
