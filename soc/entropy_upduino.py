#!/usr/bin/env python3
"""Capture the UPduino's SB_LFOSC for the entropy assessment, and look for a periodic lock.

  entropy_upduino.py intervals --bit B --out FILE [--frames 24]        raw edge intervals, one run per line
  entropy_upduino.py capture   --bit B --out FILE [--samples N]        consecutive sample bits, one byte each
  entropy_upduino.py restart   --bit B --out FILE [--rows 1000]        the first samples after each reboot
  entropy_upduino.py periodic  FILE...                                 look for a repeating pattern in a sample-bit file

The board is the UPduino's FT232H, which a user process cannot open: iceprog writes the bitstream to the
configuration flash once (`iceprog -S` leaves CDONE low on this board), `iceprog -t` then reboots the
FPGA from it without writing, and ftread reads the UART, all as the root-owned copies in
/usr/local/libexec/little-cpu/bin run through `sudo -n`. ftread prints a byte outside 0x20-0x7e as
`<xx>`, which `unescape` undoes; the frames' CRC and sum catch the rare byte pair that reads the same
either way. The frames are soc/entropy_interval_capture.v's and soc/entropy_bit_framer.v's.
"""

import argparse
import os
import re
import subprocess
import sys
import time

import entropy_assess
import entropy_stream

SOC = os.path.dirname(os.path.abspath(__file__))
TOOLS = "/usr/local/libexec/little-cpu/bin"
BAUD = 115200
ESCAPED = re.compile(rb"<([0-9a-f]{2})>")
MAX_LAG = 4096
NEIGHBOURS = 8
PROMINENCE_LIMIT = 0.02
LOCK_WAIT_S = 2 * 3600
LOAD_TRIES, LOAD_S = 6, 600


def unescape(data):
    """The bytes ftread was shown, from its text: literal printable bytes and `<xx>` for the others."""
    return ESCAPED.sub(lambda m: bytes([int(m.group(1), 16)]), data)


def root_binary(name, check=True):
    """The path of an installed root-owned tool, after soc/check_root_binary.sh has walked its parents.

    The walk reads every directory from / down and takes minutes on a loaded machine; check=False
    leaves the guarantee to the sudoers rule, which names these two paths and no others.
    """
    if not check:
        return os.path.join(TOOLS, name)
    done = subprocess.run([os.path.join(SOC, "check_root_binary.sh"), os.path.join(TOOLS, name)],
                          capture_output=True, text=True)
    if done.returncode:
        sys.exit("*** %s is not a root-owned binary this script may run under sudo:\n%s" % (name, done.stderr))
    return done.stdout.strip()


def sudo(argv, seconds):
    """Run a root-owned tool and return (status, stdout, stderr); a tool still running after `seconds`
    is sent SIGTERM through sudo, which relays it, and reads as status None."""
    proc = subprocess.Popen(["sudo", "-n"] + argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        out, err = proc.communicate(timeout=seconds)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            out, err = proc.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            out = err = b""
        return None, out, err
    return proc.returncode, out, err


def load(bit, iceprog):
    """Write the bitstream to the configuration flash and verify it; the FPGA then boots it.

    iceprog -S is not used: on the UPduino it ends with the FPGA's CDONE low, unconfigured.
    """
    for attempt in range(LOAD_TRIES):
        status, out, err = sudo([iceprog, bit], LOAD_S)
        if status == 0 and b"VERIFY OK" in out + err:
            return True
        if attempt + 1 < LOAD_TRIES:
            time.sleep(3)
    return False


def reboot(iceprog):
    """Reset the FPGA so it configures itself from the flash again, reading the flash and writing nothing."""
    return sudo([iceprog, "-t"], 60)[0] == 0


def read(ftread, seconds):
    """What the UART sent in `seconds`, decoded."""
    return unescape(sudo([ftread, str(BAUD), str(int(seconds * 1000))], seconds + 30)[1])


def run_from_boot(tools, samples, seconds, first_zero=True):
    """Reboot, read, and keep the frames; with first_zero only if they begin at sequence 0 and never skip."""
    run = entropy_stream.Run(samples, first_zero=first_zero)
    if not reboot(tools["iceprog"]):
        return None, run
    run.feed(read(tools["ftread"], seconds))
    return run.result(), run


def tools_for(args):
    return {name: root_binary(name, not args.no_root_check) for name in ("iceprog", "ftread")}


def intervals(args):
    tools = tools_for(args)
    started = time.monotonic()
    with entropy_stream.locked(args.lock, LOCK_WAIT_S):
        if not load(args.bit, tools["iceprog"]):
            sys.exit("*** iceprog did not load %s" % args.bit)
        frames, dropped = entropy_assess.extract_frames(bytearray(read(tools["ftread"], args.seconds)))
    runs = frames[: args.frames]
    with open(args.out, "w") as handle:
        for run in runs:
            handle.write(" ".join(map(str, run)) + "\n")
    print("entropy_upduino: %d of %d interval frames in %.0fs, %d dropped" % (
        len(runs), args.frames, time.monotonic() - started, dropped))
    return 0 if len(runs) == args.frames else 1


def spliced(frames, count):
    """The first `count` sample bits of the frames in order, and how many places a lost frame joins two runs."""
    sequences = [(seq, epoch) for seq, epoch, _ in frames]
    splices = sum((b[0] - a[0]) % entropy_stream.SEQ_MOD != 1 or a[1] != b[1] for a, b in zip(sequences, sequences[1:]))
    return entropy_stream.frames_to_samples(frames, count), splices


def capture(args):
    """Samples from one boot. The reader loses about one frame in a hundred (a CRC failure or a skipped
    sequence number), so the run is the good frames in order with a seam where one was lost, and the
    seams are counted and printed for whoever quotes the result."""
    tools = tools_for(args)
    started = time.monotonic()
    with entropy_stream.locked(args.lock, LOCK_WAIT_S):
        if not load(args.bit, tools["iceprog"]):
            sys.exit("*** iceprog did not write and verify %s" % args.bit)
        if not reboot(tools["iceprog"]):
            sys.exit("*** iceprog -t did not reboot the board")
        frames, _ = entropy_stream.parse_frames(bytearray(read(tools["ftread"], args.seconds)))
    samples, splices = spliced(frames, args.samples)
    if len(samples) < args.samples:
        print("entropy_upduino: %d samples from %d frames, short of %d" % (len(samples), len(frames), args.samples))
        return 1
    with open(args.out + ".part", "wb") as handle:
        handle.write(bytes(samples))
    os.replace(args.out + ".part", args.out)
    print("entropy_upduino: %d samples in %.0fs from %d frames, %d seams where a frame was lost, ones %.4f"
          % (len(samples), time.monotonic() - started, len(frames), splices, sum(samples) / len(samples)))
    return 0


def restart(args):
    tools = tools_for(args)
    done, failures = 0, 0
    entropy_stream.bind(args.out + ".meta.json", {"bitstream_sha256": entropy_stream.file_sha256(args.bit),
                                                   "row_samples": args.samples})
    if os.path.exists(args.out):
        size = os.path.getsize(args.out)
        if size % args.samples:
            sys.exit("*** %s holds %d bytes, not a whole number of %d-sample rows" % (args.out, size, args.samples))
        done = size // args.samples
    if done >= args.rows:
        return 0
    with entropy_stream.locked(args.lock, LOCK_WAIT_S), open(args.out, "ab") as handle:
        if not load(args.bit, tools["iceprog"]):
            sys.exit("*** iceprog did not write and verify %s" % args.bit)
        while done < args.rows:
            samples, _ = run_from_boot(tools, args.samples, args.seconds)
            if samples is None:
                failures += 1
                print("entropy_upduino: row %d failed; retrying" % done, flush=True)
                if failures > args.max_failures:
                    sys.exit("*** %d failed rows; stopping" % failures)
                continue
            handle.write(bytes(samples))
            handle.flush()
            done += 1
            if done % 25 == 0 or done == args.rows:
                print("entropy_upduino: %d of %d restarts, %d failed rows" % (done, args.rows, failures), flush=True)
    return 0


def self_matches(bits, max_lag=MAX_LAG):
    """(lag, fraction of positions where bit i equals bit i+lag) for each lag, over the whole stream."""
    n = len(bits)
    word = int("".join(map(str, bits)), 2)
    out = []
    for lag in range(1, min(max_lag, n // 2) + 1):
        span = n - lag
        differ = ((word ^ (word >> lag)) & ((1 << span) - 1)).bit_count()
        out.append((lag, 1 - differ / span))
    return out


def periodic_peak(bits, max_lag=MAX_LAG):
    """The lag whose self-match stands furthest above (or below) its neighbours', and by how much.

    A stream that repeats with period P matches itself far better at multiples of P than a few lags
    either side; slow drift of the oscillator's phase moves the match smoothly with the lag and
    leaves no peak, so the prominence is the match less the median of the eight lags either side.
    """
    match = dict(self_matches(bits, max_lag))
    last = max(match)
    best = (0, 0.0)
    for lag in range(2, last + 1):
        near = sorted(match[k] for k in range(max(1, lag - NEIGHBOURS), min(last, lag + NEIGHBOURS) + 1) if k != lag)
        prominence = match[lag] - near[len(near) // 2]
        if abs(prominence) > abs(best[1]):
            best = (lag, prominence)
    return best


def periodic(args):
    status = 0
    for path in args.files:
        bits = list(open(path, "rb").read())
        lag, prominence = periodic_peak(bits)
        verdict = "PERIODIC" if abs(prominence) >= PROMINENCE_LIMIT else "no periodic lock"
        status |= abs(prominence) >= PROMINENCE_LIMIT
        print("%s: %d samples, ones %.4f; self-match at lag 1 %.4f; most prominent lag in 2-%d is %d, standing %+.4f "
              "from its neighbours (limit %.2f): %s"
              % (path, len(bits), sum(bits) / len(bits), self_matches(bits, 1)[0][1], MAX_LAG, lag, prominence,
                 PROMINENCE_LIMIT, verdict))
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    for name in ("intervals", "capture", "restart"):
        p = sub.add_parser(name)
        p.add_argument("--bit", required=True, help="the capture bitstream to load into SRAM first")
        p.add_argument("--out", required=True)
        p.add_argument("--no-root-check", action="store_true",
                       help="skip soc/check_root_binary.sh; the sudoers rule names the two tools")
        p.add_argument("--lock", help="a directory made for the length of each board access and removed after")
    sub.choices["intervals"].add_argument("--frames", type=int, default=24)
    sub.choices["intervals"].add_argument("--seconds", type=float, default=60.0)
    sub.choices["capture"].add_argument("--samples", type=int, default=1_000_000)
    sub.choices["capture"].add_argument("--seconds", type=float, default=150.0)
    rp = sub.choices["restart"]
    rp.add_argument("--rows", type=int, default=1000)
    rp.add_argument("--samples", type=int, default=1000)
    rp.add_argument("--seconds", type=float, default=4.5)
    rp.add_argument("--max-failures", type=int, default=3000)
    pp = sub.add_parser("periodic")
    pp.add_argument("files", nargs="+")
    args = parser.parse_args()
    return {"intervals": intervals, "capture": capture, "restart": restart, "periodic": periodic}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
