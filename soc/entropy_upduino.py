#!/usr/bin/env python3
"""Capture the UPduino's SB_LFOSC for the entropy assessment, and look for a periodic lock.

  entropy_upduino.py intervals --bit B --out FILE [--frames 24]        raw edge intervals, one run per line
  entropy_upduino.py capture   --bit B --out FILE [--samples N]        consecutive sample bits, one byte each
  entropy_upduino.py restart   --bit B --out FILE [--rows 1000]        the first samples after each reboot
  entropy_upduino.py periodic  FILE...                                 look for a repeating pattern in a sample-bit file

The board is the UPduino's FT232H, which a user process cannot open: iceprog writes the bitstream to the
configuration flash once (`iceprog -S` leaves CDONE low on this board), `iceprog -t` then reboots the
FPGA from it without writing, and ftread reads the UART. All three go through the root-owned wrappers
lc-flash and lc-uart in /usr/local/libexec/little-cpu/bin run through `sudo -n`: lc-flash takes a bitstream
under a build/ directory or -t, lc-uart takes milliseconds, and neither takes anything else. ftread prints
a byte outside 0x20-0x7e as `<xx>`, which `unescape` undoes; the frames' CRC and sum catch the rare byte pair that reads the same
either way. The frames are soc/entropy_interval_capture.v's and soc/entropy_bit_framer.v's.
"""

import argparse
import os
import re
import subprocess
import sys
import tempfile
import time

import entropy_assess
import entropy_stream

SOC = os.path.dirname(os.path.abspath(__file__))
TOOLS = "/usr/local/libexec/little-cpu/bin"
ESCAPED = re.compile(rb"<([0-9a-f]{2})>")
MAX_LAG = 4096
NEIGHBOURS = 8
PROMINENCE_LIMIT = 0.02
LOCK_WAIT_S = 2 * 3600
LOAD_TRIES, LOAD_S, STALL_S, PRIME_S, PAUSE_S = 4, 180, 30, 30, 3
REBOOT_S, READ_SLACK_S = 60, 30
SUDO = ["sudo", "-n"]
NO_DEVICE = b"Can't find iCE FTDI USB device"


def unescape(data):
    """The bytes ftread was shown, from its text: literal printable bytes and `<xx>` for the others."""
    return ESCAPED.sub(lambda m: bytes([int(m.group(1), 16)]), data)


def root_binary(name, check=True):
    """The path of an installed root-owned wrapper, after soc/check_root_binary.sh has walked its parents.

    The walk reads every directory from / down and takes minutes on a loaded machine; check=False
    leaves the guarantee to the sudoers rule, which names these two wrappers and no others.
    """
    if not check:
        return os.path.join(TOOLS, name)
    done = subprocess.run([os.path.join(SOC, "check_root_binary.sh"), os.path.join(TOOLS, name)],
                          capture_output=True, text=True)
    if done.returncode:
        sys.exit("*** %s is not a root-owned binary this script may run under sudo:\n%s" % (name, done.stderr))
    return done.stdout.strip()


def sudo(argv, seconds, stall=None):
    """Run a root-owned wrapper and return (status, stdout, stderr); a tool still running after `seconds`, or
    silent on both streams for `stall` seconds, is sent SIGTERM through sudo, which relays it, and reads
    as status None. A tool that finds no board stops the script, whatever it was asked to do."""
    with tempfile.TemporaryFile() as out, tempfile.TemporaryFile() as err:
        proc = subprocess.Popen(SUDO + argv, stdout=out, stderr=err)
        started = quiet_since = time.monotonic()
        written, stopped = -1, False
        while proc.poll() is None:
            now = time.monotonic()
            size = os.fstat(out.fileno()).st_size + os.fstat(err.fileno()).st_size
            if size != written:
                written, quiet_since = size, now
            if now - started > seconds or (stall and now - quiet_since > stall):
                proc.terminate()
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    pass
                stopped = True
                break
            time.sleep(0.2)
        status = None if stopped else proc.returncode
        out.seek(0)
        err.seek(0)
        got = (status, out.read(), err.read())
    if NO_DEVICE in got[1] + got[2] or (status == 2 and got[2].startswith(b"open: ")):
        sys.exit("*** %s found no UPduino on USB; replug it and rerun (an interrupted restart matrix resumes "
                 "from its rows on disk)" % os.path.basename(argv[0]))
    return got


def prime(flash):
    """Reset the FPGA once with a read, so the FT232H's receive FIFO is empty when a flash write starts.

    The FT232H returns to UART mode when iceprog exits, and a design that streams fills its FIFO. On the
    next iceprog one or two of those bytes survive its purge and shift every read, the flash ID first
    (`0xFF 0xEF 0x70 0x16`), so the erase polls a status byte that never clears and the write hangs or
    verifies wrong. A read-only `lc-flash -t` takes the strays instead, and the write after it is clean.
    """
    return sudo([flash, "-t"], PRIME_S)[0] == 0


def load(bit, flash):
    """Write the bitstream to the configuration flash and verify it; the FPGA then boots it.

    iceprog -S is not used: on the UPduino it ends with the FPGA's CDONE low, unconfigured. An attempt
    that prints nothing for STALL_S is a hung one, and is stopped and retried.
    """
    for attempt in range(LOAD_TRIES):
        prime(flash)
        status, out, err = sudo([flash, bit], LOAD_S, STALL_S)
        if status == 0 and b"VERIFY OK" in out + err:
            return True
        print("entropy_upduino: load attempt %d of %d failed (%s)" % (
            attempt + 1, LOAD_TRIES, "stopped after silence or timeout" if status is None else "status %d, no VERIFY OK" % status),
            flush=True)
        if attempt + 1 < LOAD_TRIES:
            time.sleep(PAUSE_S)
    return False


def capture_budget(read_seconds):
    """The longest a capture may take without the lock wait: every load attempt, the reboot and the read."""
    return LOAD_TRIES * (PRIME_S + LOAD_S + PAUSE_S) + REBOOT_S + read_seconds + READ_SLACK_S


def reboot(flash):
    """Reset the FPGA so it configures itself from the flash again, reading the flash and writing nothing."""
    return sudo([flash, "-t"], REBOOT_S)[0] == 0


def read(uart, seconds):
    """What the UART sent in `seconds`, decoded."""
    return unescape(sudo([uart, str(int(seconds * 1000))], seconds + READ_SLACK_S)[1])


def run_from_boot(tools, samples, seconds, first_zero=True):
    """Reboot, read, and keep the frames; with first_zero only if they begin at sequence 0 and never skip."""
    run = entropy_stream.Run(samples, first_zero=first_zero)
    if not reboot(tools["flash"]):
        return None, run
    run.feed(read(tools["uart"], seconds))
    return run.result(), run


def tools_for(args):
    return {name.split("-")[1]: root_binary(name, not args.no_root_check) for name in ("lc-flash", "lc-uart")}


def intervals(args):
    tools = tools_for(args)
    started = time.monotonic()
    with entropy_stream.locked(args.lock, LOCK_WAIT_S):
        if not load(args.bit, tools["flash"]):
            sys.exit("*** iceprog did not load %s" % args.bit)
        frames, dropped = entropy_assess.extract_frames(bytearray(read(tools["uart"], args.seconds)))
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
    need = capture_budget(args.seconds)
    if args.budget is not None and args.budget < need:
        sys.exit("*** the capture's phases can take %.0fs (%d load attempts, a reboot, a %.0fs read) and the "
                 "budget is %.0fs" % (need, LOAD_TRIES, args.seconds, args.budget))
    tools = tools_for(args)
    started = time.monotonic()
    with entropy_stream.locked(args.lock, LOCK_WAIT_S):
        if not load(args.bit, tools["flash"]):
            sys.exit("*** iceprog did not write and verify %s" % args.bit)
        if not reboot(tools["flash"]):
            sys.exit("*** iceprog -t did not reboot the board")
        frames, _ = entropy_stream.parse_frames(bytearray(read(tools["uart"], args.seconds)))
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
        if not load(args.bit, tools["flash"]):
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
        p.add_argument("--bit", required=True, help="the capture bitstream to write to the flash first; lc-flash accepts only a path under a build/ directory")
        p.add_argument("--out", required=True)
        p.add_argument("--no-root-check", action="store_true",
                       help="skip soc/check_root_binary.sh; the sudoers rule names the two wrappers")
        p.add_argument("--lock", help="a directory made for the length of each board access and removed after")
    sub.choices["intervals"].add_argument("--frames", type=int, default=24)
    sub.choices["intervals"].add_argument("--seconds", type=float, default=60.0)
    sub.choices["capture"].add_argument("--samples", type=int, default=1_000_000)
    sub.choices["capture"].add_argument("--seconds", type=float, default=150.0)
    sub.choices["capture"].add_argument("--budget", type=float,
                                        help="refuse unless the phases' worst case, lock wait excluded, fits in this many seconds")
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
