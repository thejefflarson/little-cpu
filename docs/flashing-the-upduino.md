# Flashing the UPduino from macOS, and reading its UART afterwards

`make prog` runs `iceprog` under `sudo` on macOS (`ICEPROG_SUDO` is `sudo` on Darwin and empty
elsewhere). This page explains why, and what happens to the serial port afterwards.

## Root runs only root-owned copies

Root must not execute a binary the user can write: `iceprog` from the OSS CAD Suite and `build/ftread`
both live in user-owned directories. Run `make install-board-tools` once (and again after a new
`iceprog` or a rebuilt `ftread`). It copies both, plus the two wrappers it compiles (below), owned by root and mode 755, into
`/usr/local/libexec/little-cpu/bin` (`BOARD_TOOLS_DIR` moves the prefix) and asks for your password
through `sudo`. The OSS CAD Suite's `bin/iceprog` is a bash wrapper, not the program: copied alone it
cannot find its siblings, and under `sudo` its `#!/usr/bin/env bash` would run whichever `bash` the
caller's `PATH` names first. So the install takes the real executable from the suite's `libexec/`,
and on macOS the libraries it loads from `@executable_path/../lib` into `lib/` beside `bin/`.
`ftread` links libftdi and libusb from Homebrew's prefix, which you own, so the install copies those
dylibs into `lib/ftread/` and repoints `ftread` and each copy at them with `install_name_tool`
(re-signing ad hoc, which arm64 requires after an edit). They stay apart from iceprog's `lib/` so two
builds of one library name cannot collide.

Whenever `ICEPROG_SUDO` is non-empty, `make prog` and `make suite-board` run the `lc-flash` and `lc-uart`
wrappers (built and installed with those copies, see below) by path and
first refuse any binary that is a symlink, is not a Mach-O or ELF executable (a `#!` script included),
or is not owned by root or is group- or world-writable, or carries an ACL that grants write. The same
test runs on every directory from `/` down (after `realpath`, so macOS's `/var` to `/private/var` is
walked as `/private/var`), on every library `otool -L` (macOS) or `env -i ldd` (Linux) lists,
transitively, and on every directory an ELF's RUNPATH names. Libraries under `/usr/lib` and
`/System/Library` are the system's and pass; `/System/Volumes`, the writable data volume, is refused,
as is any other path that fails the test, or an `@rpath` reference. The check prints the resolved
path it walked, and that is the path sudo runs, so a symlink in the name given cannot be repointed
between the check and the run (`soc/check_root_binary.sh`, graded by `test/board_verdict_test.sh`).
If `/usr/local` or
`BOARD_TOOLS_DIR`'s parents are user-writable (some Intel Homebrew setups), the check refuses and
`BOARD_TOOLS_DIR` must move to a root-owned prefix. On Linux
`ICEPROG_SUDO` is empty and nothing changes: `iceprog` comes from `PATH` and `ftread` from `build/`.

## Running the board targets without a password

macOS's `sudo` caches a password per terminal, and a shell with no terminal (an editor's task
runner, or an agent) never sees that cache. `make install-board-sudoers` writes
`/etc/sudoers.d/little-cpu`, which lets you run two root-owned wrappers with no password, with any
arguments: `lc-flash` and `lc-uart` in `BOARD_TOOLS_DIR/bin`. The real `iceprog` and `ftread` beside
them are not in the rule. The wrappers are small compiled programs (`soc/lc_flash.c`, `soc/lc_uart.c`;
`make install-board-tools` builds them against the installed copies' absolute paths and installs them
with those). The rule refuses unless both wrappers pass the same root-owned check the board targets run.
It never sits in a file you can write: it is piped into a root-owned file in `/etc/sudoers.d`,
named with a dot so sudo ignores it until it is moved into place, and `visudo -cf` checks that
file before `mv` installs it at mode 0440. Probe it with `sudo -n /usr/local/libexec/little-cpu/bin/lc-uart`
(it prints its usage and exits 2 without touching the board); `sudo -n true` stays refused. Remove it with `sudo rm /etc/sudoers.d/little-cpu`.

**What the rule lets a process running as you do.** sudo cannot restrict arguments, so the wrappers do,
and each refuses everything outside the shape the board targets use:

- `lc-flash [-d i:VID:PID] FILE` writes FILE to the configuration flash. `lc-flash [-d i:VID:PID] -t`
  reboots the FPGA from it, writing nothing. No other iceprog flag is passed on, so `-r` (which writes
  its argument as root), `-R`, `-o`, `-S` and the rest are refused.
- FILE is opened with your user and group ids, not root's, so a file you cannot read is refused by the
  kernel (the privileged process takes your identity from the `SUDO_UID` and `SUDO_GID` that sudo sets,
  and refuses to run without them). The wrapper then judges the open file, not the name: its canonical
  path, after every symlink and `..`, must sit under a `build/` directory, it must be a regular file of
  yours, and it must be 1 byte to 4 MiB. Because the checks read the descriptor, a symlink swapped in after
  the open changes nothing. iceprog is handed `/dev/fd/N` for that descriptor rather than a path, and
  runs with a scrubbed environment (`PATH` and `LC_ALL` only).
- `lc-uart MILLISECONDS` reads the UART at 115200 baud for 1 to 3,600,000 ms. Nothing else is accepted.
- `lc-uart -q MILLISECONDS` is the same read with every FT232H pin an input. In UART mode the chip drives
  TXD, RTS# and DTR# (ADBUS0, 2 and 4) onto the flash's clock, data output and select, so a design that
  reads the flash through the shared pins sees RTS# holding the data output high. Quiet mode samples the
  pins in asynchronous bit-bang mode at eight to sixteen times the baud, measures the rate it actually
  got and halves the clock until the USB path keeps up, decodes 8N1 from ADBUS1 in software
  (`soc/uart_soft_rx.h`, graded by `test/uart_soft_rx_test.c`), and prints each ADBUS pin's high
  fraction on stderr. A UART mode read of the same board leaves those three pins driven for as long as
  the port is open.

So the rule no longer hands a process of yours root. What it still allows is writing any bitstream-sized
file of yours that lives under a `build/` directory into the board's flash, and holding the USB
adapter for up to an hour. The `build/` requirement is a statement of what the target is for, not a
privilege boundary: any directory you create named `build` satisfies it, because the privilege boundary is
the file being opened as you. `test/board_wrappers_test.sh` compiles both wrappers against stub tools and
drives each refusal (`-r`, an out-of-tree path, a symlink escaping `build/`, an unreadable file, among
others), each with a forced-red probe in `make probe-gates`. It runs unprivileged, so the
root-to-user identity switch is not exercised there.

## Why flashing needs root

Run unprivileged, every libftdi tool reports **zero devices**, even while `ioreg` shows the board.
The cause is macOS's own driver: `com.apple.DriverKit-AppleUSBFTDI` claims the FT232H's only
interface, so a user process can't open it. Root can.

The same result came back from `iceprog -d i:0x0403:0x6014`, from `openFPGALoader`, and from a
Homebrew `iceprog` built against a different libusb. So the cause is the system driver's claim on
the interface, not the libusb build.

Don't try to unload the driver. It's a DriverKit extension (a dext), not a kernel extension, so
advice to run `kextunload` is out of date and `kextstat` lists nothing.

## The suite on the part

`make suite-board` ran on an UPduino v3.0 on 2026-10-24 at 650aa07c (plus the flash drain below): **79 of 79
programs passed, 0 failed, 0 without a report, in eight batches** of 4, 6, 7, 7, 10, 13, 20 and 12, every
batch on the first flash and the first read. Each read opened with its own `B<batch> <count>` tag. The
runner skips `rvc.S` (12,256 bytes against the 8,192-byte ROM), so the run covers 79 of the 80 `.S` files
in `test/OBSERVED_FLOOR`; the manifest's one `.c` program, `datainit.c`, is outside the board runner's `.S` glob.
The whole run was through the root-owned `lc-flash` and `lc-uart` wrappers under `sudo`, their first
use on the board, and both worked: eight bitstream writes (7 to 9 s each) and eight reads, no password.

The runner writes each bitstream through `flash_bitstream` (`soc/board_verdict.sh`): a read-only
`lc-flash -t` first, then the write, each under `soc/bounded.py` (30 s and 180 s). The `-t` takes the stray UART
bytes an earlier design leaves in the FT232H, which otherwise shift the write's reads and hang its erase.
`bounded.py` sends SIGTERM and waits ten seconds before SIGKILL, because `sudo` relays only SIGTERM to a root
child.

## Reading the UART after `iceprog`

The FT232H is both the programmer and the serial port. `iceprog` leaves it in MPSSE mode, and
`/dev/cu.usbserial-*` disappears.

- **Unplugging and replugging the board** brings the device node back.
- **If the driver has been unloaded**, nothing attaches after a replug. `make ftread` builds
  `./ftread`, which talks libftdi directly and reads the UART with no device node. Run the root-owned copy:
  `sudo /usr/local/libexec/little-cpu/ftread 115200 8000`. `make suite-board` builds and uses it, running the script as you and only `iceprog` and `ftread` under `ICEPROG_SUDO`.
  The verdicts it reads off the UART are untrusted text: bash evaluates array subscripts inside
  `$(( ))`, so `grade_verdict` (`soc/board_verdict.sh`) accepts digits only and reports anything else as a
  parse error. `test/board_verdict_test.sh` grades that. The same text never reaches your terminal
  raw either: the script shows it through `display_safe`, which keeps printable bytes and newlines
  only, so an escape sequence on the wire cannot drive the terminal. The captures in
  `build/suite_board_raw` stay byte-exact.

The iCESugar-Pro doesn't have this problem. Its serial port is a CDC device on the iCELink
debugger, a device node that flashing never takes away (`soc/board_read.py`).

The iCELink also mounts a drag-and-drop volume at `/Volumes/iCELink` (DAPLink's emulated FAT
drive), and Finder can hang reading it. `diskutil eject /Volumes/iCELink` removes it without
affecting JTAG loading or the UART. The board flows use only JTAG and the CDC port, never the
volume, and every iCESugar target and script ejects it when it is mounted, before it touches the loader or the
serial port and again after a load, since the volume can return when the board re-enumerates
(`soc/icelink_eject.sh`).

Always pass `--vid 0x1d50 --pid 0x602b` to a hand-run `openFPGALoader -c cmsisdap`, as the make targets do: the bare
`--detect` failed with "no USB backend available" on 2026-10-17, and the form with the IDs detected the LFE5U-25 on the
same plug-in.

## Loading without a write, and reading a sparse stream

`iceprog -S` (SRAM) is not usable on the UPduino: it finishes with the FPGA's CDONE low, unconfigured, on
2026-10-18. `iceprog -t` reads the flash's ID and resets the FPGA around it, so the FPGA configures itself
from the flash again with nothing written; `soc/entropy_upduino.py` uses it to restart a design a thousand
times for one flash write. Both tools run through `sudo -n` on the root-owned copies, which is the only
thing a sudoers rule for the board needs to name.

`ftread` drops most frames of a stream that leaves the line idle between bursts (it kept 28% of a
15-byte frame every 6.4 ms, with stray bytes ahead of each marker) and keeps nearly all of a stream that
never idles. `soc/entropy_bit_framer.v` has a `FILL` parameter that sends a zero byte in every idle slot
for that reason. `ftread` also prints a byte outside 0x20 to 0x7e as `<xx>`, which
`soc/entropy_upduino.py` decodes. A flash write under a heavily loaded host takes minutes, not the 6 s of
an idle one; the write must not be interrupted, because the erase has already happened.

## A flash write after a streaming design hangs unless a read drains the FT232H first

`iceprog` returns the FT232H to UART mode when it exits, and a design that streams on the UART
(`soc/entropy_capture_upduino.v`, `soc/seed_probe.S`) fills the chip's receive FIFO from then on. The next
`iceprog` purges that FIFO as it switches to MPSSE, but one or two bytes arrive after the purge and
survive it. They shift every byte `iceprog` reads back by that many places. The shift is visible in the
flash ID line (`0xFF 0xEF 0x70 0x16` for a W25Q32, `0xFF 0xFF 0xEF ...` for two strays) and in the reset
step's `cdone: high` where a good run reads low. The erase then polls a status byte that is always 0xFF,
which reads as "busy" forever, so `iceprog` sits at 0% CPU until killed; with two strays the erase
completes and the write verifies wrong (exit 3). A read-only `iceprog -t` does not mind the shift, takes
the strays itself, and the write after it starts clean.

Measured on the UPduino v3.0 on 2026-10-24 (the root-owned `iceprog` and `ftread`, the streaming capture
bitstream, one board, no replug):

| Sequence | First attempt |
|---|---|
| `ftread` 5 s, then `iceprog FILE` | 1 clean of 5; 3 hung at the first erase (stopped at 30 or 90 s), 1 wrote and failed verify (exit 3) |
| `iceprog FILE` twice back to back | 2 of 2 clean |
| `iceprog FILE` after the board sat streaming for minutes | 1 hung at the first erase, then 3 of 3 clean |
| `ftread` 5 s, `iceprog -t`, `iceprog FILE` | 9 of 9 clean; the ID line of `-t` showed a stray `0xFF` in 9 of the session's 12 runs |
| `soc/entropy_upduino.py`'s loader after `ftread` 5 s | 8 of 8 clean on the first attempt, 7.1 to 10.2 s each |

A killed hung `iceprog` is recoverable: the next write succeeded in 3 of 4 tries and failed once with the
same shifted ID (exit 2). After the very first hang of the day, `iceprog -t` failed twice in a row
(`Write error (single byte, rc=-1, expected 1)`, then forty `unexpected rx byte: FF`) until one `ftread`
ran, whose `BITMODE_RESET` and purge cleared the chip. A replug and a host-side FTDI reset were not run:
the board was not touched by hand, and the drain above made them unnecessary to find the cause. A clean
write of the 104 KB capture bitstream takes about 9 s, so a hang is 30 s of silence rather than a long
wait for one.

`soc/entropy_upduino.py` therefore runs `iceprog -t` before every flash write and stops a write that prints
nothing on either stream for `STALL_S` (30 s) or runs past `LOAD_S` (180 s), then drains and retries, up to
`LOAD_TRIES` (4) attempts. A tool that reports no board on USB (`Can't find iCE FTDI USB device` from
`iceprog`, `open:` from `ftread`) stops the whole run at once, so a restart matrix whose board left the bus
resumes from the rows on disk after a replug rather than spending its failure allowance. The capture's
timeout in `soc/entropy_nist.py` is the lock wait plus `capture_budget`, the sum of every load attempt, the
reboot and the read, and the reader refuses a `--budget` below that sum.
