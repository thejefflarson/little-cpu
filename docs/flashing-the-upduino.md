# Flashing the UPduino from macOS, and reading its UART afterwards

`make prog` runs `iceprog` under `sudo` on macOS (`ICEPROG_SUDO` is `sudo` on Darwin and empty
elsewhere). This page explains why, and what happens to the serial port afterwards.

## Root runs only root-owned copies

Root must not execute a binary the user can write: `iceprog` from the OSS CAD Suite and `build/ftread`
both live in user-owned directories. Run `make install-board-tools` once (and again after a new
`iceprog` or a rebuilt `ftread`). It copies both, owned by root and mode 755, into
`/usr/local/libexec/little-cpu/bin` (`BOARD_TOOLS_DIR` moves the prefix) and asks for your password
through `sudo`. The OSS CAD Suite's `bin/iceprog` is a bash wrapper, not the program: copied alone it
cannot find its siblings, and under `sudo` its `#!/usr/bin/env bash` would run whichever `bash` the
caller's `PATH` names first. So the install takes the real executable from the suite's `libexec/`,
and on macOS the libraries it loads from `@executable_path/../lib` into `lib/` beside `bin/`.
`ftread` links libftdi and libusb from Homebrew's prefix, which you own, so the install copies those
dylibs into `lib/ftread/` and repoints `ftread` and each copy at them with `install_name_tool`
(re-signing ad hoc, which arm64 requires after an edit). They stay apart from iceprog's `lib/` so two
builds of one library name cannot collide.

Whenever `ICEPROG_SUDO` is non-empty, `make prog` and `make suite-board` run those copies by path and
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
`/etc/sudoers.d/little-cpu`, which lets you run the two root-owned copies above with no password,
with any arguments. It refuses unless both pass the same root-owned check the board targets run. The
rule never sits in a file you can write: it is piped into a root-owned file in `/etc/sudoers.d`,
named with a dot so sudo ignores it until it is moved into place, and `visudo -cf` checks that
file before `mv` installs it at mode 0440. Probe it with
`sudo -n /usr/local/libexec/little-cpu/bin/iceprog --help`; `sudo -n true` stays refused. Remove
it with `sudo rm /etc/sudoers.d/little-cpu`.

**The rule is root-equivalent for any process running as you.** sudo cannot restrict the
arguments, and `iceprog` takes a path: `iceprog -r FILE` writes FILE as root, and `iceprog FILE`
reads FILE as root and writes it to the board's flash, where `-r` can read it back. So the rule
removes the password between your account and root. It does not add a separate privilege boundary.
Install it only on a machine where that trade is acceptable, and remove it when the board work is
done.

## Why flashing needs root

Run unprivileged, every libftdi tool reports **zero devices**, even while `ioreg` shows the board.
The cause is macOS's own driver: `com.apple.DriverKit-AppleUSBFTDI` claims the FT232H's only
interface, so a user process can't open it. Root can.

The same result came back from `iceprog -d i:0x0403:0x6014`, from `openFPGALoader`, and from a
Homebrew `iceprog` built against a different libusb. So the cause is the system driver's claim on
the interface, not the libusb build.

Don't try to unload the driver. It's a DriverKit extension (a dext), not a kernel extension, so
advice to run `kextunload` is out of date and `kextstat` lists nothing.

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
