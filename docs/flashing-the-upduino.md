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
