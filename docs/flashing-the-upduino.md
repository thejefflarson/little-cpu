# Flashing the UPduino from macOS, and reading its UART afterwards

`make prog` runs `iceprog` under `sudo` on macOS (`ICEPROG_SUDO` is `sudo` on Darwin and empty
elsewhere). This page explains why, and what happens to the serial port afterwards.

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
  `./ftread`, which talks libftdi directly and reads the UART with no device node. Run it as root:
  `sudo ./ftread 115200 8000`. `make suite-board` builds and uses it the same way.

The iCESugar-Pro doesn't have this problem. Its serial port is a CDC device on the iCELink
debugger, a device node that flashing never takes away (`soc/board_read.py`).
