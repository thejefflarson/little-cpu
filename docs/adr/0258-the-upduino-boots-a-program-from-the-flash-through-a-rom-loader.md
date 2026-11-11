# ADR-0258: The UPduino boots a program from the flash through a ROM loader

**Status:** Accepted · 2026-10-27. Builds on ADR-0135 (the flash is reachable, shape (a): read it into the
block RAM fetch already uses) and ADR-0255 (the board top's cell headroom). Measured on `main` at 0ce429f7
plus this tree, yosys 0.68+48, nextpnr 0.11-1-g62e659ed (OSS CAD Suite 20260811). The bench numbers are from
simulation and the board run is under Measured on the board.

## Decision

**A 812-byte loader in the bitstream's ROM copies a program image from flash offset `0x20000` into
instruction memory at `0x400`, executes `fence.i` and jumps to it. The fetch path is untouched.** A
program change then writes about 7 KB at an offset past the bitstream instead of re-packing, re-placing and
re-flashing 104,090 bytes.

- **Layout, in `soc/boot_layout.h` and nowhere else.** The loader, the flash wrapper, the image tool and the
  Makefile read it. The bitstream is 104,090 bytes (`icepack -s`, every build: the part's bitstream size is
  fixed); `iceprog` erases whole 64 KiB blocks from the offset it is given, so the image sits at the next
  block, `0x20000`, 26,982 bytes clear of the bitstream's end. A bitstream write erases blocks 0 and 1 and
  stops short of the image, so re-flashing the bitstream keeps the program.
- **Image format.** Three little-endian words (magic `LCIM`, payload length in bytes, checksum), then the
  payload, which is linked at `0x400` by `soc/boot_image.lds` through `test/crt0.S` and copied verbatim. The
  checksum is seeded with the length and folds each word as rotate-left-one then xor. It catches a flipped bit
  or a truncation; it is not a signature, and the flash is trusted as the bitstream is.
- **The loader reads only.** The one flash command it shifts out is `0x03` with a 24-bit address. It stores
  into instruction memory as it reads, accumulates the checksum, deselects, and only then prints, because the
  UART shares pin 14 with the flash's data input while the select is held (ADR-0135). `BOOT NOIMAGE` (bad
  magic, which an erased flash reads as `FFFFFFFF`), `BOOT BADLEN` (zero, past the ROM, or not whole words)
  and `BOOT BADSUM` stop in a loop; nothing is executed. Success prints `BOOT OK len=N cycles=N`.
- **Room.** The loader owns ROM below `0x400`; an image has `0x2000 - 0x400 = 7,168` bytes including its
  `.data` image. `make boot-loader-test` fails if the loader outgrows `0x400`.
- **The host side.** `make boot-image BOOT_PROG=x.c` builds through the existing `soc-rom` recipe (the ISA string
  stays stated once) and `soc/boot_image.py` packs it; `make prog-image` runs the usual `-t` drain and one
  `lc-flash -o 131072`. `lc-flash` gained `-o OFFSET`, hardened as its other modes are: plain decimal digits
  with no leading zero, a 64 KiB boundary, at or past `BOOT_FLASH_OFFSET`, below 4 MiB; the file must carry the
  image magic, a length matching its size and be at most 64 KiB, under `build/`, opened as the caller; iceprog
  is handed `-o` and `/dev/fd/N` built by the wrapper, never the caller's text. The sudoers rule names the
  binary, not its arguments, so it needs no change; `make install-board-tools` must be re-run to install the
  new wrapper.

## Measurements

Load time, simulated on `rtl/littlesoc.v` against `test/spiflash_model.v`: 9,641 cycles for a 188-byte
payload and 11,351 for 224 bytes, 47.5 cycles a byte (the controller shifts a bit per two core clocks and
software polls between bytes). A full 7,168-byte image is about 341,000 cycles, 28 ms at 12 MHz. The longest
stretch the flash clock sat low with the select held was 54 cycles against the pin lockout's 128;
`test/boot_loader_sim.v` fails at 128.

Cells: no RTL changed, so the board top is the same netlist with different block RAM contents. `upduino_top`
packed under six instance names (`soc core cpu u_soc dut s0`), `ICESTORM_LC` of 5,280, default ROM against
the loader ROM on this tree:

| name | default ROM | loader ROM |
|---|---|---|
| soc | 5,101 | 5,101 |
| core | 5,118 | 5,118 |
| cpu | 5,118 | 5,118 |
| u_soc | 5,113 | 5,113 |
| dut | 5,118 | 5,118 |
| s0 | 5,098 | 5,098 |

The worst name leaves 162 cells locally. A different image is a different block RAM init and not a
different netlist, which is the point.

## Measured on the board

Run on an UPduino v3.0 on 2026-11-11, the loader built with `BOARD_OSC=internal` (12 MHz) and each image
written with `make prog-image`, with no re-place between them:

| Flash at 0x20000 | UART |
|---|---|
| erased | `BOOT NOIMAGE FFFFFFFF 00000000` |
| image A, 328 bytes | `IMAGE A sum=10`, `LOAD cycles=15721` |
| image B, 364 bytes | `IMAGE B product=006480`, `LOAD cycles=17431` |
| image A with one payload bit flipped | `BOOT BADSUM 509A79C9 529A79C9`, never run |

The two loads differ by 1,710 cycles over 36 bytes, 47.5 cycles a byte, the figure simulation predicted, plus
about 140 cycles of fixed cost; a full 7,168-byte image is about 340,600 cycles, 28.4 ms at 12 MHz. The
corrupt image had to be written past `make prog-image`, which refuses a bad checksum on the host; `lc-flash`
checks only the header and the offset, so the loader's own check is the one this row grades.

A line printed once at power-up is gone before a host can open the UART after `iceprog` releases the part,
so the loader repeats a failure line about four times a second, and the test images repeat theirs with the
load's cycle count, which the loader leaves in `mscratch` because the C startup overwrites the argument
registers. The first `lc-uart` read after a flash returns nothing and the second reads normally.

## Declined

Fetching from the SPRAM (ADR-0135's shape (b)) and a loader that keeps the image in RAM and jumps there
(the core fetches only from the ROM). A CRC-32 in place of the rotate-xor: it costs loader bytes the ROM
budget is short of for a flash nobody attacks. A write-capable controller or a loader that updates the
flash: the controller stays read-only so no firmware can erase the bitstream.
