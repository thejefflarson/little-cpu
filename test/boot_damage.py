#!/usr/bin/env python3
"""usage: boot_damage.py IMAGE OUTDIR MAX_PAYLOAD

Writes four damaged copies of a boot image into OUTDIR: one payload bit flipped, a payload length four bytes past
what the ROM holds, a length that is not a whole number of words, and an erased flash's 0xFF where the header
should be."""

import struct
import sys
from pathlib import Path

source, out, limit = Path(sys.argv[1]), Path(sys.argv[2]), int(sys.argv[3])
image = source.read_bytes()

flipped = bytearray(image)
flipped[-5] ^= 0x01
(out / "flipped.img").write_bytes(flipped)

for name, length in (("long", limit + 4), ("odd", 7)):
    damaged = bytearray(image)
    damaged[4:8] = struct.pack("<I", length)
    (out / f"{name}.img").write_bytes(damaged)

(out / "erased.img").write_bytes(b"\xff" * 16)
