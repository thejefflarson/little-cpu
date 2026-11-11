#!/usr/bin/env python3
"""Packs and unpacks the images soc/boot_loader.c reads from the configuration flash.

An image is three little-endian words, then the payload: BOOT_MAGIC, the payload's length in bytes, and a
checksum that soc/boot_loader.c recomputes while it copies. The payload is linked at BOOT_IMAGE_BASE
(soc/boot_image.lds) and is copied there verbatim. Every number comes from soc/boot_layout.h.
"""

import argparse
import re
import struct
import sys
from pathlib import Path

LAYOUT = Path(__file__).with_name("boot_layout.h")
HEADER_BYTES = 12


def layout():
    values = {}
    for line in LAYOUT.read_text().splitlines():
        m = re.fullmatch(r"#define\s+(BOOT_\w+)\s+(0x[0-9a-fA-F]+|\d+)", line.strip())
        if m:
            values[m.group(1)] = int(m.group(2), 0)
    return values


def checksum(payload):
    """Seeded with the length, then rotate left one and xor each word: soc/boot_loader.c's fold."""
    total = len(payload)
    for (word,) in struct.iter_unpack("<I", payload):
        total = (((total << 1) | (total >> 31)) & 0xFFFFFFFF) ^ word
    return total


def pack(payload):
    lay = layout()
    payload = payload + b"\0" * (-len(payload) % 4)
    limit = lay["BOOT_ROM_BYTES"] - lay["BOOT_IMAGE_BASE"]
    if not payload or len(payload) > limit:
        sys.exit(f"boot_image: payload is {len(payload)} bytes; it must be 1 to {limit}")
    return struct.pack("<III", lay["BOOT_MAGIC"], len(payload), checksum(payload)) + payload


def unpack(image):
    """The payload of a well-formed image; exits with the reason when it is not one."""
    lay = layout()
    if len(image) < HEADER_BYTES:
        sys.exit("boot_image: shorter than the header")
    magic, length, want = struct.unpack_from("<III", image)
    payload = image[HEADER_BYTES:]
    if magic != lay["BOOT_MAGIC"]:
        sys.exit(f"boot_image: magic is {magic:#010x}")
    if length != len(payload) or length % 4:
        sys.exit(f"boot_image: header says {length} bytes, the file holds {len(payload)}")
    if checksum(payload) != want:
        sys.exit("boot_image: the checksum does not match the payload")
    return payload


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("layout", help="print one value from soc/boot_layout.h")
    p.add_argument("name")
    p = sub.add_parser("pack", help="payload binary -> image")
    p.add_argument("payload")
    p.add_argument("image")
    p = sub.add_parser("check", help="exit nonzero unless the image is well formed")
    p.add_argument("image")
    p = sub.add_parser("bytehex", help="image -> one hex byte per line, for $readmemh")
    p.add_argument("image")
    p.add_argument("out")
    p.add_argument("--pad", type=int, default=0, help="fill to this many bytes with 0xFF, erased flash")
    p = sub.add_parser("vhex", help="flat binary -> objcopy's 32-bit verilog format, for rom_banks.py")
    p.add_argument("binary")
    p.add_argument("out")
    args = ap.parse_args()

    if args.cmd == "layout":
        print(layout()[args.name])
    elif args.cmd == "pack":
        Path(args.image).write_bytes(pack(Path(args.payload).read_bytes()))
    elif args.cmd == "check":
        unpack(Path(args.image).read_bytes())
    elif args.cmd == "bytehex":
        data = Path(args.image).read_bytes()
        data += b"\xff" * (args.pad - len(data))
        Path(args.out).write_text("".join(f"{b:02x}\n" for b in data))
    else:
        data = Path(args.binary).read_bytes()
        data += b"\0" * (-len(data) % 4)
        words = (w for (w,) in struct.iter_unpack("<I", data))
        Path(args.out).write_text("@00000000\n" + "".join(f"{w:08x}\n" for w in words))


if __name__ == "__main__":
    main()
