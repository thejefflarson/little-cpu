#!/bin/bash
# A predicted target narrower than sixteen bits sends this 64 KB program's far jal astray, so its store never lands.
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1
: "${SOC_SRCS:?SOC_SRCS must name the SoC RTL; run this through make window64-test}"
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
ROM_WORDS=16384

mkdir -p "$WORK/soc"
riscv-none-elf-gcc -march=rv32imac_zicsr_zifencei_zkt_zkr -mabi=ilp32 -nostdlib -Wl,-Ttext=0 \
  -o "$WORK/prog.elf" test/window64/prog.S || exit 1
riscv-none-elf-objcopy -O binary -j .text "$WORK/prog.elf" "$WORK/prog.bin" || exit 1
bytes=$(wc -c < "$WORK/prog.bin" | tr -d ' ')
if [ "$bytes" -le 16384 ]; then
  echo "*** window64: the program is $bytes bytes, no larger than a 16 KB window"
  exit 1
fi
python3 soc/boot_image.py vhex "$WORK/prog.bin" "$WORK/prog.hex" || exit 1
python3 soc/rom_banks.py "$WORK/prog.hex" "$WORK/soc/rom_even.hex" "$WORK/soc/rom_odd.hex" --rom-words "$ROM_WORDS" > /dev/null || exit 1

read -ra srcs <<< "$SOC_SRCS"
iverilog -g2012 -I rtl -o "$WORK/sim.vvp" "${srcs[@]}" test/window64/sim.v || { echo "*** window64: the bench does not elaborate"; exit 1; }
got=$(cd "$WORK" && vvp -n sim.vvp 2>&1 | grep -E '^(STORE|TRAP|TIMEOUT)')
if ! [[ $got =~ ^STORE\ 12\ cycles=[0-9]+$ ]]; then
  echo "*** window64: wanted the store of 12, the bench said: ${got:-nothing}"
  exit 1
fi
echo "window64: a $bytes-byte program in a 64 KB window ran its far jal and its straddling branch."
