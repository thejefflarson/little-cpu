#!/bin/bash
# Links one batch of `.S` suite programs against test/board/board_suite.S and
# writes the ROM banks for it, plus the index->name map the host needs to read
# the verdicts back.
#
# SYMBOL COLLISIONS ARE THE REASON FOR objcopy. Every program defines `_start`
# and `tohost` and reuses the same local labels, so several cannot be linked
# together as they are. Each is assembled alone and then has ITS OWN symbols
# prefixed -- p0_, p1_ -- which makes each `_start` a distinct address the
# driver's table can hold.
#
# Usage: build_batch.sh <out-dir> <prog.S> [prog.S ...]
set -euo pipefail
OUT=$1; shift
# The part's instruction memory, in words, read from the Makefile so it cannot
# drift from what soc/rom_banks.py is told to pad to.
ROM_WORDS=${ROM_WORDS:-$(awk -F'= *' '/^SOC_ROM_WORDS/{print $2; exit}' "$(dirname "$0")/../../Makefile" | tr -d ' ')}
: "${ROM_WORDS:=2048}"
mkdir -p "$OUT"
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)

CC=""
for c in riscv64-elf-gcc riscv64-unknown-elf-gcc; do
  command -v "$c" >/dev/null 2>&1 && { CC=$c; break; }
done
[ -n "$CC" ] || { echo "no RISC-V cross compiler" >&2; exit 1; }
OBJCOPY=${CC%gcc}objcopy

objs=(); names=(); i=0
for prog in "$@"; do
  base=$(basename "$prog")
  # TEXT_PAST is the top of the instruction memory and differs between a
  # simulator and the part -- 4096 words against 2048. loadfault.S and
  # storefault.S probe just below it, so a program built with the simulator's
  # value tests an address this machine does not have and fails on a core that
  # refused it correctly.
  $CC -march=rv32imac_zicsr_zifencei -mabi=ilp32 -nostdlib -DBOARD_SUITE \
      -DTEXT_PAST=$(( ROM_WORDS * 4 )) \
      -I "$ROOT/test/asm" -c -o "$OUT/p$i.o" "$prog"
  $OBJCOPY --prefix-symbols="p${i}_" "$OUT/p$i.o"
  # --prefix-symbols renames UNDEFINED symbols too, so the program's reference to
  # the driver's `board_next` became `pN_board_next` and resolves to nothing.
  # Put every external the driver provides back the way it was.
  $OBJCOPY --redefine-sym "p${i}_board_next=board_next" "$OUT/p$i.o"
  objs+=("$OUT/p$i.o"); names+=("$base"); i=$((i+1))
done

# The table the driver indexes. Written here rather than in the driver because
# only this script knows how many programs went in.
{
  echo '  .section .rodata'
  echo '  .globl board_table'
  echo 'board_table:'
  for n in $(seq 0 $((i-1))); do echo "  .word p${n}__start"; done
  echo '  .globl board_count'
  echo 'board_count:'
  echo "  .word $i"
} > "$OUT/table.S"
$CC -march=rv32imac_zicsr_zifencei -mabi=ilp32 -nostdlib -c -o "$OUT/table.o" "$OUT/table.S"

$CC -march=rv32imac_zicsr_zifencei -mabi=ilp32 -nostdlib -DBOARD_SUITE \
    -I "$ROOT/test/asm" -T "$HERE/board.lds" -o "$OUT/batch.elf" \
    "$HERE/board_suite.S" "$OUT/table.o" "${objs[@]}"

# ld does not police the 8K rom region for an overflow it can satisfy by
# spilling, so the size is checked here rather than assumed.
bytes=$(${CC%gcc}size "$OUT/batch.elf" | awk 'NR==2{print $1+$2}')
if [ "$bytes" -gt 8192 ]; then
  echo "error: batch is $bytes bytes and the ROM is 8192." >&2
  exit 1
fi

$OBJCOPY -O verilog --verilog-data-width=4 -j .text -j .data "$OUT/batch.elf" "$OUT/rom.hex"
python3 "$ROOT/soc/rom_banks.py" "$OUT/rom.hex" "$ROOT/soc/rom_even.hex" "$ROOT/soc/rom_odd.hex" \
  --rom-words 2048 >/dev/null
printf '%s\n' "${names[@]}" > "$OUT/names.txt"
echo "batch: $i programs, $bytes bytes of 8192"
