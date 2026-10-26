#!/bin/bash
# Links one batch of `.S` suite programs against test/board/board_suite.S and writes the
# ROM banks for it, plus the index->name map the host needs to read the verdicts back.
set -euo pipefail
OUT=$1; shift
ROM_WORDS=${ROM_WORDS:-$(awk -F'= *' '/^SOC_ROM_WORDS/{print $2; exit}' "$(dirname "$0")/../../Makefile" | tr -d ' ')}
: "${ROM_WORDS:=2048}"
mkdir -p "$OUT"
ROM_BYTES=$(( ROM_WORDS * 4 ))
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)

CC=""
command -v riscv-none-elf-gcc >/dev/null 2>&1 && CC=riscv-none-elf-gcc
[ -n "$CC" ] || { echo "no RISC-V cross compiler; run 'make riscv-gcc-setup'" >&2; exit 1; }
OBJCOPY=${CC%gcc}objcopy

objs=(); names=(); i=0
for prog in "$@"; do
  base=$(basename "$prog")
  TEXT_PAST=$ROM_BYTES "$HERE/compile_program.sh" "$prog" "$OUT/p$i.o"
  $OBJCOPY --prefix-symbols="p${i}_" "$OUT/p$i.o"
  $OBJCOPY --redefine-sym "p${i}_board_next=board_next" --redefine-sym "p${i}_init_memory=init_memory" "$OUT/p$i.o"
  objs+=("$OUT/p$i.o"); names+=("$base"); i=$((i+1))
done

{
  echo '  .section .rodata'
  echo '  .align 2'
  echo '  .globl board_table'
  echo 'board_table:'
  for n in $(seq 0 $((i-1))); do echo "  .word p${n}__start"; done
  echo '  .align 2'
  echo '  .globl board_tag'
  echo 'board_tag:'
  echo "  .asciz \"B${BATCH_ID:-0} $i\\n\""
  echo '  .align 2'
  echo '  .globl board_count'
  echo 'board_count:'
  echo "  .word $i"
} > "$OUT/table.S"
$CC -march=rv32imac_zicsr_zifencei_zkt_zkr -mabi=ilp32 -nostdlib -c -o "$OUT/table.o" "$OUT/table.S"

$CC -march=rv32imac_zicsr_zifencei_zkt_zkr -mabi=ilp32 -nostdlib -DBOARD_SUITE \
    -I "$ROOT/test/asm" -T "$HERE/board.lds" -Wl,--defsym=ROM_BYTES=$ROM_BYTES -o "$OUT/batch.elf" \
    "$HERE/board_suite.S" "$OUT/table.o" "${objs[@]}"

bytes=$(${CC%gcc}size "$OUT/batch.elf" | awk 'NR==2{print $1+$2}')
if [ "$bytes" -gt "$ROM_BYTES" ]; then
  echo "error: batch is $bytes bytes and the ROM is $ROM_BYTES." >&2
  exit 1
fi

$OBJCOPY -O verilog --verilog-data-width=4 -j .text -j .data "$OUT/batch.elf" "$OUT/rom.hex"
python3 "$ROOT/soc/rom_banks.py" "$OUT/rom.hex" "$ROOT/soc/rom_even.hex" "$ROOT/soc/rom_odd.hex" \
  --rom-words "$ROM_WORDS" >/dev/null
printf '%s\n' "${names[@]}" > "$OUT/names.txt"
echo "batch: $i programs, $bytes bytes of $ROM_BYTES"
