#!/bin/bash
# Compiles one suite program for a board batch into the relocatable object $2. A `.c` program is built as soc-rom builds it and
# joined with test/board/c_start.S, so its entry, runtime_init and verdict hook sit inside the one object build_batch.sh prefixes.
set -euo pipefail
prog=$1; out=$2
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
CC=riscv-none-elf-gcc
arch=(-march=rv32imac_zicsr_zifencei_zkt_zkr -mabi=ilp32 -nostdlib)
case $prog in
  *.c)
    $CC "${arch[@]}" -Os -std=c11 -ffreestanding -fno-tree-loop-distribute-patterns -Wall -Wextra -Werror \
        -DBOARD_SUITE -I "$ROOT/test/asm" -c -o "$out.prog.o" "$prog"
    $CC "${arch[@]}" -c -o "$out.start.o" "$HERE/c_start.S"
    $CC "${arch[@]}" -r -o "$out" "$out.prog.o" "$out.start.o"
    rm -f "$out.prog.o" "$out.start.o";;
  *)
    $CC "${arch[@]}" -DBOARD_SUITE ${TEXT_PAST:+-DTEXT_PAST=$TEXT_PAST} -I "$ROOT/test/asm" -c -o "$out" "$prog";;
esac
