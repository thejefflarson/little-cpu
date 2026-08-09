/*
 * Dhrystone Benchmark, Version 2.1 (Language: C) — module 2 of 2.
 *
 * IT IS A SEPARATE TRANSLATION UNIT AND MUST STAY ONE. Six of the benchmark's
 * eleven procedures live here so the compiler cannot inline them into the
 * measured loop in dhry_1.c. Merge the two files, or build them with `-flto`,
 * and most of Dhrystone becomes dead code the optimiser deletes — the published
 * numbers this exists to be compared against are all from a two-module,
 * no-LTO build.
 */

#include "dhry.h"
#include "dhry_port.h"

void Proc_6(Enumeration Enum_Val_Par, Enumeration *Enum_Ref_Par) {
  *Enum_Ref_Par = Enum_Val_Par;
  if (!Func_3(Enum_Val_Par)) {
    *Enum_Ref_Par = Ident_4;
  }
  switch (Enum_Val_Par) {
  case Ident_1:
    *Enum_Ref_Par = Ident_1;
    break;
  case Ident_2:
    if (Int_Glob > 100) {
      *Enum_Ref_Par = Ident_1;
    } else {
      *Enum_Ref_Par = Ident_4;
    }
    break;
  case Ident_3:
    *Enum_Ref_Par = Ident_2;
    break;
  case Ident_4:
    break;
  case Ident_5:
    *Enum_Ref_Par = Ident_3;
    break;
  }
}

void Proc_7(One_Fifty Int_1_Par_Val, One_Fifty Int_2_Par_Val,
            One_Fifty *Int_Par_Ref) {
  One_Fifty Int_Loc;

  Int_Loc = Int_1_Par_Val + 2;
  *Int_Par_Ref = Int_2_Par_Val + Int_Loc;
}

void Proc_8(Arr_1_Dim Arr_1_Par_Ref, Arr_2_Dim Arr_2_Par_Ref,
            int Int_1_Par_Val, int Int_2_Par_Val) {
  REG One_Fifty Int_Index;
  REG One_Fifty Int_Loc;

  Int_Loc = Int_1_Par_Val + 5;
  Arr_1_Par_Ref[Int_Loc] = Int_2_Par_Val;
  Arr_1_Par_Ref[Int_Loc + 1] = Arr_1_Par_Ref[Int_Loc];
  Arr_1_Par_Ref[Int_Loc + 30] = Int_Loc;
  for (Int_Index = Int_Loc; Int_Index <= Int_Loc + 1; ++Int_Index) {
    Arr_2_Par_Ref[Int_Loc][Int_Index] = Int_Loc;
  }
  Arr_2_Par_Ref[Int_Loc][Int_Loc - 1] += 1;
  Arr_2_Par_Ref[Int_Loc + 20][Int_Loc] = Arr_1_Par_Ref[Int_Loc];
  Int_Glob = 5;
}

Enumeration Func_1(Capital_Letter Ch_1_Par_Val, Capital_Letter Ch_2_Par_Val) {
  Capital_Letter Ch_1_Loc;
  Capital_Letter Ch_2_Loc;

  Ch_1_Loc = Ch_1_Par_Val;
  Ch_2_Loc = Ch_1_Loc;
  if (Ch_2_Loc != Ch_2_Par_Val) {
    return Ident_1;
  } else {
    Ch_1_Glob = Ch_1_Loc;
    return Ident_2;
  }
}

/*
 * `Ch_Loc` is read on a path the published source never initialises it on. The
 * loop below always runs at least once and always assigns it, so the read is
 * unreachable rather than merely unlikely, but gcc cannot see that — hence the
 * initialiser in the declaration. Any value works; the two comparisons that
 * read it are false for every char but 'W'..'Y' and 'R', neither of which the
 * loop can produce.
 */
Boolean Func_2(Str_30 Str_1_Par_Ref, Str_30 Str_2_Par_Ref) {
  REG One_Thirty Int_Loc;
  Capital_Letter Ch_Loc = '\0';

  Int_Loc = 2;
  while (Int_Loc <= 2) {
    if (Func_1(Str_1_Par_Ref[Int_Loc], Str_2_Par_Ref[Int_Loc + 1]) == Ident_1) {
      Ch_Loc = 'A';
      Int_Loc += 1;
    }
  }
  if (Ch_Loc >= 'W' && Ch_Loc < 'Z') {
    Int_Loc = 7;
  }
  if (Ch_Loc == 'R') {
    return true;
  } else {
    if (strcmp(Str_1_Par_Ref, Str_2_Par_Ref) > 0) {
      Int_Loc += 7;
      Int_Glob = Int_Loc;
      return true;
    } else {
      return false;
    }
  }
}

Boolean Func_3(Enumeration Enum_Par_Val) {
  Enumeration Enum_Loc;

  Enum_Loc = Enum_Par_Val;
  if (Enum_Loc == Ident_3) {
    return true;
  } else {
    return false;
  }
}
