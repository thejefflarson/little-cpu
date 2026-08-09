/*
 * Dhrystone Benchmark, Version 2.1 (Language: C) — module 1 of 2.
 *
 * The published main loop, unchanged. What differs from the file Weicker
 * published is only the scaffolding around it: `malloc` becomes a static pool,
 * the `times()`/`time()` block becomes two `mcycle` reads, and the closing
 * `printf` block becomes the self-check plus dhry_report(). Nothing inside the
 * timed loop is touched, and dhry_2.c stays a separate translation unit so the
 * optimiser cannot inline the work away.
 *
 * THE SELF-CHECK IS NOT DECORATION. The published version prints the final
 * value of every global and leaves the comparison to a human reading the
 * output against a table. Nothing here reads that output, so the comparison is
 * made in the program instead: without it, most of Dhrystone's results are
 * unused and a link-time optimiser -- or a future -flto -- would be within its
 * rights to delete the loop that produced them, leaving a benchmark that
 * measures an empty `for`.
 */

#include "dhry.h"
#include "dhry_port.h"

#ifndef DHRY_RUNS
#error "DHRY_RUNS must be defined with the iteration count"
#endif

Rec_Pointer Ptr_Glob, Next_Ptr_Glob;
int         Int_Glob;
Boolean     Bool_Glob;
char        Ch_1_Glob, Ch_2_Glob;
int         Arr_1_Glob[50];
int         Arr_2_Glob[50][50];

void Proc_1(Rec_Pointer Ptr_Val_Par) {
  REG Rec_Pointer Next_Record = Ptr_Val_Par->Ptr_Comp;

  structassign(*Ptr_Val_Par->Ptr_Comp, *Ptr_Glob);
  Ptr_Val_Par->variant.var_1.Int_Comp = 5;
  Next_Record->variant.var_1.Int_Comp = Ptr_Val_Par->variant.var_1.Int_Comp;
  Next_Record->Ptr_Comp = Ptr_Val_Par->Ptr_Comp;
  Proc_3(&Next_Record->Ptr_Comp);
  if (Next_Record->Discr == Ident_1) {
    Next_Record->variant.var_1.Int_Comp = 6;
    Proc_6(Ptr_Val_Par->variant.var_1.Enum_Comp,
           &Next_Record->variant.var_1.Enum_Comp);
    Next_Record->Ptr_Comp = Ptr_Glob->Ptr_Comp;
    Proc_7(Next_Record->variant.var_1.Int_Comp, 10,
           &Next_Record->variant.var_1.Int_Comp);
  } else {
    structassign(*Ptr_Val_Par, *Ptr_Val_Par->Ptr_Comp);
  }
}

/* `Enum_Loc` is read by the `while` before anything has written it on the
 * published source's own terms. The body always runs first and always assigns
 * it -- Ch_1_Glob is 'A' at every call -- so the initialiser here only tells
 * gcc what it cannot work out for itself. */
void Proc_2(One_Fifty *Int_Par_Ref) {
  One_Fifty   Int_Loc;
  Enumeration Enum_Loc = Ident_2;

  Int_Loc = *Int_Par_Ref + 10;
  do {
    if (Ch_1_Glob == 'A') {
      Int_Loc -= 1;
      *Int_Par_Ref = Int_Loc - Int_Glob;
      Enum_Loc = Ident_1;
    }
  } while (Enum_Loc != Ident_1);
}

void Proc_3(Rec_Pointer *Ptr_Ref_Par) {
  if (Ptr_Glob != Null) {
    *Ptr_Ref_Par = Ptr_Glob->Ptr_Comp;
  }
  Proc_7(10, Int_Glob, &Ptr_Glob->variant.var_1.Int_Comp);
}

void Proc_4(void) {
  Boolean Bool_Loc;

  Bool_Loc = Ch_1_Glob == 'A';
  Bool_Glob = Bool_Loc | Bool_Glob;
  Ch_2_Glob = 'B';
}

void Proc_5(void) {
  Ch_1_Glob = 'A';
  Bool_Glob = false;
}

/*
 * The values the published output table says a correct run ends with. Checked
 * here rather than printed for a human to check, so a run that computed the
 * wrong thing reports FAIL through `tohost` and the number is not quoted.
 * Int_Glob and Arr_2_Glob[8][7] both depend on the loop count, so they are
 * derived from it rather than written out.
 */
static int self_check(int runs, One_Fifty Int_1_Loc, One_Fifty Int_2_Loc,
                      One_Fifty Int_3_Loc, Enumeration Enum_Loc,
                      const char *Str_1_Loc, const char *Str_2_Loc) {
  return Int_Glob == 5 && Bool_Glob == 1 && Ch_1_Glob == 'A' &&
         Ch_2_Glob == 'B' && Arr_1_Glob[8] == 7 &&
         Arr_2_Glob[8][7] == runs + 10 && Ptr_Glob->Discr == Ident_1 &&
         Ptr_Glob->variant.var_1.Enum_Comp == Ident_3 &&
         Ptr_Glob->variant.var_1.Int_Comp == 17 &&
         strcmp(Ptr_Glob->variant.var_1.Str_Comp,
                "DHRYSTONE PROGRAM, SOME STRING") == 0 &&
         Next_Ptr_Glob->Discr == Ident_1 &&
         Next_Ptr_Glob->variant.var_1.Enum_Comp == Ident_2 &&
         Next_Ptr_Glob->variant.var_1.Int_Comp == 18 &&
         strcmp(Next_Ptr_Glob->variant.var_1.Str_Comp,
                "DHRYSTONE PROGRAM, SOME STRING") == 0 &&
         Int_1_Loc == 5 && Int_2_Loc == 13 && Int_3_Loc == 7 &&
         Enum_Loc == Ident_2 &&
         strcmp(Str_1_Loc, "DHRYSTONE PROGRAM, 1'ST STRING") == 0 &&
         strcmp(Str_2_Loc, "DHRYSTONE PROGRAM, 2'ND STRING") == 0;
}

int main(void) {
  One_Fifty      Int_1_Loc;
  REG One_Fifty  Int_2_Loc;
  One_Fifty      Int_3_Loc;
  REG char       Ch_Index;
  Enumeration    Enum_Loc;
  Str_30         Str_1_Loc;
  Str_30         Str_2_Loc;
  REG int        Run_Index;
  REG int        Number_Of_Runs;
  unsigned       begin_cycle, end_cycle, begin_instret, end_instret;

  Next_Ptr_Glob = dhry_alloc_record();
  Ptr_Glob = dhry_alloc_record();

  Ptr_Glob->Ptr_Comp = Next_Ptr_Glob;
  Ptr_Glob->Discr = Ident_1;
  Ptr_Glob->variant.var_1.Enum_Comp = Ident_3;
  Ptr_Glob->variant.var_1.Int_Comp = 40;
  strcpy(Ptr_Glob->variant.var_1.Str_Comp, "DHRYSTONE PROGRAM, SOME STRING");
  strcpy(Str_1_Loc, "DHRYSTONE PROGRAM, 1'ST STRING");

  Arr_2_Glob[8][7] = 10;

  Number_Of_Runs = DHRY_RUNS;

  begin_instret = dhry_minstret();
  begin_cycle = dhry_mcycle();

  for (Run_Index = 1; Run_Index <= Number_Of_Runs; ++Run_Index) {
    Proc_5();
    Proc_4();
    Int_1_Loc = 2;
    Int_2_Loc = 3;
    strcpy(Str_2_Loc, "DHRYSTONE PROGRAM, 2'ND STRING");
    Enum_Loc = Ident_2;
    Bool_Glob = !Func_2(Str_1_Loc, Str_2_Loc);
    while (Int_1_Loc < Int_2_Loc) {
      Int_3_Loc = 5 * Int_1_Loc - Int_2_Loc;
      Proc_7(Int_1_Loc, Int_2_Loc, &Int_3_Loc);
      Int_1_Loc += 1;
    }
    Proc_8(Arr_1_Glob, Arr_2_Glob, Int_1_Loc, Int_3_Loc);
    Proc_1(Ptr_Glob);
    for (Ch_Index = 'A'; Ch_Index <= Ch_2_Glob; ++Ch_Index) {
      if (Enum_Loc == Func_1(Ch_Index, 'C')) {
        Proc_6(Ident_1, &Enum_Loc);
        strcpy(Str_2_Loc, "DHRYSTONE PROGRAM, 3'RD STRING");
        Int_2_Loc = Run_Index;
        Int_Glob = Run_Index;
      }
    }
    Int_2_Loc = Int_2_Loc * Int_1_Loc;
    Int_1_Loc = Int_2_Loc / Int_3_Loc;
    Int_2_Loc = 7 * (Int_2_Loc - Int_3_Loc) - Int_1_Loc;
    Proc_2(&Int_1_Loc);
  }

  end_cycle = dhry_mcycle();
  end_instret = dhry_minstret();

  dhry_report(Number_Of_Runs, end_cycle - begin_cycle,
              end_instret - begin_instret,
              self_check(Number_Of_Runs, Int_1_Loc, Int_2_Loc, Int_3_Loc,
                         Enum_Loc, Str_1_Loc, Str_2_Loc));
  return 0;
}
