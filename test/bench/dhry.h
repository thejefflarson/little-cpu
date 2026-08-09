/*
 * Dhrystone Benchmark, Version 2.1 (Language: C), by Reinhold P. Weicker.
 * Public-domain, and reproduced here as it is published, because a trimmed
 * Dhrystone produces a number nothing else can be compared against — which is
 * the only reason this benchmark is in the tree at all.
 *
 * Two deliberate departures from the published file, neither of which changes
 * what is measured:
 *
 *   - the declarations are ANSI prototypes rather than K&R. gcc 14 removed
 *     support for the old form outright, so keeping it would mean the source
 *     no longer builds rather than the source staying faithful;
 *   - the `TIMES`/`TIME`/`printf`/`malloc` scaffolding is gone. This core has
 *     no host clock, no allocator and no console. dhry_port.h supplies the
 *     three things dhry_1.c actually needs — a cycle counter, static storage
 *     for the two records, and a report buffer.
 *
 * `REG` is left undefined, which is the published default: the run is compiled
 * WITHOUT the `register` attribute, and the report says so.
 */

#ifndef DHRY_H
#define DHRY_H

#define Null 0
#define true 1
#define false 0

/* Undefined on purpose; see the header comment. */
#ifdef REG
#define REG register
#else
#define REG
#endif

typedef enum { Ident_1, Ident_2, Ident_3, Ident_4, Ident_5 } Enumeration;

typedef int  One_Thirty;
typedef int  One_Fifty;
typedef char Capital_Letter;
typedef int  Boolean;
typedef char Str_30[31];
typedef int  Arr_1_Dim[50];
typedef int  Arr_2_Dim[50][50];

typedef struct record {
  struct record *Ptr_Comp;
  Enumeration    Discr;
  union {
    struct {
      Enumeration Enum_Comp;
      int         Int_Comp;
      char        Str_Comp[31];
    } var_1;
    struct {
      Enumeration E_Comp_2;
      char        Str_2_Comp[31];
    } var_2;
    struct {
      char Ch_1_Comp;
      char Ch_2_Comp;
    } var_3;
  } variant;
} Rec_Type, *Rec_Pointer;

#define structassign(d, s) d = s

/* Globals, defined in dhry_1.c. dhry_2.c reads Int_Glob and Ch_1_Glob. */
extern Rec_Pointer Ptr_Glob, Next_Ptr_Glob;
extern int         Int_Glob;
extern Boolean     Bool_Glob;
extern char        Ch_1_Glob, Ch_2_Glob;
extern int         Arr_1_Glob[50];
extern int         Arr_2_Glob[50][50];

void        Proc_1(Rec_Pointer Ptr_Val_Par);
void        Proc_2(One_Fifty *Int_Par_Ref);
void        Proc_3(Rec_Pointer *Ptr_Ref_Par);
void        Proc_4(void);
void        Proc_5(void);
void        Proc_6(Enumeration Enum_Val_Par, Enumeration *Enum_Ref_Par);
void        Proc_7(One_Fifty Int_1_Par_Val, One_Fifty Int_2_Par_Val,
                   One_Fifty *Int_Par_Ref);
void        Proc_8(Arr_1_Dim Arr_1_Par_Ref, Arr_2_Dim Arr_2_Par_Ref,
                   int Int_1_Par_Val, int Int_2_Par_Val);
Enumeration Func_1(Capital_Letter Ch_1_Par_Val, Capital_Letter Ch_2_Par_Val);
Boolean     Func_2(Str_30 Str_1_Par_Ref, Str_30 Str_2_Par_Ref);
Boolean     Func_3(Enumeration Enum_Par_Val);

#endif /* DHRY_H */
