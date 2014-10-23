/*
** ADSP-BF609 user heap source file generated on Oct 20, 2014 at 09:14:55.
*/
/*
** Copyright (C) 2000-2013 Analog Devices Inc., All Rights Reserved.
**
** This file is generated automatically based upon the options selected
** in the System Configuration utility. Changes to the Heap configuration
** should be made by modifying the appropriate options rather than editing
** this file. To access the System Configuration utility, double-click the
** system.svc file from a navigation view.
**
** Custom additions can be inserted within the user-modifiable sections,
** these are bounded by comments that start with "$VDSG". Only changes
** placed within these sections are preserved when this file is re-generated.
**
** Product      : CrossCore Embedded Studio 1.0.2.0
** Tool Version : 6.0.2.32
*/

#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_1_1)
#pragma diag(suppress:misra_rule_2_2)
#pragma diag(suppress:misra_rule_6_3)
#pragma diag(suppress:misra_rule_8_10)
#pragma diag(suppress:misra_rule_10_1_a)
#pragma diag(suppress:misra_rule_11_3)
#pragma diag(suppress:misra_rule_12_7)
#else
#pragma diag(suppress:1124)
#endif /* _MISRA_RULES */


extern "asm" int ldf_heap_space;
extern "asm" int ldf_heap_length;
extern "asm" int MyHeap2_space;
extern "asm" int MyHeap2_length;
/*$VDSG<insert-new-sections>*/
extern "asm" int MyHeap_space;
extern "asm" int MyHeap_length;
/*$VDSG<insert-new-sections>*/
struct heap_table_t
{
  void          *base;
  unsigned long  length;
  long int       userid;
};

#pragma file_attr("libData=HeapTable")
#pragma section("constdata")
struct heap_table_t heap_table[4] =
{

  { &ldf_heap_space, (unsigned long) &ldf_heap_length, 0 },
  { &MyHeap2_space, (unsigned long) &MyHeap2_length, 2 },
  /*$VDSG<insert-new-sections>*/
  { &MyHeap_space, (unsigned long) &MyHeap_length, 1 },
  /*$VDSG<insert-new-sections>*/
  { 0, 0, 0 }
};

#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */
