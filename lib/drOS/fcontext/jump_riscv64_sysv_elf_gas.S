/*
   Distributed under the Boost Software License, Version 1.0.
      (See accompanying file LICENSE_1_0.txt or copy at
          http://www.boost.org/LICENSE_1_0.txt)
*/
/*******************************************************
 *                                                     *
 *  -------------------------------------------------  *
 *  |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  *
 *  -------------------------------------------------  *
 *  | 0x0 | 0x4 | 0x8 | 0xc | 0x10| 0x14| 0x18| 0x1c|  *
 *  -------------------------------------------------  *
 *  |    fs0    |    fs1    |    fs2    |    fs3    |  *
 *  -------------------------------------------------  *
 *  -------------------------------------------------  *
 *  |  8  |  9  |  10 |  11 |  12 |  13 |  14 |  15 |  *
 *  -------------------------------------------------  *
 *  | 0x20| 0x24| 0x28| 0x2c| 0x30| 0x34| 0x38| 0x3c|  *
 *  -------------------------------------------------  *
 *  |    fs4    |    fs5    |    fs6    |    fs7    |  *
 *  -------------------------------------------------  *
 *  -------------------------------------------------  *
 *  |  16 |  17 |  18 |  19 |  20 |  21 |  22 |  23 |  *
 *  -------------------------------------------------  *
 *  | 0x40| 0x44| 0x48| 0x4c| 0x50| 0x54| 0x58| 0x5c|  *
 *  -------------------------------------------------  *
 *  |    fs8    |    fs9    |    fs10   |    fs11   |  *
 *  -------------------------------------------------  *
 *  -------------------------------------------------  *
 *  |  24 |  25 |  26 |  27 |  28 |  29 |  30 |  31 |  *
 *  -------------------------------------------------  *
 *  | 0x60| 0x64| 0x68| 0x6c| 0x70| 0x74| 0x78| 0x7c|  *
 *  -------------------------------------------------  *
 *  |    s0     |    s1     |    s2     |    s3     |  *
 *  -------------------------------------------------  *
 *  -------------------------------------------------  *
 *  |  32 |  33 |  34 |  35 |  36 |  37 |  38 |  39 |  *
 *  -------------------------------------------------  *
 *  | 0x80| 0x84| 0x88| 0x8c| 0x90| 0x94| 0x98| 0x9c|  *
 *  -------------------------------------------------  *
 *  |    s4     |    s5     |    s6     |     s7    |  *
 *  -------------------------------------------------  *
 *  -------------------------------------------------  *
 *  |  40 |  41 |  42 | 43  |  44 | 45  |  46 | 47  |  *
 *  -------------------------------------------------  *
 *  | 0xa0| 0xa4| 0xa8| 0xac| 0xb0| 0xb4| 0xb8| 0xbc|  *
 *  -------------------------------------------------  *
 *  |     s8    |     s9    |    s10    |    s11    |  *
 *  -------------------------------------------------  *
 *  -------------------------------------------------  *
 *  |  48 |  49 |  50 | 51  |     |     |     |     |  *
 *  -------------------------------------------------  *
 *  | 0xc0| 0xc4| 0xc8| 0xcc|     |     |     |     |  *
 *  -------------------------------------------------  *
 *  |     ra    |     pc    |           |           |  *
 *  -------------------------------------------------  *
 *                                                     *
 *******************************************************/

#ifdef __riscv

#if __riscv_xlen == 32
#define fsd fsw
#define fld flw
#define sd sw
#define ld lw
#endif

.file "jump_riscv64_sysv_elf_gas.S"
.text
.align  1
.global jump_fcontext
.hidden jump_fcontext
.type   jump_fcontext, %function
jump_fcontext:
    # prepare stack for GP + FPU
    addi  sp, sp, -26 * __SIZEOF_SIZE_T__

#ifndef __riscv_float_abi_soft
    # save fs0 - fs11
    fsd  fs0, 0*__SIZEOF_SIZE_T__ (sp)
    fsd  fs1, 1*__SIZEOF_SIZE_T__ (sp)
    fsd  fs2, 2*__SIZEOF_SIZE_T__ (sp)
    fsd  fs3, 3*__SIZEOF_SIZE_T__ (sp)
    fsd  fs4, 4*__SIZEOF_SIZE_T__ (sp)
    fsd  fs5, 5*__SIZEOF_SIZE_T__ (sp)
    fsd  fs6, 6*__SIZEOF_SIZE_T__ (sp)
    fsd  fs7, 7*__SIZEOF_SIZE_T__ (sp)
    fsd  fs8, 8*__SIZEOF_SIZE_T__ (sp)
    fsd  fs9, 9*__SIZEOF_SIZE_T__ (sp)
    fsd  fs10, 10*__SIZEOF_SIZE_T__ (sp)
    fsd  fs11, 11*__SIZEOF_SIZE_T__ (sp)
#endif

    # save s0-s11, ra
    sd  s0, 12 * __SIZEOF_SIZE_T__ (sp)
    sd  s1, 13 * __SIZEOF_SIZE_T__ (sp)
    sd  s2, 14 * __SIZEOF_SIZE_T__ (sp)
    sd  s3, 15 * __SIZEOF_SIZE_T__ (sp)
    sd  s4, 16 * __SIZEOF_SIZE_T__ (sp)
    sd  s5, 17 * __SIZEOF_SIZE_T__ (sp)
    sd  s6, 18 * __SIZEOF_SIZE_T__ (sp)
    sd  s7, 19 * __SIZEOF_SIZE_T__ (sp)
    sd  s8, 20 * __SIZEOF_SIZE_T__ (sp)
    sd  s9, 21 * __SIZEOF_SIZE_T__ (sp)
    sd  s10, 22 * __SIZEOF_SIZE_T__ (sp)
    sd  s11, 23 * __SIZEOF_SIZE_T__ (sp)
    sd  ra, 24 * __SIZEOF_SIZE_T__ (sp)

    # save RA as PC
    sd  ra, 25 * __SIZEOF_SIZE_T__ (sp)

    # store SP (pointing to context-data) in A2
    mv  a2, sp

    # restore SP (pointing to context-data) from A0
    mv  sp, a0

    # load fs0 - fs11
#ifndef __riscv_float_abi_soft
    fld  fs0, 0*__SIZEOF_SIZE_T__ (sp)
    fld  fs1, 1*__SIZEOF_SIZE_T__ (sp)
    fld  fs2, 2*__SIZEOF_SIZE_T__ (sp)
    fld  fs3, 3*__SIZEOF_SIZE_T__ (sp)
    fld  fs4, 4*__SIZEOF_SIZE_T__ (sp)
    fld  fs5, 5*__SIZEOF_SIZE_T__ (sp)
    fld  fs6, 6*__SIZEOF_SIZE_T__ (sp)
    fld  fs7, 7*__SIZEOF_SIZE_T__ (sp)
    fld  fs8, 8*__SIZEOF_SIZE_T__ (sp)
    fld  fs9, 9*__SIZEOF_SIZE_T__ (sp)
    fld  fs10, 10*__SIZEOF_SIZE_T__ (sp)
    fld  fs11, 11*__SIZEOF_SIZE_T__ (sp)
#endif

    # load s0-s11,ra
    ld  s0, 12 * __SIZEOF_SIZE_T__ (sp)
    ld  s1, 13 * __SIZEOF_SIZE_T__ (sp)
    ld  s2, 14 * __SIZEOF_SIZE_T__ (sp)
    ld  s3, 15 * __SIZEOF_SIZE_T__ (sp)
    ld  s4, 16 * __SIZEOF_SIZE_T__ (sp)
    ld  s5, 17 * __SIZEOF_SIZE_T__ (sp)
    ld  s6, 18 * __SIZEOF_SIZE_T__ (sp)
    ld  s7, 19 * __SIZEOF_SIZE_T__ (sp)
    ld  s8, 20 * __SIZEOF_SIZE_T__ (sp)
    ld  s9, 21 * __SIZEOF_SIZE_T__ (sp)
    ld  s10, 22 * __SIZEOF_SIZE_T__ (sp)
    ld  s11, 23 * __SIZEOF_SIZE_T__ (sp)
    ld  ra, 24 * __SIZEOF_SIZE_T__ (sp)

    # return transfer_t from jump
    # pass transfer_t as first arg in context function
    # a0 == FCTX, a1 == DATA
    mv a0, a2

    # load pc
    ld  a2, 25 * __SIZEOF_SIZE_T__(sp)

    # restore stack from GP + FPU
    addi  sp, sp, 26 * __SIZEOF_SIZE_T__

    jr a2
.size   jump_fcontext,.-jump_fcontext
# Mark that we don't need executable stack.
.section .note.GNU-stack,"",%progbits

#endif
