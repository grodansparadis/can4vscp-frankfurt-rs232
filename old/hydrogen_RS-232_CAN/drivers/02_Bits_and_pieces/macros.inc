/*
   macros.inc - macros for use in assembler sources

   Contributors:
     Created by Marek Michalkiewicz <marekm@linux.org.pl>

   THIS SOFTWARE IS NOT COPYRIGHTED

   This source code is offered for use in the public domain.  You may
   use, modify or distribute it freely.

   This code is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY.  ALL WARRANTIES, EXPRESS OR IMPLIED ARE HEREBY
   DISCLAIMED.  This includes but is not limited to warranties of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <io-avr.h>

/* if not defined, assume old version with underscores */
#ifndef __USER_LABEL_PREFIX__
#define __USER_LABEL_PREFIX__ _
#endif

#ifndef __REGISTER_PREFIX__
#define __REGISTER_PREFIX__
#endif

/* the assembler line separator (just in case it ever changes) */
#define _L $

#define CONCAT1(a, b) CONCAT2(a, b)
#define CONCAT2(a, b) a ## b

#define _U(x) CONCAT1(__USER_LABEL_PREFIX__, x)

#define _R(x) CONCAT1(__REGISTER_PREFIX__, x)

/* these should help to fix the "can't have function named r1()" bug
   which may require adding '%' in front of register names.  */

#define r0 _R(r0)
#define r1 _R(r1)
#define r2 _R(r2)
#define r3 _R(r3)
#define r4 _R(r4)
#define r5 _R(r5)
#define r6 _R(r6)
#define r7 _R(r7)
#define r8 _R(r8)
#define r9 _R(r9)
#define r10 _R(r10)
#define r11 _R(r11)
#define r12 _R(r12)
#define r13 _R(r13)
#define r14 _R(r14)
#define r15 _R(r15)
#define r16 _R(r16)
#define r17 _R(r17)
#define r18 _R(r18)
#define r19 _R(r19)
#define r20 _R(r20)
#define r21 _R(r21)
#define r22 _R(r22)
#define r23 _R(r23)
#define r24 _R(r24)
#define r25 _R(r25)
#define r26 _R(r26)
#define r27 _R(r27)
#define r28 _R(r28)
#define r29 _R(r29)
#define r30 _R(r30)
#define r31 _R(r31)

#ifndef __tmp_reg__
#define __tmp_reg__ r0
#endif

#ifndef __zero_reg__
#define __zero_reg__ r1
#endif

#if FLASHEND > 0x2000  /* ATmega */
  #define XJMP jmp
  #define XCALL call
#else
  #define XJMP rjmp
  #define XCALL rcall
#endif

/* used only by fplib/strtod.S - libgcc internal function calls */
#define PROLOGUE_SAVES(offset) XJMP (__prologue_saves__ + 2 * (offset))
#define EPILOGUE_RESTORES(offset) XJMP (__epilogue_restores__ + 2 * (offset))

#if FLASHEND > 0x10000  /* ATmega103 */
  #define BIG_CODE 1
#else
  #define BIG_CODE 0
#endif

/*
   Smart version of movw (use register numbers as arguments):
    - uses "movw" if possible (supported by MCU, and both registers even)
    - handles overlapping register pairs correctly
    - no instruction generated if source and destination are the same
   (may expand to 0, 1 or 2 instructions).
 */

	.macro	X_movw dest src
		.if ((\src) - (\dest))  /* different registers */
			.if (((\src) | (\dest)) & 0x01)
				.if (((\src)-(\dest)) & 0x80) /* src < dest */
					mov	(\dest)+1, (\src)+1
					mov	(\dest), (\src)
				.else  /* src > dest */
					mov	(\dest), (\src)
					mov	(\dest)+1, (\src)+1
				.endif
			.else  /* both even -> overlap not possible */
#if __AVR_ENHANCED__
				movw	(\dest), (\src)
#else
				mov	(\dest), (\src)
				mov	(\dest)+1, (\src)+1
#endif
			.endif
		.endif
	.endm

#if __AVR_ENHANCED__
  #define LOAD_X(lo, hi) movw XL, lo
  #define LOAD_Z(lo, hi) movw ZL, lo
#else
  #define LOAD_X(lo, hi)		\
	mov	XL, lo			_L\
	mov	XH, hi

  #define LOAD_Z(lo, hi)		\
	mov	ZL, lo			_L\
	mov	ZH, hi
#endif

/* LOAD_X_CONST(p) loads constant P into pointer register X.  */
#define LOAD_X_CONST(p)		\
	ldi	XL, lo8(p)		_L\
	ldi	XH, hi8(p)

/*
   LPM_R0_ZPLUS_INIT is used before the loop to initialize RAMPZ
   for future devices with RAMPZ:Z auto-increment - [e]lpm r0, Z+.

   LPM_R0_ZPLUS_NEXT is used inside the loop to load a byte from
   the program memory at [RAMPZ:]Z to R0, and increment [RAMPZ:]Z.

   The argument in both macros is a register that contains the
   high byte (bits 23-16) of the address, bits 15-0 should be in
   the Z (r31:r30) register.  It can be any register except for:
   r0, r1 (__zero_reg__ - assumed to always contain 0), r30, r31.
 */

	.macro	LPM_R0_ZPLUS_INIT hhi
#if __AVR_ENHANCED__
  #if BIG_CODE
	out	RAMPZ, \hhi
  #endif
#endif
	.endm

	.macro	LPM_R0_ZPLUS_NEXT hhi
#if __AVR_ENHANCED__
  #if BIG_CODE
    /* ELPM with RAMPZ:Z post-increment, load RAMPZ only once */
	elpm	r0, Z+
  #else
    /* LPM with Z post-increment, max 64K, no RAMPZ (ATmega83/161/163/32) */
	lpm	r0, Z+
  #endif
#else
  #if BIG_CODE
    /* ELPM without post-increment, load RAMPZ each time (ATmega103) */
	out	RAMPZ, \hhi
	elpm
	adiw	ZL, 1
	adc	\hhi, __zero_reg__
  #else
    /* LPM without post-increment, max 64K, no RAMPZ (AT90S*) */
	lpm
	adiw	ZL, 1
  #endif
#endif
	.endm

	.macro  LPM_R0_ZP
#if __AVR_ENHANCED__
	lpm	r0, Z+
#else
	lpm
	adiw	ZL, 1
#endif
	.endm

