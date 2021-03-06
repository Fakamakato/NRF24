/*
 * Filter Coefficients (C Source) generated by the Filter Design and Analysis Tool
 * Generated by MATLAB(R) 9.1 and the Signal Processing Toolbox 7.3.
 * Generated on: 24-May-2018 10:25:02
 */

/*
 * Discrete-Time FIR Filter (real)
 * -------------------------------
 * Filter Structure  : Direct-Form FIR
 * Filter Length     : 16
 * Stable            : Yes
 * Linear Phase      : Yes (Type 2)
 */
#include "stdint.h"
#define FIR_ORD  32
/* General type conversion for MATLAB generated C-code  */
/* 
 * Expected path to tmwtypes.h 
 * C:\Program Files\MATLAB\R2016b\extern\include\tmwtypes.h 
 */

static uint16_t fir_coeff[]={      341,    259,    347,    447,    558,    677,    801,    927,   1052,
                                   1172,   1284,   1383,   1467,   1532,   1577,   1600,   1600,   1577,
                                   1532,   1467,   1383,   1284,   1172,   1052,    927,    801,    677,
                                    558,    447,    347,    259,    341 };
uint_fast32_t FirNorm;
uint32_t delay[FIR_ORD];

uint_fast64_t FIR (uint16_t,uint16_t *cc);
uint_fast32_t FIR_NORM(uint16_t *cc);

