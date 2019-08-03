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

/* General type conversion for MATLAB generated C-code  */
#include "fir_stm.h"
/* 
 * Expected path to tmwtypes.h 
 * C:\Program Files\MATLAB\R2016b\extern\include\tmwtypes.h 
 */




uint_fast64_t FIR (uint16_t input,uint16_t *cc)
{
  //double input = nsine[arr_count];
  //unsigned int input = nsine[arr_count];
  uint_fast8_t i;
  uint_fast64_t summ = 0;
  static volatile uint_fast64_t delay[FIR_ORD]={0};
  for (i = 0; i <= (FIR_ORD - 2); i++)
    {
      delay[i] = delay[i + 1];
      summ += (delay[i] * cc[i]) / FirNorm;
    }
  delay[FIR_ORD - 1] = input;
  summ += (delay[FIR_ORD - 1] * cc[FIR_ORD - 1]) / FirNorm;
return summ;
}
uint_fast32_t FIR_NORM(uint16_t *cc)
{
 int i=0;
 uint_fast32_t summ;
 for(i = 0; i < FIR_ORD;i++)
    summ+=cc[i];
  return summ;
}
