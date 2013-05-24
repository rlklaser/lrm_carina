/*********************************************************
  gp_rede24.c
  --------------------------------------------------------
  generated at Mon Nov 19 15:53:58 2007
  by snns2c ( Bernward Kett 1995 )
*********************************************************/

#include <math.h>

#define Act_Logistic(sum, bias)  ( (sum+bias<10000.0) ? ( 1.0/(1.0 + exp(-sum-bias) ) ) : 0.0 )
#define Act_Identity(sum, bias)     ( sum )
#define Act_IdentityPlusBias(sum, bias) (sum + bias)
#ifndef NULL
#define NULL (void *)0
#endif

typedef struct UT {
          float act;         /* Activation       */
          float Bias;        /* Bias of the Unit */
          int   NoOfSources; /* Number of predecessor units */
   struct UT   **sources; /* predecessor units */
          float *weights; /* weights from predecessor units */
        } UnitType, *pUnit;

  /* Forward Declaration for all unit types */
  static UnitType Units[34];

  /* Sources definition section */
  static pUnit Sources[] =  {
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7,
Units + 8, Units + 9, Units + 10, Units + 11, Units + 12, Units + 13, Units + 14, Units + 15, Units + 16, Units + 17,
Units + 18, Units + 19, Units + 20, Units + 21, Units + 22, Units + 23, Units + 24, Units + 25, Units + 26, Units + 27,
Units + 28, Units + 29, Units + 30, Units + 31,
Units + 8, Units + 9, Units + 10, Units + 11, Units + 12, Units + 13, Units + 14, Units + 15, Units + 16, Units + 17,
Units + 18, Units + 19, Units + 20, Units + 21, Units + 22, Units + 23, Units + 24, Units + 25, Units + 26, Units + 27,
Units + 28, Units + 29, Units + 30, Units + 31,

  };

  /* Weigths definition section */
  static float Weights[] =  {
-0.066650, 0.070400, -0.008940, -0.052840, 0.130070, -0.033040, -0.012890,
0.011740, -0.004120, 0.087800, -0.087390, 0.766110, -1.220830, -0.921070,
-0.002700, 0.000500, -0.093350, -0.016480, -0.014700, -0.007220, -0.096480,
0.001230, -0.001270, -0.091290, -0.017780, -0.015380, -0.010050, -0.095680,
-0.036490, 0.051020, -0.148590, 0.066060, -0.026840, -0.012830, -0.171380,
0.003350, -0.000770, -0.098130, -0.023790, -0.014090, -0.011020, -0.089940,
-0.017940, 0.018840, -0.590610, -0.374020, 0.187320, -0.115770, 0.015800,
-0.014140, 0.008240, -0.112630, 0.005430, -0.004200, 0.001700, -0.102580,
0.002570, -0.001400, -0.092280, -0.020310, -0.013700, -0.013120, -0.092190,
1.071130, -0.902750, -1.461690, -0.084430, 0.187680, -0.323370, -0.409090,
0.004600, -0.008300, -0.130570, -0.005830, -0.038670, 0.075610, -0.201100,
-0.002860, -0.000510, -0.102660, -0.011020, -0.014660, -0.007190, -0.095990,
0.020910, -0.005910, 0.010000, -0.045670, 0.061040, 0.368410, -3.643920,
-0.006980, 0.010820, -0.080250, -0.016360, -0.014510, -0.006900, -0.095960,
0.051460, -0.049110, -0.115960, -0.000250, 0.100760, 0.040260, -0.038700,
-0.000840, 0.002130, -0.077100, -0.018630, -0.007410, -0.019930, -0.095620,
-0.004740, 0.003310, -0.091350, -0.016580, -0.014630, -0.007270, -0.096590,
1.072340, -0.944840, -0.036750, 0.407240, -0.137100, -0.140830, 1.479620,
0.122190, -0.118020, -0.398930, -0.068950, 0.219620, 0.207160, 0.220460,
0.015360, -0.009170, -0.090160, -0.023590, -0.014750, -0.021290, -0.076210,
0.008810, -0.006480, -3.007200, 0.068970, -0.163900, -0.022040, 0.439110,
0.002770, -0.004980, -0.515030, -0.029940, 0.008930, -0.613250, -0.531050,
-0.004760, 0.002340, -0.165510, -0.009000, -0.017900, -0.008370, -0.174020,
-0.001230, 0.003480, 0.165460, -0.057000, 0.021410, -0.749490, -0.011590,
0.595180, -0.458720, -0.008030, -0.185670, 0.168460, 0.906940, 0.714080, 0.283840, -0.081240, 0.921260,
0.302290, 0.440310, -0.853660, -0.037670, -0.736120, -1.603120, -0.050940, 0.425420, 0.367840, 0.516590,
1.051170, -0.108100, -0.323160, -0.884440,
0.073650, -0.133140, 0.245980, 0.615600, 0.244840, 1.600020, 0.017120, 2.580620, 0.640290, 0.130340,
2.013630, 0.254820, -0.024850, 0.245390, 0.229280, 0.373620, 0.245750, 0.163880, -0.281570, 1.968520,
-0.000490, 1.222100, -4.127430, 0.290880,

  };

  /* unit definition section (see also UnitType) */
  static UnitType Units[34] =
  {
    { 0.0, 0.0, 0, NULL , NULL },
    { /* unit 1 (Input) */
      0.0, 0.999990, 0,
       &Sources[0] ,
       &Weights[0] ,
      },
    { /* unit 2 (Input) */
      0.0, 0.999980, 0,
       &Sources[0] ,
       &Weights[0] ,
      },
    { /* unit 3 (Input) */
      0.0, 0.999990, 0,
       &Sources[0] ,
       &Weights[0] ,
      },
    { /* unit 4 (Input) */
      0.0, 0.999990, 0,
       &Sources[0] ,
       &Weights[0] ,
      },
    { /* unit 5 (Input) */
      0.0, 0.999990, 0,
       &Sources[0] ,
       &Weights[0] ,
      },
    { /* unit 6 (Input) */
      0.0, 0.999980, 0,
       &Sources[0] ,
       &Weights[0] ,
      },
    { /* unit 7 (Input) */
      0.0, 0.999990, 0,
       &Sources[0] ,
       &Weights[0] ,
      },
    { /* unit 8 (Hidden) */
      0.0, -3.112470, 7,
       &Sources[0] ,
       &Weights[0] ,
      },
    { /* unit 9 (Hidden) */
      0.0, 66.679688, 7,
       &Sources[7] ,
       &Weights[7] ,
      },
    { /* unit 10 (Hidden) */
      0.0, 22.014570, 7,
       &Sources[14] ,
       &Weights[14] ,
      },
    { /* unit 11 (Hidden) */
      0.0, 21.754520, 7,
       &Sources[21] ,
       &Weights[21] ,
      },
    { /* unit 12 (Hidden) */
      0.0, 36.191120, 7,
       &Sources[28] ,
       &Weights[28] ,
      },
    { /* unit 13 (Hidden) */
      0.0, 21.599581, 7,
       &Sources[35] ,
       &Weights[35] ,
      },
    { /* unit 14 (Hidden) */
      0.0, 39.723789, 7,
       &Sources[42] ,
       &Weights[42] ,
      },
    { /* unit 15 (Hidden) */
      0.0, 22.243679, 7,
       &Sources[49] ,
       &Weights[49] ,
      },
    { /* unit 16 (Hidden) */
      0.0, 21.603350, 7,
       &Sources[56] ,
       &Weights[56] ,
      },
    { /* unit 17 (Hidden) */
      0.0, -11.052280, 7,
       &Sources[63] ,
       &Weights[63] ,
      },
    { /* unit 18 (Hidden) */
      0.0, 27.457701, 7,
       &Sources[70] ,
       &Weights[70] ,
      },
    { /* unit 19 (Hidden) */
      0.0, 22.040670, 7,
       &Sources[77] ,
       &Weights[77] ,
      },
    { /* unit 20 (Hidden) */
      0.0, 69.250313, 7,
       &Sources[84] ,
       &Weights[84] ,
      },
    { /* unit 21 (Hidden) */
      0.0, 22.022230, 7,
       &Sources[91] ,
       &Weights[91] ,
      },
    { /* unit 22 (Hidden) */
      0.0, 0.811120, 7,
       &Sources[98] ,
       &Weights[98] ,
      },
    { /* unit 23 (Hidden) */
      0.0, 21.570511, 7,
       &Sources[105] ,
       &Weights[105] ,
      },
    { /* unit 24 (Hidden) */
      0.0, 22.052700, 7,
       &Sources[112] ,
       &Weights[112] ,
      },
    { /* unit 25 (Hidden) */
      0.0, 1.970290, 7,
       &Sources[119] ,
       &Weights[119] ,
      },
    { /* unit 26 (Hidden) */
      0.0, 2.206010, 7,
       &Sources[126] ,
       &Weights[126] ,
      },
    { /* unit 27 (Hidden) */
      0.0, 21.788050, 7,
       &Sources[133] ,
       &Weights[133] ,
      },
    { /* unit 28 (Hidden) */
      0.0, 58.040771, 7,
       &Sources[140] ,
       &Weights[140] ,
      },
    { /* unit 29 (Hidden) */
      0.0, 144.667084, 7,
       &Sources[147] ,
       &Weights[147] ,
      },
    { /* unit 30 (Hidden) */
      0.0, 33.995781, 7,
       &Sources[154] ,
       &Weights[154] ,
      },
    { /* unit 31 (Hidden) */
      0.0, 16.537630, 7,
       &Sources[161] ,
       &Weights[161] ,
      },
    { /* unit 32 (noName) */
      0.0, -0.675300, 24,
       &Sources[168] ,
       &Weights[168] ,
      },
    { /* unit 33 (noName) */
      0.0, -9.731100, 24,
       &Sources[192] ,
       &Weights[192] ,
      }

  };

int gp_rede24(float *in, float *out, int init)
{
  int member, source;
  float sum;
  enum{OK, Error, Not_Valid};
  pUnit unit;


  /* layer definition section (names & member units) */

  static pUnit Input[7] = {Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7}; /* members */

  static pUnit Hidden1[24] = {Units + 8, Units + 9, Units + 10, Units + 11, Units + 12, Units + 13, Units + 14, Units + 15, Units + 16, Units + 17, Units + 18, Units + 19, Units + 20, Units + 21, Units + 22, Units + 23, Units + 24, Units + 25, Units + 26, Units + 27, Units + 28, Units + 29, Units + 30, Units + 31}; /* members */

  static pUnit Output1[2] = {Units + 32, Units + 33}; /* members */

  static int Output[2] = {32, 33};

  for(member = 0; member < 7; member++) {
    Input[member]->act = in[member];
  }

  for (member = 0; member < 24; member++) {
    unit = Hidden1[member];
    sum = 0.0;
    for (source = 0; source < unit->NoOfSources; source++) {
      sum += unit->sources[source]->act
             * unit->weights[source];
    }
    unit->act = Act_Logistic(sum, unit->Bias);
  };

  for (member = 0; member < 2; member++) {
    unit = Output1[member];
    sum = 0.0;
    for (source = 0; source < unit->NoOfSources; source++) {
      sum += unit->sources[source]->act
             * unit->weights[source];
    }
    unit->act = Act_IdentityPlusBias(sum, unit->Bias);
  };

  for(member = 0; member < 2; member++) {
    out[member] = Units[Output[member]].act;
  }

  return(OK);
}

