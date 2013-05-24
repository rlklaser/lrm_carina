///*********************************************************
//  rna.c
//  --------------------------------------------------------
//  generated at Thu Jul 19 20:06:26 2007
//  by snns2c ( Bernward Kett 1995 ) 
//*********************************************************/
//
//#include <math.h>
//
//#define Act_Logistic(sum, bias)  ( (sum+bias<10000.0) ? ( 1.0/(1.0 + exp(-sum-bias) ) ) : 0.0 )
//#define Act_Identity(sum, bias)     ( sum )
//#define Act_IdentityPlusBias(sum, bias) (sum + bias)
//#ifndef NULL
//#define NULL (void *)0
//#endif
//
//typedef struct UT {
//          float act;         /* Activation       */
//          float Bias;        /* Bias of the Unit */
//          int   NoOfSources; /* Number of predecessor units */
//   struct UT   **sources; /* predecessor units */
//          float *weights; /* weights from predecessor units */
//        } UnitType, *pUnit;
//
//  /* Forward Declaration for all unit types */
//  static UnitType Units[36];
//  /* Sources definition section */
//  static pUnit Sources[] =  {
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9, 
//Units + 10, Units + 11, Units + 12, Units + 13, Units + 14, Units + 15, Units + 16, Units + 17, Units + 18, Units + 19, 
//Units + 20, Units + 21, Units + 22, Units + 23, Units + 24, Units + 25, Units + 26, Units + 27, Units + 28, Units + 29, 
//Units + 30, Units + 31, Units + 32, Units + 33, 
//Units + 10, Units + 11, Units + 12, Units + 13, Units + 14, Units + 15, Units + 16, Units + 17, Units + 18, Units + 19, 
//Units + 20, Units + 21, Units + 22, Units + 23, Units + 24, Units + 25, Units + 26, Units + 27, Units + 28, Units + 29, 
//Units + 30, Units + 31, Units + 32, Units + 33, 
//
//  };
//
//  /* Weigths definition section */
//  static float Weights[] =  {
//-0.122010, 0.037450, -0.031310, -0.041450, 0.229910, 0.199240, -0.159690, 0.409580, 0.411890, 
//-0.357470, 0.045330, 1.081080, -0.033490, 0.071870, -0.010730, -0.034980, 0.607470, 0.120660, 
//-0.159750, 0.031840, -0.090090, 0.222440, -0.004050, 0.002390, -0.034200, 1.362120, 0.034980, 
//-0.161980, 0.039320, -0.115950, 0.279070, -0.002410, -0.007100, -0.034380, 1.381280, 0.187870, 
//-0.331610, 0.071850, 0.085260, -0.079650, 0.060430, -0.029080, -0.045850, 0.817870, 0.169890, 
//-0.021280, -0.224700, 0.012410, -0.012780, 0.458320, 0.458210, 0.084740, -0.103950, 0.633740, 
//-0.195460, 0.037560, -0.136380, 0.092400, 0.035010, 0.040000, 0.002670, 1.717530, 0.454680, 
//-0.018880, -0.218840, 0.061620, 0.030710, 0.501810, 0.501690, 0.174040, -0.051440, 0.634820, 
//-0.121540, 0.121090, -0.020580, -0.051830, 0.094720, -0.002770, -0.493990, 1.034550, -0.595310, 
//-0.156450, 0.004640, -0.141170, -0.020010, 0.123560, 0.018830, 0.032430, 1.139800, 0.057370, 
//-0.178560, 0.037350, 0.012490, 0.652170, -0.004020, -0.021640, -0.082090, 1.370250, 0.140120, 
//-0.167750, 0.061030, 0.002130, 0.492910, -0.003170, -0.011150, -0.038970, 1.162080, -0.026220, 
//-0.025380, -0.094030, 0.239960, -0.042030, -0.027550, -0.008770, 0.011530, 0.152230, 0.459050, 
//-0.081420, 0.081170, -0.403550, -0.041480, 0.366100, 0.467890, 0.034690, -1.207660, 0.717000, 
//-0.259130, 0.066400, -0.007360, -0.123680, 0.049310, -0.029260, -0.045510, 0.925410, 0.451260, 
//0.344810, -0.346540, -0.051840, -0.060020, 0.452970, 0.455170, 0.007880, -1.503290, 0.390790, 
//-0.119340, 0.078240, -0.060750, -0.278620, 0.292300, 0.289520, -0.007930, -0.276470, 0.240980, 
//-0.154370, -0.007550, -0.108250, 0.376360, 0.267810, 0.067860, -0.010060, 1.043520, 0.122640, 
//-0.163740, 0.002590, 0.126830, 0.279300, 0.350300, 0.265790, -0.002350, 1.075420, -0.802070, 
//-0.171730, -0.113090, 0.375960, -0.004400, 0.072260, 0.072230, 0.046620, 0.823890, 0.399360, 
//-0.009870, -0.068370, -0.339690, -0.040450, -0.015810, -0.015350, 0.368840, 0.379870, 0.429120, 
//-0.059080, -0.145380, 0.216270, -0.024370, 0.001020, -0.001530, -0.032460, 0.915450, 0.179840, 
//-0.161410, 0.105030, -0.281780, 0.219330, -0.015760, -0.003150, -0.024460, 1.359030, -0.371990, 
//-0.156290, 0.001720, -0.029090, 0.007830, 0.157600, 0.056840, 0.004210, 1.057050, -0.019730, 
//-0.561620, -0.868980, 0.101430, -0.014340, -0.249420, -0.320750, 0.201150, -0.320340, -0.161430, -0.247430, 
//-0.023310, -0.024940, 0.450680, 0.687460, 0.557150, -0.228190, 0.786220, -0.229210, -0.259800, 0.545820, 
//0.856090, -0.135980, 0.043210, -0.236310, 
//-1.483070, 0.046800, 0.191320, 0.131890, -0.092870, -0.313560, 0.337110, -0.318940, -2.517160, -0.197110, 
//0.133750, 0.132450, 0.677730, -0.132500, -0.196970, -2.593550, 0.425200, 0.139660, 0.208610, 0.518880, 
//0.589040, -0.873710, 0.132290, 0.052610, 
//
//  };
//
//  /* unit definition section (see also UnitType) */
//  static UnitType Units[36] = 
//  {
//    { 0.0, 0.0, 0, NULL , NULL },
//    { /* unit 1 (Input) */
//      0.0, 0.999990, 0,
//       &Sources[0] , 
//       &Weights[0] , 
//      },
//    { /* unit 2 (Input) */
//      0.0, 1.000000, 0,
//       &Sources[0] , 
//       &Weights[0] , 
//      },
//    { /* unit 3 (Input) */
//      0.0, 0.999980, 0,
//       &Sources[0] , 
//       &Weights[0] , 
//      },
//    { /* unit 4 (Input) */
//      0.0, 0.999970, 0,
//       &Sources[0] , 
//       &Weights[0] , 
//      },
//    { /* unit 5 (Input) */
//      0.0, 0.999980, 0,
//       &Sources[0] , 
//       &Weights[0] , 
//      },
//    { /* unit 6 (Input) */
//      0.0, 1.000000, 0,
//       &Sources[0] , 
//       &Weights[0] , 
//      },
//    { /* unit 7 (Input) */
//      0.0, 1.000000, 0,
//       &Sources[0] , 
//       &Weights[0] , 
//      },
//    { /* unit 8 (Input) */
//      0.0, 0.999980, 0,
//       &Sources[0] , 
//       &Weights[0] , 
//      },
//    { /* unit 9 (Input) */
//      0.0, 1.000000, 0,
//       &Sources[0] , 
//       &Weights[0] , 
//      },
//    { /* unit 10 (Hidden) */
//      0.0, 0.752620, 9,
//       &Sources[0] , 
//       &Weights[0] , 
//      },
//    { /* unit 11 (Hidden) */
//      0.0, 0.350050, 9,
//       &Sources[9] , 
//       &Weights[9] , 
//      },
//    { /* unit 12 (Hidden) */
//      0.0, 0.069400, 9,
//       &Sources[18] , 
//       &Weights[18] , 
//      },
//    { /* unit 13 (Hidden) */
//      0.0, 0.071930, 9,
//       &Sources[27] , 
//       &Weights[27] , 
//      },
//    { /* unit 14 (Hidden) */
//      0.0, 0.339840, 9,
//       &Sources[36] , 
//       &Weights[36] , 
//      },
//    { /* unit 15 (Hidden) */
//      0.0, 1.349000, 9,
//       &Sources[45] , 
//       &Weights[45] , 
//      },
//    { /* unit 16 (Hidden) */
//      0.0, 0.106410, 9,
//       &Sources[54] , 
//       &Weights[54] , 
//      },
//    { /* unit 17 (Hidden) */
//      0.0, 1.480670, 9,
//       &Sources[63] , 
//       &Weights[63] , 
//      },
//    { /* unit 18 (Hidden) */
//      0.0, 0.375370, 9,
//       &Sources[72] , 
//       &Weights[72] , 
//      },
//    { /* unit 19 (Hidden) */
//      0.0, 0.413180, 9,
//       &Sources[81] , 
//       &Weights[81] , 
//      },
//    { /* unit 20 (Hidden) */
//      0.0, 0.025740, 9,
//       &Sources[90] , 
//       &Weights[90] , 
//      },
//    { /* unit 21 (Hidden) */
//      0.0, 0.067210, 9,
//       &Sources[99] , 
//       &Weights[99] , 
//      },
//    { /* unit 22 (Hidden) */
//      0.0, 0.001700, 9,
//       &Sources[108] , 
//       &Weights[108] , 
//      },
//    { /* unit 23 (Hidden) */
//      0.0, 1.125590, 9,
//       &Sources[117] , 
//       &Weights[117] , 
//      },
//    { /* unit 24 (Hidden) */
//      0.0, 0.330920, 9,
//       &Sources[126] , 
//       &Weights[126] , 
//      },
//    { /* unit 25 (Hidden) */
//      0.0, 1.431900, 9,
//       &Sources[135] , 
//       &Weights[135] , 
//      },
//    { /* unit 26 (Hidden) */
//      0.0, 0.950960, 9,
//       &Sources[144] , 
//       &Weights[144] , 
//      },
//    { /* unit 27 (Hidden) */
//      0.0, 0.587070, 9,
//       &Sources[153] , 
//       &Weights[153] , 
//      },
//    { /* unit 28 (Hidden) */
//      0.0, 0.628830, 9,
//       &Sources[162] , 
//       &Weights[162] , 
//      },
//    { /* unit 29 (Hidden) */
//      0.0, 0.119410, 9,
//       &Sources[171] , 
//       &Weights[171] , 
//      },
//    { /* unit 30 (Hidden) */
//      0.0, 0.001560, 9,
//       &Sources[180] , 
//       &Weights[180] , 
//      },
//    { /* unit 31 (Hidden) */
//      0.0, 0.048190, 9,
//       &Sources[189] , 
//       &Weights[189] , 
//      },
//    { /* unit 32 (Hidden) */
//      0.0, 0.070900, 9,
//       &Sources[198] , 
//       &Weights[198] , 
//      },
//    { /* unit 33 (Hidden) */
//      0.0, 0.442290, 9,
//       &Sources[207] , 
//       &Weights[207] , 
//      },
//    { /* unit 34 (noName) */
//      0.0, -0.292890, 24,
//       &Sources[216] , 
//       &Weights[216] , 
//      },
//    { /* unit 35 (noName) */
//      0.0, 3.190020, 24,
//       &Sources[240] , 
//       &Weights[240] , 
//      }
//
//  };
//
//
//
//int rna(float *in, float *out, int init)
//{
//  int member, source;
//  float sum;
//  enum{OK, Error, Not_Valid};
//  pUnit unit;
//
//
//  /* layer definition section (names & member units) */
//
//  static pUnit Input[9] = {Units + 1, Units + 2, Units + 3, Units + 4, Units + 5, Units + 6, Units + 7, Units + 8, Units + 9}; /* members */
//
//  static pUnit Hidden1[24] = {Units + 10, Units + 11, Units + 12, Units + 13, Units + 14, Units + 15, Units + 16, Units + 17, Units + 18, Units + 19, Units + 20, Units + 21, Units + 22, Units + 23, Units + 24, Units + 25, Units + 26, Units + 27, Units + 28, Units + 29, Units + 30, Units + 31, Units + 32, Units + 33}; /* members */
//
//  static pUnit Output1[2] = {Units + 34, Units + 35}; /* members */
//
//  static int Output[2] = {34, 35};
//
//  for(member = 0; member < 9; member++) {
//    Input[member]->act = in[member];
//  }
//
//  for (member = 0; member < 24; member++) {
//    unit = Hidden1[member];
//    sum = 0.0;
//    for (source = 0; source < unit->NoOfSources; source++) {
//      sum += unit->sources[source]->act
//             * unit->weights[source];
//    }
//    unit->act = Act_Logistic(sum, unit->Bias);
//  };
//
//  for (member = 0; member < 2; member++) {
//    unit = Output1[member];
//    sum = 0.0;
//    for (source = 0; source < unit->NoOfSources; source++) {
//      sum += unit->sources[source]->act
//             * unit->weights[source];
//    }
//    unit->act = Act_IdentityPlusBias(sum, unit->Bias);
//  };
//
//  for(member = 0; member < 2; member++) {
//    out[member] = Units[Output[member]].act;
//  }
//
//  return(OK);
//}
