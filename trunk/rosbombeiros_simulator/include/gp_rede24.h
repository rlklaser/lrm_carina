/*********************************************************
  gp_rede24.h
  --------------------------------------------------------
  generated at Mon Nov 19 15:53:58 2007
  by snns2c ( Bernward Kett 1995 ) 
*********************************************************/

extern "C" int gp_rede24(float *in, float *out, int init);

static struct {
  int NoOfInput;    /* Number of Input Units  */
  int NoOfOutput;   /* Number of Output Units */
  int(* propFunc)(float *, float*, int);
} gp_rede24REC = {7,2,gp_rede24};
