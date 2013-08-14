/*++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*                                                      */
/* File:  ControlVelocityBrake.c                        */
/*                                                      */
/* Author: Automatically generated by Xfuzzy            */
/*                                                      */
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdio.h>
#include <math.h>
#include <controle/ControlVelocityBrake.h>

/*======================================================*/
/*  Common function to compute a fuzzy number           */
/*======================================================*/

static double compute(FuzzyNumber fn,double x) {
 int length = fn.length;
 int i;
 double imp = fn.imp(fn.degree[0],fn.conc[0].equal(x));
 double mu = imp;

 for(i=1; i<length; i++) {
  imp = fn.imp(fn.degree[i],fn.conc[i].equal(x));
  mu = fn.also(mu,imp);
 }
 return mu;
}

/*======================================================*/
/*  MembershipFunction FMF_xfl_triangular               */
/*======================================================*/

/*------------------------------------------------------*/
/* Function to compute an equal relationship            */
/*------------------------------------------------------*/

static double FMF_xfl_triangular_equal(double x, int i, double min, double max, double step, double *p, int length) {
      double a = (i==0? min-1 : (i==1 ? min : p[i-2]));
      double b = (i==0? min : (i==length+1? max : p[i-1]));
      double c = (i==length? max : (i==length+1? max+1 : p[i]));
      return (a<x && x<=b? (x-a)/(b-a) : (b<x && x<c? (c-x)/(c-b) : 0));
}

/*======================================================*/
/*  Operatorset OP__default_                            */
/*======================================================*/

/*------------------------------------------------------*/
/* Description of the operator AND                      */
/*------------------------------------------------------*/

static double OP__default__And(double a, double b) {
    return (a<b? a : b); 

}

/*------------------------------------------------------*/
/* Description of the operator ALSO                     */
/*------------------------------------------------------*/

static double OP__default__Also(double a, double b) {
    return (a>b? a : b); 

}

/*------------------------------------------------------*/
/* Description of the operator IMPLICATION              */
/*------------------------------------------------------*/

static double OP__default__Imp(double a, double b) {
    return (a<b? a : b); 

}

/*------------------------------------------------------*/
/* Description of the defuzzification method            */
/*------------------------------------------------------*/

static double OP__default__Defuz(FuzzyNumber mf) {
 double min = mf.min;
 double max = mf.max;
 double step = mf.step;
     double x, m, num=0, denom=0;
     for(x=min; x<=max; x+=step) {
      m = compute(mf,x);
      num += x*m;
      denom += m;
     }
     if(denom==0) return (min+max)/2;
     return num/denom;

}


/*======================================================*/
/*  Type TP_erroV                                       */
/*======================================================*/

/*------------------------------------------------------*/
/* Description of the family fam                        */
/*------------------------------------------------------*/

static double TP_erroV_fam_equal(double x, int i){
   double list[3] = {-15.0,0.0,15.0};
   return FMF_xfl_triangular_equal(x,i,-30.0,30.0,0.006000600060006,list,3);
}

/*------------------------------------------------------*/
/* Description of the label muitoDevagar                */
/*------------------------------------------------------*/

static double TP_erroV_muitoDevagar_equal(double x){
   return TP_erroV_fam_equal(x,0);}

/*------------------------------------------------------*/
/* Description of the label devagar                     */
/*------------------------------------------------------*/

static double TP_erroV_devagar_equal(double x){
   return TP_erroV_fam_equal(x,1);}

/*------------------------------------------------------*/
/* Description of the label velocidadeCruzeiro          */
/*------------------------------------------------------*/

static double TP_erroV_velocidadeCruzeiro_equal(double x){
   return TP_erroV_fam_equal(x,2);}

/*------------------------------------------------------*/
/* Description of the label rapido                      */
/*------------------------------------------------------*/

static double TP_erroV_rapido_equal(double x){
   return TP_erroV_fam_equal(x,3);}

/*------------------------------------------------------*/
/* Description of the label muitoRapido                 */
/*------------------------------------------------------*/

static double TP_erroV_muitoRapido_equal(double x){
   return TP_erroV_fam_equal(x,4);}

/*======================================================*/
/*  Type TP_accelAjust                                  */
/*======================================================*/

/*------------------------------------------------------*/
/* Description of the family fam                        */
/*------------------------------------------------------*/

static double TP_accelAjust_fam_equal(double x, int i){
   double list[3] = {-5.0,0.0,5.0};
   return FMF_xfl_triangular_equal(x,i,-10.0,10.0,0.002000200020002,list,3);
}

/*------------------------------------------------------*/
/* Description of the label desacelerarMuito            */
/*------------------------------------------------------*/

static double TP_accelAjust_desacelerarMuito_equal(double x){
   return TP_accelAjust_fam_equal(x,0);}

/*------------------------------------------------------*/
/* Description of the label desacelerar                 */
/*------------------------------------------------------*/

static double TP_accelAjust_desacelerar_equal(double x){
   return TP_accelAjust_fam_equal(x,1);}

/*------------------------------------------------------*/
/* Description of the label manterAceleracao            */
/*------------------------------------------------------*/

static double TP_accelAjust_manterAceleracao_equal(double x){
   return TP_accelAjust_fam_equal(x,2);}

/*------------------------------------------------------*/
/* Description of the label acelerar                    */
/*------------------------------------------------------*/

static double TP_accelAjust_acelerar_equal(double x){
   return TP_accelAjust_fam_equal(x,3);}

/*------------------------------------------------------*/
/* Description of the label acelerarMuito               */
/*------------------------------------------------------*/

static double TP_accelAjust_acelerarMuito_equal(double x){
   return TP_accelAjust_fam_equal(x,4);}

/*======================================================*/
/*  Type TP_dV                                          */
/*======================================================*/

/*------------------------------------------------------*/
/* Description of the family fam                        */
/*------------------------------------------------------*/

static double TP_dV_fam_equal(double x, int i){
   double list[3] = {-2.5,0.0,2.5};
   return FMF_xfl_triangular_equal(x,i,-5.0,5.0,0.001000100010001,list,3);
}

/*------------------------------------------------------*/
/* Description of the label desacelerandoMuito          */
/*------------------------------------------------------*/

static double TP_dV_desacelerandoMuito_equal(double x){
   return TP_dV_fam_equal(x,0);}

/*------------------------------------------------------*/
/* Description of the label desacelerando               */
/*------------------------------------------------------*/

static double TP_dV_desacelerando_equal(double x){
   return TP_dV_fam_equal(x,1);}

/*------------------------------------------------------*/
/* Description of the label velocidadeConstante         */
/*------------------------------------------------------*/

static double TP_dV_velocidadeConstante_equal(double x){
   return TP_dV_fam_equal(x,2);}

/*------------------------------------------------------*/
/* Description of the label acelerando                  */
/*------------------------------------------------------*/

static double TP_dV_acelerando_equal(double x){
   return TP_dV_fam_equal(x,3);}

/*------------------------------------------------------*/
/* Description of the label acelerandoMuito             */
/*------------------------------------------------------*/

static double TP_dV_acelerandoMuito_equal(double x){
   return TP_dV_fam_equal(x,4);}

/*======================================================*/
/*  Type TP_brakeAjust                                  */
/*======================================================*/

/*------------------------------------------------------*/
/* Description of the family fam                        */
/*------------------------------------------------------*/

static double TP_brakeAjust_fam_equal(double x, int i){
   double list[3] = {-5.0,0.0,5.0};
   return FMF_xfl_triangular_equal(x,i,-10.0,10.0,0.002000200020002,list,3);
}

/*------------------------------------------------------*/
/* Description of the label frearMuito                  */
/*------------------------------------------------------*/

static double TP_brakeAjust_frearMuito_equal(double x){
   return TP_brakeAjust_fam_equal(x,4);}

/*------------------------------------------------------*/
/* Description of the label aumentarFrenagem            */
/*------------------------------------------------------*/

static double TP_brakeAjust_aumentarFrenagem_equal(double x){
   return TP_brakeAjust_fam_equal(x,3);}

/*------------------------------------------------------*/
/* Description of the label manterFrenagem              */
/*------------------------------------------------------*/

static double TP_brakeAjust_manterFrenagem_equal(double x){
   return TP_brakeAjust_fam_equal(x,2);}

/*------------------------------------------------------*/
/* Description of the label liberarBastante             */
/*------------------------------------------------------*/

static double TP_brakeAjust_liberarBastante_equal(double x){
   return TP_brakeAjust_fam_equal(x,0);}

/*------------------------------------------------------*/
/* Description of the label liberar                     */
/*------------------------------------------------------*/

static double TP_brakeAjust_liberar_equal(double x){
   return TP_brakeAjust_fam_equal(x,1);}

/*======================================================*/
/*  Rulebase RL_AccelRules                              */
/*======================================================*/

static void RL_AccelRules(double erroV, double dV, double *accelAjust) {
 double _rl;

 double _accelAjust_degree[25];
 Consequent _accelAjust_conc[25];
 FuzzyNumber _accelAjust;
 _accelAjust.min = -10.0;
 _accelAjust.max = 10.0;
 _accelAjust.step = 0.002000200020002;
 _accelAjust.imp = OP__default__Imp;
 _accelAjust.also = OP__default__Also;
 _accelAjust.length = 25;
 _accelAjust.degree = _accelAjust_degree;
 _accelAjust.conc = _accelAjust_conc;
 int _accelAjust_i = 0;

 double _erroV_eq[5];
 _erroV_eq[0] = TP_erroV_muitoDevagar_equal(erroV);
 _erroV_eq[1] = TP_erroV_devagar_equal(erroV);
 _erroV_eq[2] = TP_erroV_velocidadeCruzeiro_equal(erroV);
 _erroV_eq[3] = TP_erroV_rapido_equal(erroV);
 _erroV_eq[4] = TP_erroV_muitoRapido_equal(erroV);

 double _dV_eq[5];
 _dV_eq[0] = TP_dV_desacelerandoMuito_equal(dV);
 _dV_eq[1] = TP_dV_desacelerando_equal(dV);
 _dV_eq[2] = TP_dV_velocidadeConstante_equal(dV);
 _dV_eq[3] = TP_dV_acelerando_equal(dV);
 _dV_eq[4] = TP_dV_acelerandoMuito_equal(dV);

 _rl = OP__default__And(_erroV_eq[0],_dV_eq[0]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_acelerarMuito_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[0],_dV_eq[1]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_acelerarMuito_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[0],_dV_eq[2]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_acelerar_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[0],_dV_eq[3]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_manterAceleracao_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[0],_dV_eq[4]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_desacelerar_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[1],_dV_eq[0]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_acelerarMuito_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[1],_dV_eq[1]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_acelerar_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[1],_dV_eq[2]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_acelerar_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[1],_dV_eq[3]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_manterAceleracao_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[1],_dV_eq[4]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_desacelerar_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[2],_dV_eq[0]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_acelerar_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[2],_dV_eq[1]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_acelerar_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[2],_dV_eq[2]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_manterAceleracao_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[2],_dV_eq[3]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_desacelerar_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[2],_dV_eq[4]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_desacelerar_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[3],_dV_eq[0]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_acelerar_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[3],_dV_eq[1]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_manterAceleracao_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[3],_dV_eq[2]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_desacelerar_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[3],_dV_eq[3]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_desacelerar_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[3],_dV_eq[4]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_desacelerarMuito_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[4],_dV_eq[0]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_manterAceleracao_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[4],_dV_eq[1]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_desacelerar_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[4],_dV_eq[2]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_desacelerarMuito_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[4],_dV_eq[3]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_desacelerarMuito_equal;
 _accelAjust_i++;

 _rl = OP__default__And(_erroV_eq[4],_dV_eq[4]);
 _accelAjust_degree[_accelAjust_i] = _rl;
 _accelAjust_conc[_accelAjust_i].equal = TP_accelAjust_desacelerarMuito_equal;
 _accelAjust_i++;

 *accelAjust = OP__default__Defuz(_accelAjust);
}

/*======================================================*/
/*  Rulebase RL_BrakeRules                              */
/*======================================================*/

static void RL_BrakeRules(double erroV, double dV, double *brakeAjust) {
 double _rl;

 double _brakeAjust_degree[25];
 Consequent _brakeAjust_conc[25];
 FuzzyNumber _brakeAjust;
 _brakeAjust.min = -10.0;
 _brakeAjust.max = 10.0;
 _brakeAjust.step = 0.002000200020002;
 _brakeAjust.imp = OP__default__Imp;
 _brakeAjust.also = OP__default__Also;
 _brakeAjust.length = 25;
 _brakeAjust.degree = _brakeAjust_degree;
 _brakeAjust.conc = _brakeAjust_conc;
 int _brakeAjust_i = 0;

 double _erroV_eq[5];
 _erroV_eq[0] = TP_erroV_muitoDevagar_equal(erroV);
 _erroV_eq[1] = TP_erroV_devagar_equal(erroV);
 _erroV_eq[2] = TP_erroV_velocidadeCruzeiro_equal(erroV);
 _erroV_eq[3] = TP_erroV_rapido_equal(erroV);
 _erroV_eq[4] = TP_erroV_muitoRapido_equal(erroV);

 double _dV_eq[5];
 _dV_eq[0] = TP_dV_desacelerandoMuito_equal(dV);
 _dV_eq[1] = TP_dV_desacelerando_equal(dV);
 _dV_eq[2] = TP_dV_velocidadeConstante_equal(dV);
 _dV_eq[3] = TP_dV_acelerando_equal(dV);
 _dV_eq[4] = TP_dV_acelerandoMuito_equal(dV);

 _rl = OP__default__And(_erroV_eq[0],_dV_eq[0]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_liberarBastante_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[0],_dV_eq[1]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_liberarBastante_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[0],_dV_eq[2]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_liberarBastante_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[0],_dV_eq[3]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_manterFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[0],_dV_eq[4]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_manterFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[1],_dV_eq[0]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_liberar_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[1],_dV_eq[1]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_liberar_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[1],_dV_eq[2]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_liberar_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[1],_dV_eq[3]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_manterFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[1],_dV_eq[4]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_aumentarFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[2],_dV_eq[0]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_liberar_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[2],_dV_eq[1]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_liberar_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[2],_dV_eq[2]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_manterFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[2],_dV_eq[3]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_aumentarFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[2],_dV_eq[4]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_aumentarFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[3],_dV_eq[0]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_manterFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[3],_dV_eq[1]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_manterFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[3],_dV_eq[2]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_aumentarFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[3],_dV_eq[3]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_aumentarFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[3],_dV_eq[4]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_aumentarFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[4],_dV_eq[0]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_manterFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[4],_dV_eq[1]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_manterFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[4],_dV_eq[2]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_aumentarFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[4],_dV_eq[3]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_aumentarFrenagem_equal;
 _brakeAjust_i++;

 _rl = OP__default__And(_erroV_eq[4],_dV_eq[4]);
 _brakeAjust_degree[_brakeAjust_i] = _rl;
 _brakeAjust_conc[_brakeAjust_i].equal = TP_brakeAjust_aumentarFrenagem_equal;
 _brakeAjust_i++;

 *brakeAjust = OP__default__Defuz(_brakeAjust);
}


/*======================================================*/
/*                   Inference Engine                   */
/*======================================================*/

void ControlVelocityBrakeInferenceEngine(double erroV, double dV, double *_d_accelAjust, double *_d_brakeAjust) {
 double accelAjust;
 double brakeAjust;
 RL_AccelRules(erroV, dV, &accelAjust);
 RL_BrakeRules(erroV, dV, &brakeAjust);
 *_d_accelAjust = accelAjust;
 *_d_brakeAjust = brakeAjust;
}

