#ifndef CONSTANTES_H_
#define CONSTANTES_H_

#define QTD_OBSTACULOS        10
#define QTD_VEICULOS          5
#define QTD_RODAS              4
#define TAM_RAIO              7   // 1
#define QTD_RAIO              21
#define QTD_CONTACT          8

#define QTD_LIXO 7
#define QTD_RECICLA 7

#define OFFSET_POS_CILINDRO   40

#define COMPRIMENTO          0.75 //1.75 //3.5
#define LARGURA              0.55 //1.25 //2.5
#define ALTURA               0.25 //0.5  //1.0
#define RAIO_RODA            0.08 //0.25 //0.5
#define TAM_SUPORTE          0.01
#define QTD_SUPORTE            28

#define RAIO_PCONTATO       0.045   // 0.09
// from ode sample
#define STARTZ    0.5 //.5    //1.0		// starting height of chassis
#define COMOFFSET -0.05 //-5	// center of mass offset


struct simulation
{
	int cCont; /* contador de pontos de contato */
	float pContact[QTD_CONTACT][3];
	//dVector3 p[QTD_SUPORTE];
	//dVector3 origin;
	//dVector3 dir;

	dWorldID world;
	dSpaceID space;
	dJointGroupID contactgroup;
	dGeomID ground;

	void (*bodyCollision)(void *, dGeomID, dGeomID);
	void (*rayCollision)(void *, dGeomID, dGeomID);
};

inline double saturation(double u, double min, double max)
{
	if (u > max)
		u = max;
	if (u < min)
		u = min;
	return u;
}


#endif
