#include <iostream>
#include <sstream>
#include <ode/ode.h>
#include <vector>
#include <drawstuff/drawstuff.h>
#include <cstdlib>
#include <cmath>
#include <ctime>

#include "veiculo.h"
#include "bussola.h"
#include "gps.h"
#include "constantes.h"
#include "mymath.h"
#include "aco.h"
#include "item.h"

/**/

dGeomID barra1geom;
dBodyID barra1body;
double pb1[] = {0,30,2};
double db1[] = {2,58,5};  // dimensao

dGeomID barra2geom;
dBodyID barra2body;
double pb2[] = {60,30,2};
double db2[] = {2,58,5}; // dimensao

dGeomID barra3geom;
dBodyID barra3body;
double pb3[] = {30,60,2};
double db3[] = {58,2,5};  // dimensao

dGeomID barra4geom;
dBodyID barra4body;
double pb4[] = {30,0,2};
double db4[] = {58,2,5};  // dimensao

/******************************************************************************
 * Posicoes
 *****************************************************************************/

int mostra_area_lixeira = 1;
int mostra_area_reciclagem = 1;
int mostra_area_bateria = 1;

int mostra_item_reciclagem = 1;
int mostra_item_lixo = 1;

double posicao_area_reciclagem_x = (double)POS_LIX_1_X;
double posicao_area_reciclagem_y = (double)POS_LIX_1_Y;

double posicao_area_bateria_x = (double)POS_LIX_2_X;
double posicao_area_bateria_y = (double)POS_LIX_2_Y;

double posicao_area_lixeira_x = (double)POS_LIX_3_X;
double posicao_area_lixeira_y = (double)POS_LIX_3_Y;



Item item_lixo[QTD_LIXO];
Item item_recicla[QTD_RECICLA];


veiculo v[QTD_VEICULOS];


/******************************************************************************
 * Coisas da ODE 
 * Algumas coisas soh estao aqui para nao criar a cada ciclo de simulacao 
 *****************************************************************************/

#ifdef _MSC_VER
#pragma warning(disable:4244 C4305 4996)  // for VC++, no precision loss complaints
#endif

static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;
static dGeomID ground;

int cCont=0; 
float pContact[QTD_CONTACT][3];

const dReal ls[3] = {TAM_SUPORTE,TAM_SUPORTE,TAM_SUPORTE}; 
dVector3 pt[QTD_SUPORTE];
dVector3 origin,dir;


/******************************************************************************
 * Function ODE de controle de colisao  
 * Called by dSpaceCollide when two objects in space are potentially colliding
 *****************************************************************************/

static void nearCallback (void *data, dGeomID o1, dGeomID o2) 
{
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnected(b1, b2)) return;	

	dContact contact[QTD_CONTACT];

	if ((dGeomGetClass (o2) == dRayClass) || (dGeomGetClass (o1) == dRayClass)) 
	{
		/* se for raio, nao faz nada */
	}
	else {
		int n = dCollide (o1,o2,QTD_CONTACT,&contact[0].geom,sizeof(dContact));
		if (n > 0) {
			for (int i=0; i<n; i++) {
				contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
				if (dGeomGetClass(o1) == dSphereClass || dGeomGetClass(o2) == dSphereClass) contact[i].surface.mu = 20;
				else contact[i].surface.mu = 0.5;
				contact[i].surface.slip1 = 0.0;
				contact[i].surface.slip2 = 0.0;
				contact[i].surface.soft_erp = 0.8;
				contact[i].surface.soft_cfm = 0.01;
				dJointID c = dJointCreateContact (world,contactgroup,contact+i);
				dJointAttach (c,dGeomGetBody(o1),dGeomGetBody(o2));
			}
		}
	}
}

/******************************************************************************
 * Function ODE de controle de colisao especial para meu raio 
 * Called by dSpaceCollide when two objects in space are potentially colliding
 *****************************************************************************/

static void nearCallbackRay (void *data, dGeomID o1, dGeomID o2) 
{
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnected(b1, b2)) return;
	
	dContactGeom contact2[QTD_CONTACT];

	if ((dGeomGetClass (o2) == dRayClass) || (dGeomGetClass (o1) == dRayClass)) {
		if((o2==data) || (o1==data)) {
			int n = dCollide (o2,o1,QTD_CONTACT,&contact2[0],sizeof(dContactGeom));
			if (n > 0) {
				pContact[cCont][0] = contact2[0].pos[0];
				pContact[cCont][1] = contact2[0].pos[1];
				pContact[cCont][2] = contact2[0].pos[2];

				/* toda primeira vez que entra faz o segundo ponto de contato receber o primeiro */

				if (cCont==0) {
					pContact[1][0] = contact2[0].pos[0];
					pContact[1][1] = contact2[0].pos[1];
					pContact[1][2] = contact2[0].pos[2];
				}
				
				/* tem um esquema aqui que é o seguinte:
				   nem sempre o primeiro ponto de contato é o objeto mais proximo,
				   assim, nao tendo outra solução mais inteligente, farei o seguinte:
				   ponto 0 é sempre para-chique do carro
				   ponto 1 é colisão
				   se tem >1, entao testa a distancia vetorial entre 1 e 2 e troca */

				if (cCont>1) {
					/* se a distancia vetorial do ponto dois for menor que do ponto 1
					   entao troca pontos */
					  double a = MyMath::getDistanciaVetorial3d(pContact[0][0],pContact[0][1],
						  pContact[0][2], pContact[1][0],pContact[1][1],
						  pContact[1][2]);

					  double b = MyMath::getDistanciaVetorial3d(pContact[0][0],pContact[0][1],
						  pContact[0][2], pContact[cCont][0],pContact[cCont][1],
						  pContact[cCont][2]);
					if (b<a) {
						/* troca pontos */ 
						pContact[1][0] = pContact[cCont][0];  
						pContact[1][1] = pContact[cCont][1];
						pContact[1][2] = pContact[cCont][2];
					}
				}
				cCont++;
			}
		}
	}
}

/******************************************************************************
 * Set viewpoint of the simulation
 *****************************************************************************/

static void start() 
{
	static float xyz[3] = {22.538477, -23.605749, 32.130016};
	static float hpr[3] = {89.500000, -39.500000, 0.250000};
	dsSetViewpoint (xyz,hpr);
	printf ("G para pegar viewpoint.\n");
}

/******************************************************************************
 * Called when a key pressed 
 *****************************************************************************/

static void command (int cmd) 
{
	switch (cmd) 
	{
		//case 't': v[0].speed -= 0.3; break;
		//case 'f': v[0].speed += 0.3; break;
		//case 'd': v[0].turn += 0.1; if (v[0].turn > 0.8) v[0].turn = 0.8; break;
		//case 'e': v[0].turn -= 0.1; if (v[0].turn < -0.8) v[0].turn = -0.8; break;
		//case ' ': v[0].speed = 0; v[0].turn = 0; break;
		case 'g': 
			float t[3]; float y[3]; 
			dsGetViewpoint (t,y);
			printf ("%f %f %f %f %f %f\n",t[0],t[1],t[2],y[0],y[1],y[2]);
			break;
	}
}

/******************************************************************************
 * Simulation loop 
 *****************************************************************************/

static void simLoop (int pause) 
{
	dsSetTexture (DS_WOOD);

	/******************************************************************************
	* Define juntas 
	******************************************************************************/

	if (!pause) 
	{
		for (int j = 0; j < QTD_VEICULOS; j++) 
		{
			for (int i=0;i<QTD_RODAS;i++) 
			{
				if (i<2) /* rodas de tras, torque e giro */
				{
					dReal curturn = dJointGetHinge2Angle1 (v[j].rodaJoint[i]);
					dJointSetHinge2Param(v[j].rodaJoint[i],dParamVel,(v[j].turn-curturn)*1.0);
					dJointSetHinge2Param(v[j].rodaJoint[i],dParamFMax,dInfinity);
					dJointSetHinge2Param(v[j].rodaJoint[i],dParamVel2,(v[j].speed)*(-1));
					dJointSetHinge2Param(v[j].rodaJoint[i],dParamFMax2,25);
					dBodyEnable(dJointGetBody(v[j].rodaJoint[i],0));
					dBodyEnable(dJointGetBody(v[j].rodaJoint[i],1));
				}
				if (i>=2) /* rodas de tras, torque SEM gira */
				{ 
					dReal curturn = dJointGetHinge2Angle1 (v[j].rodaJoint[i]);
					dJointSetHinge2Param(v[j].rodaJoint[i],dParamVel,0);
					dJointSetHinge2Param(v[j].rodaJoint[i],dParamFMax,dInfinity);
					dJointSetHinge2Param(v[j].rodaJoint[i],dParamVel2,(v[j].speed)*(-1));
					dJointSetHinge2Param(v[j].rodaJoint[i],dParamFMax2,25);
					dBodyEnable(dJointGetBody(v[j].rodaJoint[i],0));
					dBodyEnable(dJointGetBody(v[j].rodaJoint[i],1));
				}
			}
		}		
		dSpaceCollide (space,0,&nearCallback);
		dWorldQuickStep (world,0.05);
		dJointGroupEmpty (contactgroup);
	}

	/******************************************************************************
	* Desenha itens (lixo, reciclagem) e areas (lixo, reciclagem, bateria) 
	******************************************************************************/

	const dReal quaternion_para_itens[] = {0.938075,  0.346433,  7.49379e-005, 0, -0.346433, 0.938075, -3.62081e-005, 0, -8.2841e-005, 8.00489e-006, 1, 0};
	const dReal quaternion_para_areas[] = {0.999812, -0.0193649, 6.87975e-005, 0, 0.0193649, 0.999812, -9.13112e-005, 0, -6.70163e-005, 9.26263e-005, 1};
	double sidesAreas[] = {8.0,8.0,.1};

	/* desenha lixeira */
	if (mostra_area_lixeira)
	{
		dsSetColor (.8,.3,.3);
		double posLixeira[] = {posicao_area_lixeira_x,posicao_area_lixeira_y,0.1};
		dsDrawBoxD (posLixeira,quaternion_para_areas,sidesAreas);
	}
	
	/* desenha lixo */
	if (mostra_item_lixo)
	{
		for (int u=0;u<QTD_LIXO;u++)
		{
			if(item_lixo[u].existe == true)
			{
			    double posCentro[] = {item_lixo[u].x,item_lixo[u].y,0.5};
			    dsDrawSphereD(posCentro,quaternion_para_itens,.5);
			}
		}
	}

	/* desenha feromonio */
	if (mostra_item_lixo)
	{
		dsSetColor (1,1,0);
		for (int u=0;u<TAM_GRID;u++)
			for (int q=0;q<TAM_GRID;q++)
		    {
				if(getFeromonio(u,q) > 0)
				{
			    	double posCentro[] = {u,q,0.5};
					dsDrawSphereD(posCentro,quaternion_para_itens,0.3);
				}
		    }
	}

	/* desenha area reciclagem */
	if (mostra_area_reciclagem)
	{
		dsSetColor (.3,.3,.8);
		double posR[] = {posicao_area_reciclagem_x,posicao_area_reciclagem_y,0.1};
		dsDrawBoxD (posR,quaternion_para_areas,sidesAreas);
	}

	/* desenha itens reciclagem */
	if (mostra_item_reciclagem)
	{
		for (int u=0;u<QTD_RECICLA;u++)
		{
			dsSetColor (.3,.3,.8);
			double posCentro[] = {item_recicla[u].x,item_recicla[u].y,0.5};
			dsDrawSphereD(posCentro,quaternion_para_itens,.5);
		}
	}

	/* desenha area bateria */
	if (mostra_area_bateria)
	{
		dsSetColor (.3,.8,.3);
		double posB[] = {posicao_area_bateria_x,posicao_area_bateria_y,0.1};
		dsDrawBoxD (posB,quaternion_para_areas,sidesAreas);
	}

	/******************************************************************************
	* Desenha o veiculo  
	******************************************************************************/

	for (int a=0;a<QTD_VEICULOS;a++) 
	{

		/* desenha lixo */
		dsSetColor (1,0,0);
		
		if(v[a].temLixo == true)
		{
		    double posCentro[] = {v[a].minhaPosicaoX,v[a].minhaPosicaoY,ALTURA+0.5};
		    dsDrawSphereD(posCentro,quaternion_para_itens,.3);
		}
		
		dsSetColor (.5,.8,.5);
		dReal sides[3] = {COMPRIMENTO,LARGURA,ALTURA};
		dsDrawBoxD (dGeomGetPosition(v[a].veicGeom),dGeomGetRotation(v[a].veicGeom),sides);
		dsSetColor (1,1,1);
		
		/* rodas */
		for (int b=0;b<QTD_RODAS;b++) dsDrawSphereD (dGeomGetPosition(v[a].rodaGeom[b]),dGeomGetRotation(v[a].rodaGeom[b]),RAIO_RODA);
		for (int t=0;t<QTD_SUPORTE;t++) dsDrawBoxD(dBodyGetPosition(v[a].supBody[t]),dBodyGetRotation(v[a].supBody[t]),ls);

		/* reposiciona raios */
		for (int r=0;r<QTD_SUPORTE;r++) dBodyCopyPosition(v[a].supBody[r], pt[r]);

		//dGeomRaySet(v[a].raioGeom[0], pt[0][0], pt[0][1], pt[0][2], pt[0][0]-pt[2][0], pt[0][1]-pt[2][1], pt[0][2]-pt[2][2]);
		dGeomRaySet(v[a].raioGeom[0], pt[0][0], pt[0][1], pt[0][2], pt[0][0]-pt[1][0], pt[0][1]-pt[1][1], pt[0][2]-pt[1][2]);
		//dGeomRaySet(v[a].raioGeom[2], pt[0][0], pt[0][1], pt[0][2], pt[0][0]-pt[3][0], pt[0][1]-pt[3][1], pt[0][2]-pt[3][2]);

		//dGeomRaySet(v[a].raioGeom[3], pt[4][0], pt[4][1], pt[4][2], pt[4][0]-pt[7][0], pt[4][1]-pt[7][1], pt[4][2]-pt[7][2]);
		dGeomRaySet(v[a].raioGeom[1], pt[4][0], pt[4][1], pt[4][2], pt[4][0]-pt[5][0], pt[4][1]-pt[5][1], pt[4][2]-pt[5][2]);
		//dGeomRaySet(v[a].raioGeom[5], pt[4][0], pt[4][1], pt[4][2], pt[4][0]-pt[6][0], pt[4][1]-pt[6][1], pt[4][2]-pt[6][2]);

		//dGeomRaySet(v[a].raioGeom[6], pt[8][0], pt[8][1], pt[8][2], pt[8][0]-pt[11][0], pt[8][1]-pt[11][1], pt[8][2]-pt[11][2]);
		dGeomRaySet(v[a].raioGeom[2], pt[8][0], pt[8][1], pt[8][2], pt[8][0]-pt[9][0], pt[8][1]-pt[9][1], pt[8][2]-pt[9][2]);
		//dGeomRaySet(v[a].raioGeom[8], pt[8][0], pt[8][1], pt[8][2], pt[8][0]-pt[10][0], pt[8][1]-pt[10][1], pt[8][2]-pt[10][2]);

		//dGeomRaySet(v[a].raioGeom[9], pt[12][0], pt[12][1], pt[12][2], pt[12][0]-pt[13][0], pt[12][1]-pt[13][1], pt[12][2]-pt[13][2]);
		dGeomRaySet(v[a].raioGeom[3], pt[12][0], pt[12][1], pt[12][2], pt[12][0]-pt[14][0], pt[12][1]-pt[14][1], pt[12][2]-pt[14][2]);
		//dGeomRaySet(v[a].raioGeom[11], pt[12][0], pt[12][1], pt[12][2], pt[12][0]-pt[15][0], pt[12][1]-pt[15][1], pt[12][2]-pt[15][2]);
		
		//dGeomRaySet(v[a].raioGeom[12], pt[16][0], pt[16][1], pt[16][2], pt[16][0]-pt[17][0], pt[16][1]-pt[17][1], pt[16][2]-pt[17][2]);
		dGeomRaySet(v[a].raioGeom[4], pt[16][0], pt[16][1], pt[16][2], pt[16][0]-pt[18][0], pt[16][1]-pt[18][1], pt[16][2]-pt[18][2]);
		//dGeomRaySet(v[a].raioGeom[14], pt[16][0], pt[16][1], pt[16][2], pt[16][0]-pt[19][0], pt[16][1]-pt[19][1], pt[16][2]-pt[19][2]);

		//GeomRaySet(v[a].raioGeom[15], pt[20][0], pt[20][1], pt[20][2], pt[20][0]-pt[21][0], pt[20][1]-pt[21][1], pt[20][2]-pt[21][2]);
		//GeomRaySet(v[a].raioGeom[16], pt[20][0], pt[20][1], pt[20][2], pt[20][0]-pt[22][0], pt[20][1]-pt[22][1], pt[20][2]-pt[22][2]);
		//dGeomRaySet(v[a].raioGeom[17], pt[20][0], pt[20][1], pt[20][2], pt[20][0]-pt[23][0], pt[20][1]-pt[23][1], pt[20][2]-pt[23][2]);

		//dGeomRaySet(v[a].raioGeom[18], pt[24][0], pt[24][1], pt[24][2], pt[24][0]-pt[25][0], pt[24][1]-pt[25][1], pt[24][2]-pt[25][2]);
		//dGeomRaySet(v[a].raioGeom[19], pt[24][0], pt[24][1], pt[24][2], pt[24][0]-pt[26][0], pt[24][1]-pt[26][1], pt[24][2]-pt[26][2]);
		//dGeomRaySet(v[a].raioGeom[20], pt[24][0], pt[24][1], pt[24][2], pt[24][0]-pt[27][0], pt[24][1]-pt[27][1], pt[24][2]-pt[27][2]);

		/* desenha raios do veiculo e testa colisao dos raios */
		for (int b=0;b<QTD_RAIO;b++) 
		{
			dSpaceCollide (space,v[a].raioGeom[b],&nearCallbackRay);

			double d = MyMath::getDistanciaVetorial3d(pContact[0][0],pContact[0][1],
						  pContact[0][2], pContact[1][0],pContact[1][1],
						  pContact[1][2]);

			dsSetColor (.5,.5,.5);
			dVector3 tpos1={pContact[0][0],pContact[0][1],pContact[0][2]};
			dsDrawSphereD(tpos1,dGeomGetRotation(v[a].veicGeom),RAIO_PCONTATO);

			dsSetColor (1,1,0);
			dVector3 tpos2={pContact[1][0],pContact[1][1],pContact[1][2]};
			dsDrawSphereD(tpos2,dGeomGetRotation(v[a].veicGeom),RAIO_PCONTATO);

			cCont=0;

			dGeomRayGet (v[a].raioGeom[b],origin,dir);
			if (d<0.00002) d=TAM_RAIO; /* duvidas? pessin@gmail.com */
			for (int j=0; j<3; j++) dir[j] = dir[j]*d + origin[j];
			dsSetColor (1,1,1);
			dsDrawLineD (origin,dir);

			v[a].raiosDistancias[b]=d;
		}

		/* arruma info dos senroses (acumula raios), assim testa apenas 0,3,6,9,12,15,18*/
		/*if (v[a].raiosDistancias[0]>v[a].raiosDistancias[1]) v[a].raiosDistancias[0]=v[a].raiosDistancias[1];
		if (v[a].raiosDistancias[0]>v[a].raiosDistancias[2]) v[a].raiosDistancias[0]=v[a].raiosDistancias[2];

		if (v[a].raiosDistancias[3]>v[a].raiosDistancias[4]) v[a].raiosDistancias[3]=v[a].raiosDistancias[4];
		if (v[a].raiosDistancias[3]>v[a].raiosDistancias[5]) v[a].raiosDistancias[3]=v[a].raiosDistancias[5];

		if (v[a].raiosDistancias[6]>v[a].raiosDistancias[7]) v[a].raiosDistancias[6]=v[a].raiosDistancias[7];
		if (v[a].raiosDistancias[6]>v[a].raiosDistancias[8]) v[a].raiosDistancias[6]=v[a].raiosDistancias[8];

		if (v[a].raiosDistancias[9]>v[a].raiosDistancias[10]) v[a].raiosDistancias[9]=v[a].raiosDistancias[10];
		if (v[a].raiosDistancias[9]>v[a].raiosDistancias[11]) v[a].raiosDistancias[9]=v[a].raiosDistancias[11];

		if (v[a].raiosDistancias[12]>v[a].raiosDistancias[13]) v[a].raiosDistancias[12]=v[a].raiosDistancias[13];
		if (v[a].raiosDistancias[12]>v[a].raiosDistancias[14]) v[a].raiosDistancias[12]=v[a].raiosDistancias[14];

		if (v[a].raiosDistancias[15]>v[a].raiosDistancias[16]) v[a].raiosDistancias[15]=v[a].raiosDistancias[16];
		if (v[a].raiosDistancias[15]>v[a].raiosDistancias[17]) v[a].raiosDistancias[15]=v[a].raiosDistancias[17];

		if (v[a].raiosDistancias[18]>v[a].raiosDistancias[19]) v[a].raiosDistancias[18]=v[a].raiosDistancias[19];
		if (v[a].raiosDistancias[18]>v[a].raiosDistancias[20]) v[a].raiosDistancias[18]=v[a].raiosDistancias[20];*/
	}

	/* */
	dsSetColor (.5,.5,.5);
	dReal sides1[3] = {db1[0],db1[1],db1[2]};
	dsDrawBoxD (dGeomGetPosition(barra1geom),dGeomGetRotation(barra1geom),sides1);

	dsSetColor (.5,.5,.5);
	dReal sides2[3] = {db2[0],db2[1],db2[2]};
	dsDrawBoxD (dGeomGetPosition(barra2geom),dGeomGetRotation(barra2geom),sides2);

	dsSetColor (.5,.5,.5);
	dReal sides3[3] = {db3[0],db3[1],db3[2]};
	dsDrawBoxD (dGeomGetPosition(barra3geom),dGeomGetRotation(barra3geom),sides3);

	dsSetColor (.5,.5,.5);
	dReal sides4[3] = {db4[0],db4[1],db4[2]};
	dsDrawBoxD (dGeomGetPosition(barra4geom),dGeomGetRotation(barra4geom),sides4);		

	/**************************************************************************
	* Comportamento   
	**************************************************************************/
	movimentaFormigas(v,item_lixo);
    atualizaFeromonio();
    zeraDelta();
	
	double minha_posicao_corrente_x,minha_posicao_corrente_y;
	double veiculo_orientacao;
	double azimute_para_destino;
	double erro_gps = ERRO_GPS; 
	double angulo_oposto;
	double diferenca_angulos;
	double distancia_para_destino;
	double distancia_para_destino_final;

	for (int i=0; i < QTD_VEICULOS; i++) 
	{
		veiculo_orientacao = v[i].GetBussola();
		v[i].GetGpsUtmPosition(&minha_posicao_corrente_x,&minha_posicao_corrente_y);
		v[i].minhaPosicaoX = minha_posicao_corrente_x;
		v[i].minhaPosicaoY = minha_posicao_corrente_y;


		azimute_para_destino = v[i].GetAngleToEndpoint(v[i].destinoX,v[i].destinoY);
		distancia_para_destino = sqrt((pow((minha_posicao_corrente_x - v[i].destinoX),2.0)) + (pow((minha_posicao_corrente_y - v[i].destinoY),2.0)));
		distancia_para_destino_final = sqrt((pow((minha_posicao_corrente_x - posicao_area_lixeira_x),2.0)) + (pow((minha_posicao_corrente_y - posicao_area_lixeira_y),2.0)));

		v[i].speed = 5.0;		
		//if (distancia_para_destino_final < erro_gps) v[i].speed = 0.0;

		

		/* 
		 * calcula angulo oposto, 
		 * angulo oposto e diferença servem p "suavizar" curva
		 * e para fazer saida direita / esquerda correta
		 */ 	

		angulo_oposto = 180.0 + veiculo_orientacao;
		if (angulo_oposto > 360.0) angulo_oposto -= 360.0;
		diferenca_angulos = veiculo_orientacao - azimute_para_destino; 
		if (diferenca_angulos < 0) diferenca_angulos *= -1;

		/**********************************************************************
		 * Controle de extercamento
		 *********************************************************************/

		v[i].turn = 0;
		
		if (distancia_para_destino > erro_gps)  
		{
			if (
				( (azimute_para_destino > veiculo_orientacao) && (azimute_para_destino < angulo_oposto) && (angulo_oposto > veiculo_orientacao) )
				||
				( !(azimute_para_destino > veiculo_orientacao) && (azimute_para_destino < angulo_oposto) && !(angulo_oposto > veiculo_orientacao) )
				||
				( (azimute_para_destino > veiculo_orientacao) && !(azimute_para_destino < angulo_oposto) && !(angulo_oposto > veiculo_orientacao) )
				)
			{
				if (diferenca_angulos < 5.0) v[i].turn = 0.1;
				else v[i].turn = 1.5;
			}
		
			else 
			{
				if (diferenca_angulos<5) v[i].turn = -0.1;
				else v[i].turn = -1.5;
			}

			/******************************************************************
			* Bloco de desvio 
			******************************************************************/

		 

			//if (v[i].raiosDistancias[4] < 4.5 ) v[i].turn = -0.6; /* ok para NE,SE e SO*/
			//if (v[i].raiosDistancias[0] < 4.5 ) v[i].turn = 0.6; /* ok para NE,SE e SO*/
			//if ((v[i].raiosDistancias[0] < 4.5 )&&(v[i].raiosDistancias[4] < 4.5 )) v[i].turn = 0.0;

			//if (v[i].raiosDistancias[3] < 6.0 ) v[i].turn = -0.6; /* ok para NE,SE e SO*/
			//if (v[i].raiosDistancias[1] < 6.0 ) v[i].turn = 0.6; /* ok para NE,SE e SO*/
			//if ((v[i].raiosDistancias[1] < 6.0 )&&(v[i].raiosDistancias[3] < 6.0 )) v[i].turn = 0.0;

			//if (v[i].raiosDistancias[3] < v[i].raiosDistancias[1]) v[i].turn = -0.2; 
			//if (v[i].raiosDistancias[1] < v[i].raiosDistancias[3]) v[i].turn = 0.2; 

		 //

			//if (v[i].raiosDistancias[4] < 1.6 ) v[i].turn = -0.6; /* ok para NE,SE e SO*/
			//if (v[i].raiosDistancias[0] < 1.6 ) v[i].turn = 0.6; /* ok para NE,SE e SO*/

			//if ((v[i].raiosDistancias[2] < 3.0 ) && (v[i].raiosDistancias[1] < 3.0 ) && (v[i].raiosDistancias[3] < 3.0)) 
			//{
			//	if (v[i].raiosDistancias[4] < v[i].raiosDistancias[0]) v[i].turn = -0.6;
			//	else v[i].turn = 0.6;
			//}
		}
	}
	
	/**************************************************************************
	* fim dos comportamentos   
	**************************************************************************/
}

/******************************************************************************
* main
******************************************************************************/

int main (int argc, char **argv) 
{

//=================ACO STUFF================================================
    srand(time(0));

    inicializaFeromonio();
    zeraDelta();
	movimentaFormigas(v,item_lixo);

	atualizaFeromonioInit();
    zeraDelta();

//============================================================================

	//srand(time(NULL));
	//srand(73);

	/* setup pointers to drawstuff callback functions */
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = "textures";

	/* setup ode */
	dInitODE();
	world = dWorldCreate();
	space = dHashSpaceCreate (0);
	contactgroup = dJointGroupCreate (0);
	dWorldSetGravity (world,0,0,-9.5);
	dWorldSetCFM (world, 1e-5);
	dWorldSetERP (world, 0.8);
	dWorldSetQuickStepNumIterations (world,20);
	ground = dCreatePlane (space,0,0,1,0);

	/* setup meus objetos */
	for (int u=0;u<QTD_LIXO;u++)    
	{ 
		item_lixo[u].x = rand()%50+5;     
		item_lixo[u].y = rand()%50+5; 
		item_lixo[u].existe = true;
	}
	for (int u=0;u<QTD_RECICLA;u++)	{ item_recicla[u].x = rand()%30+10;  item_recicla[u].y = rand()%30+10; }
	for (int i=0;i<QTD_VEICULOS;i++) v[i].genesis(world,space);

	double massa = 50.0;
	/* 1 */
	dMass m;
	barra1body = dBodyCreate (world);
	dBodySetPosition (barra1body,pb1[0],pb1[1],pb1[2]);
	dMassSetBox (&m,1,db1[0],db1[1],db1[2]);
	dMassAdjust (&m,massa);
	dBodySetMass (barra1body,&m);
	barra1geom = dCreateBox (space,db1[0],db1[1],db1[2]);
	dGeomSetBody (barra1geom,barra1body);

	/* 2 */
	barra2body = dBodyCreate (world);
	dBodySetPosition (barra2body,pb2[0],pb2[1],pb2[2]);
	dMassSetBox (&m,1,db2[0],db2[1],db2[2]);
	dMassAdjust (&m,massa);
	dBodySetMass (barra2body,&m);
	barra2geom = dCreateBox (space,db2[0],db2[1],db2[2]);
	dGeomSetBody (barra2geom,barra2body);

	/* 3 */
	barra3body = dBodyCreate (world);
	dBodySetPosition (barra3body,pb3[0],pb3[1],pb3[2]);
	dMassSetBox (&m,1,db3[0],db3[1],db3[2]);
	dMassAdjust (&m,massa);
	dBodySetMass (barra3body,&m);
	barra3geom = dCreateBox (space,db3[0],db3[1],db3[2]);
	dGeomSetBody (barra3geom,barra3body);

	/* 4 */
	barra4body = dBodyCreate (world);
	dBodySetPosition (barra4body,pb4[0],pb4[1],pb4[2]);
	dMassSetBox (&m,1,db4[0],db4[1],db4[2]);
	dMassAdjust (&m,massa);
	dBodySetMass (barra4body,&m);
	barra4geom = dCreateBox (space,db4[0],db4[1],db4[2]);
	dGeomSetBody (barra4geom,barra4body);

	/**/

	dJointID barraJoint = dJointCreateFixed(world, 0);
	dJointAttach(barraJoint, barra4body, barra2body);
	dJointSetFixed(barraJoint);
	
    dJointID barraJoint1 = dJointCreateFixed(world, 0);
	dJointAttach(barraJoint1, barra2body, barra1body);
	dJointSetFixed(barraJoint1);

    dJointID barraJoint2 = dJointCreateFixed(world, 0);
	dJointAttach(barraJoint2, barra2body, barra3body);
	dJointSetFixed(barraJoint2);

	/* run simulaion */
	dsSimulationLoop (argc,argv,1100,640,&fn);

	/* close ode */	
	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);
	dCloseODE();
	return 0;
}