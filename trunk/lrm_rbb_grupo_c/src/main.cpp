#include <iostream>
#include <sstream>
#include <ode/ode.h>
#include <vector>
#include <drawstuff/drawstuff.h>
#include <stdlib.h>
#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>

//#include "antenna.h"
#include "obstaculo.h"
#include "veiculo.h"
#include "bussola.h"
#include "gps.h"
#include "constantes.h"
#include "mymath.h"


static struct simulation sim;

int mostraReciclagem = 1;
int mostraBateria = 1;
int mostraLixo = 1;

double posicaoReciclagemX = 0.0;
double posicaoReciclagemY = 0.0;

double posicaoBateriaX = 0.0;
double posicaoBateriaY = 65.0;

double posicaoLixeiraX = 65.0;
double posicaoLixeiraY = 65.0;

class Coisa
{
public:
	double x;
	double y;
};

Coisa lixo[QTD_LIXO];
Coisa recicla[QTD_RECICLA];

//obstaculo troncos[QTD_OBSTACULOS];
veiculo* v[QTD_VEICULOS];
//Antenna* antenna;

/* this is called by dSpaceCollide when two objects in space are potentially colliding */
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnected(b1, b2)) {
		//std::cout << "poutz..." << std::endl;
		return;
	}

	dContact contact[QTD_CONTACT];

	if ((dGeomGetClass(o2) == dRayClass) || (dGeomGetClass(o1) == dRayClass))
	{
		/* se for raio, nao faz nada */
	}
	else
	{
		int n = dCollide(o1, o2, QTD_CONTACT, &contact[0].geom, sizeof(dContact));
		if (n > 0)
		{
			for (int i = 0; i < n; i++)
			{
				contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
				if (dGeomGetClass(o1) == dSphereClass || dGeomGetClass(o2) == dSphereClass)
					contact[i].surface.mu = 20;
				else
					contact[i].surface.mu = 0.5;
				contact[i].surface.slip1 = 0.0;
				contact[i].surface.slip2 = 0.0;
				contact[i].surface.soft_erp = 0.8;
				contact[i].surface.soft_cfm = 0.01;
				dJointID c = dJointCreateContact(sim.world, sim.contactgroup, contact + i);
				dJointAttach(c, dGeomGetBody(o1), dGeomGetBody(o2));
			}
		}
	}
}

/* this is called by dSpaceCollide when two objects in space are potentially colliding */
static void nearCallbackRay(void *data, dGeomID o1, dGeomID o2)
{
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnected(b1, b2))
		return;

	dContactGeom contact2[QTD_CONTACT];

	if ((dGeomGetClass(o2) == dRayClass) || (dGeomGetClass(o1) == dRayClass))
	{
		if ((o2 == data) || (o1 == data))
		{
			int n = dCollide(o2, o1, QTD_CONTACT, &contact2[0], sizeof(dContactGeom));
			if (n > 0)
			{
				sim.pContact[sim.cCont][0] = contact2[0].pos[0];
				sim.pContact[sim.cCont][1] = contact2[0].pos[1];
				sim.pContact[sim.cCont][2] = contact2[0].pos[2];

				/* toda primeira vez que entra faz o segundo ponto de contato receber o primeiro */
				if (sim.cCont == 0)
				{
					sim.pContact[1][0] = contact2[0].pos[0];
					sim.pContact[1][1] = contact2[0].pos[1];
					sim.pContact[1][2] = contact2[0].pos[2];
				}

				/* tem um esquema aqui que eh o seguinte:
				 nem sempre o primeiro ponto de contato eh o objeto mais proximo,
				 assim, nao tendo outra solucao mais inteligente, farei o seguinte:
				 ponto 0 eh sempre para-chique do carro
				 ponto 1 eh colisao
				 se tem >1, entao testa a distancia vetorial entre 1 e 2 e troca */
				if (sim.cCont > 1)
				{
					/* se a distancia vetorial do ponto dois for menor que do ponto 1
					 entao troca pontos */
					double a = MyMath::getDistanciaVetorial3d(
							sim.pContact[0][0],
							sim.pContact[0][1],
							sim.pContact[0][2],
							sim.pContact[1][0],
							sim.pContact[1][1],
							sim.pContact[1][2]);

					double b = MyMath::getDistanciaVetorial3d(
							sim.pContact[0][0],
							sim.pContact[0][1],
							sim.pContact[0][2],
							sim.pContact[sim.cCont][0],
							sim.pContact[sim.cCont][1],
							sim.pContact[sim.cCont][2]);

					if (b < a)
					{
						/* troca pontos */
						sim.pContact[1][0] = sim.pContact[sim.cCont][0];
						sim.pContact[1][1] = sim.pContact[sim.cCont][1];
						sim.pContact[1][2] = sim.pContact[sim.cCont][2];
					}
				}
				sim.cCont++;
			}
		}
	}
}

/* start simulation - set viewpoint */
static void start()
{
	static float xyz[3] =
	//{3.8855,-0.1660,0.5600};
	{ 22.538477, -23.605749, 32.130016 };
	//{26.44f,-26.73f,29.89f};
	static float hpr[3] =
	//{59.0000,-13.5000,0.2500};
	{ 89.500000, -39.500000, 0.250000 };
	//{90.0f,-25.0f,0.25f};

	dsSetViewpoint(xyz, hpr);
	printf("Frente,aTras,Direita,Esquerta,GetViewPoint,EpacoZeraMotor\n");
}

/* called when a key pressed */
static void command(int cmd)
{
	switch (cmd)
	{
	case 't':
		v[0]->speed -= 0.3;
		break;
	case 'f':
		v[0]->speed += 0.3;
		break;
	case 'd':
		v[0]->turn += 0.1;
		if (v[0]->turn > 0.8)
			v[0]->turn = 0.8;
		break;
	case 'e':
		v[0]->turn -= 0.1;
		if (v[0]->turn < -0.8)
			v[0]->turn = -0.8;
		break;
	case ' ':
		v[0]->speed = 0;
		v[0]->turn = 0;
		break;
	case 'g':
		float t[3];
		float y[3];
		dsGetViewpoint(t, y);
		printf("%f %f %f %f %f %f\n", t[0], t[1], t[2], y[0], y[1], y[2]);
		break;
	}
}

/* simulation loop */
static void simLoop(int pause)
{
	ros::spinOnce();

	//dsSetTexture (DS_WOOD);
	dsSetTexture(DS_NONE);
	if (!pause)
	{
//#pragma omp parallel for
		for (int j = 0; j < QTD_VEICULOS; j++)
		{
			v[j]->exodus();
		}
		dSpaceCollide(sim.space, 0, sim.bodyCollision);
		dWorldQuickStep(sim.world, 0.05);
		dJointGroupEmpty(sim.contactgroup);
	}

	// desenha bolinha	
	const static dReal g2[] = 	{ 0.938075, 0.346433, 7.49379e-005, 0, -0.346433, 0.938075, -3.62081e-005, 0, -8.2841e-005, 8.00489e-006, 1, 0 };

	// desenha lixeira
	const static dReal quatNinho[] = { 0.999812, -0.0193649, 6.87975e-005, 0, 0.0193649, 0.999812, -9.13112e-005, 0, -6.70163e-005, 9.26263e-005, 1 };

//	antenna->apparition(sim);

	double static sidesNinho[] = { 8.0, 8.0, .1 };
	double static posLixeira[] = { posicaoLixeiraX, posicaoLixeiraY, 0.1 };
	dsSetColor(.8, .3, .3);
	dsDrawBoxD(posLixeira, quatNinho, sidesNinho);

	if (mostraLixo == 1)
	{
		for (int u = 0; u < QTD_LIXO; u++)
		{
			double posCentro[] = { lixo[u].x, lixo[u].y, 0.5 };
			dsDrawSphereD(posCentro, g2, .5);
		}
	}
	// fim desenha ninho

	// desenha reciclagem
	if (mostraReciclagem == 1)
	{
		dsSetColor(.3, .3, .8);
		static double posR[] = { posicaoReciclagemX, posicaoReciclagemY, 0.1 };
		dsDrawBoxD(posR, quatNinho, sidesNinho);

		for (int u = 0; u < QTD_RECICLA; u++)
		{
			double posCentro[] = { recicla[u].x, recicla[u].y, 0.5 };
			dsDrawSphereD(posCentro, g2, .5);
			//recicla[u].x = rand()%30+10;
			//recicla[u].y = rand()%30+10;
		}

	}

	// desenha bateria
	if (mostraBateria == 1)
	{
		dsSetColor(.3, .8, .3);
		static double posB[] = { posicaoBateriaX, posicaoBateriaY, 0.1 };
		dsDrawBoxD(posB, quatNinho, sidesNinho);
	}

	/* draw the veiculo */
	for (int a = 0; a < QTD_VEICULOS; a++)
	{
		v[a]->apparition(sim);
		v[a]->reincarnation(sim);
	}

	//***********************************************************************************************************************
	// create the robot brain here, padawan
	// may the force be with you
	//***********************************************************************************************************************
//#pragma omp parallel for
	for (int i = 0; i < QTD_VEICULOS; i++)
	{
		v[i]->gotobase();
		//v[i].wander();
		//v[i]->reactive();

		double gx, gy;
		v[i]->GetGpsUtmPosition(&gx, &gy);
//		double dBm = antenna->RSSI(gx, gy);
//		v[i]->UpdateSignal(dBm);
		/*
		if(QTD_VEICULOS==1) {
			std::cout << i << ") RSSI=" << dBm << " dBm (" <<
			gx << ":" << gy << ")" <<  "-(" <<
			antenna->PosicaoX << ":" << antenna->PosicaoY << ")" <<
			std::endl;
		}
		*/
	}
	//***********************************************************************************************************************
	// end of robot brain 
	//***********************************************************************************************************************

}
/*
void callbackTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
	// Copy the incoming twist command.
	//twist_ = *msg;
	//last_cmd_ = robot_->getTime();
	v[0]->turn = msg->angular.z;
	//v[0]->speed = msg->linear.x;

	std::cout << "z:" << msg->angular.z << std::endl;
}
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "robombeiros");
	ros::NodeHandle n;

	//ros::Subscriber sub_twist = n.subscribe("cmd_vel", 10, &callbackTwist);

	//srand(66);

	srand(ros::Time::now().nsec);

	/* setup pointers to drawstuff callback functions */
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;

	if(argc>1) {
		std::cout << argc << ":" << argv[1] << std::endl;
		fn.path_to_textures = argv[1];
	}
	else {
		fn.path_to_textures = "textures";
	}

	sim.bodyCollision = &nearCallback;
	sim.rayCollision = &nearCallbackRay;

	/* setup ode */
	dInitODE();
	sim.world = dWorldCreate();
	sim.space = dHashSpaceCreate(0);
	sim.contactgroup = dJointGroupCreate(0);
	dWorldSetGravity(sim.world, 0, 0, -9.8);
	dWorldSetCFM(sim.world, 1e-5);
	dWorldSetERP(sim.world, 0.8);
	dWorldSetQuickStepNumIterations(sim.world, 30);
	sim.ground = dCreatePlane(sim.space, 0, 0, 1, 0);

	/* setup meus objetos */
	for (int u = 0; u < QTD_LIXO; u++)
	{
		lixo[u].x = rand() % 30 + 10;
		lixo[u].y = rand() % 30 + 10;
	}
	for (int u = 0; u < QTD_RECICLA; u++)
	{
		recicla[u].x = rand() % 30 + 10;
		recicla[u].y = rand() % 30 + 10;
	}

//	antenna = new Antenna(); //n
//	antenna->genesis(sim.world, sim.space);

	for (int i = 0; i < QTD_VEICULOS; i++) {
		v[i] = new veiculo();
		v[i]->genesis(sim.world, sim.space);
	}

	/* run simulaion */
	dsSimulationLoop(argc, argv, 1024, 768, &fn);

	/* close ode */
	dJointGroupDestroy(sim.contactgroup);
	dSpaceDestroy(sim.space);
	dWorldDestroy(sim.world);
	dCloseODE();
	return 0;
}
