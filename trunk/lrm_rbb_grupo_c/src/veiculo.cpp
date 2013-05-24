#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <ode/ode.h>

#include "veiculo.h"
#include "constantes.h"
#include "bussola.h"
#include "mymath.h"

void veiculo::callbackTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
	//o comando reativo precede
	if(!_reactive) {
		turn = -saturation(msg->angular.z, -0.95, 0.95);
		speed = saturation(msg->linear.x, -5, 5);
		//std::cout << "z:" << msg->angular.z << std::endl;
	}
}

veiculo::veiculo()
: destinoX(0), destinoY(0), _reactive(false)
{
	//veiculo::id++;
	veiculo_id++;
	std::ostringstream topic_odom;
	std::ostringstream topic_vel;
	std::ostringstream topic_signal;
	std::ostringstream topic_god;
	std::ostringstream obase_link;

	topic_odom << "vehicle/" << veiculo_id << "/odom";
	topic_vel << "vehicle/" << veiculo_id << "/cmd_vel";
	topic_signal << "vehicle/" << veiculo_id << "/signal";
	topic_god << "vehicle/" << veiculo_id << "/ground_truth";
	obase_link << "vehicle/" << veiculo_id << "/base_link";

	_odom_pub = _nh.advertise<nav_msgs::Odometry>(topic_odom.str(), 2);
	//_signal_pub = _nh.advertise<lrm_rbb_grupo_c::Signal>(topic_signal.str(), 2);
	_god_pose_pub = _nh.advertise<geometry_msgs::Pose2D>(topic_god.str(), 2);
	_sub_twist = _nh.subscribe(topic_vel.str(), 2, &veiculo::callbackTwist, this);

	world_link = "/world";
	odom_link = "/" + topic_odom.str();
	base_link = "/" + obase_link.str();
}

void veiculo::genesis(dWorldID world, dSpaceID space)
{
	minhaPosicaoX = float(rand() % 50); // minha posicao
	minhaPosicaoY = float(rand() % 50); // minha posicao

	destinoX = (float(rand() % 200)) - 100.0;
	destinoY = (float(rand() % 200)) - 100.0;

	dMass m;

	/* chassis body */
	veicBody = dBodyCreate(world);
	dBodySetPosition(veicBody, minhaPosicaoX, minhaPosicaoY, STARTZ);
	dMassSetBox(&m, 1, COMPRIMENTO, LARGURA, ALTURA);
	dMassAdjust(&m, 1 / 2.0);
	dBodySetMass(veicBody, &m);
	veicGeom = dCreateBox(space, COMPRIMENTO, LARGURA, ALTURA);
	dGeomSetBody(veicGeom, veicBody);

	/* raios */
	for (int t = 0; t < QTD_RAIO; t++)
		raioGeom[t] = dCreateRay(space, TAM_RAIO);

	/* suportes */
	for (int t = 0; t < QTD_SUPORTE; t++)
	{
		supBody[t] = dBodyCreate(world);
		supGeom[t] = dCreateBox(space, dReal(TAM_SUPORTE), dReal(TAM_SUPORTE), dReal(TAM_SUPORTE));
		dMassSetBox(&m, dReal(0.0001), dReal(0.0001), dReal(0.0001), dReal(0.0001)); // esse eh o cara pra para os tremelique!
		//dMassSetZero(&m);
		dBodySetMass(supBody[t], &m);
		dGeomSetBody(supGeom[t], supBody[t]);
	}

	int x = minhaPosicaoX;
	int y = minhaPosicaoY;

	dBodySetPosition(supBody[0], dReal(x + 0.4 * COMPRIMENTO), dReal(y + LARGURA * 0.52), dReal(ALTURA + 0.250));
	dBodySetPosition(supBody[1], dReal(x + 0.12 * COMPRIMENTO), dReal(y + LARGURA * 0.4), dReal(ALTURA + 0.253));
	dBodySetPosition(supBody[2], dReal(x + 0.24 * COMPRIMENTO), dReal(y + LARGURA * 0.4), dReal(ALTURA + 0.253));
	dBodySetPosition(supBody[3], dReal(x + 0.30 * COMPRIMENTO), dReal(y + LARGURA * 0.4), dReal(ALTURA + 0.253));

	dBodySetPosition(supBody[4], dReal(x + 0.52 * COMPRIMENTO), dReal(y + LARGURA * 0.5), dReal(ALTURA + 0.250));
	dBodySetPosition(supBody[5], dReal(x + 0.2 * COMPRIMENTO), dReal(y + LARGURA * 0.5), dReal(ALTURA + 0.255));
	dBodySetPosition(supBody[6], dReal(x + 0.2 * COMPRIMENTO), dReal(y + LARGURA * 0.40), dReal(ALTURA + 0.255));
	dBodySetPosition(supBody[7], dReal(x + 0.2 * COMPRIMENTO), dReal(y + LARGURA * 0.45), dReal(ALTURA + 0.255));

	dBodySetPosition(supBody[8], dReal(x + 0.52 * COMPRIMENTO), dReal(y), dReal(ALTURA + 0.250));
	dBodySetPosition(supBody[9], dReal(x + 0.38 * COMPRIMENTO), dReal(y), dReal(ALTURA + 0.253));
	dBodySetPosition(supBody[10], dReal(x + 0.38 * COMPRIMENTO), dReal(y + 0.05), dReal(ALTURA + 0.253));
	dBodySetPosition(supBody[11], dReal(x + 0.38 * COMPRIMENTO), dReal(y - 0.05), dReal(ALTURA + 0.253));

	dBodySetPosition(supBody[12], dReal(x + 0.52 * COMPRIMENTO), dReal(y - LARGURA * 0.5), dReal(ALTURA + 0.250));
	dBodySetPosition(supBody[13], dReal(x + 0.2 * COMPRIMENTO), dReal(y - LARGURA * 0.5), dReal(ALTURA + 0.255));
	dBodySetPosition(supBody[14], dReal(x + 0.2 * COMPRIMENTO), dReal(y - LARGURA * 0.40), dReal(ALTURA + 0.255));
	dBodySetPosition(supBody[15], dReal(x + 0.2 * COMPRIMENTO), dReal(y - LARGURA * 0.45), dReal(ALTURA + 0.255));

	dBodySetPosition(supBody[16], dReal(x + 0.4 * COMPRIMENTO), dReal(y - LARGURA * 0.52), dReal(ALTURA + 0.250));
	dBodySetPosition(supBody[17], dReal(x + 0.12 * COMPRIMENTO), dReal(y - LARGURA * 0.4), dReal(ALTURA + 0.253));
	dBodySetPosition(supBody[18], dReal(x + 0.24 * COMPRIMENTO), dReal(y - LARGURA * 0.4), dReal(ALTURA + 0.253));
	dBodySetPosition(supBody[19], dReal(x + 0.30 * COMPRIMENTO), dReal(y - LARGURA * 0.4), dReal(ALTURA + 0.253));

	dBodySetPosition(supBody[20], dReal(x + 0.35 * COMPRIMENTO), dReal(y + LARGURA * 0.52), dReal(ALTURA + 0.250));
	dBodySetPosition(supBody[21], dReal(x + 0.28 * COMPRIMENTO), dReal(y + LARGURA * 0.4), dReal(ALTURA + 0.253));
	dBodySetPosition(supBody[22], dReal(x + 0.30 * COMPRIMENTO), dReal(y + LARGURA * 0.4), dReal(ALTURA + 0.253));
	dBodySetPosition(supBody[23], dReal(x + 0.32 * COMPRIMENTO), dReal(y + LARGURA * 0.4), dReal(ALTURA + 0.253));

	dBodySetPosition(supBody[24], dReal(x + 0.35 * COMPRIMENTO), dReal(y - LARGURA * 0.52), dReal(ALTURA + 0.250));
	dBodySetPosition(supBody[25], dReal(x + 0.28 * COMPRIMENTO), dReal(y - LARGURA * 0.4), dReal(ALTURA + 0.253));
	dBodySetPosition(supBody[26], dReal(x + 0.30 * COMPRIMENTO), dReal(y - LARGURA * 0.4), dReal(ALTURA + 0.253));
	dBodySetPosition(supBody[27], dReal(x + 0.32 * COMPRIMENTO), dReal(y - LARGURA * 0.4), dReal(ALTURA + 0.253));

	for (int t = 0; t < QTD_SUPORTE; t++)
	{
		supJoint[t] = dJointCreateFixed(world, 0);
		dJointAttach(supJoint[t], veicBody, supBody[t]);
		dJointSetFixed(supJoint[t]);
	}

	/* wheel bodies */
	for (int a = 0; a < QTD_RODAS; a++)
	{
		rodaBody[a] = dBodyCreate(world);
		dQuaternion q;
		dQFromAxisAndAngle(q, 1, 0, 0, M_PI * 0.5);
		dBodySetQuaternion(rodaBody[a], q);
		dMassSetSphere(&m, 1, RAIO_RODA);
		dMassAdjust(&m, 1);
		dBodySetMass(rodaBody[a], &m);
		rodaGeom[a] = dCreateSphere(space, RAIO_RODA);
		//rodaGeom[a] = dCreateCylinder(space, RAIO_RODA, 0.1);
		dGeomSetBody(rodaGeom[a], rodaBody[a]);
	}

	dBodySetPosition(rodaBody[0], dReal(x + 0.4 * COMPRIMENTO - 0.5 * RAIO_RODA), dReal(y + LARGURA * 0.5), dReal(0.25));
	dBodySetPosition(rodaBody[1], dReal(x + 0.4 * COMPRIMENTO - 0.5 * RAIO_RODA), dReal(y - LARGURA * 0.5), dReal(0.25));
	dBodySetPosition(rodaBody[2], dReal(x - 0.4 * COMPRIMENTO + 0.5 * RAIO_RODA), dReal(y + LARGURA * 0.5), dReal(0.25));
	dBodySetPosition(rodaBody[3], dReal(x - 0.4 * COMPRIMENTO + 0.5 * RAIO_RODA), dReal(y - LARGURA * 0.5), dReal(0.25));

	/* front and back wheel hinges */
	for (int i = 0; i < QTD_RODAS; i++)
	{
		if (i < 2)
		{ /* rodas da frente, torque e giro */
			rodaJoint[i] = dJointCreateHinge2(world, 0);
			dJointAttach(rodaJoint[i], veicBody, rodaBody[i]);
			const dReal *a = dBodyGetPosition(rodaBody[i]);
			dJointSetHinge2Anchor(rodaJoint[i], a[0], a[1], a[2]);
			dJointSetHinge2Axis1(rodaJoint[i], 0, 0, 1);
			dJointSetHinge2Axis2(rodaJoint[i], 0, 1, 0);
			dJointSetHinge2Param(rodaJoint[i], dParamSuspensionERP, dReal(0.8));
			dJointSetHinge2Param(rodaJoint[i], dParamSuspensionCFM, dReal(1e-5));
			dJointSetHinge2Param(rodaJoint[i], dParamVel2, 0);
			dJointSetHinge2Param(rodaJoint[i], dParamFMax2, 25);
			//dJointSetHinge2Param(rodaJoint[i], dParamLoStop, -0.96);
			//dJointSetHinge2Param(rodaJoint[i], dParamHiStop, 0.96);
		}
		if (i >= 2)
		{ /* rodas de traz, torque mas nao gira */
			rodaJoint[i] = dJointCreateHinge2(world, 0);
			dJointAttach(rodaJoint[i], veicBody, rodaBody[i]);
			const dReal *a = dBodyGetPosition(rodaBody[i]);
			dJointSetHinge2Anchor(rodaJoint[i], a[0], a[1], a[2]);
			dJointSetHinge2Axis1(rodaJoint[i], 0, 0, 1);
			dJointSetHinge2Axis2(rodaJoint[i], 0, 1, 0);
			dJointSetHinge2Param(rodaJoint[i], dParamSuspensionERP,  dReal(0.8));
			dJointSetHinge2Param(rodaJoint[i], dParamSuspensionCFM, dReal(1e-5));
			dJointSetHinge2Param(rodaJoint[i], dParamVel2, 0);
			dJointSetHinge2Param(rodaJoint[i], dParamFMax2, 25);
			dJointSetHinge2Param(rodaJoint[i], dParamLoStop, 0);
			dJointSetHinge2Param(rodaJoint[i], dParamHiStop, 0);
		}
	}

	/* center of mass offset body. (hang another copy of the body COMOFFSET units below it by a fixed joint) */
	dBodyID b = dBodyCreate(world);
	dBodySetPosition(b, x, y, STARTZ + COMOFFSET);
	dMassSetBox(&m, 1, COMPRIMENTO, LARGURA, ALTURA);
	dMassAdjust(&m, 1 / 2.0);
	dBodySetMass(b, &m);
	dJointID j = dJointCreateFixed(world, 0);
	dJointAttach(j, veicBody, b);
	dJointSetFixed(j);
}

void veiculo::apparition(struct simulation& sim)
{
	dsSetColor(.5, .8, .5);
	dsDrawBoxD(dGeomGetPosition(veicGeom), dGeomGetRotation(veicGeom), sides);

	dsSetColor(1, 1, 1);
	// rodas
	for (int b = 0; b < QTD_RODAS; b++) {
		dsDrawSphereD(dGeomGetPosition(rodaGeom[b]), dGeomGetRotation(rodaGeom[b]), RAIO_RODA);
		//dsDrawCylinderD(dGeomGetPosition(rodaGeom[b]), dGeomGetRotation(rodaGeom[b]), 0.1, RAIO_RODA);
	}
	dsSetColor(1, 1, 0);
	for (int t = 0; t < QTD_SUPORTE; t++) {
		dsDrawBoxD(dBodyGetPosition(supBody[t]), dBodyGetRotation(supBody[t]), ls);
	}
}

//test collisions
void veiculo::reincarnation(struct simulation& sim)
{
	/* reposiciona raios */
	for (int r = 0; r < QTD_SUPORTE; r++) {
		dBodyCopyPosition(supBody[r], p[r]);
	}
/*
	RAY_GEOM(X, Y)
		dGeomRaySet(raioGeom[X-3], p[Y][0], p[0][1], p[0][2], p[0][0] - p[2][0], p[0][1] - p[2][1], p[0][2] - p[2][2]);
		dGeomRaySet(raioGeom[X-2], p[Y][0], p[0][1], p[0][2], p[0][0] - p[1][0], p[0][1] - p[1][1], p[0][2] - p[1][2]);
		dGeomRaySet(raioGeom[X-1], p[Y][0], p[0][1], p[0][2], p[0][0] - p[3][0], p[0][1] - p[3][1], p[0][2] - p[3][2]);
*/
	dGeomRaySet(raioGeom[0], p[0][0], p[0][1], p[0][2], p[0][0] - p[2][0], p[0][1] - p[2][1], p[0][2] - p[2][2]);
	dGeomRaySet(raioGeom[1], p[0][0], p[0][1], p[0][2], p[0][0] - p[1][0], p[0][1] - p[1][1], p[0][2] - p[1][2]);
	dGeomRaySet(raioGeom[2], p[0][0], p[0][1], p[0][2], p[0][0] - p[3][0], p[0][1] - p[3][1], p[0][2] - p[3][2]);

	dGeomRaySet(raioGeom[3], p[4][0], p[4][1], p[4][2], p[4][0] - p[7][0], p[4][1] - p[7][1], p[4][2] - p[7][2]);
	dGeomRaySet(raioGeom[4], p[4][0], p[4][1], p[4][2], p[4][0] - p[5][0], p[4][1] - p[5][1], p[4][2] - p[5][2]);
	dGeomRaySet(raioGeom[5], p[4][0], p[4][1], p[4][2], p[4][0] - p[6][0], p[4][1] - p[6][1], p[4][2] - p[6][2]);

	dGeomRaySet(raioGeom[6], p[8][0], p[8][1], p[8][2], p[8][0] - p[11][0], p[8][1] - p[11][1], p[8][2] - p[11][2]);
	dGeomRaySet(raioGeom[7], p[8][0], p[8][1], p[8][2], p[8][0] - p[9][0], p[8][1] - p[9][1], p[8][2] - p[9][2]);
	dGeomRaySet(raioGeom[8], p[8][0], p[8][1], p[8][2], p[8][0] - p[10][0], p[8][1] - p[10][1], p[8][2] - p[10][2]);

	dGeomRaySet(raioGeom[9], p[12][0], p[12][1], p[12][2], p[12][0] - p[13][0], p[12][1] - p[13][1], p[12][2] - p[13][2]);
	dGeomRaySet(raioGeom[10], p[12][0], p[12][1], p[12][2], p[12][0] - p[14][0], p[12][1] - p[14][1], p[12][2] - p[14][2]);
	dGeomRaySet(raioGeom[11], p[12][0], p[12][1], p[12][2], p[12][0] - p[15][0], p[12][1] - p[15][1], p[12][2] - p[15][2]);

	dGeomRaySet(raioGeom[12], p[16][0], p[16][1], p[16][2], p[16][0] - p[17][0], p[16][1] - p[17][1], p[16][2] - p[17][2]);
	dGeomRaySet(raioGeom[13], p[16][0], p[16][1], p[16][2], p[16][0] - p[18][0], p[16][1] - p[18][1], p[16][2] - p[18][2]);
	dGeomRaySet(raioGeom[14], p[16][0], p[16][1], p[16][2], p[16][0] - p[19][0], p[16][1] - p[19][1], p[16][2] - p[19][2]);

	dGeomRaySet(raioGeom[15], p[20][0], p[20][1], p[20][2], p[20][0] - p[21][0], p[20][1] - p[21][1], p[20][2] - p[21][2]);
	dGeomRaySet(raioGeom[16], p[20][0], p[20][1], p[20][2], p[20][0] - p[22][0], p[20][1] - p[22][1], p[20][2] - p[22][2]);
	dGeomRaySet(raioGeom[17], p[20][0], p[20][1], p[20][2], p[20][0] - p[23][0], p[20][1] - p[23][1], p[20][2] - p[23][2]);

	dGeomRaySet(raioGeom[18], p[24][0], p[24][1], p[24][2], p[24][0] - p[25][0], p[24][1] - p[25][1], p[24][2] - p[25][2]);
	dGeomRaySet(raioGeom[19], p[24][0], p[24][1], p[24][2], p[24][0] - p[26][0], p[24][1] - p[26][1], p[24][2] - p[26][2]);
	dGeomRaySet(raioGeom[20], p[24][0], p[24][1], p[24][2], p[24][0] - p[27][0], p[24][1] - p[27][1], p[24][2] - p[27][2]);

	/* draw os raios do veiculo e testa colisao dos raios */
	for (int b = 0; b < QTD_RAIO; b++)
	{
		dSpaceCollide(sim.space, raioGeom[b], sim.rayCollision);

		double d = MyMath::getDistanciaVetorial3d(
				sim.pContact[0][0],
				sim.pContact[0][1],
				sim.pContact[0][2],
				sim.pContact[1][0],
				sim.pContact[1][1],
				sim.pContact[1][2]);

		dsSetColor(.5, .5, .5);
		dVector3 tpos1 = { sim.pContact[0][0], sim.pContact[0][1], sim.pContact[0][2] };
		dsDrawSphereD(tpos1, dGeomGetRotation(veicGeom), RAIO_PCONTATO);

		dsSetColor(1, 1, 0);
		dVector3 tpos2 ={ sim.pContact[1][0], sim.pContact[1][1], sim.pContact[1][2] };
		dsDrawSphereD(tpos2, dGeomGetRotation(veicGeom), RAIO_PCONTATO);

		sim.cCont = 0;

		dGeomRayGet(raioGeom[b], origin, dir);
		if (d < 0.00002)
			d = TAM_RAIO; /* duvidas? pessin@gmail.com */
		for (int j = 0; j < 3; j++)
			dir[j] = dir[j] * d + origin[j];
		dsSetColor(1, 1, 1);
		dsDrawLineD(origin, dir);

		raiosDistancias[b] = d;
	}

	/* arruma info dos senroses (acumula raios), assim testa apenas 0,3,6,9,12,15,18*/
	if (raiosDistancias[0] > raiosDistancias[1])
		raiosDistancias[0] = raiosDistancias[1];
	if (raiosDistancias[0] > raiosDistancias[2])
		raiosDistancias[0] = raiosDistancias[2];

	if (raiosDistancias[3] > raiosDistancias[4])
		raiosDistancias[3] = raiosDistancias[4];
	if (raiosDistancias[3] > raiosDistancias[5])
		raiosDistancias[3] = raiosDistancias[5];

	if (raiosDistancias[6] > raiosDistancias[7])
		raiosDistancias[6] = raiosDistancias[7];
	if (raiosDistancias[6] > raiosDistancias[8])
		raiosDistancias[6] = raiosDistancias[8];

	if (raiosDistancias[9] > raiosDistancias[10])
		raiosDistancias[9] = raiosDistancias[10];
	if (raiosDistancias[9] > raiosDistancias[11])
		raiosDistancias[9] = raiosDistancias[11];

	if (raiosDistancias[12] > raiosDistancias[13])
		raiosDistancias[12] = raiosDistancias[13];
	if (raiosDistancias[12] > raiosDistancias[14])
		raiosDistancias[12] = raiosDistancias[14];

	if (raiosDistancias[15] > raiosDistancias[16])
		raiosDistancias[15] = raiosDistancias[16];
	if (raiosDistancias[15] > raiosDistancias[17])
		raiosDistancias[15] = raiosDistancias[17];

	if (raiosDistancias[18] > raiosDistancias[19])
		raiosDistancias[18] = raiosDistancias[19];
	if (raiosDistancias[18] > raiosDistancias[20])
		raiosDistancias[18] = raiosDistancias[20];
}

void veiculo::exodus()
{
	_reactive = false; //will move, reset states

	for (int i = 0; i < QTD_RODAS; i++)
	{
		if (i < 2)
		{ /* rodas de traz, torque e giro */
			dReal curturn = dJointGetHinge2Angle1(rodaJoint[i]);
			dJointSetHinge2Param(rodaJoint[i], dParamVel, (turn - curturn) * 1.0);
			dJointSetHinge2Param(rodaJoint[i], dParamFMax, dInfinity);
			dJointSetHinge2Param(rodaJoint[i], dParamVel2, (speed) * (-1));
			dJointSetHinge2Param(rodaJoint[i], dParamFMax2, 25);
			dBodyEnable(dJointGetBody(rodaJoint[i], 0));
			dBodyEnable(dJointGetBody(rodaJoint[i], 1));
		}
		if (i >= 2)
		{ /* rodas de traz, torque mas nao gira */
			//dReal curturn = dJointGetHinge2Angle1(v[j].rodaJoint[i]);
			dJointSetHinge2Param(rodaJoint[i], dParamVel, 0);
			dJointSetHinge2Param(rodaJoint[i], dParamFMax, dInfinity);
			dJointSetHinge2Param(rodaJoint[i], dParamVel2, (speed) * (-1));
			dJointSetHinge2Param(rodaJoint[i], dParamFMax2, 25);
			dBodyEnable(dJointGetBody(rodaJoint[i], 0));
			dBodyEnable(dJointGetBody(rodaJoint[i], 1));
		}
	}

	ros::Time current_time = ros::Time::now();

	//world reference frame (where born)
	//cannot be used by the robot!!!!
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(minhaPosicaoX, minhaPosicaoY, 0.0));
	transform.setRotation(tf::createQuaternionFromYaw(0.0));
	_odom_broadcaster.sendTransform(tf::StampedTransform(transform, current_time, world_link/* "/world"*/, odom_link /*"odom"*/));

	////////////////
	//God's odometry
	////////////////
	dReal pos[3];
	//dQuaternion qt;
	dGeomCopyPosition(veicGeom, pos);
	//dGeomGetQuaternion(veicGeom, qt);


	double yaw = GetBussola()-90; //muda para direcionar para o "sul"
	//pow, em graus Pessin !?!?!
	yaw = -(yaw/360) * (2*M_PI);

	//std::cout << "x:" << qt[0] << " y:" << qt[1] << " z:" << qt[2] << " w:" << qt[3] << std::endl;
	//std::cout << yaw/M_PI*180 << std::endl;

	//a odometria nao sabe onde o veiculo nasceu...comeca de zero.
	//God's odometry != Thief's odometry
	_pose_x = pos[0] - minhaPosicaoX;
	_pose_y = pos[1] - minhaPosicaoY;
	_pose_theta = yaw;

	geometry_msgs::Quaternion q =  tf::createQuaternionMsgFromYaw(yaw);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = odom_link; //"/odom";
	odom_trans.child_frame_id = base_link;// "/base_link";
	odom_trans.transform.translation.x = _pose_x;
	odom_trans.transform.translation.y = _pose_y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = q;
	_odom_broadcaster.sendTransform(odom_trans);

	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id=odom_link;// "/odom";
	odom.child_frame_id =base_link;// "/base_link";
	odom.pose.pose.position.x = _pose_x;
	odom.pose.pose.position.y = _pose_y;
	odom.pose.pose.position.z = 0 ;
	odom.pose.pose.orientation = q;
	odom.twist.twist.linear.x = speed;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.angular.z = turn;
	_odom_pub.publish(odom);


	//Now WE CAN!
	//Ground truth: God's view of the robot
	_ground_truth_pose.x = pos[0];
	_ground_truth_pose.y = pos[1];
	_ground_truth_pose.theta = yaw;
	_god_pose_pub.publish(_ground_truth_pose);
}

double veiculo::GetBussola()
{
	dReal posrf[3];
	dReal posrt[3];

	dGeomCopyPosition(rodaGeom[0], posrf);
	dGeomCopyPosition(rodaGeom[2], posrt);

	double xi = posrf[0];
	double yi = posrf[1];
	double xf = posrt[0];
	double yf = posrt[1];

	double ang = 0.0;
	ang = Bussola::GetMyAngle(xi, yi, xf, yf);
	/*std::cout << xi << " " << yi << " " << xf << " " << yf << "\n";*/
	return (ang);
}

double veiculo::GetAngleToEndpoint(double AlvoX, double AlvoY)
{
	dReal pos[3];

	dGeomCopyPosition(rodaGeom[2], pos);

	double MyBodyX = pos[0];
	double MyBodyY = pos[1];

	double ang = 0.0;
	ang = Bussola::GetMyAngle(AlvoX, AlvoY, MyBodyX, MyBodyY);
	return (ang);
}

void veiculo::GetGpsUtmPosition(double *gx, double *gy)
{
	dReal pos[3];

	dGeomCopyPosition(veicGeom, pos);

	(*gx) = pos[0];
	(*gy) = pos[1];
}

void veiculo::wander()
{
	double veicAngle = GetBussola();
	double gx, gy;
	GetGpsUtmPosition(&gx, &gy);

	double dAzimute = GetAngleToEndpoint(destinoX, destinoY);

	float egps = 18.0; // erro_gps

	if ((gx > destinoX + egps) || (gx < destinoX - egps) || (gy > destinoY + egps) || (gy < destinoY - egps))
	{
		speed = 2.0;
	}
	else
	{
		destinoX = (float(rand() % 200)) - 100.0;
		destinoY = (float(rand() % 200)) - 100.0;
	}

	/* calcula angulo oposto */
	float fAnguloOposto = 180.0 + veicAngle;
	if (fAnguloOposto > 360.0)
		fAnguloOposto -= 360.0;

	double fDif = veicAngle - dAzimute; // pega diferenca para "suavizar" curva
	if (fDif < 0)
		fDif *= -1;

	if (1 == 1)
	{
		turn = 0;

		if ((gx > destinoX + egps) || (gx < destinoX - egps) || (gy > destinoY + egps) || (gy < destinoY - egps))
		{
			if (((dAzimute > veicAngle) && (dAzimute < fAnguloOposto) && (fAnguloOposto > veicAngle)) || (!(dAzimute > veicAngle) && (dAzimute < fAnguloOposto) && !(fAnguloOposto > veicAngle))
					|| ((dAzimute > veicAngle) && !(dAzimute < fAnguloOposto) && !(fAnguloOposto > veicAngle)))/*{*/
			{
				if (fDif < 5)
					turn = 0.1;
				else
					turn = 0.5;
			}

			else
			{
				if (fDif < 5)
					turn = -0.1;
				else
					turn = -0.5;
			}
		}

	}
}

void veiculo::reactive()
{
	//bool changed = false;
	/*desvio*/
	if (raiosDistancias[18] < 3 /*4*/) {
		turn = -0.6; /* ok para NE,SE e SO*/
		_reactive = true;
	}
	if (raiosDistancias[15] < 3 /*4*/) {
		turn = 0.6; /* ok para NE,SE e SO*/
		_reactive = true;
	}
	if ((raiosDistancias[18] < 3 /*4*/) && (raiosDistancias[15] < 4.0)) {
		turn = 0.0;
		_reactive = true;
	}
	if (raiosDistancias[12] < 4.5 /*6*/) {
		turn = -0.6; /* ok para NE,SE e SO*/
		_reactive = true;
	}
	if (raiosDistancias[0] < 4.5 /*6*/) {
		turn = 0.6; /* ok para NE,SE e SO*/
		_reactive = true;
	}
	if ((raiosDistancias[0] < 4.5 /*6*/) && (raiosDistancias[12] < 6)) {
		turn = 0.0;
		_reactive = true;
	}
	if (raiosDistancias[9] < 6 /*8*/) {
		turn = -0.6; /* ok para NE,SE e SO*/
		_reactive = true;
	}
	if (raiosDistancias[3] < 6 /*8*/) {
		turn = 0.6; /* ok para NE,SE e SO*/
		_reactive = true;
	}
	if ((raiosDistancias[3] < 6 /*8*/) && (raiosDistancias[9] < 8)) {
		turn = 0.0;
		_reactive = true;
	}
	if (raiosDistancias[9] < raiosDistancias[3]) {
		turn = -0.2;
		_reactive = true;
	}
	if (raiosDistancias[3] < raiosDistancias[9]) {
		turn = 0.2;
		_reactive = true;
	}
	if (raiosDistancias[18] < 1.6) {
		turn = -0.6; /* ok para NE,SE e SO*/
		_reactive = true;
	}
	if (raiosDistancias[15] < 1.6) {
		turn = 0.6; /* ok para NE,SE e SO*/
		_reactive = true;
	}
	if (raiosDistancias[12] < 1.6) {
		turn = -0.6; /* ok para NE,SE e SO*/
		_reactive = true;
	}
	if (raiosDistancias[0] < 1.6) {
		turn = 0.6; /* ok para NE,SE e SO*/
		_reactive = true;
	}
	if ((raiosDistancias[6] < 3 /*4*/) && (raiosDistancias[3] < 3 /*4*/) && (raiosDistancias[9] < 3 /*4*/))
	{
		if ((raiosDistancias[18] < raiosDistancias[15]) || (raiosDistancias[12] < raiosDistancias[0])) {
			turn = -0.6;
			_reactive = true;
		}
		else {
			turn = 0.6;
			_reactive = true;
		}
	}

	//if(_reactive) std::cout << "pan pan pan..., mayday, mayday!" << std::endl;
}

void veiculo::UpdateSignal(double rssi)
{
	_rssi = rssi;
	lrm_rbb_grupo_c::Signal msg;

	msg.header.stamp = ros::Time::now();
	msg.rssi = rssi;
	msg.pose.x = _pose_x;
	msg.pose.y = _pose_x;
	msg.pose.theta = _pose_x;

	//_signal_pub.publish(msg);
}

void veiculo::gotobase()
{
	//double gx, gy;
	//GetGpsUtmPosition(&gx, &gy);

	/*
	double dAzimute = GetAngleToEndpoint(destinoX, destinoY);

	float egps = 18.0; // erro_gps

	if ((gx > destinoX + egps) || (gx < destinoX - egps) || (gy > destinoY + egps) || (gy < destinoY - egps))
	{
		speed = 0.5;
	}
	else
	{
		destinoX = (float(rand() % 200)) - 100.0;
		destinoY = (float(rand() % 200)) - 100.0;
	}
	*/
	//speed = 0;
	//turn = 0;
}
