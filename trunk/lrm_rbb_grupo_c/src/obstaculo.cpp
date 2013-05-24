#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <ode/ode.h>
#include "obstaculo.h"
#include "constantes.h"

obstaculo::obstaculo()
{
}

void obstaculo::genesisEsfera(dWorldID world, dSpaceID space, float x, float y)
{
	dMass m;

	raio = dReal(.6);
	massa = 10.0;
	body = dBodyCreate(world);
	geom = dCreateSphere(space, raio);
	dMassSetSphereTotal(&m, massa, raio);
	dBodySetMass(body, &m);
	dGeomSetBody(geom, body);
	dBodySetPosition(body, x, y, raio);
}

void obstaculo::genesisCilindro(dWorldID world, dSpaceID space)
{
	dReal x = float(rand() % 80) + OFFSET_POS_CILINDRO;
	dReal y = float(rand() % 100) - 50;
	dMass m;

	raio = 1.0;
	massa = 20.0;
	altura = 9.0;

	body = dBodyCreate(world);
	dMassSetSphere(&m, 1, raio);
	dMassAdjust(&m, massa);
	dBodySetMass(body, &m);
	geom = dCreateCylinder(space, raio, altura);
	dGeomSetBody(geom, body);
	dBodySetPosition(body, x, y, altura);
}