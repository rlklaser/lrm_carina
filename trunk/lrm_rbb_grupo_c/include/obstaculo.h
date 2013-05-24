#include <ode/ode.h>

class obstaculo
{
public:
	obstaculo();
	void genesisEsfera(dWorldID world, dSpaceID space, float x, float y);
	void genesisCilindro(dWorldID world, dSpaceID space);

	dGeomID geom;
	dBodyID body;
	dReal raio;
	dReal massa;
	dReal altura;
};
