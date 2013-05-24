#include <ode/ode.h>
#include "constantes.h"

class MyMath
{
public:
	MyMath();
	static double getDistanciaVetorial2d(double x1, double y1, double x2, double y2);
	static double getDistanciaVetorial3d(double x1, double y1, double z1, double x2, double y2, double z2);
};
