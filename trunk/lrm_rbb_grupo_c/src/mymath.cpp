#include <iostream>
#include <math.h>
#include <stdlib.h>
#include "mymath.h"
#include "constantes.h"

MyMath::MyMath()
{
}

double MyMath::getDistanciaVetorial3d(double x1, double y1, double z1, double x2, double y2, double z2)
{
	double p1 = ((x2 - x1) * (x2 - x1));
	double p2 = ((y2 - y1) * (y2 - y1));
	double p3 = ((z2 - z1) * (z2 - z1));
	double p4 = sqrt(p1 + p2 + p3);

	return (p4);
}