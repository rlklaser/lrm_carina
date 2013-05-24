#include <iostream>
#include <math.h>
#include <ode/ode.h>
#include "bussola.h"

double Bussola::GetMyAngle(double xi, double yi, double xf, double yf)
{

	if (yf == yi)
		yi += 0.0001;
	if (xf == xi)
		xi += 0.0001;

	double AngR = atan((yf - yi) / (xf - xi));
	double AngG = (AngR / (2 * M_PI)) * 360;

	/* 0 == x, trigonometria */
	//if ((xi<xf)&&(yi<yf)) return 180+AngG;
	//if ((xi>xf)&&(yi>yf)) return AngG;
	//if ((xi>xf)&&(yi<yf)) return 360+AngG;
	//if ((xi<xf)&&(yi>yf)) return 180+AngG;
	/* isso eh uma bussola de verdade, 0 == norte [y] e angulos aumentam em sentido horario  */
	if ((xi < xf) && (yi < yf))
		return 360 - (90 + AngG);
	if ((xi > xf) && (yi > yf))
		return 360 - (270 + AngG);
	if ((xi > xf) && (yi < yf))
		return 360 - (270 + AngG);
	if ((xi < xf) && (yi > yf))
		return 360 - (90 + AngG);

	/* nunca entra aqui, mas... */
	return 0;
}
