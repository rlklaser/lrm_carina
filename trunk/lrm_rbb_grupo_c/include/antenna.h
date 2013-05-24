/*
 *  Copyright (C) 2012, Laboratorio de Robotica Movel - ICMC/USP
 *  Rafael Luiz Klaser <rlklaser@gmail.com>
 *  http://lrm.icmc.usp.br
 *
 *  Apoio FAPESP: 2012/04555-4
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file antenna.h
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 26, 2012
 *
 */

#ifndef ANTENNA_H_
#define ANTENNA_H_

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <ns3/ptr.h>
#include <ns3/core-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/jakes-propagation-loss-model.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/buildings-propagation-loss-model.h>
#include <ns3/propagation-environment.h>
#include <ns3/yans-wifi-phy.h>

#include "constantes.h"


using namespace ns3;

class Antenna //: public Spirit
{
private:
	 Ptr<ConstantPositionMobilityModel> _a;
	 Ptr<ConstantPositionMobilityModel> _b;
	 Ptr<PropagationLossModel> _model;
	 Ptr<YansWifiPhy> _wifi_phy;

public:

	dBodyID body;
	dGeomID geom;

	double txPowerDbm;// = +20; // dBm

	dReal raio;
	dReal massa;
	dReal altura;

	dReal PosicaoX, PosicaoY;

	void genesis(dWorldID world, dSpaceID space);
	void apparition(struct simulation& sim);

	Antenna();
	double RSSI(dReal X, dReal Y);
};


#endif /* ANTENNA_H_ */
