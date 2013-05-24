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
 * @file antenna.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 26, 2012
 *
 */

#include "antenna.h"

Antenna::Antenna()
{
	_a = CreateObject<ConstantPositionMobilityModel>();
	_b = CreateObject<ConstantPositionMobilityModel>();

	//ItuR1411LosPropagationLossModel
	_model = CreateObject<LogDistancePropagationLossModel>();
	_wifi_phy = CreateObject<YansWifiPhy>();

	txPowerDbm = +17.5;//+20; // dBm
}

/**
 * Physics definitions
 */
void Antenna::genesis(dWorldID world, dSpaceID space)
{
	dReal x = float(rand() % 200) - 100;
	dReal y = float(rand() % 200) - 100;
	dMass m;

	x = 0;
	y = 0;

	PosicaoX = x;
	PosicaoY = y;

//	_a->SetPosition(Vector(x, y, 0.0));

	raio = 0.25;
	massa = 20.0;
	altura = 15.0;

	body = dBodyCreate(world);
	dMassSetSphere(&m, 1, raio);
	dMassAdjust(&m, massa);
	dBodySetMass(body, &m);
	geom = dCreateCylinder(space, raio, altura);
	dGeomSetBody(geom, body);
	dBodySetPosition(body, x, y, altura);
}

/**
 * Visual definitions
 */
void Antenna::apparition(struct simulation& sim)
{
	static double qt[] = {
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0 };
	static double pa[] = {PosicaoX, PosicaoY, 0};

	dsSetColor(1, 1, 0);
	dsDrawCylinderD(pa, qt, altura, raio);
}

double Antenna::RSSI(dReal X, dReal Y)
{
	_a->SetPosition(Vector(X, Y, 0.0));

	double dbm = _model->CalcRxPower(txPowerDbm, _b, _a);
	//private :-( double w = _wifi_phy.DbmToW(dbm);

	return dbm;
}
