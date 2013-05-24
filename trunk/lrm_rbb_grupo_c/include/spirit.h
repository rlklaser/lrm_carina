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
 * @file spirit.h
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 27, 2012
 *
 */

#ifndef SPIRIT_H_
#define SPIRIT_H_

#include <ode/ode.h>

/**
 *  the world of spirits...
 */

class Spirit
{

protected:
	//struct simulation& sim;

public:
	virtual void genesis(dWorldID world, dSpaceID space) = 0;
	virtual void apparition(struct simulation& sim) = 0;
	virtual void exodus() = 0;
	virtual void incarnation();

	//Spirit():sim(sim){};
	virtual ~Spirit(){};
};

#endif /* SPIRIT_H_ */
