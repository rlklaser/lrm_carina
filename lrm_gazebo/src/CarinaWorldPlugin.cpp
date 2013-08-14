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
 * @file CarinaWorldPlugin.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Dec 28, 2012
 *
 */

#include <gazebo/gazebo.hh>

namespace gazebo
{
class CarinaWorldPlugin: public WorldPlugin
{
private:
	physics::WorldPtr _world;
	sdf::ElementPtr _sdf;

public:
	CarinaWorldPlugin() : WorldPlugin()
	{
		printf("CaRINA World Plugin: Hello World!\n");
	}

	void Load(physics::WorldPtr world, sdf::ElementPtr sdf)
	{
		_world = world;
		_sdf = sdf;

		sdf::SDF model;
		model.SetFromString(
		"\
				<gazebo version='1.2'>\
					<model name='model'>\
						<pose>0 0 0 0 0 0</pose>\
						<static>true</static>\
						<allow_auto_disable>true</allow_auto_disable>\
						<link name='link'>\
							<pose>0 0 0 0 0 0</pose>\
							<inertial>\
								<mass>1</mass>\
								<pose>0 0 0 0 0 0</pose>\
								<inertia>\
									<ixx>1</ixx>\
									<ixy>0</ixy>\
									<ixz>0</ixz>\
									<iyy>1</iyy>\
									<iyz>0</iyz>\
									<izz>1</izz>\
								</inertia>\
							</inertial>\
							<collision name='collision'>\
								<geometry>\
									<cylinder>\
										<radius>0.07</radius>\
										<length>5.0</length>\
									</cylinder>\
								</geometry>\
							</collision>\
							<visual name='visual'>\
								<geometry>\
									<mesh>\
										<uri>model://bougainvillier.dae</uri>\
										<scale>1 1 1</scale>\
									</mesh>\
								</geometry>\
							</visual>\
						</link>\
					</model>\
				</gazebo>\
		");
		//_world->InsertModelSDF(model);
		//<uri>model://Penguinville_1/asiainsurancebox.mesh</uri>
	}
};
GZ_REGISTER_WORLD_PLUGIN (CarinaWorldPlugin)
}
