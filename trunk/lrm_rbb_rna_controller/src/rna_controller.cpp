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
 * @file rna_controller.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 14, 2012
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "lrm_rbb_rna_controller/RNAInput.h"


/*
 * Trecho de codigo do RoBombeiros (G.Pesin)
 */
void arrumaRaios()
{
	/* arruma info dos senroses (acumula raios), assim testa apenas 0,3,6,9,12*/
	if (laser_distances_[0]>laser_distances_[1]) laser_distances_[0]=laser_distances_[1];
	if (laser_distances_[0]>laser_distances_[2]) laser_distances_[0]=laser_distances_[2];
	if (laser_distances_[0]>laser_distances_[15]) laser_distances_[0]=laser_distances_[15];
	if (laser_distances_[0]>laser_distances_[16]) laser_distances_[0]=laser_distances_[16];
	if (laser_distances_[0]>laser_distances_[17]) laser_distances_[0]=laser_distances_[17];

	if (laser_distances_[3]>laser_distances_[4]) laser_distances_[3]=laser_distances_[4];
	if (laser_distances_[3]>laser_distances_[5]) laser_distances_[3]=laser_distances_[5];
	if (laser_distances_[6]>laser_distances_[7]) laser_distances_[6]=laser_distances_[7];
	if (laser_distances_[6]>laser_distances_[8]) laser_distances_[6]=laser_distances_[8];
	if (laser_distances_[9]>laser_distances_[10]) laser_distances_[9]=laser_distances_[10];
	if (laser_distances_[9]>laser_distances_[11]) laser_distances_[9]=laser_distances_[11];

	if (laser_distances_[12]>laser_distances_[13]) laser_distances_[12]=laser_distances_[13];
	if (laser_distances_[12]>laser_distances_[14]) laser_distances_[12]=laser_distances_[14];
	if (laser_distances_[12]>laser_distances_[18]) laser_distances_[12]=laser_distances_[18];
	if (laser_distances_[12]>laser_distances_[19]) laser_distances_[12]=laser_distances_[19];
	if (laser_distances_[12]>laser_distances_[20]) laser_distances_[12]=laser_distances_[20];
}

double getDefaultTorque()
{
	double torque = 0.0;

	if ((laser_distances_[0] > 30.0) && (laser_distances_[3] > 30.0) &&
			(laser_distances_[6] > 30.0) && (laser_distances_[9] > 30.0) && (laser_distances_[12] > 30.0))
		torque = VELOCIDADE_LIVRE;

	if ((laser_distances_[0] < 30.0) || (laser_distances_[3] < 30.0) ||
		(laser_distances_[6] < 30.0) || (laser_distances_[9] < 30.0) || (laser_distances_[12] < 30.0))
		torque = VELOCIDADE_OBSTACULO_LONGE;

	if ((laser_distances_[0] < 18.0) || (laser_distances_[3] < 18.0) ||
		(laser_distances_[6] < 18.0) || (laser_distances_[9] < 18.0) || (laser_distances_[12] < 18.0))
		torque = VELOCIDADE_OBSTACULO_PERTO;

	return torque;
}

double getMyAngle(double xi, double yi, double xf, double yf) {

	if (yf==yi) yi += 0.0001;
	if (xf==xi) xi += 0.0001;

	double AngR = atan((yf-yi)/(xf-xi));
	double AngG = (AngR/(2*M_PI))*360;

	/* 0 == x, trigonometria */
	//if ((xi<xf)&&(yi<yf)) return 180+AngG;
	//if ((xi>xf)&&(yi>yf)) return AngG;
	//if ((xi>xf)&&(yi<yf)) return 360+AngG;
	//if ((xi<xf)&&(yi>yf)) return 180+AngG;

	/* isso e uma bussola de verdade, 0 == norte [y] e angulos aumentam em sentido horario  */
	if ((xi<xf)&&(yi<yf)) return 360-(90+AngG);
	if ((xi>xf)&&(yi>yf)) return 360-(270+AngG);
	if ((xi>xf)&&(yi<yf)) return 360-(270+AngG);
	if ((xi<xf)&&(yi>yf)) return 360-(90+AngG);

	/* nunca entra aqui, mas... */
	return 0;
}

double getOrientation()
{
	return yaw_*180/M_PI;
}

double getAzimuth()
{
	return getMyAngle(target_.x, target_.y, current_.x, current_.y);
}


ros::Publisher vel_pub_;
//lrm_rbb_rna_controller::RNAInput input_;
geometry_msgs::Twist twist_;

void inputCallback(const lrm_rbb_rna_controller::RNAInput::ConstPtr& msg)
{

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rna_controller");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	int freq = 30;

	ros::Rate rate(freq);

	ros::Subscriber sub = nh.subscribe<lrm_rbb_rna_controller::RNAInput>("input", 1, &inputCallback);
	vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", freq);

	while(ros::ok()) {

		vel_pub_.publish(twist_);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

