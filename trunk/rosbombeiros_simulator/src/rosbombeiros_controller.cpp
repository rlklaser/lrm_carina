
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#include "gp_rede24.h"

//typedef nav_msgs::Odometry imu_type;
//typedef geometry_msgs::PoseWithCovarianceStamped imu_type;
typedef sensor_msgs::Imu imu_type;

#define TAM_RAIO_FRENTE    6
//4*12
#define TAM_RAIO_LATERAL   6
//3*11
#define QTD_RAIO           21

/* velocidde padrao dos robos moveis*/
#define VELOCIDADE_LIVRE 6.0
#define VELOCIDADE_OBSTACULO_LONGE 3.0
#define VELOCIDADE_OBSTACULO_PERTO 1.5

struct point_3d {
	double x;
	double y;
	double z;
};

struct quaternion {
	double x;
	double y;
	double z;
	double w;
};

class ROSBombeirosController {

private:

	geometry_msgs::Twist cmd;

	ros::NodeHandle n_;
	ros::Publisher vel_pub_;
	tf::TransformListener listener_;
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
	tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
	message_filters::Subscriber<imu_type> imu_sub_;
	tf::MessageFilter<imu_type> imu_notifier_;
	ros::Publisher scan_pub_;

	float laser_distances_[QTD_RAIO];

	struct point_3d target_;
	struct point_3d current_;
	//struct quaternion orientation_;
	int imu_print_rate_;
	int ann_print_rate_;
	double yaw_;
	double plannar_angle_;

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

public:

	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
		sensor_msgs::LaserScan msg = *scan_in;
		printf("laser seq=%d, r[0]=%.3f, sz=%d\n", msg.header.seq, msg.ranges[0], msg.ranges.size());

		for(int i=0; i<QTD_RAIO; i++) {
			laser_distances_[i] = msg.ranges[i];
		}
	}

	void imuCallback(const imu_type::ConstPtr& odom_in)
	{
		imu_type msg = *odom_in;
/*
		current_.x = msg.pose.pose.position.x;
		current_.y = msg.pose.pose.position.y;
		current_.z = msg.pose.pose.position.z;

		orientation_.x = msg.pose.pose.orientation.x;
		orientation_.y = msg.pose.pose.orientation.y;
		orientation_.z = msg.pose.pose.orientation.z;
		orientation_.w = msg.pose.pose.orientation.w;

		tf::Quaternion orientation;
		tf::Transform transform;
		tf::quaternionMsgToTF(msg.pose.pose.orientation, orientation);
		transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0) );
		transform.setRotation(orientation);

		yaw_ = orientation.getAngle();
*/
		yaw_ = tf::getYaw(msg.orientation);
	}

	ROSBombeirosController(ros::NodeHandle n)
	: 	n_(n),
		laser_sub_(n_, "/base_scan", 10),
		laser_notifier_(laser_sub_, listener_, "laser_link", 10),
		imu_sub_(n_, "/imu_data", 10),
		imu_notifier_(imu_sub_, listener_, "imu_link", 10)
	{
		imu_print_rate_ = 0;
		ann_print_rate_ = 0;

		laser_notifier_.registerCallback(
				boost::bind(&ROSBombeirosController::laserCallback, this, _1));

		imu_notifier_.registerCallback(
				boost::bind(&ROSBombeirosController::imuCallback, this, _1));

		vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	}

	void init()
	{
		cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

		ros::NodeHandle n_private("~");
		n_private.param("target_x", target_.x, 10.0);
		n_private.param("target_y", target_.y, 10.0);
		n_private.param("target_z", target_.z, 0.0);
	}

	bool testSuccess()
	{
		bool success = (current_.x==target_.x && current_.y==-target_.y);
		if(success)
			printf("Robot Succeeded!\n");
		return success;
	}

	void stop() {
		cmd.linear.x = 0;
		cmd.linear.y = 0;
		cmd.angular.z = 0;
		vel_pub_.publish(cmd);
	}

	void loop()
	{
		/*
		 * Trecho de codigo do RoBombeiros (G.Pesin)
		 */
		float lista_in[7];
		float lista_out[2];

		lista_in[0] = getOrientation();
		lista_in[1] = getAzimuth();

		arrumaRaios();

		lista_in[2]=(laser_distances_[0]/(TAM_RAIO_LATERAL))*100;
		lista_in[3]=(laser_distances_[3]/(TAM_RAIO_FRENTE))*100;
		lista_in[4]=(laser_distances_[6]/(TAM_RAIO_FRENTE))*100;
		lista_in[5]=(laser_distances_[9]/(TAM_RAIO_FRENTE))*100;
		lista_in[6]=(laser_distances_[12]/(TAM_RAIO_LATERAL))*100;

		// chama rede
		gp_rede24(lista_in, lista_out, 1);

		ann_print_rate_++;
		if(ann_print_rate_>40) {
			ann_print_rate_ = 0;
			printf("lin=%.3f ang=%.3f (az=%.3f or=%.3f qu=%.3f) (tg.x=%.3f tg.y=%.3f) - ",
					lista_out[1], lista_out[0],
					lista_in[1], lista_in[0], plannar_angle_,
					target_.x, target_.y
			);
			printf("([0]=%.2f [3]=%.2f [6]=%.2f [9]=%.2f [12]=%.2f) - ",
					lista_in[2],
					lista_in[3],
					lista_in[4],
					lista_in[5],
					lista_in[6]
			);
			printf("(x=%.2f y=%.2f z=%.2f) - ",
					current_.x,
					current_.y,
					current_.z
			);
			printf("yaw_rad=%.2f yaw_dgr=%.2f",
					yaw_,
					yaw_*180/M_PI
			);
			printf("\n");
		}

		cmd.linear.x = -lista_out[1] / 10.0;
		cmd.angular.z = lista_out[0];
		vel_pub_.publish(cmd);

		/*
		// chegada no final final
		if ( (v[i].sSituacao == 'X') && (dVeiculoPosX <= v[i].fDestinoX+fEgps) &&
			 (dVeiculoPosX >= v[i].fDestinoX-fEgps) && (dVeiculoPosY <= v[i].fDestinoY+fEgps) &&
			 (dVeiculoPosY >= v[i].fDestinoY-fEgps) )
			 v[i].Torque = 0.0;
		else
		{

			if ((laser_distances_[0] > 30.0) && (laser_distances_[3] > 30.0) &&
				(laser_distances_[6] > 30.0) && (laser_distances_[9] > 30.0) && (laser_distances_[12] > 30.0))
				v[i].Torque = -1.0 * VELOCIDADE_LIVRE;

			if ((laser_distances_[0] < 30.0) || (laser_distances_[3] < 30.0) ||
				(laser_distances_[6] < 30.0) || (laser_distances_[9] < 30.0) || (laser_distances_[12] < 30.0))
				v[i].Torque = -1.0 * VELOCIDADE_OBSTACULO_LONGE;

			if ((laser_distances_[0] < 18.0) || (laser_distances_[3] < 18.0) ||
				(laser_distances_[6] < 18.0) || (laser_distances_[9] < 18.0) || (laser_distances_[12] < 18.0))
				v[i].Torque = -1.0 * VELOCIDADE_OBSTACULO_PERTO;
		}
		*/
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "rosbombeiros_controller");
	ros::NodeHandle n;
	ROSBombeirosController controller(n);

	controller.init();

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		controller.loop();
		if (controller.testSuccess()) {
			controller.stop();
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	exit(0);
}

