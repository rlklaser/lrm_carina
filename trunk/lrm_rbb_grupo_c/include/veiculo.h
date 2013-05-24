#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "constantes.h"
#include "spirit.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>

#include "lrm_rbb_grupo_c/Signal.h"

const static dReal ls[3] = {TAM_SUPORTE, TAM_SUPORTE, TAM_SUPORTE};
const static dReal sides[3] = {COMPRIMENTO, LARGURA, ALTURA};

static int veiculo_id = 0;

class veiculo //: public Spirit
{

private:
	//static veiculo() : _id(0) {};
	void callbackTwist(const geometry_msgs::Twist::ConstPtr& msg);

	std::string odom_link;
	std::string base_link;
	std::string world_link;

public:
	//static int id;
	veiculo();

	void genesis(dWorldID world, dSpaceID space);
	void exodus();
	void apparition(struct simulation& sim);
	void reincarnation(struct simulation& sim);

	void reactive();
	void wander();
	void gotobase();

	double GetBussola();
	double GetAngleToEndpoint(double AlvoX, double AlvoY);
	void GetGpsUtmPosition(double *gx, double *gy);

	void UpdateSignal(double rssi);

	dBodyID veicBody;
	dGeomID veicGeom;
	dGeomID raioGeom[QTD_RAIO];     // infrared
	dBodyID rodaBody[QTD_RODAS];
	dJointID rodaJoint[QTD_RODAS];
	dGeomID rodaGeom[QTD_RODAS];
	dBodyID supBody[QTD_SUPORTE];
	dJointID supJoint[QTD_SUPORTE];
	dGeomID supGeom[QTD_SUPORTE];

	float raiosDistancias[QTD_RAIO];

	dReal turn;
	dReal speed;
	dReal rodaRaio;
	dReal rodaMassa;
	dReal cmassa;
	dReal minhaPosicaoX, minhaPosicaoY;

	double destinoX;
	double destinoY;

	dVector3 p[QTD_SUPORTE];
	dVector3 origin;
	dVector3 dir;

	ros::NodeHandle _nh;
	ros::Publisher _odom_pub;
	ros::Publisher _god_pose_pub;
	ros::Publisher _signal_pub;
	ros::Subscriber _sub_twist;
	tf::TransformBroadcaster _odom_broadcaster;
	geometry_msgs::Pose2D _ground_truth_pose;

	double _pose_x;
	double _pose_y;
	double _pose_theta;
	double _rssi;
	bool _reactive;
};
