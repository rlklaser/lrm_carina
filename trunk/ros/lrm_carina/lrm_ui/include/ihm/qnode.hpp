/**
 * @file /include/ihm/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ihm_QNODE_HPP_
#define ihm_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <atuacao/Throttle.h>
#include <atuacao/Brake.h>
#include <atuacao/VehicleState.h>
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ihm {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
    void brakeCallback(const atuacao::Brake::ConstPtr& state);
    void throttleCallback(const atuacao::Throttle::ConstPtr& state);
    void canCallback(const atuacao::VehicleState::ConstPtr& state);

signals:
    void velocitySig(double);
    void brakeSig(int);
    void throttleSig(int);
    void handBrakeSig(int);
    void engineSpeedSig(int);
    void gearShiftSig(int);
    void rosShutdown();

private:
	int init_argc;
    int handBrake;
    int engine_speed;
    int gearShift;
	char** init_argv;
    ros::NodeHandle * nh;
    ros::NodeHandle * nh_priv;
    ros::Subscriber can_sub;
    ros::Subscriber brake_sub;
    ros::Subscriber throttle_sub;
    double velocity;
    double brake;
    double throttle;
};

}  // namespace ihm

#endif /* ihm_QNODE_HPP_ */
