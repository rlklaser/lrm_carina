#include <stdio.h>
#include <sys/select.h>
#include <termios.h>
#include <stropts.h>
#include <sys/ioctl.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>

FILE * fHandle;
double gpsx, gpsy;
int vel;

int kbhit(){

    static const int STDIN = 0;
    static bool initialized= false;

    if (!initialized) {
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int byteswaiting;
    ioctl(STDIN, FIONREAD, &byteswaiting);
    return byteswaiting;
}

void callback(const geometry_msgs::PoseStampedPtr pose)
{
    gpsx = pose->pose.position.x;
    gpsy = pose->pose.position.y;
}

void writePoint()
{
    printf("Gravando: %.2lf %.2lf %d\n", gpsx, gpsy, vel);
    fprintf(fHandle, "%.2lf %.2lf %d\n", gpsx, gpsy, vel);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "GPSCoordinateCapture");
    ros::NodeHandle nh;

    fHandle = fopen("points.txt", "wt");

    ros::Subscriber subPose = nh.subscribe("lse_xsens_mti/xsens_1/pose", 1, callback);

    ros::Publisher velPub = nh.advertise<std_msgs::Int32>("/throttle_command", 1);

    while (ros::ok())
    {
        if(kbhit()) {
            char c;
            c = getchar();
            vel = (int) c - 48;
            writePoint();

            //publica velocidade
            std_msgs::Int32 msg;
            msg.data = vel;
            velPub.publish(msg);
        }
    	ros::spinOnce();
    }

    fclose(fHandle);
    return 0;
}
