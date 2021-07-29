#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Int8.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>

double x; //global x robot
double y; //global y robot
double theta; //global theta robot (heading)
geometry_msgs::Quaternion rot_q; //global rot_q robot (orienation_quaternion)

void newOdom(const nav_msgs::Odometry& msg)
{
    //set x,y, and rot_q from odometry
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    rot_q = msg.pose.pose.orientation;

    //define quaternion
    tf::Quaternion q(rot_q.x, rot_q.y, rot_q.z, rot_q.w);
    tf::Matrix3x3 m(q);

    //define roll, pitch, yaw
    double R, P, Y;

    //Quaternion to Euler conversion      
    m.getRPY(R,P,Y);

    //set theta as Y (yaw)
    theta = Y;
}

int main (int argc, char **argv)
{
    //Node Name : robot_1
    ros::init(argc, argv, "robot_1"); //Initialize ROS
    
    ros::NodeHandle nh; //create node

    ros::Subscriber sub = nh.subscribe("/robot_1/odom", 2000, newOdom);

    ros::Rate loop_rate(1000); //1000 Hz 

    while (ros::ok()) //SIGINT handler
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}