#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Int8.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <math.h>


//GLOBAL VARIABLES
ros::Publisher pub;
double x; //x robot
double y; //y robot
double theta; //theta robot (heading)
geometry_msgs::Quaternion rot_q; //rot_q robot (orienation_quaternion)
double goal_x; 
double goal_y; 
geometry_msgs::Twist speed;
double angle_to_goal;

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
    //Node Definition
    //Node Name : robot_1
    ros::init(argc, argv, "robot_1"); //Initialize ROS
    ros::NodeHandle nh; //create node

    //Publisher & Subscriber Definition
    ros::Subscriber sub = nh.subscribe("/robot_1/odom", 2000, newOdom); //odometry
    pub = nh.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel",2000);  //robot control

    ros::Rate loop_rate(1000); //1000 Hz 

    goal_x = 2.0;
    goal_y = 2.0;

    double Dx;
    double Dy; 
    while (ros::ok()) //SIGINT handler
    {
        Dx = goal_x - x;
        Dy = goal_y - y;
        angle_to_goal = atan2(Dy,Dx);

        if ((angle_to_goal-theta) > 0.2)
        {
            speed.linear.x = 0.0;
            speed.angular.z = 0.3;
        }
        else if ((angle_to_goal-theta) < -0.2)
        {
            speed.linear.x = 0.0;
            speed.angular.z = -0.3;
        }
        else
        {
            speed.linear.x = 0.5;
            speed.angular.z = 0.0;
        }

        pub.publish(speed);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}