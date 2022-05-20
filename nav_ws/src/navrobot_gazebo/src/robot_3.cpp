//Proportional Controller

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Int8.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <math.h>
#include <climits>
#include <vector>
#include <bits/stdc++.h>
#include "gazebo_msgs/SetModelState.h"


//GLOBAL VARIABLES
ros::Publisher pub;
ros::ServiceClient clientSetModel;
double x; //x robot
double y; //y robot
double theta; //theta robot (heading)
geometry_msgs::Quaternion rot_q; //rot_q robot (orienation_quaternion)
geometry_msgs::Twist speed; //robot's speed (linear & angular) variable
std::vector<int> path; //vertex path sequence

/*
 Axes Colors Reference in Gazebo
 X : RED Axis
 Y : GREEN Axis
 Z : BLUE Axis
 */

//FUNCTIONS AND PROCEDURES
void newOdom(const nav_msgs::Odometry& msg)
{
    //update x,y, and rot_q from odometry msg
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    rot_q = msg.pose.pose.orientation;

    //quaternion declaration
    tf::Quaternion q(rot_q.x, rot_q.y, rot_q.z, rot_q.w);
    tf::Matrix3x3 m(q);

    //roll, pitch, yaw declaration
    double R, P, Y;

    //Quaternion to Euler conversion      
    m.getRPY(R,P,Y);

    //set theta as Y (yaw)
    theta = Y;
}

geometry_msgs::Twist speedcontrol(double goalx, double goaly, double x_pos, double y_pos, double tht, double Kp_lin, double Kp_rot)
{
    //Robot P-Controller to Goal Position
    double angle_to_goal;
    double err_x;
    double err_y;
    double err_lin;
    double err_ort;
    err_x = goalx - x_pos;
    err_y = goaly - y_pos;
    err_lin = sqrt( (pow(err_x,2) + pow(err_y, 2)) );
    angle_to_goal = atan2(err_y,err_x);
    err_ort = angle_to_goal - tht;
    // std::cout<<angle_to_goal<<std::endl;
    geometry_msgs::Twist spd;
    spd.linear.x = Kp_lin * err_lin;
    spd.angular.z = Kp_rot * err_ort;
    // std::cout<<spd<<std::endl;
    return spd;
}

char numtochar(int i)
{
    char a;
    a = 65+i; //ASCII decimal conversion
    return a;
}


//MAIN PROGRAM
int main (int argc, char **argv)
{
    //Node Initialization, Node Name : robot_3
    ros::init(argc, argv, "robot_3"); //Initialize ROS
    ros::NodeHandle nh("~");          //Create node with ability to receive multiple rosrun arguments

    //Param Load
    double x_goal; 
    double y_goal; 
    double Kp_lin;
    double Kp_rot;
    nh.getParam("x_goal", x_goal);
    nh.getParam("y_goal", y_goal);
    nh.getParam("Kp_lin", Kp_lin);
    nh.getParam("Kp_rot", Kp_rot);

    //Subscriber Declaration and Assignment
    ros::Subscriber sub = nh.subscribe("/robot_3/odom", 1, newOdom); //odometry

    //Publisher Assignment
    pub = nh.advertise<geometry_msgs::Twist>("/robot_3/cmd_vel",1);  //robot control

    //Node Frequency 
    ros::Rate loop_rate(50); //50 Hz 

    while (ros::ok()) //SIGINT handler
    {
        speed = speedcontrol(x_goal, y_goal, x, y, theta, Kp_lin, Kp_rot);
        pub.publish(speed);
        //ER : En Route
        ROS_INFO("\n\nGX:%.3f, GY:%.3f, X:%.3f, Y:%.3f\n", x_goal, y_goal, x, y);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}