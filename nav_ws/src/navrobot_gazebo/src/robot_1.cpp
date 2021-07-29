#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


int main (int argc, char **argv)
{
    ros::init(argc, argv, "robot_1"); //Initialize ROS
    
    ros::NodeHandle nh; //create node

    ros::Rate loop_rate(60); //60 Hz 

    while (ros::ok()) //SIGINT handler
    {
        loop_rate.sleep();
    }

    return 0;
}