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

//GLOBAL VARIABLES
ros::Publisher pub;
double x; //x robot
double y; //y robot
double theta; //theta robot (heading)
geometry_msgs::Quaternion rot_q; //rot_q robot (orienation_quaternion)
double goal_x; 
double goal_y; 
geometry_msgs::Twist speed;
std::vector<int> path;

//GLOBAL WEIGHTED ADJACENCY MATRIX
int graph[6][6]={
		{0, 10, 20, 0, 0, 0},
		{10, 0, 0, 50, 10, 0},
		{20, 0, 0, 20, 33, 0},
		{0, 50, 20, 0, 20, 2},
		{0, 10, 33, 20, 0, 1},
		{0, 0, 0, 2, 1, 0}};

//FUNCTIONS AND PROCEDURES
void newOdom(const nav_msgs::Odometry& msg)
{
    //set x,y, and rot_q from odometry
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

int minimumDist(int dist[], bool Tset[]) 
{
	int min=INT_MAX,index;
              
	for(int i=0;i<6;i++) 
	{
		if(Tset[i]==false && dist[i]<=min)      
		{
			min=dist[i];
			index=i;
		}
	}
	return index;
}

int *dijkstra(int graph[6][6],int src) // adjacency matrix used is 6x6
{
	//dijkstra algo modified from https://www.educative.io/edpresso/how-to-implement-dijkstras-algorithm-in-cpp

    int dist[6]; // integer array to calculate minimum distance for each node.                            
	bool Tset[6];// boolean array to mark visted/unvisted for each node.
	static int prv[6];
	// set the nodes with infinity distance
	// except for the initial node and mark
	// them unvisited.  
	for(int i = 0; i<6; i++)
	{
		dist[i] = INT_MAX;
		Tset[i] = false;
        prv[i] = -1;
	}
	
	dist[src] = 0;   // Source vertex distance is set to zero.             
	
	for(int i = 0; i<6; i++)                           
	{
		int m=minimumDist(dist,Tset); // vertex not yet included.
		Tset[m]=true;// m with minimum distance included in Tset.
		for(int i = 0; i<6; i++)                  
		{
			// Updating the minimum distance for the particular node.
			if(!Tset[i] && graph[m][i] && dist[m]!=INT_MAX && dist[m]+graph[m][i]<dist[i])
				{
                    prv[i] = m;
                    dist[i]=dist[m]+graph[m][i];
                }
		}
	}
    return prv;
}

std::vector<int> shortestpath(int src, int end)
{
    int *prvs = dijkstra(graph, src);
    std::vector<int> a;
    int i;
    i = end;
    a.push_back(i);
    while (prvs[i] != -1)
    {
        a.push_back(prvs[i]);
        i = prvs[i];
    }
    reverse(a.begin(), a.end());
    return a;  
}

geometry_msgs::Twist speedcontrol(double goalx, double goaly, double x_pos, double y_pos, double tht)
{
    double angle_to_goal;
    double Dx;
    double Dy;
    Dx = goalx - x_pos;
    Dy = goaly - y_pos;
    angle_to_goal = atan2(Dy,Dx);
    geometry_msgs::Twist spd;

    if ((angle_to_goal-tht) > 0.2)
    {
        spd.linear.x = 0.0;
        spd.angular.z = 0.3;
    }
    else if ((angle_to_goal-tht) < -0.2)
    {
        spd.linear.x = 0.0;
        spd.angular.z = -0.3;
    }
    else
    {
        spd.linear.x = 0.5;
        spd.angular.z = 0.0;
    }
    return spd;
}

//MAIN PROGRAM
int main (int argc, char **argv)
{
    //Node Declaration
    //Node Name : robot_1
    ros::init(argc, argv, "robot_1"); //Initialize ROS
    ros::NodeHandle nh("~"); //Create node with ability to receive multiple rosrun arguments

    //Source Vertex and End Vertex Argument Getter
    int src;
    int end;
    nh.getParam("src", src);
    nh.getParam("end", end);

    //Subscriber Declaration and Assignment
    ros::Subscriber sub = nh.subscribe("/robot_1/odom", 2000, newOdom); //odometry

    //Publisher Assignment
    pub = nh.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel",2000);  //robot control

    //Frequency Definition
    ros::Rate loop_rate(1000); //1000 Hz 

    goal_x = 2.0;
    goal_y = 2.0;

    path = shortestpath(0, 5);
    while (ros::ok()) //SIGINT handler
    {
        speed = speedcontrol(goal_x, goal_y, x, y, theta);
        pub.publish(speed);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}