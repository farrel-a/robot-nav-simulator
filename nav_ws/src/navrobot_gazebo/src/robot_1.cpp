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
double goal_x; 
double goal_y; 
geometry_msgs::Twist speed; //robot's speed (linear & angular) variable
std::vector<int> path; //vertex path sequence

/*
 Axes Colors Reference in Gazebo
 X : RED Axis
 Y : GREEN Axis
 Z : BLUE Axis
 */

//WEIGHTED ADJACENCY MATRIX GLOBAL VARIABLE
double graph[6][6]={
    {0.0, 2.5, 5.0, 0.0, 0.0, 0.0},
    {2.5, 0.0, 0.0, 10.0, 12.9, 0.0},
    {5.0, 0.0, 0.0, 7.8, 8.5, 0.0},
    {0.0, 10.0, 7.8, 0.0, 5.05, 3.75},
    {0.0, 12.9, 8.5, 5.05, 0.0, 2.0},
    {0.0, 0.0, 0.0, 3.75, 2.0, 0.0}};

/*
VERTEX LOCATIONS
Vertex : ( x , y )
A/0 : (0.0 , 0.0)
B/1 : (1.0 , -2.275)      
C/2 : (4.55 , 2.076)   
D/3 : (11.0 , -2.275)
E/4 : (13.075 , 2.375)
F/5 : (13.55 , 0.45)
*/

//VERTEX POSITION ARRAY GLOBAL VARIABLE
double vpos[6][2] = {
                    {0.0,  0.0},
                    {1.0, -2.275},
                    {4.55, 2.075},
                    {11.0,-2.275},
                    {13.075, 2.375},
                    {13.55, 0.45}};

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

int minimumDist(double dist[], bool vis[]) 
{
	double min=std::numeric_limits<double>::infinity();
    int index;
              
	for(int i=0;i<6;i++) 
	{
		if(!vis[i] && dist[i]<=min)      
		{
			min=dist[i];
			index=i;
		}
	}
	return index;
}

int *dijkstra(double graph[6][6],int src) // adjacency matrix used is 6x6
{
	//dijkstra algorithm modified from https://www.educative.io/edpresso/how-to-implement-dijkstras-algorithm-in-cpp

    double dist[6];     // integer array of minimum distance for each node.                            
	bool vis[6];        // boolean array of visited or unvisited
	static int prv[6];  // previous vertex for each vertex to reach shortest path
    int m;              // unvisited vertex index

	// set all vertex distance as infinity
	for(int i = 0; i<6; i++)
	{
		dist[i] = std::numeric_limits<double>::infinity();
		vis[i] = false;
        prv[i] = -1;
	}
	
	dist[src] = 0;   // Source vertex distance is set to zero.             
	
	for(int i = 0; i<6; i++)                           
	{
		m=minimumDist(dist,vis); // unvisited vertex
		vis[m]=true;             // m with minimum distance included in vis.
		for(int i = 0; i<6; i++)                  
		{
			// Updating the minimum distance for the particular vertex
			if(!vis[i] && (graph[m][i] != 0.0) && dist[m]!=std::numeric_limits<double>::infinity() && dist[m]+graph[m][i]<dist[i])
				{
                    prv[i] = m;  //set previous vertex
                    dist[i]=dist[m]+graph[m][i]; //update distance
                }
		}
	}
    return prv;
}

std::vector<int> shortestpath(int src, int end)
{
    //Vertex Sequence of The Shortest Path
    int *prvs = dijkstra(graph, src);

    for (int i = 0 ; i<6 ; i++)
    {
        std::cout<<prvs[i]<<" ";
    }
    std::cout<<""<<std::endl;

    //Vertex sequence from previous vertex of each vertex
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
    //Robot Speed Control to Goal
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

void checkPos(double x_pos, double y_pos)
{
    //Check whether the robot has arrived at certain vertex
    if (((vpos[path[0]][0]-0.05 <= x_pos) && (x_pos <= vpos[path[0]][0]+0.05) 
         && (vpos[path[0]][1]-0.05 <= y_pos) && (y_pos <= vpos[path[0]][0]+0.05)) && path.size()!=0)
    {
        reverse(path.begin(), path.end());
        path.pop_back();
        reverse(path.begin(), path.end());
    }
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
    //Node Initialization, Node Name : robot_1
    ros::init(argc, argv, "robot_1"); //Initialize ROS
    ros::NodeHandle nh("~");          //Create node with ability to receive multiple rosrun arguments

    //Source Vertex and End Vertex Argument Getter through rosrun
    int src;
    int end;
    nh.getParam("src", src);
    nh.getParam("end", end);
    if (src<0 || src>5 || end<0 || end>5)
    {
        ROS_INFO("Invalid Argument! src and end must be 0<=(src,end)<=5");
        ros::shutdown();
    }

    //Subscriber Declaration and Assignment
    ros::Subscriber sub = nh.subscribe("/robot_1/odom", 2000, newOdom); //odometry

    //Publisher Assignment
    pub = nh.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel",2000);  //robot control

    //Service Call Assignment
    clientSetModel = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState robotState;
    robotState.request.model_state.model_name = "robot_1";
    robotState.request.model_state.pose.position.x = vpos[src][0];
    robotState.request.model_state.pose.position.y = vpos[src][1];
    clientSetModel.call(robotState);

    //Node Frequency 
    ros::Rate loop_rate(1000); //1000 Hz 

    //Path Sequence
    path = shortestpath(src, end);
    


    while (ros::ok()) //SIGINT handler
    {
        char vertex;
        goal_x = vpos[path[0]][0];
        goal_y = vpos[path[0]][1];
        speed = speedcontrol(goal_x, goal_y, x, y, theta);
        pub.publish(speed);
        checkPos(x,y);
        vertex = numtochar(path[0]);
        //ER : En Route
        ROS_INFO("\n\nER:%c/%d, GX:%.3f, GY:%.3f, X:%.3f, Y:%.3f\n", vertex, path[0], goal_x, goal_y, x, y);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}