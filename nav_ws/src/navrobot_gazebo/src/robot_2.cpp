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
ros::ServiceClient clientSetModelState;
double x; //x robot
double y; //y robot
double theta; //theta robot (heading)
geometry_msgs::Quaternion rot_q; //rot_q robot (orienation_quaternion)
double goal_x; 
double goal_y; 
geometry_msgs::Twist speed; //robot's speed (linear & angular) variable
std::vector<int> path; //vertex path sequence
std::string a[14];

//ROBOT GLOBAL CONFIG
double speed_linear = 0.3;
double speed_angular = 0.2;

/*
 Axes Colors Reference in Gazebo
 X : RED Axis
 Y : GREEN Axis
 Z : BLUE Axis
 */

//WEIGHTED ADJACENCY MATRIX GLOBAL VARIABLE (Expanded Obstacle)
double graph[14][14]={
/*S*/ {0.0, 0.5, 0.7, 0.8, 0.0, 1.3, 1.5, 0.0, 0.0, 0.0, 0.0, 1.4, 0.0, 0.0},
/*C*/ {0.5, 0.0, 0.3, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
/*D*/ {1.3, 0.3, 0.0, 0.0, 0.3, 0.5, 0.8, 0.7, 0.0, 0.0, 0.7, 0.0, 0.0, 1.4},
/*A*/ {0.8, 0.3, 0.0, 0.0, 0.3, 0.0, 0.0, 0.8, 0.0, 0.4, 0.6, 0.7, 0.0, 0.0},
/*B*/ {0.0, 0.0, 0.3, 0.3, 0.0, 0.5, 0.0, 0.5, 0.0, 0.4, 0.4, 0.0, 0.0, 1.2},
/*G*/ {1.3, 0.0, 0.5, 0.0, 0.5, 0.0, 0.3, 0.3, 0.0, 0.8, 0.5, 0.0, 0.9, 0.0},
/*H*/ {1.5, 0.0, 0.8, 0.0, 0.0, 0.3, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0},
/*E*/ {0.0, 0.0, 0.7, 0.0, 0.5, 0.3, 0.0, 0.0, 0.3, 0.7, 0.4, 0.0, 0.5, 0.7},
/*F*/ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.0, 0.9, 0.7, 0.0, 0.8, 0.6},
/*I*/ {0.0, 0.0, 0.0, 0.4, 0.4, 0.8, 0.0, 0.7, 0.9, 0.0, 0.3, 0.3, 0.0, 0.0},
/*J*/ {0.0, 0.0, 0.0, 0.6, 0.4, 0.5, 0.0, 0.4, 0.7, 0.3, 0.0, 0.0, 0.3, 0.9},
/*K*/ {1.4, 0.0, 0.0, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.3, 1.1},
/*L*/ {0.0, 0.0, 0.0, 0.0, 0.0, 0.8, 0.0, 0.5, 0.8, 0.0, 0.3, 0.3, 0.0, 0.8},
/*GL*/{0.0, 0.0, 1.4, 0.0, 1.2, 0.0, 0.0, 0.7, 0.6, 0.0, 0.9, 1.1, 0.8, 0.0}
//      S    C    D    A    B    G    H    E    F     I    J    K    L   GL
};

std::string node[14] = {"S","C","D","A","B","G","H","E","F","I","J","K","L","GL"};

/*
VERTEX LOCATIONS
Vertex/Num : ( x , y ) --> d = degree
S/0 : (0.0 , 0.0)   --> START, d(S) = 6
C/1 : (0.3 , 0.4) --> d(C) = 3
D/2 : (0.6, 0.4) --> d(D) = 8
A/3 : (0.3, 0.7) --> d(A) = 7
B/4 : (0.6, 0.7) --> d(B) = 7
G/5 : (1.1, 0.6) --> d(G) = 8
H/6 : (1.4, 0.6) --> d(H) = 4
E/7 : (1.1, 0.9) --> d(E) = 9
F/8 : (1.4, 0.9) --> d(F) = 6
I/9 : (0.5, 1.0) --> d(I) = 7
J/10 : (0.8, 1.0) --> d(J) = 9
K/11 : (0.5, 1.3) --> d(K) = 5
L/12 : (0.8, 1.3) --> d(L) = 6
GL/13 : (1.5, 1.5) --> GOAL --> d(GL) = 7 
*/

//VERTEX POSITION ARRAY GLOBAL VARIABLE
double vpos[14][2] = {
                    {0.0, 0.0},
                    {0.3, 0.4},
                    {0.6, 0.4},
                    {0.3, 0.7},
                    {0.6, 0.7},
                    {1.1, 0.6},
                    {1.4, 0.6},
                    {1.1, 0.9},
                    {1.4, 0.9},
                    {0.5, 1.0},
                    {0.8, 1.0},
                    {0.5, 1.3},
                    {0.8, 1.3},
                    {1.5, 1.5}
                    };

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
    int index,i;
              
	for(i=0;i<14;i++) 
	{
		if(!vis[i] && dist[i]<=min)      
		{
			min=dist[i];
			index=i;
		}
	}
	return index;
}

int *dijkstra(double graph[14][14], int src) // adjacency matrix is 14x14
{
	//dijkstra algorithm implementation modified from https://www.educative.io/edpresso/how-to-implement-dijkstras-algorithm-in-cpp

    double dist[14];     // integer array of minimum distance for each node.                            
	bool vis[14];        // boolean array of visited or unvisited
	static int prv[14];  // previous vertex for each vertex to reach shortest path
    int m;              // unvisited vertex index

	// set all vertex distance as infinity, all vis as false, and all prv elem as -1
	for(int i = 0; i<14; i++)
	{
		dist[i] = std::numeric_limits<double>::infinity();
		vis[i] = false;
        prv[i] = -1;
	}
	
	dist[src] = 0;   // source vertex distance is set to zero.             
	
	for(int i = 0; i<14; i++)                           
	{
		m=minimumDist(dist,vis); // find minimum in list dist and must be unvisited vertex
		vis[m]=true;             // m with minimum distance included in vis.
		for(int i = 0; i<14; i++)                  
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

void printList(int list[], int size)
{
    std::cout<<"Prev List: ";
    for (int i = 0 ; i<size ; i++)
    {
        std::cout<<list[i]<<" ";
    }
    std::cout<<""<<std::endl;
}

void printSequence(std::vector<int> list, int size)
{
    std::cout<<"Node Sequence : ";
    for (int i = 0; i<size ; i++)
    {
        std::cout<<node[list[i]];
        if (i!=size-1)
        {
            std::cout<<" --> ";
        }
    }
    std::cout<<std::endl;
}

std::vector<int> shortestpath(int src, int end)
{
    //Vertex Sequence of The Shortest Path
    int *prvs = dijkstra(graph, src);

    printList(prvs, 14);

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
    printSequence(a, a.size());
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

    if ((angle_to_goal-tht) > 0.2) //rotate left
    {
        spd.linear.x = 0.0;
        spd.angular.z = speed_angular;
    }
    else if ((angle_to_goal-tht) < -0.2)  //rotate right
    {
        spd.linear.x = 0.0;
        spd.angular.z = -speed_angular;
    }
    else if (path.size() != 0)  //forward
    {
        spd.linear.x = speed_linear;
        spd.angular.z = 0.0;
    }
    else //arrived
    {
        spd.linear.x = 0.0;
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
    else if (path.size() == 0)
    {
        std::cout<<"Robot has arrived at goal (1.5, 1.5)"<<std::endl;
        exit(0);
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
    //Node Initialization, Node Name : robot_2
    ros::init(argc, argv, "robot_2"); //Initialize ROS
    ros::NodeHandle nh("~");          //Create node with ability to receive multiple rosrun arguments

    //Source Vertex and End Vertex Argument Getter through rosrun
    int src = 0;
    int end = 13;

    //Subscriber Declaration and Assignment
    ros::Subscriber sub = nh.subscribe("/robot_1/odom", 20, newOdom); //odometry

    //Publisher Assignment
    pub = nh.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel",20);  //robot control

    //Service Call Assignment
    clientSetModelState = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState robotState;
    robotState.request.model_state.model_name = "robot_1";
    robotState.request.model_state.pose.position.x = vpos[src][0];
    robotState.request.model_state.pose.position.y = vpos[src][1];
    clientSetModelState.call(robotState);

    //Node Frequency 
    ros::Rate loop_rate(100); //100 Hz 

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
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}