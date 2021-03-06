#include <ros/ros.h>
#include <ros/time.h>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <vector>
#include <iterator>
#include <string>

#include <rosie_servo_controller/ControlGates.h>
#include <rosie_map_controller/RequestRerun.h>
#include <rosie_map_controller/StartRRT.h>
#include <rosie_map_controller/RequestLoading.h>
#include <rosie_object_detector/RAS_Evidence.h>
#include <rosie_path_finder/rrtService.h>

#include <rosie_map_controller/MapStoring.h>
#include <rosie_map_controller/ObjectStoring.h>
#include <rosie_map_controller/BatteryPosition.h>
#include <rosie_map_controller/ObjectPosition.h>
#include <rosie_map_controller/WallDefinition.h>
#include <rosie_path_navigator/PathNavigationService.h>

ros::ServiceClient pathClient;

// RRT parameter
float PI = 3.1415926f;
float EPSILON = 0.050f;
float XDIM = 2.40;
float YDIM = 2.40;
float startx = 0.200f;
float starty = 0.400f;
float OFFSET[] = {0.0f, 0.00f};
float HOME[] = {startx, starty};
int NUMNODES = 10000;
float robotsize = 0.2f;
int mode;

// Publisher
ros::Publisher path_pub;
ros::ServiceClient loadClient;
rosie_map_controller::RequestLoading loadSrv;
rosie_path_navigator::PathNavigationService pathSrv;

// Target pose
boost::shared_ptr<geometry_msgs::PoseStamped> targetPose_ptr;
boost::shared_ptr<geometry_msgs::PoseStamped> lastTargetPose_ptr;

// Objects and Obstacles (Walls and Batteries)
std::vector<float> wallArray;
std::vector<float> ALL_OBS;
std::vector<float> ALL_OBJ;
std::vector<float> ALL_WALL;
float objSize = 0.05f;
int objNumber = 14;
std::vector<float> objPoseX;
std::vector<float> objPoseY;
float batSize = 0.10;
int batNumber = 4;
std::vector<float> batPoseX;
std::vector<float> batPoseY;

char newPathGenerated = 0;

// for target calculations
int target_num = 0;

// path variables
nav_msgs::Path path;
std::vector<float> finalpathx;
std::vector<float> finalpathy;
std::vector<geometry_msgs::PoseStamped> allposes;

// state parameters
int pathseq = 0;
char mapInitialized = 0;
int pathPublished = 0;
int pathInitialized = 0;
bool pathFound = 0; //for most valuable object search

// **************************
float certaintyValue = 100;
float certaintyLimit = 0.012;

void certaintyCallback(std_msgs::Float32 msg){
	certaintyValue = msg.data;
}

// addToObs parameter
float origin[] = {0.0f,0.0f};
float temp[2];
float m;
float r = 0.13f;
float x_1,y_1,x_2,y_2;
float diffy;
float diffx;
float P[8];
float p1_new[2];
float p2_new[2];
float n;
float co;
float si;

bool addToObs(float data[], int obsOrObj){
			float p[4];

			//float abst = sqrt(pow(OBS[i+2]-OBS[i+0],2.0)+pow(OBS[i+3]-OBS[i+1],2.0));
			//float m1 = (OBS[i+2]-OBS[i+0])/dist;
			//float m2 = (OBS[i+3]-OBS[i+1])/dist;
			p[0] =data[0];
			p[1] =data[1];
			p[2] =data[2];
			p[3] =data[3];
		//	ROS_INFO("x1 %f y1 %f x2 %f y2 %f", p1[0], p1[1], p2[0], p2[1] );


			if((p[3]< p[1]) || (p[3] == p[1] and p[2]<p[0])){
				temp[0] = p[0];
				temp[1] = p[1];
				//ROS_INFO("%f %f", p[0], p[1]);
				p[0] = p[2];
				p[1] = p[3];
				//ROS_INFO("%f %f", p[2], p[3]);
				p[2] = temp[0];
				p[3] = temp[1];

			}
			diffx = p[2]-p[0];
			diffy = p[3]-p[1];
			m = (float) atan2(diffy, diffx);
			//ROS_ERROR("m %f",m);
			co = r*cos(m);
			si = r*sin(m);

			x_1 = p[0] - co;
			y_1 = p[1] - si;
			x_2 = p[2] + co;
			y_2 = p[3] + si;
			si = r*cos(m);
			co = r*sin(m);
			 P[0] = x_1-co;
			 P[1] = y_1+si;
			 P[2] = x_1+co;
			 P[3] = y_1-si;
			 P[4] = x_2+co;
			 P[5] = y_2-si;
			 P[6] = x_2-co;
			 P[7] = y_2+si;
			 //ROS_ERROR("%f %f %f %f %f %f %f %f", P[0], P[1], P[2], P[3], P[4], P[5], P[6], P[7] );
		for(int i=0;i<8;i++){
			if(obsOrObj==1){
				 ALL_OBS.push_back(P[i]);
			}else if(obsOrObj==0){
				 ALL_OBJ.push_back(P[i]);
			}else if(obsOrObj ==2){
				ALL_WALL.push_back(P[i]);
			}

	 }
	 //ROS_INFO("addToObs");
	 return true;
}

nav_msgs::Odometry pose;
void currentPoseCallback(nav_msgs::Odometry msg){ // for re-calculation of the path when needed
    	pose = msg;
		//pose.pose.pose.position.x = 0.25f;
		//pose.pose.pose.position.y = 0.40f;
}

float objTemp1[4];
float objTemp2[4];
std::vector<int> objectID;
std::vector<float> objWeighting;
rosie_map_controller::ObjectStoring objStack;
bool objCallInitializing = 0;
bool objInitialized = 0;
void objCallback(rosie_map_controller::ObjectStoring msg){
	if(objCallInitializing){
		return;
	}
	objCallInitializing = 1;
	ALL_OBJ.clear();
	ALL_OBS.clear();
	objStack = msg;
	float posX;
	float posY;
	//ROS_ERROR("objStack %d %d",objStack.Batteries.size(), objStack.Objects.size());
	for(int i = 0; i< objStack.Batteries.size(); i++){
		posX = objStack.Batteries[i].x;
		posY = objStack.Batteries[i].y;
		objTemp1[0] = posX-batSize/2.0f;
		objTemp1[1] = posY-batSize/2.0f;
		objTemp1[2] = posX-batSize/2.0f;
		objTemp1[3] = posY+batSize/2.0f;
		//ROS_ERROR("bat %f %f %f %f",objTemp1[0], objTemp1[1], objTemp1[2], objTemp1[3] );
		objTemp2[0] = posX+batSize/2.0f;
		objTemp2[1] = posY-batSize/2.0f;
		objTemp2[2] = posX+batSize/2.0f;
		objTemp2[3] = posY+batSize/2.0f;
		//("bat %f %f %f %f",objTemp2[0], objTemp2[1], objTemp2[2], objTemp2[3] );
		bool test1 = addToObs(objTemp1,1);
		bool test2 = addToObs(objTemp2,1);
	}
	for(int i = 0; i< objStack.Objects.size(); i++){
		posX = objStack.Objects[i].x;
		posY = objStack.Objects[i].y;
		objTemp1[0] = posX-objSize/2.0f;
		objTemp1[1] = posY-objSize/2.0f;
		objTemp1[2] = posX-objSize/2.0f;
		objTemp1[3] = posY+objSize/2.0f;
		//ROS_ERROR("obj %f %f %f %f",objTemp1[0], objTemp1[1], objTemp1[2], objTemp1[3] );
		objTemp2[0] = posX+objSize/2.0f;
		objTemp2[1] = posY-objSize/2.0f;
		objTemp2[2] = posX+objSize/2.0f;
		objTemp2[3] = posY+objSize/2.0f;
		//ROS_ERROR("obj %f %f %f %f",objTemp2[0], objTemp2[1], objTemp2[2], objTemp2[3] );
		bool test1 = addToObs(objTemp1,0);
		bool test2 = addToObs(objTemp2,0);
	}
	objInitialized = 1;
}


rosie_map_controller::MapStoring wallStack;

bool mapInitializing = 0;
void wallCallback2(rosie_map_controller::MapStoring msg){
	if(mapInitializing && mapInitialized){
		return;
	}	
	mapInitializing = 1;
	ALL_WALL.clear();
	wallStack = msg;
	float posX;
	float posY;

	float minX= 0, minY=0, maxX= 0, maxY = 0;
	float sX, sY, eX, eY;

	for(int i = 0; i< wallStack.NewWalls.size(); i++){
		if(wallStack.NewWalls[i].certainty >= 0){
			objTemp1[0] = wallStack.NewWalls[i].x1;
			objTemp1[1] = wallStack.NewWalls[i].y1;
			objTemp1[2] = wallStack.NewWalls[i].x2;
			objTemp1[3] = wallStack.NewWalls[i].y2;
			//ROS_ERROR("wall %f %f %f %f %d",objTemp1[0] ,objTemp1[1], objTemp1[2], objTemp1[3], wallStack.NewWalls[i].certainty);
			bool test1 = addToObs(objTemp1,2);
		}

		if(!mapInitialized){
			if(objTemp1[0] < minX){
				minX = objTemp1[0];
			}
			if(objTemp1[0] > maxX){
				maxX = objTemp1[0];
			}
			if(objTemp1[2] < minX){
				minX = objTemp1[2];
			}
			if(objTemp1[2] > maxX){
				maxX = objTemp1[2];
			}
			if(objTemp1[1] < minY){
				minY = objTemp1[1];
			}
			if(objTemp1[1] > maxY){
				maxY = objTemp1[1];
			}
			if(objTemp1[3] < minY){
				minY = objTemp1[3];
			}
			if(objTemp1[3] > maxY){
				maxY = objTemp1[3];
			}
		}
	}

	OFFSET[0] = minX;
	OFFSET[1] = minY;
	//ROS_ERROR("Offset %f, %f", OFFSET[0], OFFSET[1]);
	XDIM = (maxX - minX);
	YDIM = (maxY - minY);
	//ROS_ERROR("XDIM %f, YDIM %f", XDIM, YDIM);
	mapInitialized = 1;
}

float oldTargetX = -1;
float oldTargetY = -1;
int oldTargetIdx = -1;

class Node{
public:
	float pos[2];
	float cost;
	int parent;

	Node(float x_in, float y_in, float c_in, int p_in) //x(x), y(y), cost(c), parent(p) {}
	{
		pos[0] = x_in;
		pos[1] = y_in;
		cost = c_in;
		parent = p_in;
	}
};

float dist(float q1[], float q2[]){
    float d = sqrt(pow(q1[0]-q2[0],2) + pow(q1[1]-q2[1],2));
		//ROS_INFO("dist in fct f%", d);
    return d;
}

float *step_from_to(float p1[], float p2[], float p_new[]){
	if(dist(p1, p2) >= EPSILON){
		//float theta = atan2(p1[1] - p2[1], p1[0] - p2[0]);
		p_new[0] = p2[0] + (p1[0]-p2[0])*EPSILON/dist(p1,p2);
		p_new[1] = p2[1] + (p1[1]-p2[1])*EPSILON/dist(p1,p2);
	}else{
		p_new[0] = p1[0];
		p_new[1] = p1[1];
	}
}

bool isIntersecting(float p1[], float p2[], float q1[], float q2[]) {
    return (((q1[0]-p1[0])*(p2[1]-p1[1]) - (q1[1]-p1[1])*(p2[0]-p1[0]))
            * ((q2[0]-p1[0])*(p2[1]-p1[1]) - (q2[1]-p1[1])*(p2[0]-p1[0])) < 0)
            and
           (((p1[0]-q1[0])*(q2[1]-q1[1]) - (p1[1]-q1[1])*(q2[0]-q1[0]))
            * ((p2[0]-q1[0])*(q2[1]-q1[1]) - (p2[1]-q1[1])*(q2[0]-q1[0])) < 0);
}

bool isIntersectingCap(float center[], float A[], float B[]){
	float x = center[0];
	float y = center[1];
	float a = (A[1]-B[1])/(A[0]-B[0]);
	float b = -1;
	float c = A[1]-m*A[0];

	int dist = (std::abs(a*x + b*y + c))/(std::sqrt(a*a + b*b));
	if(r>=dist){
		return 1;
	}else{
		return 0;
	}
}


float p1[2];
float p2[2];
float p3[2];
float p4[2];
bool nc = true;
bool ints1;
bool ints2;
bool ints3;
bool ints4;
bool ints5;
bool ints6;

bool checkIntersect(Node n2, Node n1){         //array definition might be wrong
    float A[]= {n1.pos[0],n1.pos[1]};
    float B[]= {n2.pos[0],n2.pos[1]};
    float center[2];
    nc = 1;
    for(int i =0 ; i<ALL_WALL.size(); i = i+8){
			//ROS_INFO("ALL OBS size %d", ALL_WALL.size());
				p1[0] =ALL_WALL[i];
				p1[1] =ALL_WALL[i+1];
				p2[0] =ALL_WALL[i+2];
				p2[1] =ALL_WALL[i+3];
				p3[0] =ALL_WALL[i+4];
				p3[1] =ALL_WALL[i+5];
				p4[0] =ALL_WALL[i+6];
				p4[1] =ALL_WALL[i+7];
				//ROS_INFO("x1 %f y1 %f x2 %f y2 %f", p1[0], p1[1], p2[0], p2[1] );

				 ints1 = isIntersecting(p1,p2,A,B);
				 ints2 = isIntersecting(p1,p4,A,B);
				 ints3 = isIntersecting(p3,p2,A,B);
				 ints4 = isIntersecting(p3,p4,A,B);

				 center[0] = (p2[0]-p1[0])/2;
				 center[1] = (p2[1]-p1[1])/2;
				 ints5 = isIntersectingCap(center, A, B);
				 center[0] = (p3[0]-p4[0])/2;
				 center[1] = (p3[1]-p4[1])/2;
				 ints6 = isIntersectingCap(center, A, B);
      	//if(ints5 == 0 and ints6 == 0 and ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==1){
      	if(ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==1){
            nc = 1; //no intersection
	    
        }else{
            nc = 0;
	    break;
        }

    }

    for(int i =0 ; i<ALL_OBS.size(); i = i+8){
			//ROS_INFO("ALL OBS size %d", ALL_OBS.size());
				p1[0] =ALL_OBS[i];
				p1[1] =ALL_OBS[i+1];
				p2[0] =ALL_OBS[i+2];
				p2[1] =ALL_OBS[i+3];
				p3[0] =ALL_OBS[i+4];
				p3[1] =ALL_OBS[i+5];
				p4[0] =ALL_OBS[i+6];
				p4[1] =ALL_OBS[i+7];
				//ROS_INFO("x1 %f y1 %f x2 %f y2 %f", p1[0], p1[1], p2[0], p2[1] );

				 ints1 = isIntersecting(p1,p2,A,B);
				 ints2 = isIntersecting(p1,p4,A,B);
				 ints3 = isIntersecting(p3,p2,A,B);
				 ints4 = isIntersecting(p3,p4,A,B);

				 center[0] = (p2[0]-p1[0])/2;
				 center[1] = (p2[1]-p1[1])/2;
				 ints5 = isIntersectingCap(center, A, B);
				 center[0] = (p3[0]-p4[0])/2;
				 center[1] = (p3[1]-p4[1])/2;
				 ints6 = isIntersectingCap(center, A, B);
      	//if(ints5 == 0 and ints6 == 0 and ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==1){
	if(ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==1){
            nc = 1;
	    
        }else{
            nc = 0;
	    break;
        }

    }
		//ROS_INFO("%d", target_num);
		for(int i =0 ; i<ALL_OBJ.size(); i = i+8){
				//ROS_INFO("ALL OBJ size %d", ALL_OBJ.size());
				if(i == target_num*16){
					//ROS_INFO("skipped");
					i = i+16;
				}
				p1[0] =ALL_OBJ[i];
				p1[1] =ALL_OBJ[i+1];
				p2[0] =ALL_OBJ[i+2];
				p2[1] =ALL_OBJ[i+3];
				p3[0] =ALL_OBJ[i+4];
				p3[1] =ALL_OBJ[i+5];
				p4[0] =ALL_OBJ[i+6];
				p4[1] =ALL_OBJ[i+7];
				//ROS_INFO("x1 %f y1 %f x2 %f y2 %f", p1[0], p1[1], p2[0], p2[1] );

				 ints1 = isIntersecting(p1,p2,A,B);
				 ints2 = isIntersecting(p1,p4,A,B);
				 ints3 = isIntersecting(p3,p2,A,B);
				 ints4 = isIntersecting(p3,p4,A,B);

				 center[0] = (p2[0]-p1[0])/2;
				 center[1] = (p2[1]-p1[1])/2;
				 ints5 = isIntersectingCap(center, A, B);
				 center[0] = (p3[0]-p4[0])/2;
				 center[1] = (p3[1]-p4[1])/2;
				 ints6 = isIntersectingCap(center, A, B);

       // if(ints5 == 0 and ints6 == 0 and ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==1){
	if(ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==1){
            nc = 1; //is outside c-space
	    
        }else{
            nc = 0; //is inside c-space
	    break;
        }

    }
		//ROS_INFO("CheckIntersect called %d", nc);
		return nc;
}

float isGoalInCSpace(float x, float y){
	float A[] = {x, y};
	float center[2];
	float m;
	nc = 1;
	for(int i =0 ; i<ALL_OBS.size(); i = i+8){

			p1[0] =ALL_OBS[i];
			p1[1] =ALL_OBS[i+1];
			p2[0] =ALL_OBS[i+2];
			p2[1] =ALL_OBS[i+3];
			p3[0] =ALL_OBS[i+4];
			p3[1] =ALL_OBS[i+5];
			p4[0] =ALL_OBS[i+6];
			p4[1] =ALL_OBS[i+7];
			center[0] = (p3[0]-p1[0])/2;
			center[1]	= (p3[1]-p1[1])/2;
			m= (float) atan2((p2[1]-p1[1]), (p2[0]-p1[0]));

			//ROS_INFO("x1 %f y1 %f x2 %f y2 %f", p1[0], p1[1], p2[0], p2[1] );

			 ints1 = isIntersecting(p1,p2,A,center);
			 ints2 = isIntersecting(p1,p4,A,center);
			 ints3 = isIntersecting(p3,p2,A,center);
			 ints3 = isIntersecting(p3,p4,A,center);

			/*	 center[0] = (p2[0]-p1[0])/2;
				 center[1] = (p2[1]-p1[1])/2;
				 ints5 = isIntersectingCap(center, A, center);
				 center[0] = (p3[0]-p4[0])/2;
				 center[1] = (p3[1]-p4[1])/2;
				 ints6 = isIntersectingCap(center, A, B);
			*/
        if( ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==1){
					nc = 0; //is inside cspace
					break;
			}else{
					nc = 1; //is outside cspace
			}

	}

	for(int i =0 ; i<ALL_OBJ.size(); i = i+8){
			if(mode == 1 && i/8 == target_num){
				continue;
			}
			p1[0] =ALL_OBJ[i];
			p1[1] =ALL_OBJ[i+1];
			p2[0] =ALL_OBJ[i+2];
			p2[1] =ALL_OBJ[i+3];
			p3[0] =ALL_OBJ[i+4];
			p3[1] =ALL_OBJ[i+5];
			p4[0] =ALL_OBJ[i+6];
			p4[1] =ALL_OBJ[i+7];
			center[0] = (p3[0]-p1[0])/2;
			center[1]	= (p3[1]-p1[1])/2;
			m= (float) atan2((p2[1]-p1[1]), (p2[0]-p1[0]));

			//ROS_INFO("x1 %f y1 %f x2 %f y2 %f", p1[0], p1[1], p2[0], p2[1] );

			 ints1 = isIntersecting(p1,p2,A,center);
			 ints2 = isIntersecting(p1,p4,A,center);
			 ints3 = isIntersecting(p3,p2,A,center);
			 ints3 = isIntersecting(p3,p4,A,center);

			/*	 center[0] = (p2[0]-p1[0])/2;
				 center[1] = (p2[1]-p1[1])/2;
				 ints5 = isIntersectingCap(center, A, center);

				 center[0] = (p3[0]-p4[0])/2;
				 center[1] = (p3[1]-p4[1])/2;
				 ints6 = isIntersectingCap(center, A, B);
			*/
        if( ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==1){
					nc = 0; //is inside cspace
					break;
			}else{
					nc = 1; //is outside cspace
			}

	}

	for(int i =0 ; i<ALL_WALL.size(); i = i+8){

			p1[0] =ALL_WALL[i];
			p1[1] =ALL_WALL[i+1];
			p2[0] =ALL_WALL[i+2];
			p2[1] =ALL_WALL[i+3];
			p3[0] =ALL_WALL[i+4];
			p3[1] =ALL_WALL[i+5];
			p4[0] =ALL_WALL[i+6];
			p4[1] =ALL_WALL[i+7];
			center[0] = (p3[0]-p1[0])/2;
			center[1]	= (p3[1]-p1[1])/2;
			m= (float) atan2((p2[1]-p1[1]), (p2[0]-p1[0]));

			//ROS_INFO("x1 %f y1 %f x2 %f y2 %f", p1[0], p1[1], p2[0], p2[1] );

			 ints1 = isIntersecting(p1,p2,A,center);
			 ints2 = isIntersecting(p1,p4,A,center);
			 ints3 = isIntersecting(p3,p2,A,center);
			 ints3 = isIntersecting(p3,p4,A,center);

			/*	 center[0] = (p2[0]-p1[0])/2;
				 center[1] = (p2[1]-p1[1])/2;
				 ints5 = isIntersectingCap(center, A, center);

				 center[0] = (p3[0]-p4[0])/2;
				 center[1] = (p3[1]-p4[1])/2;
				 ints6 = isIntersectingCap(center, A, B);
			*/
        if( ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==1){
					nc = 0; //is inside cspace
					break;
			}else{
					nc = 1; //is outside cspace
			}

	}
	/*if(nc == 1){
		return m;
	}else{
		return 100.0;
	}*/
	return nc;
}

float randnum;
float randZO(float min, float max){
    randnum = (rand() % ((int) max*100))/100.0f; // + (min))/100.0f;
	  //ROS_INFO("r: %f",r);
		return randnum;
}

float calculatePathDistance(){
	float sumPathDist;
	float stepDistP1[2];
	float stepDistP2[2];
	sumPathDist = 0;
	for(int i = 0; i<finalpathx.size()-1; i++){
		stepDistP1[0] = finalpathx[i];
		stepDistP1[1] = finalpathy[i];
		stepDistP2[0] = finalpathx[i+1];
		stepDistP2[1] = finalpathy[i+1];
		sumPathDist += dist(stepDistP1, stepDistP2);
	}
	return sumPathDist;
}

void runRRT(float goalPositionX, float goalPositionY){
	  ros::Time start_time = ros::Time::now();
		int p = 0;
		//ROS_INFO("Start: startx %f starty %f", startx, starty);
		std::vector<Node> nodes;
		std::vector<Node> q_nearest;
 		std::vector<float> ndist;
		float startposex=pose.pose.pose.position.x;
		float startposey=pose.pose.pose.position.y;
		ROS_ERROR("pose: %f, %f", pose.pose.pose.position.x, pose.pose.pose.position.y);
		Node start (pose.pose.pose.position.x, pose.pose.pose.position.y,0, 0);
		/*while(!isGoalInCSpace(startposex,startposey)){
			std::srand(unsigned ( std::time(0) )); 
			startposex = ((float)(std::rand()%30 - 15.0))/100.0f + pose.pose.pose.position.x;
			//std::srand(ros::Time::now().toSec()); 
			startposey = ((float)(std::rand()%30 - 15.0))/100.0f + pose.pose.pose.position.y;
			ROS_ERROR("%f %f", startposex, startposey);
		}*/
		
		Node goal (0,0,0,0);
		goal = Node(goalPositionX, goalPositionY, 0,0);
		nodes.push_back(start);
		pathFound = 0;
		Node q_rand (randZO(0, XDIM),randZO(0,YDIM),0,0);
		float r1 = 0.1500f;
		float r2 = 0.22f;
		float temp_dist = 0.0f;
		float ncoord[2];
		float smallest_dist = 1.0f;
		int smallest_dist_indx = 0;
		Node q_near (0,0,0,0);
		Node q_new (0, 0, 0, 0);
		Node q_end (0,0,0,0);
		Node q_final (0,0,0,0);
		Node q_min (0,0,0,0);
		float c_min = 0.0f;
		std::vector<float> distances;
		std::vector<float> points;
		for(int ctr = 0; ctr<NUMNODES; ctr++){
			//ROS_ERROR(" ctr %d",ctr);
			for(int j = 0; j<nodes.size(); j++){
					if(  (sqrt(pow(nodes[j].pos[0]-goal.pos[0], 2) + pow(nodes[j].pos[1]-goal.pos[1],2)) <= r1)){
						pathFound = 1;
					}
			}
			if(pathFound == 1){
				ROS_ERROR("node set to goal");
				break;
			}
			q_rand.pos[0] = randZO(0,XDIM);
			q_rand.pos[1] = randZO(0,YDIM);
			ndist.clear();
			//ROS_INFO("loop 1");
			for(int j = 0; j<nodes.size();j++){
				temp_dist = dist(nodes[j].pos, q_rand.pos);
				//ROS_INFO("dist %f", temp_dist);
				ndist.push_back(temp_dist);
				//ROS_INFO("temp_dist = %f",temp_dist);
			}
			smallest_dist = ndist[0];
			smallest_dist_indx = 0;
			//ROS_INFO("loop 2");
			for(int j = 0; j <ndist.size();j++){
				if(ndist[j]<smallest_dist){
					smallest_dist = ndist[j];
					smallest_dist_indx = j;
				}
			}
			//ROS_INFO("smallest_dist_dist = %f, index = %d",smallest_dist, smallest_dist_indx);

			q_near = nodes[smallest_dist_indx];
			//ROS_INFO("checkpoint 2");

			step_from_to(q_rand.pos, q_near.pos, q_new.pos);
			//ROS_INFO("2  %f %f", q_new_pos[0], q_new_pos[1]);
			if(checkIntersect(q_rand, q_near)){
				//ROS_INFO("checkpoint 3");

				q_new.cost = dist(q_new.pos, q_near.pos) + q_near.cost;
				q_nearest.clear();
				for(int j= 0; j<nodes.size(); j++){
					if((checkIntersect(nodes[j], q_new)) and (dist(nodes[j].pos,q_new.pos)<=r2)){
						//ROS_INFO("Second intersect check");
						q_nearest.push_back(nodes[j]);
					}
				}
				q_min.pos[0]= q_near.pos[0];
				q_min.pos[1] =q_near.pos[1];
				c_min = q_new.cost;
				//ROS_INFO("2  %f %f", q_min[0], q_min[1]);
				//ROS_INFO("loop 4");
				for(int j= 0; j<q_nearest.size(); j++){
					//ROS_INFO("i %d, j %d", ctr, j);
					if((checkIntersect(q_nearest[j], q_new)) and (q_nearest[j].cost + dist(q_nearest[j].pos, q_new.pos) < c_min)){
						  //ROS_INFO("Third intersect check");
							q_min.pos[0] = q_nearest[j].pos[0];
							q_min.pos[1] = q_nearest[j].pos[1];
							c_min = q_nearest[j].cost + dist(q_nearest[j].pos,q_new.pos);
					}
				}
				//ROS_INFO("loop 5");
				for(int j= 0; j<nodes.size(); j++){
					if(nodes[j].pos[0] == q_min.pos[0] and nodes[j].pos[1] == q_min.pos[1]){
						q_new.parent = j;
					}
				}
			  nodes.push_back(q_new);
			}
		}
		distances.clear();
		float td;
		//float np[2];

		for(int c =0; c < nodes.size(); c++){
			td = dist(nodes[c].pos, goal.pos);
			distances.push_back(td);
		}
		smallest_dist = distances[0];
		smallest_dist_indx = 0;

		for(int j = 0; j <nodes.size();j++){
			if(distances[j]<smallest_dist){
				smallest_dist = distances[j];
				smallest_dist_indx = j;
			}
		}

		q_final = nodes[smallest_dist_indx];
		goal.parent = smallest_dist_indx;
		q_end = goal;

		nodes.push_back(goal);
		int idx = 0;
		finalpathx.clear();
		finalpathy.clear();

		points.clear();
		finalpathx.push_back(q_end.pos[0]);
		finalpathy.push_back(q_end.pos[1]);
		//ROS_INFO("checkpoint");
		while(q_end.parent != 0){

			idx = q_end.parent;
			finalpathx.push_back(nodes[idx].pos[0]);
			finalpathy.push_back(nodes[idx].pos[1]);
			//ROS_INFO(" idx %d", idx);
			q_end = nodes[idx];

		}
		finalpathx.push_back(start.pos[0]);
		finalpathy.push_back(start.pos[1]);
		//ROS_INFO("Checkpoint");
		newPathGenerated = 1;
}

int getBestObject(){
	std::vector<float> pathDistances;
	float distance;
	float smallest_dist;
	int smallest_dist_indx;
	std::vector<bool> pathExists;

	for(int i = 0; i<objStack.Objects.size(); i++){
		target_num = i;
		runRRT(objStack.Objects[i].x, objStack.Objects[i].y);
		pathExists.push_back(pathFound);
		distance = calculatePathDistance();
		pathDistances.push_back(distance*(10000-objStack.Objects[i].value));
	}

	smallest_dist = pathDistances[0];
	smallest_dist_indx = 0;
	for(int j = 0; j <pathDistances.size();j++){
		if(pathDistances[j]<smallest_dist){
			if(pathExists[j]=0){
				continue;
			}
			smallest_dist = pathDistances[j];
			smallest_dist_indx = j;
		}
	}
	return smallest_dist_indx;
}

float goalx;
float goaly;

bool runrrt = 0;
bool rrtCallback(rosie_path_finder::rrtService::Request &req, rosie_path_finder::rrtService::Response &res){
	ROS_ERROR("----- sth comming");
	ros::Rate loop_rate(10);
	while(!mapInitialized){
		loop_rate.sleep();
		ros::spinOnce();
		ROS_ERROR("----- Map not initialized");
	}
		
	goalx = req.goalx;
	goaly = req.goaly;
	mode = req.mode;
	//ROS_ERROR("goalx %f, goal y %f, mode %d", goalx,goaly, mode);
	if(mode == 0){
		target_num = -1;
		res.tar_num = target_num;
		runrrt = 1;
	}else if(mode == 1){
		target_num = getBestObject();
		goalx = objStack.Objects[target_num].x;
		goaly = objStack.Objects[target_num].y;
		runrrt = 1;
		res.tar_num = target_num;
	}
	pathInitialized = 0;
	return true;
}

char onePathPublished = 0;
void publishPath(){
	geometry_msgs::PoseStamped newpose;
	allposes.clear();

	ros::Time now = ros::Time::now();
	if(newPathGenerated){
		pathseq++;
		newPathGenerated = 0;
	}else{
		return;
	}
	path.header.seq=1;//pathseq;
	path.header.stamp = now;
	path.header.frame_id = "map";
	int seq = 0;
	for(int i = finalpathx.size()-1; i >= 0; i--){

			newpose.header.seq=seq;
			newpose.header.stamp = now;
			newpose.header.frame_id = "pathpoint";

			newpose.pose.position.x = finalpathx[i];
			newpose.pose.position.y = finalpathy[i];
			newpose.pose.position.z = 0;
			newpose.pose.orientation.x = 0;
			newpose.pose.orientation.y = 0;
			newpose.pose.orientation.w = 1;
			if(i > 0){
					newpose.pose.orientation.z =(float) atan2((finalpathy[i]-finalpathy[i+1]),(finalpathx[i]-finalpathx[i+1]));
			}else if( i == 0){
				/*//int cspaceCheck = isGoalInCSpace(finalpathx[i], finalpathy[i]);				
				Node scdlast (finalpathx[i+1],finalpathx[i+1],0,0);
				Node goalpose (finalpathx[i],finalpathy[i],0,0);
				if(!checkIntersect(goalpose,scdlast)){
					int retries = 0;
					while(!checkIntersect(goalpose,scdlast) && retries++ < 1000){
						goalpose.pos[0] = ((float)(std::rand()%15 - 7.5))/100.0f + finalpathx[i];
						//std::srand(ros::Time::now().toSec()); 
						goalpose.pos[1] = ((float)(std::rand()%15 - 7.5))/100.0f + finalpathy[i];
						ROS_ERROR("%f %f", goalpose.pos[0], goalpose.pos[1]);
					}
					newpose.pose.orientation.z =(float) atan2((finalpathy[i]-goalpose.pos[0]),(finalpathx[i]-goalpose.pos[1]));
				}else{
					newpose.pose.orientation.z += 0;
				}*/
				newpose.pose.orientation.z += 0;
			}
			//poses[i] = newpose;
			allposes.push_back(newpose);

			seq = seq +1 ;
	}
	path.poses.resize(allposes.size());

	for(int i = 0; i < allposes.size(); ++i){
		path.poses[i] = allposes[i];
	}
	pathInitialized = 1;
	//path.poses = poses;
	//ROS_INFO("Checkpoint");

}

int main(int argc, char **argv){
	ros::init(argc, argv, "rosie_rrt");
	std::srand(unsigned ( std::time(0) )); 

	//target_tfl_ptr.reset(new tf::TransformListener);

	ros::NodeHandle n;
	pathClient = n.serviceClient<rosie_path_navigator::PathNavigationService>("/rosie_path_service");

	//ros::Subscriber wall_sub = n.subscribe<visualization_msgs::MarkerArray>("/maze_map", 1000, wallCallback);
	ros::Subscriber loc_cert_sub = n.subscribe<std_msgs::Float32>("/localization_certainty", 10, certaintyCallback);
	ros::Subscriber wallStack_sub = n.subscribe<rosie_map_controller::MapStoring>("/wall_stack", 1000, wallCallback2);
	ros::Subscriber objStack_sub = n.subscribe<rosie_map_controller::ObjectStoring>("/object_stack", 1000, objCallback);
	ros::Subscriber pose_sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, currentPoseCallback);
	//ros::Subscriber evidence_sub = n.subscribe<rosie_object_detector::RAS_Evidence>("/evidence",10, evidenceCallback);
	path_pub = n.advertise<nav_msgs::Path>("/rosie_path",10);
	ros::ServiceServer rrtService = n.advertiseService<rosie_path_finder::rrtService::Request,
							 rosie_path_finder::rrtService::Response>("/rrt", rrtCallback);
	loadClient = n.serviceClient<rosie_map_controller::RequestLoading>("request_load_mapping");

	static tf::TransformBroadcaster br;
	ros::Rate loop_rate(10);
	ros::Time load_time = ros::Time::now();

    while(ros::ok()){
			
		if(mapInitialized && (certaintyValue < certaintyLimit)){
			if(runrrt){
				runrrt = 0;
				ROS_ERROR("RUNRRT");
				runRRT(goalx, goaly);
				publishPath();
				mapInitializing= 0;
				objCallInitializing = 0;
				mapInitialized= 0;
			}
			if(pathInitialized){
				
				
				pathSrv.request.path=path;
				pathSrv.request.id = pathseq;
				if(pathClient.call(pathSrv)){
					onePathPublished = 1;
				}
		
				//For visualization
				path_pub.publish(path);
				
				//ROS_INFO("path published");
			}
			tf::Transform transform;
			transform.setOrigin( tf::Vector3(0,0, 0) );
			tf::Quaternion qtf;
			qtf.setRPY(0, 0, 0);
			transform.setRotation( qtf );
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "path"));
		}
		
		if(!mapInitialized){
			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}		
		
		// =0 ;
		ros::spinOnce();
		loop_rate.sleep();
	}	
}
