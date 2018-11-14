#include <ros/ros.h>
#include <ros/time.h>

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

#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <string>

#include <rosie_servo_controller/ControlGates.h>
#include <rosie_map_controller/RequestRerun.h>

float PI = 3.1415926;
float EPSILON = 0.4;
float XDIM = 2.40;
float YDIM = 2.40;
float startx = 0.200;
float starty = 0.400;
float xpoint = 1.500;
float ypoint = 1.200;
float OFFSET[] = {0.0, 0.00};
float HOME[] = {startx, starty};
int NUMNODES = 10000;
int numnodes_tot = 5000;
float RADIUS = 0.20;
int j = 0;
float alpha = 0.6;
float beta = 0.4;

float distance = 0.0f;
float resolution = 0.01;
float robotsize = 0.2;

//float* wallArray;
//float* OBS;
ros::Publisher path_pub;

std::vector<float> wallArray;
std::vector<float> OBS;

int numbMarkers;
//colors:
  // yellow = 1
  // green = 2
  // orange = 3
  // red = 4
  // blue = 5
  // purple = 6
//shapes:
  // ball = 1
  // square = 2
  // zylinder = 3
  // plus = 4
  // star = 5
  // triangle = 6
int objSize = 0.05;
int objNumber = 14;
int objColor[] = {1,1,2,2,2,3,3,4,4,4,5,5,6,6};
int objShape[] = {1,2,2,3,2,4,5,3,2,1,2,6,4,5};
//std::vector<float> objPoseX(14,-1.0);
//std::vector<float> objPoseY(14,-1.0);
float objPoseX[] = {2.0, 1.20,2.10,2.10,2.10,2.10,2.10,2.10,2.10,2.10,2.10,2.10,2.10,2.10};
float objPoseY[] = {2.0, 1.50,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30,0.30};
std::vector<float> objPose();

int batSize = 0.10;
int batNumber = 4;
//std::vector<float> batPoseX(4,-1.0);
//std::vector<float> batPoseY(4,-1.0);
float batPoseX[] = {1.20,1.20,1.20,1.20};
float batPoseY[] = {2.10,2.20,2.10,2.10};
int valueOrder[] = {6, 5, 4, 3, 2, 1};

nav_msgs::Odometry pose;
nav_msgs::Path path;

std::string lastMode = "wait";
std::string mode = "goto";

//float* finalpathx;
//float* finalpathy;

std::vector<float> finalpathx;
std::vector<float> finalpathy;

int rerun_rrt = 0;
int pathseq = 0;
int mapInitialized = 0;
int pathInitialized = 0;
int pathPublished = 0;

void wallCallback(const visualization_msgs::MarkerArray msg){
	if(!mapInitialized){
		ROS_INFO("Initializing!");


		numbMarkers = msg.markers.size();
		//markers = (visualization_msgs::Marker*)malloc(sizeof(visualization_msgs::Marker)*numbMarkers);
		//markers = msg.markers;

		int czone = robotsize/(2.0) + 0.02; //additional extra security distance

		wallArray.clear();
		wallArray.resize(4*numbMarkers);
		for(int i = 0; i < 4*numbMarkers; ++i){
			wallArray[i] = -1.0f;
		}

		float minX, minY, maxX, maxY;
		minX = minY = maxX = maxY = 0;
		for(int k = 0; k < numbMarkers; ++k){
			//Extract point data
			std::string pointsText = msg.markers[k].text;
			std::stringstream ss(pointsText);
			float sX, sY, eX, eY;
			ss>>sX;
			ss>>sY;
			ss>>eX;
			ss>>eY;

			//Set point data on every 4th index
			wallArray[k<<2]=sX;
			wallArray[(k<<2)+1]=sY;
			wallArray[(k<<2)+2]=eX;
			wallArray[(k<<2)+3]=eY;

			if(sX < minX){
				minX = sX;
			}
			if(sX > maxX){
				maxX = sX;
			}
			if(eX < minX){
				minX = eX;
			}
			if(eX > maxX){
				maxX = eX;
			}
			if(sY < minY){
				minY = sY;
			}
			if(sY > maxY){
				maxY = sY;
			}
			if(eY < minY){
				minY = eY;
			}
			if(eY > maxY){
				maxY = eY;
			}
			//ROS_INFO("sX: %f, sY: %f, eX: %f, eY: %f", wallArray[k], wallArray[k+1], wallArray[k+2], wallArray[k+3]);
		}

		float offsetX = minX;
		float offsetY = minY;

		XDIM = (maxX - minX);
		YDIM = (maxY - minY);

		//ROS_INFO("minX: %f, minY: %f, maxX: %f, maxY: %f, offsetX: %f, Y: %f", minX, minY, maxX, maxY, offsetX, offsetY);
		//ROS_INFO("XDIM: %f, YDIM: %f", XDIM, YDIM);

		mapInitialized = 1;
	}
}

void poseCallback(nav_msgs::Odometry msg){ // for re-calculation of the path when needed
    pose = msg;
		pose.pose.pose.position.x = 0.25;
		pose.pose.pose.position.y = 0.40;
    //q_start.coord = {pose.pose.pose.position.y + OFFSET[0] ,pose.pose.pose.position.x + OFFSET[1]};
}

void objectCallback(visualization_msgs::Marker mrk){ //get object positions (sorted after value and distance)
  //colors:
    // yellow = 1
    // green = 2
    // orange = 3
    // red = 4
    // blue = 5
    // purple = 6
  //shapes:
    // ball = 1
    // square = 2
    // zylinder = 3
    // plus = 4
    // star = 5
    // triangle = 6


		// from here on please uncommend
		/*
    int shape = mrk.id;
    int color = mrk.type;
    int sh_nr = 0;
    int col_nr = 0;
    switch(shape){
      case 1: sh_nr = 1; // quader
      case 2: sh_nr = 2; // ball
      case 3: sh_nr = 3; // triangle
      case 4: sh_nr = 4; // plus
      case 5: sh_nr = 5; // star
      case 6: sh_nr = 6; // cylinder
      default: sh_nr = 0;
    }
    switch(color){
      case 1: col_nr = 1; //yellow
      case 2: col_nr = 2; //green
      case 3: col_nr = 3; //orange
      case 4: col_nr = 4; // red
      case 5: col_nr = 5; // blue
      case 6: col_nr = 6; // purple
      default: col_nr = 0;
    }
    int index = 0;
    for(int i = 0; i< objNumber; ++i){
      if(objColor[i] == col_nr and objShape[i] == sh_nr){
          index = i;
          break;
      }else{
        index = 0;
      }

	    float posX = mrk.pose.position.x + pose.pose.pose.position.x;
	    float posY = pose.pose.pose.position.y;
	    float r = 0.05; //5cm radius for faulty measurement
	    if(objPoseX[i] == -1 and objPoseY[i] == -1){
	      objPoseX[i] = posX;
	      objPoseY[i] = posY;
	      ROS_INFO("NEW object");
	    }else{
	      if((posX-objPoseX[i])*(posX-objPoseX[i])+(posY-objPoseY[i])*(posY-objPoseY[i]) > (r*r)){
	        objPoseX[i] = posX;
	        objPoseY[i] = posY;
	        ROS_INFO("This object has been moved.");
	      }else{
	        ROS_INFO("This object is already mapped.");
	      }
	    }
		}
		*/
    //sortObjects();

}

void batteryCallback(visualization_msgs::Marker btr){ //get object positions (sorted after value and distance)
		/*
	  int index = 100;
    float posX = btr.pose.position.x + pose.pose.pose.position.x;
    float posY = pose.pose.pose.position.y;
    float r = 0.1; //10cm radius for faulty measurement
    for(int i = 0; i< batPoseX.size(); ++i){
        if((posX-objPoseX[i])*(posX-objPoseX[i])+(posY-objPoseY[i])*(posY-objPoseY[i]) < (r*r)){
          index = i;
          ROS_INFO("Battery is already mapped.");
        }else{
          index = 100;
          ROS_INFO("NEW battery");
        }
    }
    if(index == 100){
      for(int i = 0; i< batPoseX.size(); ++i){
        if(batPoseX[i] == -1 and batPoseY[i] == -1){
            batPoseX[i] = posX;
            batPoseY[i] = posY;
            break;
        }
      }
    }
		*/
}
/*
class Cost{
  public:
		float x;
		float y;
		float cost;
		int name;
		int parent;

		Cost(int x, int y, int p) : x(x), y(x), cost(0), name(p), parent(0) {}
};
*/


class Node{
public:
	float x;
	float y;
	float cost;
	int parent;

	Node(float x_in, float y_in, float c_in, int p_in) //x(x), y(y), cost(c), parent(p) {}
	{
		x = x_in;
		y = y_in;
		cost = c_in;
		parent = p_in;
	}
};


float angle(float p1[], float p2[]){
		 float angle = (float) atan2(p2[1]-p1[1], p2[0]-p1[0]);
		 //ROS_INFO("p1[0]: %f, p1[1]: %f, p2[0]: %f, p2[1]: %f,", p1[0],p1[1],p2[0],p2[1]);
		 return angle;
}

bool ccw(float A[], float B[], float C[]){
     bool val = (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0]);
     return val;
}

float dist(float q1[], float q2[]){
    float d = sqrt(pow(q1[0]-q2[0],2) + pow(q1[1]-q2[1],2));
		//ROS_INFO("dist in fct f%", d);
    return d;
}

float *step_from_to(float p1[], float p2[], float p_new[]){
	if(dist(p1, p2) >= EPSILON){
		float theta = atan2(p1[1] - p2[1], p1[0] - p2[0]);
		p_new[0] = p2[0] + EPSILON*cos(theta);
		p_new[1] = p2[1] + EPSILON*sin(theta);
	}else{
		p_new[0] = p1[0];
		p_new[1] = p1[1];
	}

}
/*
float steer(float qr[], float qn[], float val, float eps){
    float qnew[] = { 0, 0};

    //Steer twoards qn with maximum step size of eps
    if(val >= eps){
        qnew[0] = qn[0] + ((qr[0]-qn[0])*eps)/dist(qr, qn);
        qnew[1] = qn[1] + ((qr[1]-qn[1])*eps)/dist(qr, qn);
    }else{
        qnew[0] = qr[0];
        qnew[1] = qr[1];
    }
    float A[] = {qnew[0], qnew[1]};
    return A;
}
*/
bool checkIntersect(Node n2, Node n1){         //array definition might be wrong
    float A[]= {n1.x,n1.y};
    float B[]= {n2.x,n2.y};
		float origin[] = {0.0,0.0};
		float temp[2];
		float m;
		int r = 0.2;
		float x1,y1,x2,y2;
		float p1[2];
		float p2[2];
    bool nc = true;
		float diffy;
		float diffx;
		float P1[2];
		float P2[2];
		float P3[2];
		float P4[2];
		bool ints1;
		bool ints2;
		bool ints3;
		bool ints4;
		bool ints5;
		bool ints6;
		float n;
    for(int i =0 ; i<OBS.size(); i = i+4){
        //float abst = sqrt(pow(OBS[i+2]-OBS[i+0],2.0)+pow(OBS[i+3]-OBS[i+1],2.0));
        //float m1 = (OBS[i+2]-OBS[i+0])/dist;
        //float m2 = (OBS[i+3]-OBS[i+1])/dist;
				p1[0] =OBS[i];
				p1[1] =OBS[i+1];
				p2[0] =OBS[i+2];
				p2[1] =OBS[i+3];

				if((dist(p2,origin) <= dist(p1,origin)) or (p2[1]<p1[1])){
					temp[0] = p1[0];
					temp[1] = p1[1];
					p1[0] =p2[0];
					p1[1] = p2[1];
					p2[0] = temp[0];
					p2[1] = temp[1];
				}
				diffx = p2[0]-p1[0];
				diffy = p2[1]-p1[1];
				m = (float) atan2(diffy, diffx);
				n = (sin(m)>cos(m))?sin(m):cos(m);


        if(0.785398 <= m){
            x1 = OBS[i+0]-r;
            y1 = OBS[i+1]-r;
            x2 = OBS[i+2]+r;
            y2 = OBS[i+3]+r;
        }else{
            x1 = OBS[i+0]+r;
            y1 = OBS[i+1]+r;
            x2 = OBS[i+2]-r;
            y2 = OBS[i+3]-r;
        }

				//ROS_INFO("m1 %f, x1 %f, y1 %f, x2 %f, y2 %f", m, x1, y1, x2, y2);
      //  m1 = 1-m1;
      //  m2 = 1-m2;
				m = m+PI;

         P1[0] = x1-r;
				 P1[1] = y1-r;
				 P2[0] = x1+r;
         P2[1] = y1-r;
				 P3[0] = x2-r;
         P3[1] = y2+r;
         P4[0] = x2+r;
				 P4[1] = y2+r;

				 ints1 = ccw(A,P1,P4) != ccw(B,P1,P4) and ccw(A,B,P1) != ccw(A,B,P4);
         ints2 = ccw(A,P1,P2) != ccw(B,P1,P2) and ccw(A,B,P1) != ccw(A,B,P2);

				 ints5 = ccw(A,P1,P3) != ccw(B,P1,P3) and ccw(A,B,P1) != ccw(A,B,P3);
         ints6 = ccw(A,P2,P4) != ccw(B,P2,P4) and ccw(A,B,P2) != ccw(A,B,P4);

         ints3 = ccw(A,P3,P2) != ccw(B,P3,P2) and ccw(A,B,P3) != ccw(A,B,P2);
         ints4 = ccw(A,P3,P4) != ccw(B,P3,P4) and ccw(A,B,P3) != ccw(A,B,P4);

				/*
        float C1[2] = P1;
        float C2[2] = P1;
        float C3[2] = P3;
        float C4[2] = P3;
        float D1[2] = P4;
        float D2[2] = P2;
        float D3[2] = P2;
        float D4[2] = P4;

        bool ints1 = ccw(A,C1,D1) != ccw(B,C1,D1) and ccw(A,B,C1) != ccw(A,B,D1);
        bool ints2 = ccw(A,C2,D2) != ccw(B,C2,D2) and ccw(A,B,C2) != ccw(A,B,D2);
        bool ints3 = ccw(A,C3,D3) != ccw(B,C3,D3) and ccw(A,B,C3) != ccw(A,B,D3);
        bool ints4 = ccw(A,C4,D4) != ccw(B,C4,D4) and ccw(A,B,C4) != ccw(A,B,D4);
				*/
        if(ints1==0 && ints2==0 && ints3==0 & ints4==0 & ints5==0 & ints6==0 & nc ==1){
            nc = true;
        }else{
            nc = false;
        }

    }
		return nc;
}
/*
Node *chooseParent(Node nn, Node newnode, Node nodes[], int p){
	float newnodep[] = {newnode.x, newnode.y};
	float nnp[] = {nn.x, nn.y};
	int parent = NUMNODES;
	for(int i = 0; i <= p; ++i){
		Node k = nodes[i];
		float kp[] = {k.x, k.y};
		if(checkIntersect(k, newnode) and dist(kp, newnodep) < RADIUS and k.cost + dist(kp, newnodep) < nn.cost + dist(nnp, newnodep)){
			nn = k;
			parent = i;
		}
	}
	newnode.cost = nn.cost + dist(nnp, newnodep);
	newnode.parent = parent;
	Node result[2] = {newnode, nn};
	return result;
}*/
/*
Node * reWire(Node nodes[], Node newnode, int p){
	float newnodep[] = {newnode.x, newnode.y};
  for(int i = 0; i <= p; ++i){
		Node k = nodes[i];
		float kp[] = {k.x, k.y};
		if( checkIntersect(k, newnode) and i != newnode.parent and dist(kp, newnodep) < RADIUS and newnode.cost + dist(kp, newnodep) < k.cost){
			k.parent = i;
			k.cost = newnode.cost + dist(kp, newnodep);
			nodes[i] = k;
		}
	}
	return nodes;

}*/
/*
int seq = 0;
void publishGrid(){
    mapgrid.header.seq = seq++;
    mapgrid.header.stamp = ros::Time::now();
    mapgrid.header.frame_id = "map";

    mapgrid.info.origin.position.x = 0;
    mapgrid.info.origin.position.y = 0;
    mapgrid.info.origin.position.z = 0;
    mapgrid.info.origin.orientation.x = 0;
    mapgrid.info.origin.orientation.y = 0;
    mapgrid.info.origin.orientation.z = 0;
    mapgrid.info.origin.orientation.w = 1;

    mapgrid.info.map_load_time = load_time;
    mapgrid.info.resolution = resolution;
    mapgrid.info.XDIM = XDIM;
    mapgrid.info.YDIM = YDIM;

    mapgrid.data.clear();

    //mapgrid.data = (std_msgs::Int8*)malloc(sizeof(std_msgs::Int8)*XDIM*YDIM);
    for(int i = 0; i < XDIM*YDIM; ++i){
        mapgrid.data.push_back(0);                              // put values in with path
    }

    grid_publisher.publish(mapgrid);
}*/

bool check_for_endposition(){
	int r = 0.05;
	if(((pose.pose.pose.position.x-xpoint)*(pose.pose.pose.position.x-xpoint)+(pose.pose.pose.position.y-ypoint)*(pose.pose.pose.position.y-ypoint)) < r*r){
		return true;
	}
}

float randZO(float min, float max)
{
    float r = (rand() % ((int) max*100))/100.0f; // + (min))/100.0f;
	  //ROS_INFO("r: %f",r);
		return r;
}



void runRRT(){
		OBS.clear();
		if(mode == "goto"){
			ROS_INFO("entered mode GOTO");
			//OBS.reserve(4*(numbMarkers+(sizeof(batPoseX)*sizeof(batPoseX[0]))+(sizeof(objPoseX)*sizeof(objPoseX[0])-1))); //Walls +  Batteries + other objects then target

			//wall elements
			for(int i = 0; i<(wallArray.size()); i++){
				OBS.push_back(wallArray[i]);
			}
			//obj elements (except first element)
			for(int i = 1; i<(sizeof(objPoseY)*sizeof(objPoseY[0])); ++i){
				float temp[] = {objPoseX[i]-objSize/2, objPoseY[i]-objSize/2, objPoseX[i]-objSize/2, objPoseY[i]+objSize/2, objPoseX[i]+objSize/2, objPoseY[i]-objSize/2, objPoseX[i]+objSize/2, objPoseY[i]+objSize/2};
				for(int j = 0; j<8; ++j){
					OBS.push_back(temp[j]);
				}
			}
			//bat elements
			for(int i = 0; i<(sizeof(batPoseY)*sizeof(batPoseY[0])); ++i){
				float temp[] = {batPoseX[i]-batSize/2, batPoseY[i]-batSize/2, batPoseX[i]-batSize/2, batPoseY[i]+batSize/2, batPoseX[i]+batSize/2, batPoseY[i]-batSize/2, batPoseX[i]+batSize/2, batPoseY[i]+batSize/2};
				for(int j = 0; j<8; ++j){
					OBS.push_back(temp[j]);
				}
				//OBS.insert(OBS.end(), temp.begin(), temp.end());
			}
		}else if(mode == "home"){
			ROS_INFO("entered mode HOME");
			//OBS.reserve(4*(numbMarkers+(sizeof(batPoseX)*sizeof(batPoseX[0]))+(sizeof(objPoseX)*sizeof(objPoseX[0])-1))); //Walls +  Batteries + other objects then target
			//alternative : OBS.reserve( )
			//wall elements
			for(int i = 0; i<(wallArray.size()); i++){
				OBS.push_back(wallArray[i]);
			}			//obj elements (except first element)
			for(int i = 1; i<(sizeof(objPoseY)*sizeof(objPoseY[0])); ++i){
				float temp[] = {objPoseX[i]-objSize/2, objPoseY[i]-objSize/2, objPoseX[i]-objSize/2, objPoseY[i]+objSize/2, objPoseX[i]+objSize/2, objPoseY[i]-objSize/2, objPoseX[i]+objSize/2, objPoseY[i]+objSize/2};
				for(int j = 0; j<8; ++j){
					OBS.push_back(temp[j]);
				}
			}
			//bat elements
			for(int i = 0; i<(sizeof(batPoseY)*sizeof(batPoseY[0])); ++i){
				float temp[] = {batPoseX[i]-batSize/2, batPoseY[i]-batSize/2, batPoseX[i]-batSize/2, batPoseY[i]+batSize/2, batPoseX[i]+batSize/2, batPoseY[i]-batSize/2, batPoseX[i]+batSize/2, batPoseY[i]+batSize/2};
				for(int j = 0; j<8; ++j){
					OBS.push_back(temp[j]);
				}
			}
		}else if(mode == "explore"){
			//OBS.reserve(4*(numbMarkers+(sizeof(batPoseX)*sizeof(batPoseX[0]))+(sizeof(objPoseX)*sizeof(objPoseX[0])-1))); //Walls +  Batteries + other objects then target
			//alternative : OBS.reserve( )
			//wall elements
			for(int i = 0; i<(wallArray.size()); i++){
				OBS.push_back(wallArray[i]);
			}			//obj elements (except first element)
			for(int i = 0; i<(sizeof(objPoseY)*sizeof(objPoseY[0])); ++i){
				float temp[] = {objPoseX[i]-objSize/2, objPoseY[i]-objSize/2, objPoseX[i]-objSize/2, objPoseY[i]+objSize/2, objPoseX[i]+objSize/2, objPoseY[i]-objSize/2, objPoseX[i]+objSize/2, objPoseY[i]+objSize/2};
				for(int j = 0; j<8; ++j){
					OBS.push_back(temp[j]);
				}
			}
			//bat elements
			for(int i = 0; i<(sizeof(batPoseY)*sizeof(batPoseY[0])); ++i){
			float temp[] = {batPoseX[i]-batSize/2, batPoseY[i]-batSize/2, batPoseX[i]-batSize/2, batPoseY[i]+batSize/2, batPoseX[i]+batSize/2, batPoseY[i]-batSize/2, batPoseX[i]+batSize/2, batPoseY[i]+batSize/2};
				for(int j = 0; j<8; ++j){
					OBS.push_back(temp[j]);
				}
			}
		}
		// Define Obsacles OBS first
	  ros::Time start_time = ros::Time::now();
		int p = 0;
		//ROS_INFO("Start: startx %f starty %f", startx, starty);

		Node *nodes = (Node*)malloc(NUMNODES*sizeof(Node));
		Node *q_nearest = (Node*)malloc(NUMNODES*sizeof(Node));
    float ndist[NUMNODES];

		Node start (startx+OFFSET[0], starty+OFFSET[1],0, 0);
		float startp[] = {start.x, start.y};
		Node goal (objPoseX[0], objPoseY[0],0,0);
		float goalp[] = {goal.x, goal.y};
		//ROS_INFO("%f, %f, %f, %f", start.x, start.y, goal.x, goal.y);
		nodes[0] = start;
		//ROS_INFO("nodes[0] x: %f, y: %f, cost: %f, parent: %d", nodes[0].x, nodes[0].y, nodes[0].cost, nodes[0].parent);
		float slope = angle(startp, goalp);
		int ctr_neargoal = 0;
		//float cons = 0.5;

		Node q_rand (randZO(0, XDIM),randZO(0,YDIM),0,p);
		float q_rand_pos[2] ;
		float r1 = 0.250;
		float r2 = 0.250;
		float temp_dist = 0;
		float ncoord[2];
		float smallest_dist = 1;
		int smallest_dist_indx = 0;
		Node q_near (0,0,0,0);
		float q_near_pos[2];
		float q_new_pos[2];
		Node q_new (0, 0, 0, 0);
		int neigh_count = 0;
		float nodes_pos[2];
		float q_min[2];
		float c_min = 0;
		float q_nearest_pos[2];
		std::vector<float> distances;
		int count = 0;
		std::vector<float> points;

		for(int ctr = 0; ctr<NUMNODES; ctr++){
			//ROS_INFO("%d",NUMNODES);
			for(int j = 0; j<=count; j++){
					//r1 = 50;
					//ROS_INFO("%d",count);

					//(ctr+1 == NUMNODES) or
					if((sqrt(pow(nodes[j].x-goal.x,(float)2) + pow(nodes[j].y-goal.y,(float)2)) <= r1)){
						ctr_neargoal = 1;
						//ROS_INFO("ctr_neargoal=1");
					}
			}
			if(ctr_neargoal == 1){

				//q_rand.x = goal.x;
				//q_rand.y = goal.y;
				//q_rand = Node(goal.x, goal.y,0,p);
				ROS_INFO("node set to goal, count: %d", count);
				//nodes[ctr] = q_rand;
				break;
			}

			//ROS_INFO("checkpoint");

			//float lower = slope - cons;
			//float higher = slope + cons;

			q_rand.x = randZO(0,XDIM);
			q_rand.y = randZO(0,YDIM);
			q_rand_pos[0] = q_rand.x;
			q_rand_pos[1] = q_rand.y;
			//ROS_INFO("q_rand x: %f, y: %f", q_rand.x, q_rand.y);
			//float slope1 = angle(startp,q_rand_pos);
			/*while(!(slope1 >= lower and slope1 <= higher)){
				ROS_INFO("Slope1 %f", slope1);
				q_rand = Node(randZO(0, XDIM),randZO(0,YDIM),0,p);
				q_rand_pos[0] = q_rand.x;
				q_rand_pos[1] = q_rand.y;
				//ROS_INFO("q_rand x: %f, y: %f, name = %f", q_rand.x, q_rand.y, q_rand.name);
				float slope1 = angle(startp,q_rand_pos);
			}*/
			//ROS_INFO("set a random node");
			//i++;

			/*if(i > 300){
				cons += 0.25;
				i = i-300;
				ROS_INFO("I has been lowered: new i = %d",i);
			}*/
			//ROS_INFO("XMAX %f, YMAX %f", XDIM, YDIM);

			for(int j = 0; j<=count;j++){
				ncoord[0] = nodes[j].x;
				ncoord[1] = nodes[j].y;
				//ROS_INFO("x %f, y %f", ncoord[0], ncoord[1]);
				temp_dist = dist(ncoord, q_rand_pos);
				//ROS_INFO("dist %f", temp_dist);
				ndist[j] = temp_dist;
				//ROS_INFO("temp_dist = %f",temp_dist);

			}

			smallest_dist = ndist[0];
			smallest_dist_indx = 0;

			for(int j = 0; j <=count;j++){
				if(ndist[j]<smallest_dist){
					smallest_dist = ndist[j];
					smallest_dist_indx = j;
				}
			}
			//ROS_INFO("smallest_dist_dist = %f, index = %d",smallest_dist, smallest_dist_indx);

			q_near = nodes[smallest_dist_indx];
			q_near_pos[0]= q_near.x;
			q_near_pos[1]= q_near.y;
			//ROS_INFO("1  %f %f", q_new_pos[0], q_new_pos[1]);
			step_from_to(q_rand_pos, q_near_pos, q_new_pos);
			//ROS_INFO("2  %f %f", q_new_pos[0], q_new_pos[1]);
			if(checkIntersect(q_rand, q_near)){
				//ROS_INFO("First intersect check");
				q_new.cost = dist(q_new_pos, q_near_pos) + q_near.cost;
				q_new.x = q_new_pos[0];
				q_new.y = q_new_pos[1];

				neigh_count = 0;

				for(int j= 0; j<=count; j++){
					//ROS_INFO("i %d, j %d", i, j);
					nodes_pos[0] = nodes[j].x;
					nodes_pos[1] = nodes[j].y;
					//ROS_INFO("1.1  %d ", (checkIntersect(nodes[j], q_new)));
					//ROS_INFO("1.2  %f %f", dist(nodes_pos,q_new_pos),r2);

					if((checkIntersect(nodes[j], q_new)) and (dist(nodes_pos,q_new_pos)<=r2)){
						//ROS_INFO("Second intersect check");
						q_nearest[neigh_count].x = nodes[j].x;
						q_nearest[neigh_count].y = nodes[j].y;
						q_nearest[neigh_count].cost = nodes[j].cost;
						neigh_count++;
					}
				}

				q_min[0]= q_near_pos[0];
				q_min[1] =q_near_pos[1];
				c_min = q_new.cost;
				//ROS_INFO("2  %f %f", q_min[0], q_min[1]);

				for(int j= 0; j<neigh_count; j++){
					//ROS_INFO("i %d, j %d", ctr, j);
					q_nearest_pos[0] = q_nearest[j].x;
					q_nearest_pos[1] = q_nearest[j].y;
					//ROS_INFO("2  %f %f", q_min[0], q_min[1]);
					//ROS_INFO("2.1  %d ", (checkIntersect(q_nearest[j], q_new)));
					//ROS_INFO("2.2  %f, %f ", q_nearest[j].cost + dist(q_nearest_pos, q_new_pos),  c_min);
					if((checkIntersect(q_nearest[j], q_new)) and (q_nearest[j].cost + dist(q_nearest_pos, q_new_pos) < c_min)){
						  //ROS_INFO("Third intersect check");
							q_min[0] = q_nearest_pos[0];
							q_min[1] = q_nearest_pos[1];
							c_min = q_nearest[j].cost + dist(q_nearest_pos,q_new_pos);
					}
				}
				for(int j= 0; j<=count; j++){
					if(nodes[j].x == q_min[0] and nodes[j].y == q_min[1]){
						q_new.parent = j;

					}

				}
				numnodes_tot = count;
				count++;
				nodes[count] = q_new;

			}


			//************************
			/*
			Node nn = nodes[0];

			float k[] = {nn.x, nn.y};
			for(int c = 0; c <= i; ++c){
				ROS_INFO("c: %f", c);
				float a[] ={nodes[c].x, nodes[c].y};
				if(dist(k,q_rand_pos) < dist(a,q_rand_pos)){
					nn = nodes[p];
				}
			}
			float *interpolateNode = step_from_to(k, q_rand_pos);
			ROS_INFO("new node");
			Node newnode (interpolateNode[0], interpolateNode[1],0, p);
			ROS_INFO("q_rand x: %f, y: %f", newnode.x, newnode.y);
			if(checkIntersect(nn, q_rand)){
				Node *result = chooseParent(nn, newnode, nodes,p);
				newnode = result[0];
				nn = result[1];
				nodes[p+1] = newnode;
				nodes = reWire(nodes, newnode, p);
			}
			float d[] = {newnode.x, newnode.y};
			float e[] ={goal.x, goal.y};


			*/
			// ***********************
			/* float q_goal_pos[] = {goal.x, goal.y};
			for(int j= 0; j<=ctr; j++){
				float nodes_pos[] = {nodes[j].x, nodes[j].y};
				if(dist(nodes_pos, q_goal_pos) < 25.0){
					ctr_neargoal = 1;
				}
				if(dist(nodes_pos, q_goal_pos) == 0){
					ctr = 0;
				}
			}*/

			//p = p+1;
			//i = i+1;
			//ROS_INFO("p: %d",p);

		}
		distances.clear();
		float td;
		float np[2];
		for(int c =0; c <= numnodes_tot; c++){
			np[0] = nodes[c].x;
			np[1] = nodes[c].y;
			td = dist(np, goalp);
			distances.push_back(td);
		}
		smallest_dist = distances[0];
		smallest_dist_indx = 0;

		for(int j = 0; j <=numnodes_tot;j++){
			if(distances[j]<smallest_dist){
				smallest_dist = distances[j];
				smallest_dist_indx = j;
			}
		}

		Node q_final = nodes[smallest_dist_indx];
		goal.parent = smallest_dist_indx;
		Node q_end = goal;

		nodes[numnodes_tot] = goal;
		int idx = 0;
		//ROS_INFO("checkpoint");
		finalpathx.clear();
		finalpathy.clear();

		points.clear();
		finalpathx.push_back(q_end.x);
		finalpathy.push_back(q_end.y);
		while(q_end.parent != 0){
			idx = q_end.parent;
			//points.push_back(nodes[idx].x);
			//points.push_back(nodes[idx].y);
			finalpathx.push_back(nodes[idx].x);
			finalpathy.push_back(nodes[idx].y);
			//ROS_INFO("%d", idx);
			q_end = nodes[idx];

		}
		finalpathx.push_back(start.x);
		finalpathy.push_back(start.y);

		free(nodes);
		free(q_nearest);
/*
		points.push_back(q_end.x);
		points.push_back(q_end.y);
		while(q_end.parent != 0){
			idx = q_end.parent;
			//points.push_back(nodes[idx].x);
			//points.push_back(nodes[idx].y);
			points.push_back(nodes[idx].x);
			points.push_back(nodes[idx].y);
			//ROS_INFO("%d", idx);
			q_end = nodes[idx];

		}
		points.push_back(start.x);
		points.push_back(start.y);
*/

				//ROS_INFO("Generate final path, numnodestotal %d", numnodes_tot);

		//finalpathx.resize(numnodes_tot);
		//finalpathy.resize(numnodes_tot);
/*		finalpathx.push_back(points[0]);
		finalpathy.push_back(points[1]);
		Node node1 (0,0,0,0);
		Node node2 (0,0,0,0);
		for(int c =2; c < points.size(); c=c+2){
			node1.x = finalpathx[0];
			node1.y = finalpathy[0];
			node2.x = points[c];
			node2.y = points[c+1];
			if(!checkIntersect(node1, node2)){
				finalpathx.push_back(points[c-2]);
				finalpathy.push_back(points[c-1]);
				node1.x = points[c-2];
				node1.y = points[c-1];

			}

		}
		finalpathx.push_back(node2.x);
		finalpathx.push_back(node2.y);

*/


}
void publishPath(){

	geometry_msgs::PoseStamped newpose;
  std::vector<geometry_msgs::PoseStamped> plan;
	plan.clear();

	ros::Time now = ros::Time::now();
	path.header.seq=pathseq;
	path.header.stamp = now;
	path.header.frame_id = "path";
	int seq = 0;
	for(int i = 0; i < finalpathx.size(); ++i){

			newpose.header.seq=seq;
			newpose.header.stamp = now;
			newpose.header.frame_id = "pathpoint";

			newpose.pose.position.x = finalpathx[i];
			newpose.pose.position.y = finalpathy[i];
			newpose.pose.position.z = 0;
			newpose.pose.orientation.x = 0;
			newpose.pose.orientation.y = 0;
			newpose.pose.orientation.w = 1;
			if(i < numnodes_tot){
					newpose.pose.orientation.z =(float) atan2((finalpathy[i+1]-finalpathy[i]),(finalpathx[i+1]-finalpathx[i]));
			}else if( i == numnodes_tot){
					newpose.pose.orientation.z =(float) atan2((finalpathy[i]-finalpathy[i-1]),(finalpathx[i]-finalpathx[i-1]));
			}
			newpose.pose.orientation.z = 0;
			ROS_INFO("%f %f", newpose.pose.position.x, newpose.pose.position.y);
			//poses[i] = newpose;
			plan.push_back(newpose);

			seq = seq +1 ;
	}
	path.poses.resize(plan.size());

	for(int i = 0; i < finalpathx.size(); ++i){
		path.poses[i] = plan[i];
	}
	pathseq++;
	//path.poses = poses;
	ROS_INFO("Checkpoint");

}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_rrt");

    ros::NodeHandle n;
		ros::Subscriber wall_sub = n.subscribe<visualization_msgs::MarkerArray>("/maze_map", 1000, wallCallback);
		ros::Subscriber pose_sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, poseCallback);
    ros::Subscriber obj_sub = n.subscribe<visualization_msgs::Marker>("/visualization_marker", 10, objectCallback);
    ros::Subscriber bat_sub = n.subscribe<visualization_msgs::Marker>("/visualization_marker_battery", 10, batteryCallback);
    //ros::Publisher grid_publisher = n.advertise<nav_msgs::OccupancyGrid>("/rosie_path_grid",1);
    path_pub = n.advertise<nav_msgs::Path>("/rosie_rrt_path",1);
    ros::ServiceClient gateClient = n.serviceClient<rosie_servo_controller::ControlGates>("control_gates");
    ros::ServiceClient reqClient = n.serviceClient<rosie_map_controller::RequestRerun>("request_rerun");
    rosie_servo_controller::ControlGates gateSrv;
    rosie_map_controller::RequestRerun reqSrv;

		static tf::TransformBroadcaster br;

    ros::Rate loop_rate(1);

	  ros::Time load_time = ros::Time::now();

    while(ros::ok()){
			//loop_rate.sleep();
			if(mapInitialized){
				if(pathInitialized){

					ROS_INFO("automatic mode");
					/*
	        if(mode == "goto"){
	            xpoint = objPoseX[0]/resolution;
	            ypoint = objPoseY[0]/resolution;
	            runRRT();
	            publishPath();
	            //lastMode = "goto";
	            //mode = "wait";
	            //gateSrv.request.control = 0;
	        }else if(mode == "home"){
	            xpoint = HOME[1];
	            xpoint = HOME[2];
	            runRRT();
	            publishPath();
	            lastMode = "home";
	            mode = "wait";
	            gateSrv.request.control = 1;
	        }else if(mode =="explore"){
	            //still to do

	        }else if(mode =="wait"){

	          check_for_endposition(); //to be implemented

						if(!reqClient.call(reqSrv)){
	              mode = "goto";
	              lastMode = "wait";
	              ROS_INFO("calculating rrt*");
	              break;
	          }

	          if(gateClient.call(gateSrv)){
	          if(lastMode == "goto"){
	              mode = "home";
	              lastMode = "wait";
	              ROS_INFO("Gripper closed");
	            } else if(lastMode == "home"){
	              mode = "goto";
	              lastMode = "wait";
	              ROS_INFO("Gripper opened");
	            }
	          }else{
	            ROS_INFO("Gripper don't react. Please have a look.");
	          }

	        }

		    ros::spinOnce();
	    	loop_rate.sleep();
				*/

			}else{
				ROS_INFO("first initialization");
				if(pathPublished == 0){ //reqClient.call(reqSrv)
						ROS_INFO("start recalculating path");
						mode = "goto";
						lastMode = "wait";
						runRRT();
						publishPath();
	   				path_pub.publish(path);
						ROS_INFO("path published");
						pathInitialized = 0;
						pathPublished = 1;

				}else{
					path_pub.publish(path);
					ROS_INFO("path published");
				}

				tf::Transform transform;
				transform.setOrigin( tf::Vector3(0,0, 0) );
				tf::Quaternion qtf;
				qtf.setRPY(0, 0, 0);
				transform.setRotation( qtf );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "path"));
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}
