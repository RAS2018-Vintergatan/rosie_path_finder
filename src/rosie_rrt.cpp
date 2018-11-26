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
#include <rosie_object_detector/RAS_Evidence.h>

#define OBJECT 0
#define RED_CYLINDER 1
#define RED_CUBE 2
#define RED_HOLLOW_CUBE 3
#define RED_BALL 4
#define ORANGE_CROSS 5
#define PATRIC 6
#define YELLOW_BALL 7
#define YELLOW_CUBE 8
#define GREEN_CUBE 9
#define GREEN_HOLLOW_CUBE 10
#define GREEN_CYLINDER 11
#define BLUE_CUBE 12
#define BLUE_TRIANGLE 13
#define PURPLE_CROSS 14
#define PURPLE_STAR 15

int red_cylinder_val = 0;
int red_cube_val = 0;
int red_hollow_cube_val = 0;
int red_ball_val = 0;

int orange_cross_val = 0;
int patric_val = 0;

int yellow_ball_val = 0;
int yellow_cube_val = 0;

int green_cube_val = 0;
int green_hollow_cube_val = 0;
int green_cylinder_val = 0;

int blue_cube_val = 0;
int blue_triangle_val = 0;

int purple_cross_val = 0;
int purple_star_val = 0;

// RRT parameter
float PI = 3.1415926f;
float EPSILON = 0.150f;
float XDIM = 2.40;
float YDIM = 2.40;
float startx = 0.200f;
float starty = 0.400f;
float OFFSET[] = {0.0f, 0.00f};
float HOME[] = {startx, starty};
int NUMNODES = 3000;
float robotsize = 0.2f;

// Publisher
ros::Publisher path_pub;
ros::ServiceClient gateClient;
rosie_servo_controller::ControlGates gateSrv;

// Objects and Obstacles (Walls and Batteries)
std::vector<float> wallArray;
std::vector<float> OBS;
std::vector<float> OBJ;
std::vector<float> ALL_OBS;
std::vector<float> ALL_OBJ;
float objSize = 0.05f;
int objNumber = 14;
std::vector<float> objPoseX;
std::vector<float> objPoseY;
float batSize = 0.10;
int batNumber = 4;
std::vector<float> batPoseX;
std::vector<float> batPoseY;

// for target calculations
int target_num = 0;

// path variables
nav_msgs::Path path;
std::vector<float> finalpathx;
std::vector<float> finalpathy;
std::vector<geometry_msgs::PoseStamped> allposes;

// state parameters
int rerun_rrt = 0;
int pathseq = 0;
int mapInitialized = 0;
int pathInitialized = 0;
int pathPublished = 0;
std::string lastMode = "wait";
std::string mode = "goto";

// **************************

// addToObs parameter
float origin[] = {0.0f,0.0f};
float temp[2];
float m;
float r = 0.1f;
float x_1,y_1,x_2,y_2;
float diffy;
float diffx;
float P[8];
float p1_new[2];
float p2_new[2];
float n;
float co;
float si;

bool addToObs(float data[], bool obsOrObj){
			float p[4];

			//float abst = sqrt(pow(OBS[i+2]-OBS[i+0],2.0)+pow(OBS[i+3]-OBS[i+1],2.0));
			//float m1 = (OBS[i+2]-OBS[i+0])/dist;
			//float m2 = (OBS[i+3]-OBS[i+1])/dist;
			p[0] =data[0];
			p[1] =data[1];
			p[2] =data[2];
			p[3] =data[3];
			//ROS_INFO("x1 %f y1 %f x2 %f y2 %f", p1[0], p1[1], p2[0], p2[1] );


			if((p[3]< p[1]) || (p[3] == p[1] and p[2]<p[0])){
				temp[0] = p[0];
				temp[1] = p[1];
				ROS_INFO("%f %f", p[0], p[1]);
				p[0] = p[2];
				p[1] = p[3];
				ROS_INFO("%f %f", p[2], p[3]);
				p[2] = temp[0];
				p[3] = temp[1];

			}
			diffx = p[2]-p[0];
			diffy = p[3]-p[1];
			m = (float) atan2(diffy, diffx);
			ROS_INFO("m %f",m);
			co = r*cos(m);
			si = r*sin(m);

			x_1 = p[0] - co;
			y_1 = p[1] - si;
			x_2 = p[2] + co;
			y_2 = p[3] + si;
			//m = m+PI/(2.0f);
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
			 ROS_INFO("%f %f %f %f %f %f %f %f", P[0], P[1], P[2], P[3], P[4], P[5], P[6], P[7] );
		for(int i=0;i<8;i++){
			if(obsOrObj){
				 ALL_OBS.push_back(P[i]);
			}else{
				 ALL_OBJ.push_back(P[i]);
			}

	 }
	 ROS_INFO("addToObs");
	 return true;
}

int numbMarkers;
float minX, minY, maxX, maxY;
float czone;
float sX, sY, eX, eY;
void wallCallback(const visualization_msgs::MarkerArray msg){
	if(!mapInitialized){
		ROS_INFO("Initializing!");
		numbMarkers = msg.markers.size();
		czone = robotsize/(2.0f) + 0.02f; //additional extra security distance

		wallArray.clear();
		wallArray.resize(4*numbMarkers);
		for(int i = 0; i < 4*numbMarkers; ++i){
			wallArray[i] = -1.0f;
		}


		minX = minY = maxX = maxY = 0;
		for(int k = 0; k < numbMarkers; ++k){
			//Extract point data
			std::string pointsText = msg.markers[k].text;
			std::stringstream ss(pointsText);

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
		}

		OFFSET[0] = minX;
		OFFSET[1] = minY;

		XDIM = (maxX - minX);
		YDIM = (maxY - minY);
		float single_wall[4];
		for(int i = 0; i<(wallArray.size()); i=i+4){
			single_wall[0] = wallArray[i];
			single_wall[1] = wallArray[i+1];
			single_wall[2] = wallArray[i+2];
			single_wall[3] = wallArray[i+3];
			ROS_INFO("Wall");
			bool test1 = addToObs(single_wall,1);
		}
		mapInitialized = 1;
		//ROS_INFO("map initialized");
	}

}

nav_msgs::Odometry pose;
void currentPoseCallback(nav_msgs::Odometry msg){ // for re-calculation of the path when needed
    pose = msg;
		//pose.pose.pose.position.x = 0.25f;
		//pose.pose.pose.position.y = 0.40f;
}

bool gotTarget = 0;
geometry_msgs::PoseStamped rviz_pose;
//boost::shared_ptr<tf::TransformListener> target_tfl_ptr;
//boost::shared_ptr<tf::StampedTransform> target_pose_tf_ptr;
void rvizTargetPoseCallback(geometry_msgs::PoseStamped msg){
	rviz_pose = msg;
	//try{
	//	(*target_tfl_ptr).waitForTransform("world", msg.header.frame_id, ros::Time(0), ros::Duration(10.0));
	//	(*target_tfl_ptr).lookupTransform("world", msg.header.frame_id, ros::Time(0), *target_pose_tf_ptr);
	//}catch(tf::TransformException ex){
	//	ROS_ERROR("%s",ex.what());
	//}
	gotTarget = 1;
	//isInside = 0;
}


float objTemp1[4];
float objTemp2[4];
bool objInitialized = 0;
std::vector<int> objectID;
std::vector<float> objWeighting;
/*void objectCallback(visualization_msgs::Marker mrk){ //get object positions (sorted after value and distance)
  //colors + shape:
    // e.g. yellow + ball = 1
		std::vector<int>::iterator index;
    bool pushed = 0;
    float posX = mrk.pose.position.x + pose.pose.pose.position.x; // might be already transformed
    float posY = pose.pose.pose.position.y;
    float accuracy = 0.05; //5cm radius for faulty measurement
    if(!(std::find(objectID.begin(), objectID.end(), mrk.id) != objectID.end())){ //true if id is not present
			objectID.push_back(mrk.id);
			if(mrk.id == 1 || mrk.id == 2 || mrk.id == 3 ){
				objWeighting.push_back(1);
			}else if(mrk.id == 4 || mrk.id == 5 || mrk.id == 5 ){
				objWeighting.push_back(2);
			}else if(mrk.id == 6 || mrk.id == 7 || mrk.id == 8 ){
				objWeighting.push_back(3);
			}else if(mrk.id == 9 || mrk.id == 10){
				objWeighting.push_back(4);
			}else if(mrk.id == 11 || mrk.id == 12){
				objWeighting.push_back(5);
			}else if(mrk.id == 13 || mrk.id == 14){
				objWeighting.push_back(6);
			}
			objPoseX.push_back(posX);
      objPoseY.push_back(posY);
      ROS_INFO("NEW object");
			pushed = 1;
    }else{
			index = std::find(objectID.begin(), objectID.end(), mrk.id);
      if((posX-objPoseX[index[0]])*(posX-objPoseX[index[0]])+(posY-objPoseY[index[0]])*(posY-objPoseY[index[0]]) > (accuracy*accuracy)){
        objPoseX[index[0]] = posX;
        objPoseY[index[0]] = posY;
				//evtl remove obj and push again -> generate new walls/lines
        ROS_INFO("This object has been moved.");
      }else{
        ROS_INFO("This object is already mapped.");
      }
    }
		if(pushed){
		objTemp1[0] = posX-objSize/2.0f;
		objTemp1[1] = posY-objSize/2.0f;
		objTemp1[2] = posX-objSize/2.0f;
		objTemp1[3] = posY+objSize/2.0f;

		objTemp2[0] = posX+objSize/2.0f;
		objTemp2[1] = posY-objSize/2.0f;
		objTemp2[2] = posX+objSize/2.0f;
		objTemp2[3] = posY+objSize/2.0f;

		ROS_INFO("Obj");
		bool test1 = addToObs(objTemp1,0);
		ROS_INFO("Obj");
		bool test2 = addToObs(objTemp2,0);
		pushed = 0;
		}

}*/

void evidenceCallback(const rosie_object_detector::RAS_Evidence evidence){
	std::string obj_string_id = evidence.object_id;
	int obj_id = 0;
	int obj_val = 0;
	float obj_size = 0.05;
	/*if(obj_string_id.compare(evidence.red_cylinder)){
		obj_id = RED_CYLINDER;
		obj_val = red_cylinder_val;
	}*/
	if(obj_string_id.compare(evidence.red_cube)){
		obj_id = RED_CUBE;
		obj_val = red_cube_val;
	}
	if(obj_string_id.compare(evidence.red_hollow_cube)){
		obj_id = RED_HOLLOW_CUBE;
		obj_val = red_hollow_cube_val;
	}
	if(obj_string_id.compare(evidence.red_ball)){
		obj_id = RED_BALL;
		obj_val = red_ball_val;
	}
	/*if(obj_string_id.compare(evidence.orange_cross)){
		obj_id = ORANGE_CROSS;
		obj_val = orange_cross_val;
	}*/
	if(obj_string_id.compare(evidence.patric)){
		obj_id = PATRIC;
		obj_val = patric_val;
	}
	if(obj_string_id.compare(evidence.yellow_ball)){
		obj_id = YELLOW_BALL;
		obj_val = yellow_ball_val;
	}
	if(obj_string_id.compare(evidence.yellow_cube)){
		obj_id = YELLOW_CUBE;
		obj_val = yellow_cube_val;
	}
	if(obj_string_id.compare(evidence.green_cube)){
		obj_id = GREEN_CUBE;
		obj_val = green_cube_val;
	}
	/*if(obj_string_id.compare(evidence.green_hollow_cube)){
		obj_id = GREEN_HOLLOW_CUBE;
		obj_val = green_hollow_cube_val;
	}*/
	if(obj_string_id.compare(evidence.green_cylinder)){
		obj_id = GREEN_CYLINDER;
		obj_val = green_cylinder_val;
	}
	if(obj_string_id.compare(evidence.blue_cube)){
		obj_id = BLUE_CUBE;
		obj_val = blue_cube_val;
	}
	if(obj_string_id.compare(evidence.blue_triangle)){
		obj_id = BLUE_TRIANGLE;
		obj_val = blue_triangle_val;
	}
	if(obj_string_id.compare(evidence.purple_cross)){
		obj_id = PURPLE_CROSS;
		obj_val = purple_cross_val;
	}
	if(obj_string_id.compare(evidence.purple_star)){
		obj_id = PURPLE_STAR;
		obj_val = purple_star_val;
	}
	else if(obj_string_id.compare(evidence.an_object)){
		obj_id = OBJECT;
		obj_val = 0;
		obj_size = 0.1;
	}


	float posX = evidence.object_location.x;
	float posY = evidence.object_location.y;

 	int pushed = -1;
	if(obj_id != OBJECT){
		float accuracy = 0.05; //5cm radius for faulty measurement
		if(!(std::find(objectID.begin(), objectID.end(), obj_id) != objectID.end())){ //true if id is not present
			objectID.push_back(obj_id);
			objWeighting.push_back(obj_val);

			objPoseX.push_back(posX);
			objPoseY.push_back(posY);
			pushed = 0;
		}else{
			std::vector<int>::iterator index;
			index = std::find(objectID.begin(), objectID.end(), obj_id);
			if((posX-objPoseX[index[0]])*(posX-objPoseX[index[0]])+(posY-objPoseY[index[0]])*(posY-objPoseY[index[0]]) > (accuracy*accuracy)){
				objPoseX[index[0]] = posX;
				objPoseY[index[0]] = posY;
				//evtl remove obj and push again -> generate new walls/lines
				ROS_INFO("This object has been moved.");
			}else{
				ROS_INFO("This object is already mapped.");
			}
		}

	}else{
		float posX = evidence.object_location.x;
		float posY = evidence.object_location.y;
		float accuracy = 0.05; //5cm radius for faulty measurement
		for(int i = 0; i< batPoseX.size(); ++i){
		    if((posX-batPoseX[i])*(posX-batPoseX[i])+(posY-batPoseY[i])*(posY-batPoseY[i]) < (accuracy*accuracy)){
		      ROS_INFO("Battery is already mapped. - Updated battery position");
						batPoseX[i] = posX;
						batPoseY[i] = posY;
		    }else{
		      ROS_INFO("NEW battery");
						batPoseX.push_back(posX);
						batPoseY.push_back(posY);
						pushed = 1;
		    }
		}
	}
	if(pushed>=0){
		objTemp1[0] = posX-obj_size/2.0f;
		objTemp1[1] = posY-obj_size/2.0f;
		objTemp1[2] = posX-obj_size/2.0f;
		objTemp1[3] = posY+obj_size/2.0f;

		objTemp2[0] = posX+obj_size/2.0f;
		objTemp2[1] = posY-obj_size/2.0f;
		objTemp2[2] = posX+obj_size/2.0f;
		objTemp2[3] = posY+obj_size/2.0f;

		ROS_INFO("Obj");
		bool test1 = addToObs(objTemp1,pushed);
		ROS_INFO("Obj");
		bool test2 = addToObs(objTemp2,pushed);
		pushed = -1;
	}
}

/*
bool objInit(){
	if(!objInitialized){
		float objX[] = {1.250f, 2.20f,2.10f,2.10f,2.10f,2.10f,2.10f,2.10f,2.10f,2.10f,2.10f,2.10f,2.10f,2.10f};
		float objY[] = {1.55f, 0.30f,0.30f,0.30f,0.30f,0.30f,0.30f,0.30f,0.30f,0.30f,0.30f,0.30f,0.30f,0.30f};

		for(int i = 0; i<14; i++){
			objPoseX.push_back(objX[i]);
			objPoseY.push_back(objY[i]);
		}


		for(int i = 0; i<(objPoseY.size()); ++i){
			objTemp1[0] = objPoseX[i]-objSize/2.0f;
			objTemp1[1] = objPoseY[i]-objSize/2.0f;
			objTemp1[2] = objPoseX[i]-objSize/2.0f;
			objTemp1[3] = objPoseY[i]+objSize/2.0f;

			objTemp2[0] = objPoseX[i]+objSize/2.0f;
			objTemp2[1] = objPoseY[i]-objSize/2.0f;
			objTemp2[2] = objPoseX[i]+objSize/2.0f;
			objTemp2[3] = objPoseY[i]+objSize/2.0f;

			ROS_INFO("Obj");
			bool test1 = addToObs(objTemp1,0);
			ROS_INFO("Obj");
			bool test2 = addToObs(objTemp2,0);
		}
	}
}
*/

/*void batteryCallback(visualization_msgs::Marker btr){ //get object positions (sorted after value and distance)

		bool pushed = 0;
    float posX = btr.pose.position.x + pose.pose.pose.position.x;
    float posY = pose.pose.pose.position.y;
    float r = 0.05; //5cm radius for faulty measurement
    for(int i = 0; i< batPoseX.size(); ++i){
        if((posX-batPoseX[i])*(posX-batPoseX[i])+(posY-batPoseY[i])*(posY-batPoseY[i]) < (r*r)){
          ROS_INFO("Battery is already mapped. - Updated battery position");
					batPoseX[i] = posX;
					batPoseY[i] = posY;
        }else{
          ROS_INFO("NEW battery");
					batPoseX.push_back(posX);
					batPoseY.push_back(posY);
					pushed = 1;
        }
    }
    if(pushed){
		batTemp1[0] = posX-batSize/2.0f;
		batTemp1[1] = posY-batSize/2.0f;
		batTemp1[2] = posX-batSize/2.0f;
		batTemp1[3] = posY+batSize/2.0f;

		batTemp2[0] = posX+batSize/2.0f;
		batTemp2[1] = posY-batSize/2.0f;
		batTemp2[2] = posX+batSize/2.0f;
		batTemp2[3] = posY+batSize/2.0f;
		ROS_INFO("bat");
		addToObs(batTemp1,1);
		ROS_INFO("bat");
		addToObs(batTemp2,1);
		}

}*/
/*
bool batInit(){
	if(!batInitialized){
	float batX[] = {1.20f,1.20f,1.20f,1.20f};
	float batY[] =  {2.10f,2.20f,2.10f,2.10f};

	for(int i = 0; i<4; i++){
		batPoseX.push_back(batX[i]);
		batPoseY.push_back(batY[i]);
	}
	batInitialized = 1;
	ROS_INFO("TEST");
		for(int i = 0; i<(batPoseY.size()); ++i){
			batTemp1[0] = batPoseX[i]-batSize/2.0f;
			batTemp1[1] = batPoseY[i]-batSize/2.0f;
			batTemp1[2] = batPoseX[i]-batSize/2.0f;
			batTemp1[3] = batPoseY[i]+batSize/2.0f;

			batTemp2[0] = batPoseX[i]+batSize/2.0f;
			batTemp2[1] = batPoseY[i]-batSize/2.0f;
			batTemp2[2] = batPoseX[i]+batSize/2.0f;
			batTemp2[3] = batPoseY[i]+batSize/2.0f;
			addToObs(batTemp1,1);
			addToObs(batTemp2,1);
		}
		ROS_INFO("bat initialized");
		/*
float objTemp1[4];
float objTemp2[4];
bool objInitialized = 0;
std::vector<int> objectID;
std::vector<float> objWeighting;
void objectCallback(rosie_object_detector::RAS_Evidence msg){ //get object positions (sorted after value and distance)
  //colors + shape:
    // e.g. yellow + ball = 1
		int index;
    bool pushed = 0;
    float posX = mrk.pose.position.x + pose.pose.pose.position.x; // might be already transformed
    float posY = pose.pose.pose.position.y;
    float accuracy = 0.05; //5cm radius for faulty measurement
    if(!(std::find(objectID.begin(), objectID.end(), mrk.id) != objectID.end())){ //true if id is not present
			objectID.push_back(mrk.id);
			if(mrk.id == 1 || mrk.id == 2 || mrk.id == 3 ){
				objWeighting.push_back(1);
			}else if(mrk.id == 4 || mrk.id == 5 || mrk.id == 5 ){
				objWeighting.push_back(2);
			}else if(mrk.id == 6 || mrk.id == 7 || mrk.id == 8 ){
				objWeighting.push_back(3);
			}else if(mrk.id == 9 || mrk.id == 10){
				objWeighting.push_back(4);
			}else if(mrk.id == 11 || mrk.id == 12){
				objWeighting.push_back(5);
			}else if(mrk.id == 13 || mrk.id == 14){
				objWeighting.push_back(6);
			}
			objPoseX.push_back(posX);
      objPoseY.push_back(posY);
      ROS_INFO("NEW object");
			pushed = 1;
    }else{
			index = std::find(objectID.begin(), objectID.end, mrk.id);
      if((posX-objPoseX[index])*(posX-objPoseX[index])+(posY-objPoseY[index])*(posY-objPoseY[index]) > (accuracy*accuracy)){
        objPoseX[index] = posX;
        objPoseY[index] = posY;
        ROS_INFO("This object has been moved.");
      }else{
        ROS_INFO("This object is already mapped.");
      }
    }
		if(pushed){
		objTemp1[0] = objPoseX[index]-objSize/2.0f;
		objTemp1[1] = objPoseY[index]-objSize/2.0f;
		objTemp1[2] = objPoseX[index]-objSize/2.0f;
		objTemp1[3] = objPoseY[index]+objSize/2.0f;

		objTemp2[0] = objPoseX[index]+objSize/2.0f;
		objTemp2[1] = objPoseY[index]-objSize/2.0f;
		objTemp2[2] = objPoseX[index]+objSize/2.0f;
		objTemp2[3] = objPoseY[index]+objSize/2.0f;

		ROS_INFO("Obj");
		bool test1 = addToObs(objTemp1,0);
		ROS_INFO("Obj");
		bool test2 = addToObs(objTemp2,0);
		pushed = 0;
		}

}

	}

}
*/
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

bool ccw(float A[], float B[], float C[]){
     bool val = abs((C[1]-A[1]) * (B[0]-A[0])) > abs((B[1]-A[1]) * (C[0]-A[0]));
     return val;
}

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

float p1[2];
float p2[2];
float p3[2];
float p4[2];
bool nc = true;
bool ints1;
bool ints2;
bool ints3;
bool ints4;

bool checkIntersect(Node n2, Node n1){         //array definition might be wrong
    float A[]= {n1.pos[0],n1.pos[1]};
    float B[]= {n2.pos[0],n2.pos[1]};
		nc = true;
    for(int i =0 ; i<ALL_OBS.size(); i = i+8){
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
				 ints3 = isIntersecting(p3,p4,A,B);

				// ints1 = ccw(A,P1,P4) != ccw(B,P1,P4) and ccw(A,B,P1) != ccw(A,B,P4);
        // ints2 = ccw(A,P1,P2) != ccw(B,P1,P2) and ccw(A,B,P1) != ccw(A,B,P2);
        // ints3 = ccw(A,P3,P2) != ccw(B,P3,P2) and ccw(A,B,P3) != ccw(A,B,P2);
        // ints4 = ccw(A,P3,P4) != ccw(B,P3,P4) and ccw(A,B,P3) != ccw(A,B,P4);
        if(ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==1){
            nc = 1;
        }else{
            nc = 0;
        }

    }
		for(int i =0 ; i<ALL_OBJ.size(); i = i+8){
				if(i = target_num*8){
					i = i+8;
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
				 ints3 = isIntersecting(p3,p4,A,B);

				// ints1 = ccw(A,P1,P4) != ccw(B,P1,P4) and ccw(A,B,P1) != ccw(A,B,P4);
        // ints2 = ccw(A,P1,P2) != ccw(B,P1,P2) and ccw(A,B,P1) != ccw(A,B,P2);
        // ints3 = ccw(A,P3,P2) != ccw(B,P3,P2) and ccw(A,B,P3) != ccw(A,B,P2);
        // ints4 = ccw(A,P3,P4) != ccw(B,P3,P4) and ccw(A,B,P3) != ccw(A,B,P4);
        if(ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==1){
            nc = 1; //is outside c-space
        }else{
            nc = 0; //is inside c-space
        }

    }
		return nc;
}

float isGoalInCSpace(float x, float y){
	float A[] = {x, y};
	float center[2];
	float m;
	nc = 0;
	for(int i =0 ; i<ALL_OBS.size(); i = i+8){
			//float abst = sqrt(pow(OBS[i+2]-OBS[i+0],2.0)+pow(OBS[i+3]-OBS[i+1],2.0));
			//float m1 = (OBS[i+2]-OBS[i+0])/dist;
			//float m2 = (OBS[i+3]-OBS[i+1])/dist;
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

			// ints1 = ccw(A,P1,P4) != ccw(B,P1,P4) and ccw(A,B,P1) != ccw(A,B,P4);
			// ints2 = ccw(A,P1,P2) != ccw(B,P1,P2) and ccw(A,B,P1) != ccw(A,B,P2);
			// ints3 = ccw(A,P3,P2) != ccw(B,P3,P2) and ccw(A,B,P3) != ccw(A,B,P2);
			// ints4 = ccw(A,P3,P4) != ccw(B,P3,P4) and ccw(A,B,P3) != ccw(A,B,P4);
			if(ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==0){
					nc = 0; //is ioutside cspace
			}else{
					nc = 1; //is inside cspace
			}

	}
	if(nc == 1){
		return m;
	}else{
		return 100.0;
	}
}

float randnum;
float randZO(float min, float max){
		srand(ros::Time::now().nsec);
    randnum = (rand() % ((int) max*100))/100.0f; // + (min))/100.0f;
	  //ROS_INFO("r: %f",r);
		return randnum;
}

bool pathFound = 0;
void runRRT(float goalPositionX, float goalPositionY){
	//float targetWorldX = (*target_pose_tf_ptr).getOrigin().x() + (*targetPose_ptr).pose.position.x;
	//loat targetWorldY = (*target_pose_tf_ptr).getOrigin().y() + (*targetPose_ptr).pose.position.y;
		/*if(mode == "home" or mode == "goto"){
			for(int i = 0 ; i < ALL_OBJ.size()/16; i++){
				if(i == target_num){
					i=i++;
				}
				for(int j = 0; j<16; j++){
					ALL_OBS.push_back(ALL_OBJ[i*8+j]);
				}
			}
		}*/
		ROS_INFO("start path calc");
	  ros::Time start_time = ros::Time::now();
		int p = 0;
		//ROS_INFO("Start: startx %f starty %f", startx, starty);
		std::vector<Node> nodes;
		std::vector<Node> q_nearest;
    std::vector<float> ndist;
		Node start (startx+OFFSET[0], starty+OFFSET[1],0, 0);
		// Node start (pose.pose.pose.position.x, pose.pose.pose.position.y,0, 0);
		Node goal (0,0,0,0);
		goal = Node(goalPositionX, goalPositionY, 0,0);
		nodes.push_back(start);
		pathFound = 0;
		Node q_rand (randZO(0, XDIM),randZO(0,YDIM),0,0);
		float r1 = 0.200f;
		float r2 = 0.20f;
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
			//ROS_INFO(" ctr %d",ctr);
			for(int j = 0; j<nodes.size(); j++){
					if(  (sqrt(pow(nodes[j].pos[0]-goal.pos[0], 2) + pow(nodes[j].pos[1]-goal.pos[1],2)) <= r1)){
						pathFound = 1;
					}
			}
			if(pathFound == 1){
				ROS_INFO("node set to goal");
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

			step_from_to(q_rand.pos, q_near.pos, q_new.pos);
			//ROS_INFO("2  %f %f", q_new_pos[0], q_new_pos[1]);
			if(checkIntersect(q_rand, q_near)){
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
		ROS_INFO("checkpoint");
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

int getBestObject(){
	std::vector<float> pathDistances;
	float distance;
	float smallest_dist;
	int smallest_dist_indx;
	std::vector<bool> pathExists;
	mode = "goto";

	for(int i = 0; i<objPoseX.size(); i++){
		target_num = i;
		runRRT(objPoseX[i], objPoseY[i]);
		pathExists.push_back(pathFound);
		distance = calculatePathDistance();
		pathDistances.push_back(distance*objWeighting[i]);
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

void publishPath(){

	geometry_msgs::PoseStamped newpose;
	allposes.clear();

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
			if(i < finalpathx.size()-1){
					newpose.pose.orientation.z =(float) atan2((finalpathy[i+1]-finalpathy[i]),(finalpathx[i+1]-finalpathx[i]));
			}else if( i == finalpathx.size()-1){
				if(isGoalInCSpace(finalpathx[i], finalpathy[i]) == 100){
					newpose.pose.orientation.z =(float) atan2((finalpathy[i]-finalpathy[i-1]),(finalpathx[i]-finalpathx[i-1]));
				}else{
					newpose.pose.orientation.z = m+PI;
				}
			}
			newpose.pose.orientation.z = 0;
			ROS_INFO("%f %f", newpose.pose.position.x, newpose.pose.position.y);
			//poses[i] = newpose;
			allposes.push_back(newpose);

			seq = seq +1 ;
	}
	path.poses.resize(allposes.size());

	for(int i = 0; i < allposes.size(); ++i){
		path.poses[i] = allposes[i];
	}
	pathseq++;
	//path.poses = poses;
	ROS_INFO("Checkpoint");

}

void deleteLastObject(int idxToDelete){
	ALL_OBJ.erase(ALL_OBJ.begin() + idxToDelete*8,ALL_OBJ.begin() + idxToDelete*8+7);
	objPoseX.erase(objPoseX.begin() + idxToDelete);
	objPoseY.erase(objPoseY.begin() + idxToDelete);
	objectID.erase(objectID.begin() + idxToDelete);
	objWeighting.erase(objWeighting.begin() + idxToDelete);
}

void actuateGripper(bool command){
	gateSrv.request.control = command;
	gateClient.call(gateSrv);
	//	ROS_INFO("");
	if(gateSrv.response.result == 1){
		if(command == 0){
			ROS_INFO("Gripper closed");
		} else if(command == 1){
			ROS_INFO("Gripper opened");
		}
	}else{
		ROS_INFO("Gripper don't react. Please have a look.");
	}
}
int startInitialized = 0;
float lastGoalX;
float lastGoalY;
int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_rrt");

		//target_tfl_ptr.reset(new tf::TransformListener);

    ros::NodeHandle n;
		ros::Subscriber wall_sub = n.subscribe<visualization_msgs::MarkerArray>("/maze_map", 1000, wallCallback);
		ros::Subscriber pose_sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, currentPoseCallback);
		ros::Subscriber evidence_sub = n.subscribe<rosie_object_detector::RAS_Evidence>("/evidence",10, evidenceCallback);
    //ros::Subscriber obj_sub = n.subscribe<visualization_msgs::Marker>("/visualization_marker", 10, objectCallback);
    //ros::Subscriber bat_sub = n.subscribe<visualization_msgs::Marker>("/visualization_marker_battery", 10, batteryCallback);
		ros::Subscriber rviz_goal = n.subscribe<geometry_msgs::PoseStamped>("/rviz_object_pose",10,rvizTargetPoseCallback);
    path_pub = n.advertise<nav_msgs::Path>("/rosie_rrt_path",1);
    gateClient = n.serviceClient<rosie_servo_controller::ControlGates>("rosie_servo_service");
    ros::ServiceClient reqClient = n.serviceClient<rosie_map_controller::RequestRerun>("request_rerun");
		ros::ServiceClient startClient = n.serviceClient<rosie_map_controller::StartRRT>("start_rrt");

    n.getParam("red_cylinder", red_cylinder_val);
    n.getParam("red_cube", red_cube_val);
    n.getParam("red_hollow_cube", red_hollow_cube_val);
    n.getParam("red_ball", red_ball_val);
    n.getParam("orange_cross", orange_cross_val);
    n.getParam("patric", patric_val);
    n.getParam("yellow_ball", yellow_ball_val);
    n.getParam("yellow_cube", yellow_cube_val);
    n.getParam("green_cube", green_cube_val);
    n.getParam("green_hollow_cube", green_hollow_cube_val);
    n.getParam("green_cylinder", green_cylinder_val);
    n.getParam("blue_cube", blue_cube_val);
    n.getParam("blue_triangle", blue_triangle_val);
    n.getParam("purple_cross", purple_cross_val);
    n.getParam("purple_star", purple_star_val);

    rosie_map_controller::RequestRerun reqSrv;
		rosie_map_controller::StartRRT startSrv;

		static tf::TransformBroadcaster br;

    ros::Rate loop_rate(10);
		ros::Rate drivingtime(0.2);
	  ros::Time load_time = ros::Time::now();

		//objInit();
		//batInit();
		startSrv.request.command = 2;
    while(ros::ok()){
			//bool objectsInitialized = objInit();
			//bool batteriesInitialized = batInit();

//***********************************
//STATE-MACHINE
//***********************************
			if(!startInitialized){
				reqClient.call(startSrv);
				if(startSrv.response.answer){
					startInitialized = 1; // normal collect objects mode
				}
			}
			if(gotTarget){
				//actually ignors other objects
				runRRT(rviz_pose.pose.position.x, rviz_pose.pose.position.y);
				publishPath();
				gotTarget = 0;
			}
			reqSrv.request.question = 1;
			reqClient.call(reqSrv);
			if(reqSrv.response.answer){
				mapInitialized = 0;
				mode = lastMode;
			}
			if(startInitialized && mapInitialized && (objPoseX.size() != 0 || gotTarget)){
				if(mode == "goto"){
					target_num = getBestObject();
					lastGoalX = objPoseX[target_num];
					lastGoalY = objPoseY[target_num];
					runRRT(lastGoalX, lastGoalY);
					publishPath();
					mode = "wait";
					lastMode = "goto";
				}else if( mode == "home"){
					lastGoalX = HOME[0];
					lastGoalY = HOME[1];
					runRRT(HOME[0], HOME[1]);
					publishPath();
					mode = "wait";
					lastMode = "home";
				}else if(mode == "explore"){

				}else if(mode == "wait"){
					if(lastMode == "goto"){
						if(0.05*0.05 < (pow(pose.pose.pose.position.x-lastGoalX,2)+pow(pose.pose.pose.position.y-lastGoalY,2)) < 0.2*0.2){
							actuateGripper(1); //open
						}else if (0 < (pow(pose.pose.pose.position.x-lastGoalX,2)+pow(pose.pose.pose.position.y-lastGoalY,2)) < 0.05*0.05){
							actuateGripper(0);
							mode = "home";
						}else if((pow(pose.pose.pose.position.x-HOME[0],2)+pow(pose.pose.pose.position.y-HOME[1],2)) > 0.2*0.2){
							actuateGripper(0);
						}
					}else if(lastMode == "home"){
						if (0 < (pow(pose.pose.pose.position.x-lastGoalX,2)+pow(pose.pose.pose.position.y-lastGoalY,2)) < 0.05*0.05){
							actuateGripper(1);
							mode = "goto";
							deleteLastObject(target_num);
						}
					}
				}
				path_pub.publish(path);
				ROS_INFO("path published");
				tf::Transform transform;
				transform.setOrigin( tf::Vector3(0,0, 0) );
				tf::Quaternion qtf;
				qtf.setRPY(0, 0, 0);
				transform.setRotation( qtf );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "path"));
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
}
