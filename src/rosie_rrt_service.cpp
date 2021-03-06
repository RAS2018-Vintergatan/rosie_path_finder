#include "ros/ros.h"
#include "arduino_servo_control/SetServoAngles.h"
#include <rosie_path_finder/RequestRerun.h>
#include <rosie_path_finder/CommissionRerun.h>
#include <rosie_path_finder/StartRRT.h>

#include <cstdlib>

int rerun_command = 0; // no rerun

bool RequestRerun(rosie_path_finder::RequestRerun::Request &req,rosie_path_finder::RequestRerun::Response &res){

	if(req.question == 0){
		res.answer = rerun_command;
	}else if(req.question == 1){
		ROS_INFO("Callback Request");
		res.answer = rerun_command;
		rerun_command = 0;
	}
	ROS_INFO("%d", rerun_command);
	return true;

}

bool CommissionRerun(rosie_path_finder::CommissionRerun::Request &req, rosie_path_finder::CommissionRerun::Response &res){

	if(req.command == 0){
		rerun_command = 0;
		res.answer = 0;
	}else if(req.command == 1){
		ROS_INFO("Callback Rerun RRT");
		rerun_command = 1;
		res.answer = 1;
	}else{
		ROS_INFO("This is an unvalid operation. Requested: %d", req.command);
		rerun_command = 0;
	}

	return true;
}

bool start_command = 0;
bool StartPathPlanning(rosie_path_finder::StartRRT::Request &req, rosie_path_finder::StartRRT::Response &res){
	if(req.command == 1){
		start_command = 1;
		res.answer = 1;
	}else if(req.command == 2){
		res.answer = start_command;
	}

	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosie_rrt_server");

  ros::NodeHandle n;

  //ros::ServiceClient client = n.serviceClient<arduino_servo_control::SetServoAngles>("SetServoAngles");
  ros::ServiceServer ReqService = n.advertiseService<rosie_path_finder::RequestRerun::Request, rosie_path_finder::RequestRerun::Response>("request_rerun", RequestRerun);
	ros::ServiceServer CommService = n.advertiseService<rosie_path_finder::CommissionRerun::Request, rosie_path_finder::CommissionRerun::Response>("commission_rerun", CommissionRerun);
	ros::ServiceServer StartService = n.advertiseService<rosie_path_finder::StartRRT::Request, rosie_path_finder::StartRRT::Response>("start_rrt", StartPathPlanning);
  //ros::Subscriber sub = n.subscribe("MoveGates", 1000, GateCallback);
  ROS_INFO("Ready to do path planning.");
  ros::spin();

  return 0;
}
