
////
//  Author: Wojciech Dudek
////

#include <ros/ros.h>
#include <stdio.h> 
#include <unistd.h>
#include <fcntl.h>
#include <sstream>
#include <string.h>
#include <elektron_tower/MoveTower.h>
#include <ros/console.h>

int fd;

std::string toString(double d) {
	std::stringstream ss;
	ss << d;
	return ss.str();
}
bool sendAngles( int yaw, int pitch){
		const char* message_char;
		std::string message_str = toString(pitch)+" "+toString(yaw)+"\n";
		message_char = message_str.c_str();
		//std::cout<<message_char<<std::endl;
	if(fd!=-1){
		write(fd,message_char,strlen(message_char));

		ROS_INFO_STREAM("New position of the Elektron tower is set:\n"
		<<"yaw: "<<yaw<<" | pitch: "<<pitch);
		return true;
	}else{
		ROS_ERROR("Elektron Tower is disconected from the robot!");
	
	}
	return false;
}

bool moveTower(elektron_tower::MoveTower::Request  &req,
         elektron_tower::MoveTower::Response &res)
{
	res.status = sendAngles(req.yaw, req.pitch);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "elektron_tower_effector");
	ros::NodeHandle nh;
std::string elektron_tower_name;
nh.param<std::string>("elektron_tower_name", elektron_tower_name, "/dev/ttyACM0");
fd = open(elektron_tower_name.c_str(),O_WRONLY);
ros::ServiceServer service = nh.advertiseService("/moveTower", moveTower);
if(fd!=-1){

	ROS_INFO("Ready to move the tower.");
	}else{
		ROS_ERROR("Elektron Tower is disconected from the robot!");
	return -1;
	}
	
	ros::spin();
	return 0;
}
