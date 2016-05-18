
////
//  Author: Wojciech Dudek
////

#include <ros/ros.h>
#include <stdio.h> 
#include <unistd.h>
#include <sstream>
#include <string.h>
#include <elektron_msgs/MoveTower.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <fcntl.h>
#include <termios.h>


int fd;
int orientation_yaw = 0;
int orientation_pitch = 0;
std::string toString(double d) {
	std::stringstream ss;
	ss << d;
	return ss.str();
}
bool setConnection(std::string elektron_tower_device){

//char port[20] = elektron_tower_device.c_str(); /* port to connect to */
speed_t baud = B9600; /* baud rate */

fd = open(elektron_tower_device.c_str(), O_WRONLY); /* connect to port */

/* set the other settings (in this case, 9600 8N1) */
struct termios settings;
tcgetattr(fd, &settings);

cfsetospeed(&settings, baud); /* baud rate */
settings.c_cflag &= ~PARENB; /* no parity */
settings.c_cflag &= ~CSTOPB; /* 1 stop bit */
settings.c_cflag &= ~CSIZE;
settings.c_cflag |= CS8 | CLOCAL; /* 8 bits */
settings.c_lflag = ICANON; /* canonical mode */
settings.c_oflag &= ~OPOST; /* raw output */

tcsetattr(fd, TCSANOW, &settings); /* apply the settings */
//tcflush(fd, TCOFLUSH);
std::cout<<fd<<std::endl;
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
bool publishFrames(tf::Transform yaw_transform, tf::Transform pitch_transform,tf::TransformBroadcaster br){

    yaw_transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    yaw_transform.setRotation( tf::Quaternion(0, 0, orientation_yaw, 1) );
    br.sendTransform(tf::StampedTransform(yaw_transform, ros::Time::now(), "head_bottom_fixed", "head_yaw_revolute"));
    pitch_transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    pitch_transform.setRotation( tf::Quaternion(0, 0, orientation_pitch, 1) );
    br.sendTransform(tf::StampedTransform(pitch_transform, ros::Time::now(), "head_upper_fixed", "head_pitch_revolute"));
}

bool moveTower(elektron_msgs::MoveTower::Request  &req,
         elektron_msgs::MoveTower::Response &res)
{
	if (req.moveJoints.size()==2){
		res.status = sendAngles((int) (req.yaw*180/3.14+90), (int) ((req.pitch+0.73)*180/3.14));
		orientation_yaw = req.yaw;
		orientation_pitch = req.pitch;
	}
	if (req.moveJoints.at(0)=="yaw"){
		res.status = sendAngles((int) (req.yaw*180/3.14+90), (int) ((req.pitch+0.73)*180/3.14));
		orientation_yaw = req.yaw;
	}
	if (req.moveJoints.at(0)=="pitch"){
		res.status = sendAngles((int) (req.yaw*180/3.14+90), (int) ((req.pitch+0.73)*180/3.14));
		orientation_pitch = req.pitch;
	}
	return false;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "elektron_tower_effector");
	ros::NodeHandle nh;
	tf::TransformBroadcaster br;
	tf::Transform yaw_transform;
	tf::Transform pitch_transform;
	std::string elektron_tower_device;
	nh.param<std::string>("elektron_tower_device", elektron_tower_device, "/dev/ttyUSB0");
	setConnection(elektron_tower_device);
	ros::Rate rate(10.0);
	std::cout<<elektron_tower_device<<std::endl;
	ros::ServiceServer service = nh.advertiseService("re_moveTowerJoint", moveTower);
	if(fd!=-1){
		ROS_INFO("Ready to move the tower.");
		while (nh.ok()){
		    publishFrames(yaw_transform,pitch_transform,br);
		    rate.sleep();
		    ros::spinOnce();
		}
	}else{
		ROS_ERROR("Elektron Tower is disconected from the robot!");
		return -1;
	}
	return 0;
}
