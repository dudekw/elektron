
#include "ros/ros.h"
#include "elektron_msgs/MoveVel.h"
#include "elektron_msgs/GetImage.h"
#include "elektron_msgs/MoveJoint.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <ros/console.h>
void move(float x, float vel, ros::ServiceClient &client){
	float interval = x/vel;
	elektron_msgs::MoveVel srv;
  	srv.request.velocity_x = vel;
  	srv.request.velocity_y = 0;
  	srv.request.velocity_theta = 0;
	
	if (client.call(srv))
	{
		ROS_INFO_STREAM("Move status: "<< srv.response.status);
	}
	else
	{
		ROS_ERROR_STREAM("Failed to call service rapp_moveVel -> setting velocity aborted");
	}

	ros::Duration(interval).sleep();
  	srv.request.velocity_x = 0;
  	srv.request.velocity_y = 0;
  	srv.request.velocity_theta = 0;

	if (client.call(srv))
	{
		ROS_INFO_STREAM("Stop status: "<< srv.response.status);
	}
	else
	{
		ROS_ERROR_STREAM("Failed to call service rapp_moveVel -> stopping robot aborted");
	}
}

sensor_msgs::Image capture(ros::ServiceClient &client){

	elektron_msgs::GetImage srv;
	srv.request.camera_id = 0;
	srv.request.resolution = 0;
	
	if (client.call(srv))
	{
		ROS_INFO_STREAM("Capture successfull");
		return srv.response.frame;
	}
	else
	{
		ROS_ERROR_STREAM("Failed to call service rapp_captureImage");
	  	return srv.response.frame;
	}
}

void moveCamera(float angle, ros::ServiceClient &client){

	elektron_msgs::MoveJoint srv;
	std::vector<std::string> names;
	names.clear();
	names.push_back("head_yaw");
	std::vector<float> angles;
	angles.clear();
	angles.push_back(angle);
	srv.request.joint_name = names;
	srv.request.joint_angle = angles;
	srv.request.speeds = 1;
	
	if (client.call(srv))
	{
		ROS_INFO_STREAM("Move joint responded with status: "<< srv.response.status);
	}
	else
	{
		ROS_ERROR_STREAM("Failed to call service rapp_moveJoint");
	}
}

void saveImage(std::string path, sensor_msgs::Image ros_image){
	cv::Mat cv_image;
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGR8);
	cv_image = cv_ptr->image;
	cv::imwrite( path, cv_image );
}

void handlePoint(int iterate_init, std::string pointId, ros::ServiceClient &client_capture, ros::ServiceClient &client_moveJoint){
	sensor_msgs::Image ros_image;
	float angle = 0;
	std::string img_path;
	img_path = "mapped_pic/" + pointId;
	int i = 0;
	std::string orient_id;
	std::ostringstream ss;

	for(i; i <  7; i++){
		angle = -M_PI/2 + i * 30 * M_PI/180;
		std::cout<< "angle :"<<angle<<std::endl;
		moveCamera( angle, client_moveJoint);
		ros::Duration(2).sleep();
		ros_image = capture(client_capture);
		if (iterate_init == 0){
			if (i > 3){
				ss.str("");
				ss << 15 - i;
				orient_id = ss.str();
			}
			else{
				ss.str("0");
				ss << 3 - i;
				orient_id = ss.str();
			}
			saveImage(img_path + "_" + orient_id + ".jpg",ros_image );
		}
		else {
			ss.str("0");
			ss << 3 + i;
			orient_id = ss.str(); 
			saveImage(img_path + "_0" + orient_id + ".jpg",ros_image);
		}
	}

}
int main(int argc, char** argv){
ros::init(argc, argv, "my_node_name");
ros::NodeHandle nh;
std::cout << "Usage: <<direction>> <<row_ID>>, direction: 0-> to E || 9-> to W"<< std::endl;
ros::ServiceClient client_moveVel = nh.serviceClient<elektron_msgs::MoveVel>("rapp_moveVel");
ros::ServiceClient client_moveJoint = nh.serviceClient<elektron_msgs::MoveJoint>("rapp_moveJoint");
ros::ServiceClient client_capture = nh.serviceClient<elektron_msgs::GetImage>("rapp_capture_image");

int direction = atoi(argv[1]);
std::string pointID = "";
int cols = 9;
std::ostringstream ss;
ss.str("");
int row = atoi(argv[2]);
ss<<row;

for (int i = 0; i <= cols; i++ ){
	ss << i;
	pointID = ss.str();
	handlePoint(direction, pointID, client_capture, client_moveJoint);
	moveCamera( -M_PI/2 , client_moveJoint);
	if (i==cols)
		break;
	move(0.5, 0.08, client_moveVel);
	ss.str("");
	int row = atoi(argv[2]);
	ss<<row;

}
return 0;
}