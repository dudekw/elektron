/**
 * @class Navigation
 * @brief Class which defines the interface for Robot navigation capabilities (movement, localization)
 * @date 10-August-2015
 * @author Wojciech Dudek <wojciechsbox@gmail.com>
 * @note This class uses pimpl pattern to make ABI as stable as possible when deploying new library versions
 */


// search vector std::find
#include <algorithm>
// inverse matrix etc.
#include <Eigen/Core>
// ros includes
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
// conversions TF to eigen and eigen to TF
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
//include rapp service headers
// GetTransform different then in NAO robot !!!!!!! 
#include "elektron_msgs/MoveTo.h"
#include "elektron_msgs/MoveVel.h"
#include "elektron_msgs/MoveStop.h"
#include "elektron_msgs/GetRobotPose.h"
#include "elektron_msgs/SetGlobalPose.h"
#include "elektron_msgs/MoveJoint.h"
#include "elektron_msgs/MoveAlongPath.h"
#include "elektron_msgs/TakePredefinedPosture.h"
#include "elektron_msgs/LookAtPoint.h"
#include "elektron_msgs/GetTransform.h"
//include rapp-api objects
#include <rapp/objects/pose/pose.hpp>
#include <rapp/objects/pose_stamped/pose_stamped.hpp>
 #include <rapp/objects/qr_code_map/qr_code_map.hpp>
#include "opencv2/core/core.hpp"

namespace rapp {
namespace robot {

class NavigationImpl {

public:

	NavigationImpl (int argc, char ** argv);
	~NavigationImpl();
	
	ros::ServiceClient client_moveTo;
	ros::ServiceClient client_moveVel;
	ros::ServiceClient client_moveStop;
	ros::ServiceClient client_moveJoint;
	ros::ServiceClient client_takePredefinedPosture;
	ros::ServiceClient client_lookAtPoint;
	ros::ServiceClient client_getRobotPose;
	ros::ServiceClient client_setGlobalPose;
	ros::ServiceClient client_moveAlongPath;
	ros::ServiceClient client_getTransform;

	ros::NodeHandle *n;
	
	bool moveTo(float x, float y, float theta);
	bool moveVel(float x, float theta);
	bool moveStop();
	bool moveJoint(std::vector<std::string> joint, std::vector<float> angle);
	bool takePredefinedPosture(std::string posture, float speed);
	bool lookAtPoint(float x, float y, float z);
	bool moveAlongPath(std::vector<rapp::object::pose_stamped> poses);
	rapp::object::pose_stamped getRobotPose();
	bool setGlobalPose(rapp::object::pose rapp_pose);
	
	std::vector<std::vector<float>> getTransform(std::string chainName, int space);/*
	Input:
		chainName: Name of the item. Could be: any joint or chain or sensor.
		space: Task frame {ROBOT_FRAME = 0, MAP_FRAME = 1} 
	Output: The matrix, which contains homogeneous transform relative to the space (frame). Axis definition: the x axis is positive toward the robot’s front, the y from right to left and the z is vertical.
	Description. This function computes the transformation matrix from one frame to another (e.g. from robot frame to camera frame).
	*/
};	
} // namespace robot
} // namespace rapp
