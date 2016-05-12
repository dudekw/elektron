/**
 * @class Navigation
 * @brief Class which defines the interface for Robot navigation capabilities (movement, localization)
 * @date 10-August-2015
 * @author Wojciech Dudek <wojciechsbox@gmail.com>
 * @note This class uses pimpl pattern to make ABI as stable as possible when deploying new library versions
 */

#include "NavigationImpl.hpp"

namespace rapp {
namespace robot {

NavigationImpl::NavigationImpl(int argc, char ** argv ){
		ros::init(argc, argv,"NavigationImpl_library");
		n = new ros::NodeHandle();

		}
NavigationImpl::~NavigationImpl() {
}
	bool NavigationImpl::moveTo(float x, float y, float theta){	

		client_moveTo = n->serviceClient<elektron_msgs::MoveTo>("rapp_moveTo");
		  elektron_msgs::MoveTo srv;
		  srv.request.x = x;
		  srv.request.y = y;
		  srv.request.theta = theta;
		  if (client_moveTo.call(srv))
		  {
	  	  	return srv.response.status;
	  	  	ROS_INFO_STREAM("Service ended with status:\n" <<srv.response.status);

			}
		  else
		  {

	  	  	return false;
		    ROS_ERROR("Failed to call service moveTo"); 
		  }

	}

	bool NavigationImpl::moveVel(float x, float theta){	

		client_moveVel = n->serviceClient<elektron_msgs::MoveVel>("rapp_moveVel");

		  elektron_msgs::MoveVel srv;
		  srv.request.velocity_x = x;
		  srv.request.velocity_y = 0;
		  srv.request.velocity_theta = theta;
		  if (client_moveVel.call(srv))
		  {
		    return srv.response.status;
	  	  	ROS_INFO("Elektron moved ");
		  }
		  else
		  {
	  	  	return false;
		    ROS_ERROR("Failed to call service moveVel"); 
		  }
	}

// HeadYaw	 HeadPitch Min	HeadPitch Max	 	HeadYaw	  HeadPitch Min  	HeadPitch Max
// 	         (degrees)						      		(radians)
// -119.52		-25.73			18.91			-2.086017	-0.449073		0.330041
// -87.49		-18.91			11.46			-1.526988	-0.330041		0.200015
// -62.45		-24.64			17.19			-1.089958	-0.430049		0.300022
// -51.74		-27.50			18.91			-0.903033	-0.479965		0.330041
// -43.32		-31.40			21.20			-0.756077	-0.548033		0.370010
// -27.85		-38.50			24.18			-0.486074	-0.671951		0.422021
//   0.0		-38.50			29.51			 0.000000	-0.671951		0.515047
//  27.85		-38.50			24.18			 0.486074	-0.671951		0.422021
//  43.32		-31.40			21.20			 0.756077	-0.548033		0.370010
//  51.74		-27.50			18.91			 0.903033	-0.479965		0.330041
//  62.45		-24.64			17.19			 1.089958	-0.430049		0.300022
//  87.49		-18.91			11.46			 1.526988	-0.330041		0.200015
//  119.52		-25.73			18.91			 2.086017	-0.449073		0.330041

	bool NavigationImpl::moveJoint(std::vector<std::string> joint, std::vector<float> angle){
		client_moveJoint = n->serviceClient<elektron_msgs::MoveJoint>("rapp_moveJoint");

		elektron_msgs::MoveJoint srv;

		srv.request.joint_name = joint;
		srv.request.joint_angle = angle;
		srv.request.speeds = 1;

	 	if (client_moveJoint.call(srv))
	 	  {
	   	  return srv.response.status;
	 	  }
	 	else
	 	  {
	  	  	return false;
	 	    ROS_ERROR("Failed to call service moveJoint"); 
	 	  }
    }		

	bool NavigationImpl::takePredefinedPosture(std::string posture, float speed){
		client_takePredefinedPosture = n->serviceClient<elektron_msgs::TakePredefinedPosture>("rapp_takePredefinedPosture");
		

		  elektron_msgs::TakePredefinedPosture srv;
		  srv.request.pose = posture;
		  srv.request.speed = speed;

		  if (client_takePredefinedPosture.call(srv))
		  {
	  	  	return srv.response.status;

		  }
		  else
		  {
	  	  	return false;
		    ROS_ERROR("Failed to call service takePredefinedPosture"); 
		  }
	}	
	bool NavigationImpl::moveStop(){

		client_moveStop = n->serviceClient<elektron_msgs::MoveStop>("rapp_moveStop");
		  elektron_msgs::MoveStop srv;
		  if (client_moveStop.call(srv))
		  {
		    return srv.response.status;
	  	  	ROS_INFO("Elektron has stopped");
		  }
		  else
		  {
	  	  	return false;
		    ROS_ERROR("Failed to call service moveStop"); 
		  }
	}

	bool NavigationImpl::lookAtPoint(float pointX,float pointY,float pointZ ){

		client_lookAtPoint = n->serviceClient<elektron_msgs::LookAtPoint>("rapp_lookAtPoint");
		  elektron_msgs::LookAtPoint srv;
		  srv.request.pointX = pointX;
		  srv.request.pointY = pointY;
		  srv.request.pointZ = pointZ;

		  if (client_lookAtPoint.call(srv))
		  {
		   return srv.response.status;
	  	  	ROS_INFO_STREAM("Service ended with status:\n" <<srv.response.status);
		  }
		  else
		  {
	  	  	return false;
		    ROS_ERROR("REUEST FAILED:  lookAtPoint"); 
		  }
	}

	bool NavigationImpl::moveAlongPath(std::vector<rapp::object::pose_stamped> poses){
			geometry_msgs::PoseStamped pose_ros;
			nav_msgs::Path poses_ros;
			//poses_ros.poses;
			poses_ros.poses.clear();
			for (int i=0; i < poses.size();i++){
				pose_ros.header.seq = poses.at(i).header.seq;
				pose_ros.header.frame_id = poses.at(i).header.frame_id;
				pose_ros.header.stamp.sec = poses.at(i).header.stamp.sec;
				pose_ros.header.stamp.nsec = poses.at(i).header.stamp.nsec;
				pose_ros.pose.position.x = poses.at(i).pose.position.x;
				pose_ros.pose.position.y = poses.at(i).pose.position.y;
				pose_ros.pose.position.z = poses.at(i).pose.position.z;
				pose_ros.pose.orientation.x = poses.at(i).pose.orientation.x;
				pose_ros.pose.orientation.y = poses.at(i).pose.orientation.y;
				pose_ros.pose.orientation.z = poses.at(i).pose.orientation.z;
				pose_ros.pose.orientation.w = poses.at(i).pose.orientation.w;
				poses_ros.poses.push_back(pose_ros);
			}


		client_moveAlongPath = n->serviceClient<elektron_msgs::MoveAlongPath>("rapp_moveAlongPath");
		  elektron_msgs::MoveAlongPath srv;
  		  srv.request.poses = poses_ros.poses;

		  if (client_moveAlongPath.call(srv))
		  {
		    return srv.response.status;
	  	  	ROS_INFO("Elektron moved along path");
		  }
		  else
		  {
	  	  	return false;
		    ROS_ERROR("Failed to call service MoveAlongPath"); 
		  }

	}
	rapp::object::pose_stamped NavigationImpl::getRobotPose(){

		client_getRobotPose = n->serviceClient<elektron_msgs::GetRobotPose>("rapp_getRobotPose");
		  elektron_msgs::GetRobotPose srv;
		  geometry_msgs::PoseStamped pose_ros;
			rapp::object::pose_stamped pose;

		  	pose_ros = srv.response.pose;

		  if (client_getRobotPose.call(srv))
		  {
	  	  	ROS_INFO("Elektron returned his pose");
		  	
		  	

			
			pose.header.seq = pose_ros.header.seq;
			pose.header.frame_id = pose_ros.header.frame_id;
			pose.header.stamp.sec = pose_ros.header.stamp.sec;
			pose.header.stamp.nsec = pose_ros.header.stamp.nsec;
			pose.pose.position.x = pose_ros.pose.position.x;
			pose.pose.position.y = pose_ros.pose.position.y;
			pose.pose.position.z = pose_ros.pose.position.z;

			pose.pose.orientation.x = pose_ros.pose.orientation.x;
			pose.pose.orientation.y = pose_ros.pose.orientation.y;	
			pose.pose.orientation.z = pose_ros.pose.orientation.z;
			pose.pose.orientation.w = pose_ros.pose.orientation.w;
		    
		    return pose;
		  }
		  else
		  {
		    return pose;
		    ROS_ERROR("Failed to call service getRobotPose"); 
		  }


	}
	bool NavigationImpl::setGlobalPose(rapp::object::pose rapp_pose){
		client_setGlobalPose = n->serviceClient<elektron_msgs::SetGlobalPose>("rapp_setGlobalPose");
		elektron_msgs::SetGlobalPose srv;
		ros::Time time_now = ros::Time::now();
		geometry_msgs::PoseStamped pose_ros;
		pose_ros.pose.position.x = rapp_pose.position.x;
		pose_ros.pose.position.y = rapp_pose.position.y;
		pose_ros.pose.position.z = rapp_pose.position.z; 
		pose_ros.pose.orientation.x = rapp_pose.orientation.x; 
		pose_ros.pose.orientation.y = rapp_pose.orientation.y; 
		pose_ros.pose.orientation.z = rapp_pose.orientation.z; 
		pose_ros.pose.orientation.w = rapp_pose.orientation.w; 
		
		pose_ros.header.seq = 0;
		pose_ros.header.stamp.sec = time_now.sec;
		pose_ros.header.stamp.nsec = time_now.nsec;
		pose_ros.header.frame_id = "map";


  		  srv.request.pose = pose_ros;
		  if (client_setGlobalPose.call(srv))
		  {
	  	  	ROS_INFO("Elektron is localized");
		    return srv.response.status;
		  }
		  else
		  {
		    return false;
		    ROS_ERROR("Failed to call service setGlobalPose"); 
		  }

}
std::vector<std::vector<float>> NavigationImpl::getTransform(std::string chainName, int space){
	client_getTransform = n->serviceClient<elektron_msgs::GetTransform>("rapp_get_transform");
	elektron_msgs::GetTransform srv;
	srv.request.chainName = chainName;
	srv.request.space = space;
	std::vector<std::vector<float>> transformMatrix;

	if (client_getTransform.call(srv)) //
	{
		geometry_msgs::PoseStamped jointPose;
		jointPose = srv.response.jointPose;
		Eigen::Affine3d eigen_affine3d;
		Eigen::Matrix4d eigen_matrix4d;
		Eigen::Matrix4f eigen_matrix4f;
		tf::poseMsgToEigen(jointPose.pose, eigen_affine3d);
		eigen_matrix4d = eigen_affine3d.matrix();
		eigen_matrix4f = eigen_matrix4d.cast <float> ();
		for (int i=0; i<eigen_matrix4f.rows(); ++i)
		{
		    const float* begin = &eigen_matrix4f.row(i).data()[0];
		    transformMatrix.push_back(std::vector<float>(begin, begin+eigen_matrix4f.cols()));
		}
		//*/
		ROS_INFO("[Rapp get transform] - Transformation matrix computed");
		return transformMatrix;
	}
	else
	{

		ROS_ERROR("[Rapp get transform] - Error calling service rapp_get_transform");
		return transformMatrix;

	}

}

	
} // namespace robot
} // namespace rapp
