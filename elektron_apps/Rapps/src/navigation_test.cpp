#include "rapp-robots-api/navigation/navigation.hpp"
#include "rapp/objects/pose_stamped/pose_stamped.hpp"
#include <iostream>
int main(int argc, char ** argv){

	rapp::robot::navigation nav;
	nav.moveStop();
	std::vector<std::string> joint_names;
	joint_names.clear();
	joint_names.push_back("head_pitch");

	std::vector<float> joint_angles;
	joint_angles.clear();
	joint_angles.push_back(0);

	nav.moveJoint(joint_names, joint_angles);

	rapp::object::pose_stamped pose;
	std::vector<rapp::object::pose_stamped> poses_list;
	poses_list.clear();
	poses_list.push_back(pose);
	poses_list.push_back(pose);
	std::cout<<poses_list.at(0).pose.position.x<<std::endl;

	std::cout<<poses_list.size()<<std::endl;
	nav.moveAlongPath(poses_list);

	return 0;
}
