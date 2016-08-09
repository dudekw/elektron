#include <move_base/my_base.h>

#include <elektron_msgs/MoveAlongPathRequest.h>
#include <elektron_msgs/MoveAlongPathResponse.h>
#include <elektron_msgs/MoveAlongPath.h>


int main(int argc, char** argv){
	ros::init(argc, argv, "move_base_node");
	tf::TransformListener tf(ros::Duration(10));

  move_base::MoveBase move_base( tf );

  ros::MultiThreadedSpinner s(5);
  s.spin();
   // ros::spin();

  return(0);
}
