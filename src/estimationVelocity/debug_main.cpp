#include <ros/ros.h>
#include <autonomous_mobile_robot/velocityEstimation.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"autonomous_mobile_robot_estimate");
	
	// ROS_INFO("velocityEstimation define");
    velocityEstimation ec; //
    ros::spin();
	return 0;
}