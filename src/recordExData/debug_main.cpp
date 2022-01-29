#include <ros/ros.h>
#include <autonomous_mobile_robot/recordExData.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"autonomous_mobile_robot_red");
	
	ROS_INFO("recordExData define");
    recordExData red; //
    ros::spin();
	return 0;
}