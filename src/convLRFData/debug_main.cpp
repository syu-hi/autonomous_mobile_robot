#include <ros/ros.h>
#include <autonomous_mobile_robot/convLRFData.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"autonomous_mobile_robot_lrf");
	
    convLRFDataClass lrf; //pan_tilt_control_instance
	// while(ros::ok())
	// {
	// }
    ros::spin();
	return 0;
}