#include<autonomous_mobile_robot/synchroImage.h>

void syncroImageClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("debugType",debugType);
}

