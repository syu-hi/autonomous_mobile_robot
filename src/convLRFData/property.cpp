#include<autonomous_mobile_robot/convLRFData.h>

void convLRFDataClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    //LRFパラメータ
    n.getParam("lrf/sensorHigh",sensorHigh);
    //マップパラメータ
    n.getParam("localMap/width/float",mapW);
    n.getParam("localMap/height/float",mapH);
    n.getParam("localMap/resolution",mapR);
	mapWi=(int)(mapW/mapR);//[pixel]
	mapHi=(int)(mapH/mapR);//[pixel]

}