#include<autonomous_mobile_robot/convLRFData.h>

convLRFDataClass::convLRFDataClass()
	:sensorHigh(0.2),mapW(4.0),mapH(4.0),mapR(0.05)
{
	//subscriber
	// nhSub.setCallbackQueue(&queue);
	sub=nhSub.subscribe("scan",1,&convLRFDataClass::sensor_callback,this);
	//publisher
    pub= nhPub.advertise<autonomous_mobile_robot::SensorMapData>("laserMapData", 10);
	// lanchファイルの読み込み
	setLaunchParam();

	// lrf localmap parameter 
	// mapW=8;//width[m]
	// mapH=8;//height[m]
	// mapR=0.05;//resolution[m]
	// mapWi=(int)(mapW/mapR);//[pixel]
	// mapHi=(int)(mapH/mapR);//[pixel]
	// sensorHigh=0.7;//sensor length
	
}
convLRFDataClass::~convLRFDataClass(){
}