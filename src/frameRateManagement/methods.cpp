#include<autonomous_mobile_robot/frameRateManagement.h>

//subscribe
void managementClass::subscribeData(){//データ受信
	queue.callOne(ros::WallDuration(1));
}
void managementClass::callback(const autonomous_mobile_robot::synchronizedImage::ConstPtr& imageMsg){
    //重複チェック
    checkDuplication(imageMsg);
    culcDifTime(imageMsg);
    //データコピー
    rgbImage = imageMsg->rgb;
    depthImage = imageMsg->depth;
}
//publish
void managementClass::publishRgbCamData(){//データ送信
    pubRgb.publish(rgbImage);
}
void managementClass::publishDepthCamData(){//データ送信
    pubDepth.publish(depthImage);
}
//check duplicated message
void managementClass::checkDuplication(const autonomous_mobile_robot::synchronizedImage::ConstPtr& imageMsg){//データ送信
    if(imageMsg->rgb.header.seq == rgbImage.header.seq){
            dupImageFlag = true;
    }
    else{
            dupImageFlag = false;        
    }
}