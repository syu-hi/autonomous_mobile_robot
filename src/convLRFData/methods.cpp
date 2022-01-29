#include<autonomous_mobile_robot/convLRFData.h>

//subscribe
// void convLRFDataClass::subscribeSensorData(){//データ受信
	// queue.callOne(ros::WallDuration(1));
// }
void convLRFDataClass::sensor_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //データコピー（読み込み）
    // sensorData.header = msg->header;
    // sensorData.angle_min = msg->angle_min;
    // sensorData.angle_max = msg->angle_max;
    // sensorData.angle_increment = msg->angle_increment;
    // sensorData.time_increment = msg->time_increment;
    // sensorData.scan_time = msg->scan_time;
    // sensorData.range_min = msg->range_min;
    // sensorData.range_max = msg->range_max;
    // sensorData.ranges = msg->ranges;
    // sensorData.intensities = msg->intensities;
    sensorData = *msg;
	// ROS_ERROR_STREAM("Could not lrfMAP."<<sensorData);
    manage();
}

void convLRFDataClass::manage(){
    create2dMap();
    // ROS_INFO("if");
	if((int)smd.pt.size() < 0){
		ROS_INFO("No cpLRFData");
	}
    publishConvLRFData();
    clearMessages();
}

void convLRFDataClass::create2dMap(){
    //仮変数
    float x_temp;
    float y_temp;
    float z_temp;
    // sensorMapData smd;
    smd.header = sensorData.header;
    smd.width.data = mapW;
    smd.height.data = mapH;
    smd.res.data = mapR;
    smd.widthInt.data = (int)(mapW/mapR);
    smd.heightInt.data = (int)(mapH/mapR);
    smd.cp.x=0;
    smd.cp.y=0;
    smd.cp.z=0;
    smd.index.resize(smd.heightInt.data*smd.widthInt.data);
    smd.size.resize(smd.heightInt.data*smd.widthInt.data);
    smd.pt.resize(smd.heightInt.data*smd.widthInt.data);
    //--インデックスデータ(sensorMapData)
    //インデックス初期化
    for(int h=0;h<smd.heightInt.data;h++){
        for(int w=0;w<smd.widthInt.data;w++){
            smd.index[h*smd.heightInt.data+w].data=-1;
            smd.size[h*smd.widthInt.data+w].data=0;
            smd.pt[h*smd.widthInt.data+w].x = 0;
            smd.pt[h*smd.widthInt.data+w].y = 0;
            smd.pt[h*smd.widthInt.data+w].z = 0;
        }
    }

    int k=0;//データカウンタ
    int dataSize=(int)((sensorData.angle_max - sensorData.angle_min)/sensorData.angle_increment);
    //2次元ローカルマップの作成
    for(int i=0;i<dataSize;i++){
        //センサーデータ格納
        float d = sensorData.ranges[i];
        float th = sensorData.angle_min + sensorData.angle_increment * i;
        //センサーエラー処理
        if(d < sensorData.range_min || // エラー値の場合
            d > sensorData.range_max || // 測定範囲外の場合
            std::isnan(d)) // 無限遠の場合
	    {
            continue;
        }
        else{
            //座標返還
            y_temp = sensorHigh;//センサの高さ
            x_temp = d*(-sin(th));//横方向
            z_temp = d*(cos(th));//奥行方向
            //データ格納
            int xi,zi;//仮変数
            if(convertToGrid(x_temp,z_temp,xi,zi)){//データ変換
                //インデックスデータ
                smd.index[zi*smd.widthInt.data+xi].data = k;
                // データサイズのインクリメント
                smd.size[i].data+=1;
                //マップデータ格納
                smd.pt[k].x=x_temp;//横方向
                smd.pt[k].y=z_temp;//奥行
                smd.pt[k].z=y_temp;//高さ
                k++;//インクリメント
            }
        }
    }
    //サイズ調整
    smd.pt.resize(k);
    // ROS_INFO("size():%d",k);
}
bool convLRFDataClass::convertToGrid(const float& x,const float& y,int& xg,int& yg){
	//マップ上の中心座標(ふつうは　センサ位置＝マップ中心座標　のため　cx=cy=0)
	float cx=0;
	float cy=0;
    //マップ原点座標を画像原点座標(左上)に移動
	float map_x = mapW/2 + (x - cx);
	float map_y = mapH/2 + ( -(y - cy) );
    //マップ外のデータの際はreturn false
	if(map_x<0 || map_x>mapW)
		return false;
	if(map_y<0 || map_y>mapH)
		return false;
    //ピクセルデータに変換
	xg = (int)(map_x/mapR);
	yg = (int)(map_y/mapR);
    //変換成功
	return true;
}
void convLRFDataClass::publishConvLRFData(){//データ送信
    pub.publish(smd);
    ROS_INFO("publish ok");
}

void convLRFDataClass::clearMessages(){
    smd.index.clear();
    smd.pt.clear();
    smd.size.clear();
}