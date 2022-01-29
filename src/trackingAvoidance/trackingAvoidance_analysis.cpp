#include<autonomous_mobile_robot/trackingAvoidance.h>
void trackingAvoidance::setDefaultCrossPointChecker(){
    //クロスポイントチェッカー入力
    // float debugEncoderVel_r;//ロボットエンコーダ速度(右車輪)
    // float debugEncoderVel_l;//ロボットエンコーダ速度(左車輪)
    // float debugCmd_vel;//ロボット目標速度
    // float debugCmd_angle;//ロボット目標角度
    // int debugIndexRef;//障害物番号
    // geometry_msgs::Point debugGpRef;//クラスタ重心
    // geometry_msgs::Twist debugTwistRef;//障害物速度
    // float debguObstacleRadius;//障害物半径
    // float debugRobotRadius;//ロボット半径
    debugFlag_crossPointChecker = false;
    debugEncoderVel_r = 0;//ロボットエンコーダ速度(右車輪)
    debugEncoderVel_l = 0;//ロボットエンコーダ速度(左車輪)
    debugCmd_vel = 0;//ロボット目標速度
    debugCmd_angle = 0;//ロボット目標角度
    debugIndexRef = 0;//障害物番号
    debugGpRef.x = 0;//クラスタ重心x
    debugGpRef.y = 0;//クラスタ重心y
    debugGpRef.z = 0;//クラスタ重心z
    debugTwistRef.linear.x = 0;//障害物速度linear x
    debugTwistRef.linear.y = 0;//障害物速度linear y
    debugTwistRef.linear.z = 0;//障害物速度linear z
    debugTwistRef.angular.x = 0;//障害物速度anguler x
    debugTwistRef.angular.y = 0;//障害物速度anguler y
    debugTwistRef.angular.z = 0;//障害物速度anguler z
    debugObstacleRadius = 1.0;//障害物半径
    debugRobotRadius = 1.0;//ロボット半径
}
void trackingAvoidance::showOdometries(){//pubDeb3
    //マーカー表示
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header= cur_robotOdom.header;
    marker.ns = "my_namespace";
    // marker.lifetime = ros::Duration(1.0);
    // marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    markerArray.markers.resize(1+1);
    int count = 0;
    //ロボット自己位置
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.ns = "cur_robotOdom";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.15;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.pose.position.x = cur_robotOdom.pose.pose.position.x;
    marker.pose.position.y = cur_robotOdom.pose.pose.position.y;
    marker.pose.position.z = cur_robotOdom.pose.pose.position.z;
    marker.id = count;
    markerArray.markers[count++] = marker;
    //ゴール
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.ns = "goalOdom";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.pose.position.x = goalOdom.pose.pose.position.x;
    marker.pose.position.y = goalOdom.pose.pose.position.y;
    marker.pose.position.z = goalOdom.pose.pose.position.z;
    marker.id = count;
    markerArray.markers[count++] = marker;
    //リサイズpublish
    markerArray.markers.resize(count);
    // ROS_INFO("odom:markerArray.markers.size():%d",(int)markerArray.markers.size());
    if(markerArray.markers.size()){
        pubDeb3.publish( markerArray );
    }   
}
void trackingAvoidance::showClusters(){//pubDeb1
    //マーカー表示
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header= cur_clstr.header;
    marker.ns = "my_namespace";
    // marker.lifetime = ros::Duration(1.0);
    // marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    int marker_size=0;
    for(int i = 0; i<cur_clstr.data.size();i++){
        marker_size += (int)cur_clstr.data[i].pt.size();
    }
    marker_size += (int)cur_clstr.data.size();
    markerArray.markers.resize(marker_size);
    int count = 0;
    for(int i = 0; i<cur_clstr.data.size();i++){
        std::string clusterNumStr = "cluster: ";
        clusterNumStr = clusterNumStr + std::to_string(i);
        marker.ns = clusterNumStr;
        //text
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = "vx,vy:("+ std::to_string(cur_clstr.twist[i].twist.linear.x) +","+ std::to_string(cur_clstr.twist[i].twist.linear.y)+")" ;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.3;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.7;
        marker.pose.position.x = cur_clstr.data[i].gc.y;
        marker.pose.position.y = -cur_clstr.data[i].gc.x;
        marker.pose.position.z = cur_clstr.data[i].gc.z + 1.0;
        marker.id = count;
        markerArray.markers[count++] = marker;
        //position 
        double yaw = std::atan2(-cur_clstr.twist[i].twist.linear.x, cur_clstr.twist[i].twist.linear.y);
        if(cur_clstr.twist[i].twist.linear.x==0 && cur_clstr.twist[i].twist.linear.y ==0){
           marker.type = visualization_msgs::Marker::SPHERE; 
        }
        else{
            marker.type = visualization_msgs::Marker::ARROW;
            marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
        for(int k = 0; k<cur_clstr.data[i].pt.size();k++){
            marker.pose.position.x = cur_clstr.data[i].pt[k].y;
            marker.pose.position.y = -cur_clstr.data[i].pt[k].x;
            marker.pose.position.z = cur_clstr.data[i].pt[k].z;
            marker.color.a = 1.0;
            marker.color.r = 255;
            marker.color.g = 255;
            marker.color.b = 255;
            marker.scale.x = 0.1;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.id = count;
            markerArray.markers[count++] = marker;
        }
    }
    //リサイズpublish
    markerArray.markers.resize(count);
    // ROS_INFO("odom:markerArray.markers.size():%d",(int)markerArray.markers.size());
    if(markerArray.markers.size()){
        pubDeb1.publish( markerArray );
    }   
}
void trackingAvoidance::showDetectedClusters(){//pubDeb2
    //マーカー表示
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header= detected_clstr.header;
    marker.ns = "my_namespace";
    // marker.lifetime = ros::Duration(1.0);
    // marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    int marker_size=0;
    for(int i = 0; i<detected_clstr.data.size();i++){
        marker_size += (int)detected_clstr.data[i].pt.size();
    }
    marker_size += (int)detected_clstr.data.size();
    markerArray.markers.resize(marker_size);
    // ROS_INFO("markerArray.markers.size():%d",(int)markerArray.markers.size());
    int count = 0;
    for(int i = 0; i<detected_clstr.data.size();i++){
        std::string clusterNumStr = "cluster: ";
        clusterNumStr = clusterNumStr + std::to_string(i);
        marker.ns = clusterNumStr;
        //text
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = "vx,vy:("+ std::to_string(detected_clstr.twist[i].twist.linear.x) +","+ std::to_string(detected_clstr.twist[i].twist.linear.y)+")" ;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.3;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.7;
        marker.pose.position.x = detected_clstr.data[i].gc.y;
        marker.pose.position.y = -detected_clstr.data[i].gc.x;
        marker.pose.position.z = detected_clstr.data[i].gc.z + 1.0;
        marker.id = count;
        markerArray.markers[count++] = marker;
        //position 
        double yaw = std::atan2(-detected_clstr.twist[i].twist.linear.x, detected_clstr.twist[i].twist.linear.y);
        if(detected_clstr.twist[i].twist.linear.x==0 && detected_clstr.twist[i].twist.linear.y ==0){
           marker.type = visualization_msgs::Marker::SPHERE; 
        }
        else{
            marker.type = visualization_msgs::Marker::ARROW;
            marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
        for(int k = 0; k<detected_clstr.data[i].pt.size();k++){
            marker.pose.position.x = detected_clstr.data[i].pt[k].y;
            marker.pose.position.y = -detected_clstr.data[i].pt[k].x;
            marker.pose.position.z = detected_clstr.data[i].pt[k].z;
            marker.color.a = 1.0;
            marker.color.r = 255;
            marker.color.g = 255;
            marker.color.b = 255;
            marker.scale.x = 0.1;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.id = count;
            markerArray.markers[count++] = marker;
        }
    }           
    //リサイズpublish
    markerArray.markers.resize(count);
    // ROS_INFO("odom:markerArray.markers.size():%d",(int)markerArray.markers.size());
    if(markerArray.markers.size()){
        pubDeb2.publish( markerArray );
    }             
}
//データ作成
void trackingAvoidance::createRobotData(){
    ros::Time set_time = ros::Time::now();
    // 必要データ
    // ロボットの幅
    // d = 0.300/2.0;
    // センサ視野角度
    // sensor_angle_min = M_PI_4;
    // sensor_angle_max = M_PI_4*3;
    // ロボットの最高速度
    // float max_speed = 0.6;
    // ロボットの位置,角度
    // float robot_x = 0.0;
    // float robot_y = 0.0;
    // float robot_angle = 0.0;
    // ロボットの初期速度
    // float robot_v = 0.0;//移動速度
    // float robot_w = 0.0;//回転角速度
    //オドメトリ
    //tf座標系で作成
    cur_robotOdom.header.frame_id= "base_link";
    cur_robotOdom.header.seq = 0;
    cur_robotOdom.header.stamp = set_time;
    cur_robotOdom.pose.pose.position.x = robot_init_y;
    cur_robotOdom.pose.pose.position.y = -robot_init_x;
    cur_robotOdom.pose.pose.position.z = 0;
    cur_robotOdom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);//正面方向
    //エンコーダー
    robotEncoder_sim.header.frame_id= "base_link";
    robotEncoder_sim.header.seq = 0;
    robotEncoder_sim.header.stamp = set_time;
    robotEncoder_sim.vel.r = robot_init_v + wheel_d * robot_init_w;
    robotEncoder_sim.vel.l = robot_init_v - wheel_d * robot_init_w;
}
void trackingAvoidance::createGoalOdom(double goal_x_temp,double goal_y_temp){
    //goalOdom
    ros::Time set_time = ros::Time::now();
    //tf座標系で作成 rviz表示 -> 一般的二次元座標 x -> y ,-y -> x
    goalOdom_sim.header.frame_id= "base_link";
    goalOdom_sim.header.seq = 0;
    goalOdom_sim.header.stamp = set_time;
    goalOdom_sim.pose.pose.position.x =  goal_y_temp;
    goalOdom_sim.pose.pose.position.y = -goal_x_temp;
    goalOdom_sim.pose.pose.position.z = 0;
    goalOdom_sim.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);//正面方向
}
//データ更新
void trackingAvoidance::updateRobotEncoder(){//ロボットオドメトリ, 命令速度依存
    //ノイズ無し
    robotEncoder_sim.vel.r = cmd_vel.linear.x + wheel_d * cmd_vel.angular.z;
    robotEncoder_sim.vel.l = cmd_vel.linear.x - wheel_d * cmd_vel.angular.z;
}
void trackingAvoidance::updateObstacleState(){//一つ前のロボット姿勢, 現時点のエンコーダ使用
    //障害物の個数
    //障害物の位置角度速度
    //障害物の大きさ（半径）
    //ヘッダ
    cur_clstr.header.frame_id = pre_clstr.header.frame_id;
    cur_clstr.header.stamp = ros::Time::now();
    cur_clstr.header.seq = pre_clstr.header.seq + 1;
    //経過時間算出
    ros::Duration delta_time_ros = cur_clstr.header.stamp - pre_clstr.header.stamp;
    double delta_time = delta_time_ros.toSec();
    //データ更新
    //--ロボットの移動方向と姿勢を取得
    double v = (robotEncoder_sim.vel.r+robotEncoder_sim.vel.l)/2.0;
    double w = (robotEncoder_sim.vel.r-robotEncoder_sim.vel.l)/(2.0*wheel_d);
    tf::Quaternion quat;
    double r,p,y;
    quaternionMsgToTF(enc_robotOdom.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(r, p, y);
    y = y + M_PI_2;//tf座標系からグローバル座標系へ
    //--ロボット速度を取得(グローバル座標系)
    double vy_r = v*sin(w * delta_time + y);
    double vx_r = v*cos(w * delta_time + y);
    // std::cout<<"vr:("<<vx_r<<","<<vy_r<<std::endl;
    //--速度
    for(int i = 0; i<cur_clstr.twist.size();i++){
        cur_clstr.twist[i].twist.linear.x = pre_clstr.twist[i].twist.linear.x + vx_r;
        cur_clstr.twist[i].twist.linear.y = pre_clstr.twist[i].twist.linear.y + vy_r;
    }
    //--位置
    //グローバル座標系で
    for(int i = 0; i<cur_clstr.data.size();i++){
        cur_clstr.data[i].gc.x = pre_clstr.data[i].gc.x + vx_r;
        cur_clstr.data[i].gc.y = pre_clstr.data[i].gc.y + vy_r;
        for(int k = 0; k<cur_clstr.data[i].pt.size();k++){
            cur_clstr.data[i].pt[k].x = pre_clstr.data[i].pt[k].x + vx_r*delta_time;
            cur_clstr.data[i].pt[k].y = pre_clstr.data[i].pt[k].y + vy_r*delta_time;
        }
    }
}
void trackingAvoidance::updateRobotState(){//ロボット速度使用
    // ロボットの位置,角度
    // ロボットの速度
    cur_robotOdom.header.frame_id = enc_robotOdom.header.frame_id;
    cur_robotOdom.header.stamp = ros::Time::now();
    cur_robotOdom.header.seq = enc_robotOdom.header.seq + 1;
    //経過時間算出
    ros::Duration delta_time_ros = cur_robotOdom.header.stamp - enc_robotOdom.header.stamp;
    double delta_time = delta_time_ros.toSec();
    //速度算出
    double v = (robotEncoder_sim.vel.r+robotEncoder_sim.vel.l)/2.0;
    double w = (robotEncoder_sim.vel.r-robotEncoder_sim.vel.l)/(2.0*wheel_d);
    //--姿勢取得(tf座標系)
    tf::Quaternion quat;
    double r,p,y;
    quaternionMsgToTF(enc_robotOdom.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(r, p, y);
    //--ロボット速度算出(tf座標系)
    double vx_r = v*cos(w * delta_time + y);
    double vy_r = v*sin(w * delta_time + y);
    //位置更新(tf座標系)
    cur_robotOdom.pose.pose.position.x = cur_robotOdom.pose.pose.position.x + vx_r *delta_time;
    cur_robotOdom.pose.pose.position.y = cur_robotOdom.pose.pose.position.y + vy_r *delta_time;
    cur_robotOdom.pose.pose.position.z = cur_robotOdom.pose.pose.position.z;
    double yaw = w * delta_time + y;
    cur_robotOdom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
}
//視野角反映
void trackingAvoidance::detectObstacle(){//ロボットから視認できる障害物を抽出
    //ロボットと障害物の角度, ロボット姿勢から視野角内の点群を抽出
    //1.ロボットと障害物の角度, ロボット姿勢をグローバル座標系に変換する
    //2.障害物の各点との角度を算出して視野角内かを確認
    //3.視野角内の点を追加
    //重心位置と障害物点群の取り扱い: 重心が見えている障害物のみを移動障害物と視認可能
    // クラスタのコピー
    detected_clstr = cur_clstr;
    //ロボットの姿勢を取得
    //tf座標系
    double robot_roll,robot_pitch,robot_yaw;
    tf::Quaternion quat;
    quaternionMsgToTF(cur_robotOdom.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(robot_roll,robot_pitch,robot_yaw);
    //--グローバル座標系に変換
    robot_yaw = robot_yaw + M_PI_2;//90度ずらす
    //ロボットの位置を取得
    double robot_x, robot_y, robot_z;
    //--グローバル座標系に変換
    robot_x = -cur_robotOdom.pose.pose.position.y;
    robot_y = cur_robotOdom.pose.pose.position.x;
    //障害物位置を取得
    for(int i = 0; i<cur_clstr.data.size();i++){
        //重心位置が視認可能かを確認
        double delta_gcx = cur_clstr.data[i].gc.x - robot_x;
        double delta_gcy = cur_clstr.data[i].gc.y - robot_y;
        double angle_gc = atan2(delta_gcy,delta_gcx);
        if(angle_gc < sensor_angle_min || angle_gc > sensor_angle_max){
            //センサ視野外->静止障害物
            detected_clstr.twist[i].twist.linear.x = 0;
            detected_clstr.twist[i].twist.linear.y = 0;
        }
        //点群データを空にしてリサイズ
        detected_clstr.data[i].pt.clear();
        detected_clstr.data[i].pt.resize(cur_clstr.data[i].pt.size());
        int itr =0;
        //点群の各点が視認可能かを確認
        for(int k = 0; k<cur_clstr.data[i].pt.size();k++){
            double delta_ptx = cur_clstr.data[i].pt[k].x - robot_x;
            double delta_pty = cur_clstr.data[i].pt[k].y - robot_y;
            double angle_pt = atan2(delta_pty,delta_ptx);
            if(angle_pt < sensor_angle_min || angle_pt > sensor_angle_max){
                //センサ視野外->静止障害物
                //スキップ
            }
            else{
                detected_clstr.data[i].pt[itr].x = cur_clstr.data[i].pt[k].x;
                detected_clstr.data[i].pt[itr].y = cur_clstr.data[i].pt[k].y;
                detected_clstr.data[i].pt[itr].z = cur_clstr.data[i].pt[k].z;
                itr++;
            }
        }
        //再度リサイズ
        detected_clstr.data[i].pt.resize(itr);
    }
}
//デバッグ方法選択メソッド
void trackingAvoidance::debug(){
    switch(debugType){
        case 1: showCrossPoints();break;
        default: ;
    }
}
// 交差位置をマーカーで表示する
void trackingAvoidance::showCrossPoints(){
    //--sample
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = clstr.header.stamp;
    marker.ns = "my_namespace";
    marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    markerArray.markers.resize((int)clstr.data.size() * 3);
    int count = 0;
    //交差位置を取得
    std::vector<crossPoint> crsPts;
	crsPts.resize((int)clstr.data.size()*2);
    // ROS_INFO("cur,cmd,ang : %f,%f,%f",debugCur_vel,debugCmd_vel,debugCmd_angle );
	// ROS_INFO_STREAM("in debug"<<":"<<debugCur_vel<<", "<<debugCur_angle_steer<<", "<<debugCmd_vel<<", "<<debugCmd_angle);
    crossPointsDetect(crsPts,debugCur_vel, debugCur_angle_steer, debugCmd_vel,debugCmd_angle);//rqt_reconfiureの値を使用
    for(int k=0; k<clstr.data.size(); k++){
        // ROS_INFO("obst X(x,y), v(x,y) : X(%f,%f),v(%f,%f)",clstr.data[k].gc.x, clstr.data[k].gc.y, clstr.twist[k].linear.x,clstr.twist[k].linear.y );
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.3;//debugObstacleRadius*2+abs(clstr.twist[k].linear.y);
        marker.scale.y = 0.1;//debugObstacleRadius*2+abs(-clstr.twist[k].linear.x);
        marker.scale.z = 0.1;
        // local -> rviz 
        marker.pose.position.x = clstr.data[k].gc.y;
        marker.pose.position.y = -clstr.data[k].gc.x;
        marker.pose.position.z = clstr.data[k].gc.z;
        //angle
        // double yaw = std::atan2(-clstr.twist[k].linear.x, clstr.twist[k].linear.y);
        double yaw = std::atan2(-clstr.twist[k].twist.linear.x, clstr.twist[k].twist.linear.y);
        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.id = count;
        markerArray.markers[count++] = marker;
        marker.type = visualization_msgs::Marker::SPHERE;
        //position
        //定義済みの交差位置構造体から取得
	    marker.color.a = 1.0;
        marker.color.r = colors[k][0];
        marker.color.g = colors[k][1];
        marker.color.b = colors[k][2];
        marker.pose.position.z = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        crossPoint crsPt = crsPts[k*2];
        // std::cout<<"crsPt["<<k*2<<"]:("<<crsPts[k*2].x<<","<<crsPts[k*2].y<<","<<crsPts[k*2].t<<","<<crsPts[k*2].vx<<","<<crsPts[k*2].vy<<","<<crsPts[k*2].safe<<std::endl;
        //危険, 安全障害物ともに同じように表示している
        marker.pose.position.x = crsPt.y;
        marker.pose.position.y = -crsPt.x;
        //add Array
        marker.id = count;
        markerArray.markers[count++] = marker;
        crsPt = crsPts[k*2+1];
        // std::cout<<"crsPt["<<k*2+1<<"]:("<<crsPts[k*2+1].x<<","<<crsPts[k*2+1].y<<","<<crsPts[k*2+1].t<<","<<crsPts[k*2+1].vx<<","<<crsPts[k*2+1].vy<<","<<crsPts[k*2+1].safe<<std::endl;
        //危険, 安全障害物ともに同じように表示している
        marker.pose.position.x = crsPt.y;
        marker.pose.position.y = -crsPt.x;
        //add Array
        marker.id = count;
        markerArray.markers[count++] = marker;
    }
    markerArray.markers.resize(count);
    // ROS_INFO("markerArray.markers.size():%d",(int)markerArray.markers.size());
    if(markerArray.markers.size()){
        pubDebMarkerArray.publish( markerArray );
    }
}
//出力と交差位置の表示
void trackingAvoidance::showOutPut(std::vector<crossPoint>& crsPts, float v, int num){
    //--sample
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = clstr.header.stamp;
    marker.ns = "my_namespace";
    marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    markerArray.markers.resize((int)crsPts.size() + (int)clstr.data.size() + 2+10 +1);
    int count = 0;
    for(int k=0; k<clstr.data.size(); k++){
        marker.ns = "obstacle_vec";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.3;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        // local -> rviz 
        marker.pose.position.x = clstr.data[k].gc.y;
        marker.pose.position.y = -clstr.data[k].gc.x;
        marker.pose.position.z = clstr.data[k].gc.z;
        //angle
        // double yaw = std::atan2(-clstr.twist[k].linear.x, clstr.twist[k].linear.y);
        double yaw = std::atan2(-clstr.twist[k].twist.linear.x, clstr.twist[k].twist.linear.y);
        // if(clstr.twist[k].linear.x==0 && clstr.twist[k].linear.y ==0){
        if(clstr.twist[k].twist.linear.x==0 && clstr.twist[k].twist.linear.y ==0){
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.5;   
            yaw = 0;
        }
        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.id = count;
        markerArray.markers[count++] = marker;
        marker.ns = "obstacle_pos";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 0.3;//debugObstacleRadius*2+abs(clstr.twist[k].linear.y);
        marker.scale.y = 0.2;//debugObstacleRadius*2+abs(-clstr.twist[k].linear.x);
        marker.scale.z = 0.2;
        //position
        //定義済みの交差位置構造体から取得
        if((int)crsPts.size() == 0){
            continue;
        }
	    marker.color.a = 1.0;
        marker.color.r = colors[k][0];
        marker.color.g = colors[k][1];
        marker.color.b = colors[k][2];
        marker.pose.position.z = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.ns = "cross_points";
        crossPoint crsPt = crsPts[k*2];
        if(!crsPt.safe){
            marker.color.r =1.0;
            marker.color.g = 0;
            marker.color.b = 0;
            // std::cout<<"crsPt["<<k*2<<"]:("<<crsPts[k*2].x<<","<<crsPts[k*2].y<<","<<crsPts[k*2].t<<","<<crsPts[k*2].vx<<","<<crsPts[k*2].vy<<std::endl;
        }
        //危険, 安全障害物ともに同じように表示している
        marker.pose.position.x = crsPt.y;
        marker.pose.position.y = -crsPt.x;
        //add Array
        marker.id = count;
        markerArray.markers[count++] = marker;
        crsPt = crsPts[k*2+1];
        if(!crsPt.safe){
            marker.color.r =1.0;
            marker.color.g = 0;
            marker.color.b = 0;
            // std::cout<<"crsPt["<<k*2+1<<"]:("<<crsPts[k*2+1].x<<","<<crsPts[k*2+1].y<<","<<crsPts[k*2+1].t<<","<<crsPts[k*2+1].vx<<","<<crsPts[k*2+1].vy<<std::endl;
        }
        //危険, 安全障害物ともに同じように表示している
        marker.pose.position.x = crsPt.y;
        marker.pose.position.y = -crsPt.x;
        //add Array
        marker.id = count;
        markerArray.markers[count++] = marker;
    }
    marker.ns = "robot";
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    //ロボットの命令値の出力を表示
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.2;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 255;
    marker.color.b = 122;
    double tagyaw = (angle_min + num * angle_div)*M_PI/180;
    // ROS_INFO("yaw:%f",yaw);
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(tagyaw-M_PI_2);
    marker.id = count;
    markerArray.markers[count++] = marker;
    //目標矢印（テキスト）
    marker.ns = "text";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "v,ang:("+ std::to_string(v) +","+ std::to_string(tagyaw*180/M_PI)+")" ;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;
    marker.pose.position.z = 0.5;
    marker.id = count;
    markerArray.markers[count++] = marker;
    //ゴール
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.ns = "goal";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.pose.position.x = relationOdom.pose.pose.position.x;
    marker.pose.position.y = relationOdom.pose.pose.position.y;
    marker.pose.position.z = relationOdom.pose.pose.position.z;
    marker.id = count;
    markerArray.markers[count++] = marker;
    markerArray.markers.resize(count);
    if(markerArray.markers.size()){
        pubDebBagOutput.publish( markerArray );
    }
}
// クロスコストマップを表示する
void trackingAvoidance::showCostMap(){
    //コストマップ生成のためのパラメータからコストマップをグラデーションで表現する
    //パラメータを取得
    //process
    //マップの全てのセルに対する評価値を取得する
    //process
    //マップを描画する
    //process
    //publishする
}
//障害物１つに対するx,y座標の交差位置を相対速度を使用して算出(交差位置をデバッグ返す) obstacle avoidance method
void trackingAvoidance::getCrossPoints(crossPoint& crsPt_x0, crossPoint& crsPt_y0, int& indexRef,geometry_msgs::Point& gpRef, geometry_msgs::Twist& twistRef, float& cur_vel, float& cur_ang, float& cmd_dV, float& cmd_ang){
	// struct crossPoint{
	// 	float x;//交差位置x
	// 	float y;//交差位置y
	// 	float dis;//交差位置とロボットの距離
	// 	float t;//交差時の時間
	// 	int index;//障害物番号
	// };	
	//ロボット速度 
	float Vrx = cur_vel * cos(cur_angVel*delta_time+M_PI_2);//*delta_time);
	float Vry = cur_vel * sin(cur_angVel*delta_time+M_PI_2);//*delta_time);
	// 目標速度(探査対象)
	//cmd_dAng は水平右をx軸, 正面をy軸とする
	float dVrx_c = (cmd_dV+cur_vel) * cos(cmd_ang);//*delta_time);//(cmd_dV+cur_vel) * cos(cmd_ang);
	float dVry_c = (cmd_dV+cur_vel) * sin(cmd_ang);//*delta_time);//(cmd_dV+cur_vel) * sin(cmd_ang);
	//障害物
	// 位置
	float Xox = gpRef.x;
	float Xoy = gpRef.y;
	double delta_r,delta_p,delta_yaw;
	tf::Quaternion quat_rot;
	quaternionMsgToTF(deltaRobotOdom.pose.pose.orientation, quat_rot);
	tf::Matrix3x3(quat_rot).getRPY(delta_r,delta_p,delta_yaw);
	//回転速度変化
	// double omega = deltaRobotOdom.twist.twist.angular.z;
	double omega = cur_angVel;
	double pos_x = gpRef.x;
	double pos_y = gpRef.y;
	double vel_x = twistRef.linear.x;
	double vel_y = twistRef.linear.y;
	double vr_x = Vrx;
	double vr_y = Vry;
	double delta_pos_x = pos_x + cos(delta_yaw)*(vel_x*delta_time - pos_x + vr_x*delta_time) + sin(delta_yaw)*(vel_y*delta_time - pos_y + vr_y*delta_time);
	double delta_pos_y = pos_y - sin(delta_yaw)*(vel_x*delta_time - pos_x + vr_x*delta_time) + cos(delta_yaw)*(vel_y*delta_time - pos_y + vr_y*delta_time);
	// 速度
	float Vox = delta_pos_x/delta_time;
	float Voy = delta_pos_y/delta_time;
	//回転
	tf::Quaternion quat;
	double r_cur,p_cur,y_cur;
	quaternionMsgToTF(robotOdom.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(r_cur,p_cur,y_cur);
	double r_pre,p_pre,y_pre;
	quaternionMsgToTF(robotOdom.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(r_pre,p_pre,y_pre);
	// double delta_theta = y_cur - y_pre;
	double delta_theta = cur_angVel*delta_theta;
	double V = std::sqrt(std::pow(Vox,2.0)+std::pow(Voy,2.0));
	double theta = std::atan2(Voy,Vox);
	double theta_rot = theta - delta_theta;
	Vox = V*cos(theta_rot);
	Voy = V*sin(theta_rot);
	// 番号
	int index = indexRef;
	//交差位置
	float Vcx = Vox - dVrx_c;
	float Vcy = Voy - dVry_c;
	crsPt_x0.vx = Vcx;
	crsPt_x0.vy = Vcy;
	crsPt_x0.safe = false;
	// 場合分け
	//相対速度ゼロ
	if(Vcx == 0&& Vcy ==0){
		crsPt_x0.safe = true;
	}
	crsPt_y0 = crsPt_x0;
	//障害物が離れていっているとき
	// 傾き
	float angle = atan2(Vcy,Vcx);
	float frontAngle = M_PI_2;
	//直線の式
	float a = Vcy/Vcx;//X軸の右が正
	float b = Xoy - a*Xox;
	//交差位置 仮
	// x = 0
	// ROS_INFO("%f x + %f ",a,b);
	crsPt_x0.x = 0;
	crsPt_x0.y = b;
	crsPt_x0.dis = crsPt_x0.y;
	crsPt_x0.t = (0-Xox)/Vcx;//
	// y = 0
	crsPt_y0.x = - b /a;	
	crsPt_y0.y = 0;
	crsPt_y0.dis = crsPt_y0.x;
	crsPt_y0.t = (0-Xoy)/Vcy;//
	//--おおよそ直線方向接近物ー＞直進接近物に近似をやめ、2つの交差位置をどちらも採用
	if(checkSafetyObstacle(crsPt_x0.t, angle,Xox,Xoy)){
		crsPt_x0.safe = true;
	}
	else{
		crsPt_x0.safe = false;
	}
	if(checkSafetyObstacle(crsPt_y0.t, angle,Xox,Xoy)){
		crsPt_y0.safe = true;
	}
	else{
		crsPt_y0.safe = false;
	}
}
void trackingAvoidance::crossPointChecker(){
    //入力
    // float debugEncoderVel_r;//ロボットエンコーダ速度(右車輪)
    // float debugEncoderVel_l;//ロボットエンコーダ速度(左車輪)
    // float debugCmd_vel;//ロボット目標速度
    // float debugCmd_angle;//ロボット目標角度
    // int debugIndexRef;//障害物番号
    // geometry_msgs::Point debugGpRef;//クラスタ重心
    // geometry_msgs::Twist debugTwistRef;//障害物速度
    // float debguObstacleRadius;//障害物半径
    // float debugRobotRadius;//ロボット半径
    //エンコーダ速度セット
    // robotEncoder.vel.r = debugEncoderVel_r;
    // robotEncoder.vel.l = debugEncoderVel_l;
    //座標変換 local to rviz
    // debugGpRef.x = 
    //交差位置を取得
    int num=0;
    //実際の環境下では相対ベクトルが得られるため
    geometry_msgs::Twist relation_vel = debugTwistRef;
    // ROS_INFO("Origin: Vo(%f,%f)",debugTwistRef.linear.x,debugTwistRef.linear.y);
    relation_vel.linear.x -= debugCur_vel * cos(debugCur_angle_steer);
    relation_vel.linear.y -= debugCur_vel * sin(debugCur_angle_steer);
    // ROS_INFO("Relation: Vo(%f,%f)",relation_vel.linear.x,relation_vel.linear.y);
             std::vector<crossPoint> crsPtTemp;
    crsPtTemp.resize(2);
    getCrossPoints(crsPtTemp[0],crsPtTemp[1], debugIndexRef, debugGpRef,relation_vel,debugCur_vel,debugCur_angle_steer, debugCmd_vel,debugCmd_angle);
    //マーカーセット
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    int k = 0;
    markerArray.markers.resize(100);
    marker.scale.x = (debugObstacleRadius+debugRobotRadius)*2;
    marker.scale.y = (debugObstacleRadius+debugRobotRadius)*2;
    marker.scale.z = 0.1;
    //position
    //定義済みの交差位置構造体から取得
    crossPoint crsPt = crsPtTemp[0];
    // ROS_INFO("t_cross:%f",crsPt.t);
    //危険, 安全障害物ともに同じように表示している
    marker.pose.position.x = crsPt.y;
    marker.pose.position.y = -crsPt.x;
    marker.pose.position.z = 0;
    marker.color.a = 1.0;
    marker.color.r = colors[k][0];
    marker.color.g = colors[k][1];
    marker.color.b = colors[k][2];
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.id = k;
    markerArray.markers[k++] = marker;
    //text crossPoint
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.4;
    marker.pose.position.z += 1.0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "xy:("+ std::to_string(crsPt.x) +","+ std::to_string(crsPt.y)+") \n t:"+std::to_string(crsPt.t)+ "dis:"+std::to_string(crsPt.dis);
    marker.id = k;
    markerArray.markers[k++] = marker;
    crsPt = crsPtTemp[1];
    // ROS_INFO("t_cross:%f",crsPt.t);
    //危険, 安全障害物ともに同じように表示している
    marker.scale.x = (debugObstacleRadius+debugRobotRadius)*2;
    marker.scale.y = (debugObstacleRadius+debugRobotRadius)*2;
    marker.scale.z = 0.1;
    marker.pose.position.x = crsPt.y;
    marker.pose.position.y = -crsPt.x;
    marker.pose.position.z = 0;
    marker.color.a = 1.0;
    marker.color.r = colors[k][0];
    marker.color.g = colors[k][1];
    marker.color.b = colors[k][2];
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.id = k;
    markerArray.markers[k++] = marker;
    //text crossPoint
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.4;
    marker.pose.position.z += 1.0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "xy:("+ std::to_string(crsPt.x) +","+ std::to_string(crsPt.y)+") \n t:"+std::to_string(crsPt.t)+ "dis:"+std::to_string(crsPt.dis);
    marker.id = k;
    markerArray.markers[k++] = marker;
    //障害物位置と速度ベクトル
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = debugObstacleRadius*2+abs(debugTwistRef.linear.y);
    marker.scale.y = debugObstacleRadius*2+abs(-debugTwistRef.linear.x);
    marker.scale.z = 0.1;
    // local -> rviz 
    marker.pose.position.x = debugGpRef.y;
    marker.pose.position.y = -debugGpRef.x;
    marker.pose.position.z = debugGpRef.z;
    //angle
    double yaw = std::atan2(-debugTwistRef.linear.x, debugTwistRef.linear.y);
    //culc Quaternion
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    marker.color.a = 1.0;
    marker.color.r = colors[k][0];
    marker.color.g = colors[k][1];
    marker.color.b = colors[k][2];
    marker.id = k;
    markerArray.markers[k++] = marker;
    //text ObstaclePoint
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;
    marker.pose.position.z += 2.0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "xy:("+ std::to_string(debugGpRef.x) +","+ std::to_string(debugGpRef.y)+")"+ "\n"
                    + "v,ang:("+ std::to_string(debugTwistRef.linear.x) +","+ std::to_string(debugTwistRef.linear.y)+")"+ "\n"
                    + "state:" + ( (crsPt.safe == true) ? "SAFE" : "WARNING");
    marker.id = k;
    markerArray.markers[k++] = marker;
    //ロボットの目標速度と目標角度
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = debugRobotRadius*2;
    marker.scale.y = debugRobotRadius*2;
    marker.scale.z = 0.1;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    //angle
    // local -> rviz 
    yaw = debugCmd_angle - M_PI_2;
    //culc Quaternion
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    marker.color.a = 1.0;
    marker.color.r = colors[k][0];
    marker.color.g = colors[k][1];
    marker.color.b = colors[k][2];
    marker.id = k;
    markerArray.markers[k++] = marker;
    //text RobotPoint
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;
    marker.pose.position.z += 0.5;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "v,ang:("+ std::to_string(debugCmd_vel) +","+ std::to_string(debugCmd_angle)+")";
    marker.id = k;
    markerArray.markers[k++] = marker;
    markerArray.markers.resize(k);
    pubDebCross.publish( markerArray );
}
//ヒストグラムの出力結果を視覚的に表示する
void trackingAvoidance::histgramChecker(){
    //ヒストグラム算出
    //パラメータ設定
    set_histgram_param(debugMinAngle,debugMaxAngle,debugDivAngle);
	set_dis_threshold(debugThresholdDistance);
	set_eta(debugEtaG, debugEtaCurAngle, debugEtaPrevAngle);
    //ヒストグラムにデータを追加
    float angleTemp;
    float disTemp;
    //obstacle1
    angleTemp = atan2(debugObstacleY1, debugObstacleX1)*180/M_PI;
    disTemp = sqrt(debugObstacleY1*debugObstacleY1 + debugObstacleX1*debugObstacleX1);
    add_histgram_dis(angleTemp, disTemp);
    //obstacle2
    angleTemp = atan2(debugObstacleY2, debugObstacleX2)*180/M_PI;
    disTemp = sqrt(debugObstacleY2*debugObstacleY2 + debugObstacleX2*debugObstacleX2);
    add_histgram_dis(angleTemp, disTemp);
    //obstacle3
    angleTemp = atan2(debugObstacleY3, debugObstacleX3)*180/M_PI;
    disTemp = sqrt(debugObstacleY3*debugObstacleY3 + debugObstacleX3*debugObstacleX3);
    add_histgram_dis(angleTemp, disTemp);
    //バイナリヒストグラムの作成 
    create_binary_histgram(debugRobotRadius, debugMarginRadius);
    //距離ヒストグラム
    std::vector<double> histgram_dis;
    get_histgram_dis(histgram_dis);
    //バイナリヒストグラム
    std::vector<bool> histgram_bi;
    get_histgram_bi(histgram_bi);
    // ROS_INFO_STREAM("histgram_bi:[");
    // for(int i=0;i<histgram_bi.size();i++){
    //     std::cout<<(angle_min + i * angle_div)<<": "<<histgram_bi[i]<<"\n";
    // }
    // ROS_INFO_STREAM("]histgram_end");
    //マーカーセット
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    int k = 0;
    markerArray.markers.resize((int)histgram_dis.size()*2);
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    //距離ヒストグラム
    for(int i=0; i<histgram_dis.size();i++){
        double l = histgram_dis[i];
        double ang = (debugMinAngle + i * debugDivAngle)*M_PI/180;//rad
        marker.color.a = 1.0;
        marker.color.r = 0;
        marker.color.g = 255;
        marker.color.b = 0;
        if(l ==-1){
            l =4;
            marker.color.r = 0;
            marker.color.g = 255;
            marker.color.b = 255;
        }
        //x, y 座標を算出
        double x = l*cos(ang);
        double y = l*sin(ang);
        marker.pose.position.x = y;
        marker.pose.position.y = -x;
        marker.pose.position.z = 0;
        marker.id = k;
        markerArray.markers[k++] = marker;
    }    
    //２値ヒストグラム
    for(int i=0; i<histgram_bi.size();i++){
        double l = debugThresholdDistance;
        double ang = (debugMinAngle + i * debugDivAngle)*M_PI/180;//rad
        marker.color.a = 1.0;
        marker.color.r = 255;
        marker.color.g = 255;
        marker.color.b = 255;
        if(!histgram_bi[i]){
            l = 1.0;
            marker.color.g = 0;
            marker.color.b = 0;
        }
        //x, y 座標を算出
        double x = l*cos(ang);
        double y = l*sin(ang);
        marker.pose.position.x = y;
        marker.pose.position.y = -x;
        // marker.pose.position.z = 1.0;
        marker.id = k;
        markerArray.markers[k++] = marker;
    }
    markerArray.markers.resize(k);
    pubDebHst.publish( markerArray );
}
//ヒストグラムの出力結果を視覚的に表示する
void trackingAvoidance::histgramCheckerEx(){
     //距離ヒストグラム
    std::vector<double> histgram_dis;
    get_histgram_dis(histgram_dis);
    //バイナリヒストグラム
    std::vector<bool> histgram_bi;
    get_histgram_bi(histgram_bi);
    // ROS_INFO_STREAM("histgram_bi:[");
    // for(int i=0;i<histgram_bi.size();i++){
    //     std::cout<<(angle_min + i * angle_div)<<": "<<histgram_bi[i]<<"\n";
    // }
    // ROS_INFO_STREAM("]histgram_end");
    //マーカーセット
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    int k = 0;
    markerArray.markers.resize((int)histgram_dis.size()*2);
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    //距離ヒストグラム
    for(int i=0; i<histgram_dis.size();i++){
        double l = histgram_dis[i];
        double ang = (debugMinAngle + i * debugDivAngle)*M_PI/180;//rad
        marker.color.a = 1.0;
        marker.color.r = 0;
        marker.color.g = 255;
        marker.color.b = 0;
        if(l ==-1){
            l =4;
            marker.color.r = 0;
            marker.color.g = 255;
            marker.color.b = 255;
        }
        //x, y 座標を算出
        double x = l*cos(ang);
        double y = l*sin(ang);
        marker.pose.position.x = y;
        marker.pose.position.y = -x;
        marker.pose.position.z = 0;
        marker.id = k;
        markerArray.markers[k++] = marker;
    }    
    //２値ヒストグラム
    for(int i=0; i<histgram_bi.size();i++){
        double l = dis_th;
        double ang = (debugMinAngle + i * debugDivAngle)*M_PI/180;//rad
        marker.color.a = 1.0;
        marker.color.r = 255;
        marker.color.g = 255;
        marker.color.b = 255;
        if(!histgram_bi[i]){
            l = 1.0;
            marker.color.g = 0;
            marker.color.b = 0;
        }
        //x, y 座標を算出
        double x = l*cos(ang);
        double y = l*sin(ang);
        marker.pose.position.x = y;
        marker.pose.position.y = -x;
        // marker.pose.position.z = 1.0;
        marker.id = k;
        markerArray.markers[k++] = marker;
    }
    markerArray.markers.resize(k);
    pubDebHst.publish( markerArray );
}
//出力（命令速度, 角度）を視覚的に表示する
void trackingAvoidance::outputVFHChecker(){
    //ヒストグラムチェッカーのパラメータを共有
    //ヒストグラム算出
    //パラメータ設定
    set_histgram_param(debugMinAngle,debugMaxAngle,debugDivAngle);
	set_dis_threshold(debugThresholdDistance);
	set_eta(debugEtaG, debugEtaCurAngle, debugEtaPrevAngle);
    //ヒストグラムにデータを追加
    float angleTemp;
    float disTemp;
    //obstacle1
    angleTemp = atan2(debugObstacleY1, debugObstacleX1)*180/M_PI;
    disTemp = sqrt(debugObstacleY1*debugObstacleY1 + debugObstacleX1*debugObstacleX1);
    add_histgram_dis(angleTemp, disTemp);
    //obstacle2
    angleTemp = atan2(debugObstacleY2, debugObstacleX2)*180/M_PI;
    disTemp = sqrt(debugObstacleY2*debugObstacleY2 + debugObstacleX2*debugObstacleX2);
    add_histgram_dis(angleTemp, disTemp);
    //obstacle3
    angleTemp = atan2(debugObstacleY3, debugObstacleX3)*180/M_PI;
    disTemp = sqrt(debugObstacleY3*debugObstacleY3 + debugObstacleX3*debugObstacleX3);
    add_histgram_dis(angleTemp, disTemp);
    //バイナリヒストグラムの作成 
    create_binary_histgram(debugRobotRadius, debugMarginRadius);
    //距離ヒストグラム
    std::vector<double> histgram_dis;
    get_histgram_dis(histgram_dis);
    //バイナリヒストグラム
    std::vector<bool> histgram_bi;
    get_histgram_bi(histgram_bi);
    double min_cost = 1;
    int min_num = -1;
    double max_cost = 0;
    int max_num = -1;
    //weight正規化
    double sum_weight = debugKg + debugKcurAngle + debugKprevAngle;
    debugKg/=sum_weight;
    debugKcurAngle/=sum_weight;
    debugKprevAngle/=sum_weight;
    // ROS_INFO("goal,angle,angleVel: %f, %f, %f", debugKg, debugKcurAngle, debugKprevAngle);
    //コスト算出
    debugGoalAng = atan2(debugGoalPosY,debugGoalPosX)*180/M_PI;
    for(int i=0; i<histgram_bi.size();i++){
        if(!histgram_bi[i]){
            continue;
        }
        double l = histgram_dis[i];
        double ang = (debugMinAngle + i * debugDivAngle);//deg

        double difAng = debugGoalAng - ang;
        if(debugGoalAng < -90){
            debugGoalAng+=360;
            difAng = debugGoalAng - ang;
        }
        if (debugGoalAng < debugMinAngle )
        {
            debugGoalAng = debugMinAngle;
        }
        else if (debugGoalAng > debugMaxAngle)
        {
            debugGoalAng = debugMaxAngle;
        }
        double goalCost = cost_goal_angle_deg(ang, debugGoalAng);
        double angCost = cost_current_angle_deg(ang, debugCurAng);
        double prevAngCost = cost_prev_select_angle_deg(ang, debugPrevTagAng);
        double cost = debugKg * goalCost + debugKcurAngle * angCost + debugKprevAngle * prevAngCost;
        if(min_cost > cost){
            min_cost = cost;
            min_num = i;
        }
        if(max_cost < cost){
            max_cost = cost;
            max_num = i;
        }
    }
    //マーカーセット
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    //目標矢印
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.2;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 255;
    marker.color.b = 122;
    int k = 0;
    markerArray.markers.resize((int)histgram_dis.size()*3);
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    double yaw = (debugMinAngle + min_num * debugDivAngle)*M_PI/180;
    // ROS_INFO("yaw:%f",yaw);
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw-M_PI_2);
    marker.id = k;
    markerArray.markers[k++] = marker;
    //目標矢印（テキスト）
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "v,ang:("+ std::to_string(debugCmd_vel) +","+ std::to_string(yaw*180/M_PI)+")";
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;
    marker.pose.position.z = 0.5;
    marker.id = k;
    markerArray.markers[k++] = marker;
    //コストでグラデーション
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    for(int i=0; i<histgram_bi.size();i++){
        if(!histgram_bi[i]){
            continue;
        }
        double l = 1.5;
        double ang = (debugMinAngle + i * debugDivAngle);//deg

        double difAng = debugGoalAng - ang;
        if(difAng < -180){
            debugGoalAng+=360;
        }
        if (debugGoalAng < debugMinAngle )
        {
            debugGoalAng = debugMinAngle;
        }
        else if (debugGoalAng > debugMaxAngle)
        {
            debugGoalAng = debugMaxAngle;
        }
        float goalCost = cost_goal_angle_deg(ang, debugGoalAng);
        float angCost = cost_current_angle_deg(ang, debugCurAng);
        float prevAngCost = cost_prev_select_angle_deg(ang, debugPrevTagAng);
        double cost = debugKg * goalCost + debugKcurAngle * angCost + debugKprevAngle * prevAngCost;
        marker.color.r = ((int)(goalCost/max_cost*255))*debugKg;
        marker.color.g = ((int)(angCost/max_cost*255))*debugKcurAngle;
        marker.color.b = ((int)(prevAngCost/max_cost*255))*debugKprevAngle;
        // std::cout<< i<<": " <<marker.color.r<<", "<<marker.color.g<<", "<<marker.color.b<<std::endl;
        //x, y 座標を算出
        double x = l*cos(ang/180*M_PI);
        double y = l*sin(ang/180*M_PI);
        marker.pose.position.x = y;
        marker.pose.position.y = -x;
        marker.pose.position.z = 0;
        marker.id = k;
        markerArray.markers[k++] = marker;
        double cost2 = cost/max_cost;
        marker.pose.position.z = cost2*5;
        marker.id = k;
        markerArray.markers[k++] = marker;
    }
    //目標位置
    //マーカー
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 255;
    marker.color.g = 255;
    marker.color.b = 0;
    marker.pose.position.x = debugGoalPosY;
    marker.pose.position.y = -debugGoalPosX;
    marker.pose.position.z = 0;
    marker.id = k;
    markerArray.markers[k++] = marker;
    //参照矢印
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.4;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 255;
    marker.color.g = 255;
    marker.color.b = 122;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    yaw = debugGoalAng*M_PI/180;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw-M_PI_2);
    marker.id = k;
    markerArray.markers[k++] = marker;
    //参照矢印（テキスト）
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "goalAng:("+ std::to_string(debugGoalAng)+")";
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;
    marker.pose.position.z = -0.5;
    marker.id = k;
    markerArray.markers[k++] = marker;
    markerArray.markers.resize(k);
    pubDebOutput.publish( markerArray );
}
void trackingAvoidance::outputCrossPointVFHChecker(){
    //ヒストグラム算出
    //パラメータ設定 outputVFHパラメータを使用
    set_histgram_param(debugMinAngle,debugMaxAngle,debugDivAngle);
	set_dis_threshold(debugThresholdDistance);
	set_eta(debugEtaG, debugEtaCurAngle, debugEtaPrevAngle);
    //ヒストグラムにデータを追加
    float angleTemp;
    float disTemp;
    //障害物1
    if((debugTwist1.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize1 > debugObstacleSizeThreshold){
        angleTemp = atan2(debugObstacleY1, debugObstacleX1)*180/M_PI;
        disTemp = sqrt(debugObstacleY1*debugObstacleY1 + debugObstacleX1*debugObstacleX1);
        add_histgram_dis(angleTemp, disTemp);
    }
    if((debugTwist2.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize2 > debugObstacleSizeThreshold){
        angleTemp = atan2(debugObstacleY2, debugObstacleX2)*180/M_PI;
        disTemp = sqrt(debugObstacleY2*debugObstacleY2 + debugObstacleX2*debugObstacleX2);
        add_histgram_dis(angleTemp, disTemp); 
    }
    if((debugTwist3.linear.x ==0 && debugTwist3.linear.y ==0) || debugObstacleSize3 > debugObstacleSizeThreshold){
        angleTemp = atan2(debugObstacleY3, debugObstacleX3)*180/M_PI;
        disTemp = sqrt(debugObstacleY3*debugObstacleY3 + debugObstacleX3*debugObstacleX3);
        add_histgram_dis(angleTemp, disTemp);
    }
    //バイナリヒストグラムの作成 
    create_binary_histgram(debugRobotRadius, debugMarginRadius);
    //距離ヒストグラム
    std::vector<double> histgram_dis;
    get_histgram_dis(histgram_dis);
    //バイナリヒストグラム
    std::vector<bool> histgram_bi;
    get_histgram_bi(histgram_bi);
    //マーカーセット
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    int k = 0;
    markerArray.markers.resize((int)histgram_dis.size()*10);
    // コストでグラデーション
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    //ゴール角度
    debugGoalAng = atan2(debugGoalPosY,debugGoalPosX)*180/M_PI;
    //weight正規化
    double sum_weight = debugKg + debugKcurAngle + debugKprevAngle + debugKcp;
    debugKg/=sum_weight;
    debugKcurAngle/=sum_weight;
    debugKprevAngle/=sum_weight;
    debugKcp/=sum_weight;
    //コスト算出
    double min_cost = 1;
    int min_num = -1;
    double max_cost = 0;
    int max_num = -1;
    // ROS_INFO("cost searching:n=%d", (int)histgram_bi.size());
    //コスト最小交差位置を取得
    std::vector<crossPoint> debugCrsPtsMin;
    for(int i=0; i<histgram_bi.size();i++){
        if(!histgram_bi[i]){
            continue;
        }
        double l = histgram_dis[i];
        double ang = (debugMinAngle + i * debugDivAngle);//deg
        float cmd_ang = ang*M_PI/180;
        ////障害物との交差位置を算出(障害物の重心��置のみを使用)
        //交差位置を取得
        std::vector<crossPoint> debugCrsPts;
        debugCrsPts.clear();
        debugCrsPts.resize(2*3);
        int ptNum =0;
        int num=0;
        if(!((debugTwist1.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize1 > debugObstacleSizeThreshold)){
            // getCrossPoint(ptNum, debugCrsPts, ptNum, debugGp1,debugTwist1,debugCur_vel,debugCmd_vel,cmd_ang);
            geometry_msgs::Twist relation_vel = debugTwist1;
            relation_vel.linear.x -= debugCur_vel * cos(debugCur_angle_steer);
            relation_vel.linear.y -= debugCur_vel * sin(debugCur_angle_steer);        
            getCrossPoints(debugCrsPts[num], debugCrsPts[num+1], num, debugGp1,relation_vel,debugCur_vel, debugCur_angle_steer,debugCmd_vel,cmd_ang);
            num += 2;
        }
        if(!((debugTwist2.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize2 > debugObstacleSizeThreshold)){
            // getCrossPoint(ptNum, debugCrsPts, ptNum,z debugGp2,debugTwist2,debugCur_vel,debugCmd_vel,cmd_ang);
            geometry_msgs::Twist relation_vel = debugTwist2;
            relation_vel.linear.x -= debugCur_vel * cos(debugCur_angle_steer);
            relation_vel.linear.y -= debugCur_vel * sin(debugCur_angle_steer);        
            getCrossPoints(debugCrsPts[num], debugCrsPts[num+1], num, debugGp2,relation_vel,debugCur_vel, debugCur_angle_steer,debugCmd_vel,cmd_ang);
            num += 2;            
        }
        if(!((debugTwist3.linear.x ==0 && debugTwist3.linear.y ==0) || debugObstacleSize3 > debugObstacleSizeThreshold)){
            // getCrossPoint(ptNum, debugCrsPts, ptNum, debugGp3,debugTwist3,debugCur_vel,debugCmd_vel,cmd_ang);
            geometry_msgs::Twist relation_vel = debugTwist3;
            relation_vel.linear.x -= debugCur_vel * cos(debugCur_angle_steer);
            relation_vel.linear.y -= debugCur_vel * sin(debugCur_angle_steer);        
            getCrossPoints(debugCrsPts[num], debugCrsPts[num+1], num, debugGp3,relation_vel,debugCur_vel, debugCur_angle_steer,debugCmd_vel,cmd_ang);
            num += 2;            
            
        }
        debugCrsPts.resize(num);
        double goalCost = cost_goal_angle_deg(ang, debugGoalAng);
        double angCost = cost_current_angle_deg(ang, debugCurAng);
        double prevAngCost = cost_prev_select_angle_deg(ang, debugPrevTagAng);
        double crossCost = getCrossPointCost(debugCrsPts,debugEtaCp);
        double cost = debugKg * goalCost + debugKcurAngle * angCost + debugKprevAngle * prevAngCost + debugKcp*crossCost;
        if(min_cost > cost){
            min_cost = cost;
            min_num = i;
            debugCrsPtsMin = debugCrsPts;
        }
        if(max_cost < cost){
            max_cost = cost;
            max_num = i;
        }
    }
    // ROS_INFO("be visualable");
    //コストでグラデーション
    for(int i=0; i<histgram_bi.size();i++){
        if(!histgram_bi[i]){
            continue;
        }
        double l = histgram_dis[i];
        double ang = (debugMinAngle + i * debugDivAngle);//deg
        float cmd_ang = ang*M_PI/180;
        ////障害物との交差位置を算出(障害物の重心位置のみを使用)
        //交差位置を取得
        std::vector<crossPoint> debugCrsPts;
        debugCrsPts.clear();
        debugCrsPts.resize(3*2);
        int num=0;
        if(!((debugTwist1.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize1 > debugObstacleSizeThreshold)){
            // getCrossPoint(ptNum, debugCrsPts, ptNum, debugGp1,debugTwist1,debugCur_vel,debugCmd_vel,cmd_ang);
            geometry_msgs::Twist relation_vel = debugTwist1;
            relation_vel.linear.x -= debugCur_vel * cos(debugCur_angle_steer);
            relation_vel.linear.y -= debugCur_vel * sin(debugCur_angle_steer);        
            getCrossPoints(debugCrsPts[num], debugCrsPts[num+1], num, debugGp1,relation_vel,debugCur_vel, debugCur_angle_steer,debugCmd_vel,cmd_ang);
            num += 2;
        }
        if(!((debugTwist2.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize2 > debugObstacleSizeThreshold)){
            // getCrossPoint(ptNum, debugCrsPts, ptNum,z debugGp2,debugTwist2,debugCur_vel,debugCmd_vel,cmd_ang);
            geometry_msgs::Twist relation_vel = debugTwist2;
            relation_vel.linear.x -= debugCur_vel * cos(debugCur_angle_steer);
            relation_vel.linear.y -= debugCur_vel * sin(debugCur_angle_steer);        
            getCrossPoints(debugCrsPts[num], debugCrsPts[num+1], num, debugGp2,relation_vel,debugCur_vel, debugCur_angle_steer,debugCmd_vel,cmd_ang);
            num += 2;            
        }
        if(!((debugTwist3.linear.x ==0 && debugTwist3.linear.y ==0) || debugObstacleSize3 > debugObstacleSizeThreshold)){
            // getCrossPoint(ptNum, debugCrsPts, ptNum, debugGp3,debugTwist3,debugCur_vel,debugCmd_vel,cmd_ang);
            geometry_msgs::Twist relation_vel = debugTwist3;
            relation_vel.linear.x -= debugCur_vel * cos(debugCur_angle_steer);
            relation_vel.linear.y -= debugCur_vel * sin(debugCur_angle_steer);        
            getCrossPoints(debugCrsPts[num], debugCrsPts[num+1], num, debugGp3,relation_vel,debugCur_vel, debugCur_angle_steer,debugCmd_vel,cmd_ang);
            num += 2;            
            
        }
        debugCrsPts.resize(num);
        double goalCost = cost_goal_angle_deg(ang, debugGoalAng);
        double angCost = cost_current_angle_deg(ang, debugCurAng);
        double prevAngCost = cost_prev_select_angle_deg(ang, debugPrevTagAng);
        double crossCost = getCrossPointCost(debugCrsPts, debugEtaCp);
        double cost = debugKg * goalCost + debugKcurAngle * angCost + debugKprevAngle * prevAngCost + debugKcp*crossCost;
        // std::cout<< i<<": " <<debugKg * goalCost<<", "<<debugKcurAngle* angCost <<", "<<debugKprevAngle * prevAngCost<<","<<debugKcp*crossCost<<std::endl;
        //コストでグラデーション
        marker.color.a = 1.0;
        marker.color.r = ((uint8_t)(goalCost/max_cost*255*50))*debugKg;//ゴール角度
        marker.color.g = ((uint8_t)(angCost/max_cost*255*50))*debugKcurAngle;//現在角度
        marker.color.b = ((uint8_t)(crossCost/max_cost*255*50))*debugKcp;//交差位置
        // std::cout<< i<<": " <<marker.color.r<<", "<<marker.color.g<<", "<<marker.color.b<<std::endl;
        //x, y 座標を算出
        if(l ==-1){
            l =4;
        }
        double x = l*cos(ang/180*M_PI);
        double y = l*sin(ang/180*M_PI);
        marker.pose.position.x = y;
        marker.pose.position.y = -x;
        marker.pose.position.z = 0;
        marker.id = k;
        markerArray.markers[k++] = marker;
        double cost2 = (cost-min_cost)/min_cost ;
        if(min_cost<0){
            cost2 =-cost2;
        }
        marker.pose.position.z = cost2*5;//高さ補正
        // std::cout<< i<<": " <<marker.pose.position.x<<", "<<marker.pose.position.y<<", "<<marker.pose.position.z<<std::endl;
        marker.id = k;
        markerArray.markers[k++] = marker;
    }
    //目標位置
    //マーカー
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 255;
    marker.color.g = 255;
    marker.color.b = 0;
    marker.pose.position.x = debugGoalPosY;
    marker.pose.position.y = -debugGoalPosX;
    marker.pose.position.z = 0;
    marker.id = k;
    markerArray.markers[k++] = marker;
    //参照矢印
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.4;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 255;
    marker.color.g = 255;
    marker.color.b = 122;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    double refyaw = debugGoalAng*M_PI/180;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(refyaw-M_PI_2);
    marker.id = k;
    markerArray.markers[k++] = marker;
    //参照矢印（テキスト）
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "goalAng:("+ std::to_string(debugGoalAng)+")";
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;
    marker.pose.position.z = -0.5;
    marker.id = k;
    markerArray.markers[k++] = marker;
    //目標矢印
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.2;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 255;
    marker.color.b = 122;
    double tagyaw = (debugMinAngle + min_num * debugDivAngle)*M_PI/180;
    // ROS_INFO("yaw:%f",yaw);
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(tagyaw-M_PI_2);
    marker.id = k;
    markerArray.markers[k++] = marker;
    //目標矢印（テキスト）
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "v,ang:("+ std::to_string(debugCmd_vel) +","+ std::to_string(tagyaw*180/M_PI)+")\n cost:" + std::to_string(min_cost);
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;
    marker.pose.position.z = 0.5;
    marker.id = k;
    markerArray.markers[k++] = marker;
    //障害物位置と速度ベクトル
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.z = 0.05;
    if(!((debugTwist1.linear.x ==0 && debugTwist1.linear.y ==0) || debugObstacleSize1 > debugObstacleSizeThreshold)){
        marker.scale.x = debugObstacleSize1;
        marker.scale.y = debugObstacleSize1;
        marker.pose.position.x = debugGp1.y;
        marker.pose.position.y = -debugGp1.x;
        marker.pose.position.z = debugGp1.z;
        //angle
        double yaw = std::atan2(-debugTwist1.linear.x, debugTwist1.linear.y);
        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.color.a = 1.0;
        marker.color.r = colors[0][0];
        marker.color.g = colors[0][1];
        marker.color.b = colors[0][2];
        marker.id = k;
        markerArray.markers[k++] = marker;
    }
    if(!((debugTwist2.linear.x ==0 && debugTwist2.linear.y ==0) || debugObstacleSize2 > debugObstacleSizeThreshold)){
        marker.scale.x = debugObstacleSize2;
        marker.scale.y = debugObstacleSize2;
        marker.pose.position.x = debugGp2.y;
        marker.pose.position.y = -debugGp2.x;
        marker.pose.position.z = debugGp2.z;
        //angle
        double yaw = std::atan2(-debugTwist2.linear.x, debugTwist2.linear.y);
        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.color.a = 1.0;
        marker.color.r = colors[1][0];
        marker.color.g = colors[1][1];
        marker.color.b = colors[1][2];
        marker.id = k;
        markerArray.markers[k++] = marker;
    }
    if(!((debugTwist3.linear.x ==0 && debugTwist3.linear.y ==0) || debugObstacleSize3 > debugObstacleSizeThreshold)){
        marker.scale.x = debugObstacleSize3;
        marker.scale.y = debugObstacleSize3;
        marker.pose.position.x = debugGp3.y;
        marker.pose.position.y = -debugGp3.x;
        marker.pose.position.z = debugGp3.z;
        //angle
        double yaw = std::atan2(-debugTwist3.linear.x, debugTwist3.linear.y);
        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.color.a = 1.0;
        marker.color.r = colors[2][0];
        marker.color.g = colors[2][1];
        marker.color.b = colors[2][2];
        marker.id = k;
        markerArray.markers[k++] = marker;
    }
    //交差位置
    for(int n=0; n<debugCrsPtsMin.size();n++){
        marker.type = visualization_msgs::Marker::SPHERE; 
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.pose.position.x = debugCrsPtsMin[n].y;
        marker.pose.position.y = -debugCrsPtsMin[n].x;
        marker.pose.position.z = 0;
        marker.color.a = 1.0;
        marker.color.r = colors[n][0];
        marker.color.g = colors[n][1];
        marker.color.b = colors[n][2];
        marker.id = k;
        markerArray.markers[k++] = marker;
        //text crossPoint
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.3;
        marker.pose.position.z = 2.0+n*0.5;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = "xy:("+ std::to_string(debugCrsPtsMin[n].x) +","+ std::to_string(debugCrsPtsMin[n].y)+") \n t:"+std::to_string(debugCrsPtsMin[n].t) + ", dis:"+std::to_string(debugCrsPtsMin[n].dis);
        marker.id = k;
        markerArray.markers[k++] = marker;
        //text ObstaclePoint
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.3;
        marker.pose.position.z = -1.0-n*0.5;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = std::string("state:") + ( (debugCrsPtsMin[n].safe == true) ? "SAFE" : "WARNING");
        marker.id = k;
        markerArray.markers[k++] = marker;
    }
    markerArray.markers.resize(k);
    pubDebCPVFHOutput.publish( markerArray );
}  
void trackingAvoidance::publish_deltaRobotOdom(){
    //deltaRobotOdom
    pubDebOdom.publish(deltaRobotOdom);
}
//回転による障害物速度の変化量を回転変化前と変化後を比較することで行う
void trackingAvoidance::rotationVelocityChecker(double omega){
    //clstr: 障害物クラスタ（ロボット座標系(時刻t)）
    //rotClstr: 障害物クラスタ（ロボット座標系(時刻t)）
    //平行移動
    //X_para = X - X_base (X ={x,y,th,vx,vy})
    //しかし,今回はX_baseがロボット座標であるため, 
    //X_para = X
    //となる
    //X_baseは基準座標であり, 今回変更したい座標の基準であるため
    //X_base={0,0,M_PI_2,0,0} (ロボット座標系)
    //回転移動
    //X_rot = R(-delta_theta) X_para
    //これにより, ロボット回転による障害物の速度が算出される
    //計算をロボット座標系基準で行う
    //クラスタのコピー
    rotClstr = clstr;
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = clstr.header.stamp;
    marker.lifetime = ros::Duration(0.3);
    marker.action = visualization_msgs::Marker::ADD;
    markerArray.markers.resize((int)clstr.data.size()*5);
    int count = 0;
    for(int k=0;k < clstr.data.size();k++){
        double v_rot_x, v_rot_y;
        double x_para_x = clstr.data[k].gc.x;
        double x_para_y = clstr.data[k].gc.y;
        //double x_para_theta = std::atan2(clstr.data[k].gc.y,clstr.data[k].gc.x)-M_PI_2;
        double delta_r,delta_p,delta_yaw;
        tf::Quaternion quat_rot;
        quaternionMsgToTF(deltaRobotOdom.pose.pose.orientation, quat_rot);
        tf::Matrix3x3(quat_rot).getRPY(delta_r,delta_p,delta_yaw);
        double x_para_theta = delta_yaw;//std::atan2(gpRef.y,gpRef.x)-M_PI_2;
        // double x_para_vx = clstr.twist[k].linear.x;
        // double x_para_vy = clstr.twist[k].linear.y;
        double x_para_vx = clstr.twist[k].twist.linear.x;
        double x_para_vy = clstr.twist[k].twist.linear.y;
        //回転算出
        // debug_trans_rotation_vel(v_rot_x,v_rot_y,x_para_x,x_para_y,x_para_theta,x_para_vx, x_para_vy,omega);
        double pos_x = clstr.data[k].gc.x;
        double pos_y = clstr.data[k].gc.y;
        // double vel_x = clstr.twist[k].linear.x;
        // double vel_y = clstr.twist[k].linear.y;
        double vel_x = clstr.twist[k].twist.linear.x;
        double vel_y = clstr.twist[k].twist.linear.y;
        double vr_x = cur_vel * cos(cur_angVel*delta_time+M_PI_2);//*delta_time);
        double vr_y = cur_vel * sin(cur_angVel*delta_time+M_PI_2);//*delta_time);
        double delta_pos_x = pos_x + cos(delta_yaw)*(vel_x*delta_time - pos_x + vr_x*delta_time) + sin(delta_yaw)*(vel_y*delta_time - pos_y + vr_y*delta_time);
        double delta_pos_y = pos_y - sin(delta_yaw)*(vel_x*delta_time - pos_x + vr_x*delta_time) + cos(delta_yaw)*(vel_y*delta_time - pos_y + vr_y*delta_time);
        // 速度
        float Vox = delta_pos_x/delta_time;
        float Voy = delta_pos_y/delta_time;
        // rotClstr.twist[k].linear.x -= v_rot_x;
        // rotClstr.twist[k].linear.y -= v_rot_y;
        // rotClstr.twist[k].linear.x += v_rot_x;
        // rotClstr.twist[k].linear.y += v_rot_y;
        // rotClstr.twist[k].linear.x = v_rot_x + clstr.twist[k].linear.x;
        // rotClstr.twist[k].linear.y = v_rot_y + clstr.twist[k].linear.y;
        // rotClstr.twist[k].linear.x = Vox;
        // rotClstr.twist[k].linear.y = Voy;
        rotClstr.twist[k].twist.linear.x = Vox;
        rotClstr.twist[k].twist.linear.y = Voy;
        marker.ns = "obstacle_vec_self";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.3;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        // local -> rviz 
        marker.pose.position.x = clstr.data[k].gc.y;
        marker.pose.position.y = -clstr.data[k].gc.x;
        marker.pose.position.z = clstr.data[k].gc.z;
        //angle
        // double yaw = std::atan2(-clstr.twist[k].linear.x, clstr.twist[k].linear.y);
        double yaw = std::atan2(-clstr.twist[k].twist.linear.x, clstr.twist[k].twist.linear.y);
        // if(clstr.twist[k].linear.x==0 && clstr.twist[k].linear.y ==0){
        //     marker.type = visualization_msgs::Marker::SPHERE;
        //     marker.scale.x = 0.3;
        //     marker.scale.y = 0.3;
        //     marker.scale.z = 0.5;
        //     yaw = 0;
        // }
        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.id = count;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        markerArray.markers[count++] = marker;
        marker.ns = "obstacle_difVel_self";
        // std::cout<<k<<","<<x_para_theta*180/M_PI<<":bef,rot:("<<clstr.twist[k].linear.x<<","<<clstr.twist[k].linear.y<<"),("<<rotClstr.twist[k].linear.x<<","<<rotClstr.twist[k].linear.y<<")"<<std::endl;
        // double yawRot = std::atan2(-rotClstr.twist[k].linear.x, rotClstr.twist[k].linear.y);
        double yawRot = std::atan2(-rotClstr.twist[k].twist.linear.x, rotClstr.twist[k].twist.linear.y);
        // std::cout<<k<<","<<x_para_theta*180/M_PI<<":bef,rot:("<<std::atan2(clstr.twist[k].linear.y, clstr.twist[k].linear.x)<<","<<std::atan2(rotClstr.twist[k].linear.y, rotClstr.twist[k].linear.x)<<std::endl;
        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yawRot);
        marker.id = count;
        marker.pose.position.z = clstr.data[k].gc.z+0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0;
        marker.color.b = 0;
        markerArray.markers[count++] = marker;
        //rot
        marker.ns = "obstacle_rotVel_self";
        double yawRotVel = std::atan2(-v_rot_x, v_rot_y);
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yawRotVel);
        marker.id = count;
        marker.pose.position.z = clstr.data[k].gc.z+0.4;
        marker.scale.x = 0.3;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1;
        marker.color.b = 0;
        markerArray.markers[count++] = marker;
        marker.ns = "obstacle_rotVel_text";
        //text rotVel
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1;
        marker.color.b = 1;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.4;
        marker.pose.position.z = clstr.data[k].gc.z+1.0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = "xy:("+ std::to_string(v_rot_x) +","+ std::to_string(v_rot_y)+")" ;
        marker.id = count;
        markerArray.markers[count++] = marker;
    }
    // ROS_INFO("pubDebRotOutput: %d",count );
    markerArray.markers.resize(count);
    if(markerArray.markers.size()){
        pubDebRotOutput.publish( markerArray );
    }
}
void trackingAvoidance::trans_rotation_vel(double& v_rot_x, double& v_rot_y, const double& x_para_x,const double& x_para_y,const double& x_para_vx,const double& x_para_vy){
    //変数作成
    double theta_base = M_PI_2;//正面方向
    double omega_base;
    //tf座標系とヨー方向回転軸は同じため,自己位置推定結果を使用
    omega_base = deltaRobotOdom.twist.twist.angular.z;
    //計算
    v_rot_x = omega_base * (-sin(theta_base)*x_para_x - cos(theta_base)*x_para_y)
            + cos(theta_base)*x_para_vx - sin(theta_base)*x_para_vy;
    v_rot_y = omega_base * (-sin(theta_base)*x_para_x - cos(theta_base)*x_para_y)
            - sin(theta_base)*x_para_vx + cos(theta_base)*x_para_vy;
}
void trackingAvoidance::debug_trans_rotation_vel(double& v_rot_x, double& v_rot_y, const double& x_para_x,const double& x_para_y,const double& x_para_theta,const double& x_para_vx,const double& x_para_vy, const double& omega_base){
    //変数作成
    // double theta_base = M_PI_2;//正面方向がベース
    //計算
    // v_rot_x = omega_base * (-sin(theta_base)*x_para_x - cos(theta_base)*x_para_y)
    //         + cos(theta_base)*x_para_vx - sin(theta_base)*x_para_vy;
    // v_rot_y = omega_base * (-sin(theta_base)*x_para_x - cos(theta_base)*x_para_y)
    //         - sin(theta_base)*x_para_vx + cos(theta_base)*x_para_vy;
    v_rot_x = omega_base * (-sin(x_para_theta)*x_para_x - cos(x_para_theta)*x_para_y)
            + cos(x_para_theta)*x_para_vx - sin(x_para_theta)*x_para_vy;
    v_rot_y = omega_base * (cos(x_para_theta)*x_para_x - sin(x_para_theta)*x_para_y)
            + sin(x_para_theta)*x_para_vx + cos(x_para_theta)*x_para_vy;
}
void trackingAvoidance::display_rotVel(){
    //--sample
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = clstr.header.stamp;
    marker.ns = "my_namespace";
    marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    markerArray.markers.resize((int)clstr.data.size() * 2);
    int count = 0;
    float Vrx = cur_vel * cos(cur_angVel*delta_time+M_PI_2);//*delta_time);
    float Vry = cur_vel * sin(cur_angVel*delta_time+M_PI_2);//*delta_time);
    double delta_r,delta_p,delta_yaw;
    tf::Quaternion quat_rot;
    quaternionMsgToTF(deltaRobotOdom.pose.pose.orientation, quat_rot);
    tf::Matrix3x3(quat_rot).getRPY(delta_r,delta_p,delta_yaw);
    //障害物判断
    // decisionObstacleType();
    float colors[12][3] ={{1.0,0,0},{0,1.0,0},{0,0,1.0},{1.0,1.0,0},{0,1.0,1.0},{1.0,0,1.0},{0.5,1.0,0},{0,0.5,1.0},{0.5,0,1.0},{1.0,0.5,0},{0,1.0,0.5},{1.0,0,0.5}};//色リスト
    for(int k=0; k<clstr.data.size(); k++){
        float pos_x = clstr.data[k].gc.x;
        float pos_y = clstr.data[k].gc.y;
        // double vel_x = clstr.twist[k].linear.x;
        // double vel_y = clstr.twist[k].linear.y;
        double vel_x = clstr.twist[k].twist.linear.x;
        double vel_y = clstr.twist[k].twist.linear.y;
        double vr_x = Vrx;
        double vr_y = Vry;
        double delta_pos_x = pos_x + cos(delta_yaw)*(vel_x*delta_time - pos_x + vr_x*delta_time) + sin(delta_yaw)*(vel_y*delta_time - pos_y + vr_y*delta_time);
        double delta_pos_y = pos_y - sin(delta_yaw)*(vel_x*delta_time - pos_x + vr_x*delta_time) + cos(delta_yaw)*(vel_y*delta_time - pos_y + vr_y*delta_time);
        // 速度
        // float Vox = v_rot_x + Vrx;//twistRef.linear.x + Vrx - v_rot_x;
        // float Voy = v_rot_y + Vry;//twistRef.linear.y + Vry - v_rot_y;
        float Vox = delta_pos_x/delta_time;
        float Voy = delta_pos_y/delta_time;
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.text = "("+std::to_string(Vox) +","+ std::to_string(Voy)+")";
        //position
        marker.pose.position.x = clstr.data[k].gc.y;
        marker.pose.position.y = -clstr.data[k].gc.x;
        marker.pose.position.z = clstr.data[k].gc.z + 1.0;
        //angle
        double yaw = std::atan2(-Vox,Voy);
        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.color.a = 1.0;
        marker.color.r = colors[k][0];
        marker.color.g = colors[k][1];
        marker.color.b = colors[k][2];
        if(Vox == 0 && Voy == 0 ){
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.text = "( static )";
            marker.type = visualization_msgs::Marker::SPHERE;
        }
        else{
            //--arrorw
            marker.type = visualization_msgs::Marker::ARROW;
        }
        //add Array
        marker.id = count;
        markerArray.markers[count++] = marker;
        //--text
        marker.scale.x = 1;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.id = count;
        markerArray.markers[count++] = marker;
    }
    // markerArray.markers.resize(count);
    // ROS_INFO("markerArray.markers.size():%d",(int)markerArray.markers.size());
    if(markerArray.markers.size()){
        pubRotVel.publish( markerArray );
    }
}