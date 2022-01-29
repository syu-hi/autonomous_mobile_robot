#include<autonomous_mobile_robot/trackingAvoidance_simulation.h>
trackingAvoidance::trackingAvoidance()
	:wheel_d(0.155),angle_min(45),angle_max(135), angle_div(1.0)
	,RECEIVED_CLUSTER(false),RECEIVED_GOAL_ODOM(false),RECEIVED_ROBOT_ODOM(false),RECEIVED_LRF_SCAN(false)
    ,SEARCH_ONLY_ANGLE(false),MAX_COST(900000),PROCESS_INIT(true),PROCESS_ONCE(true)
	,max_pot(100),min_pot(-100)
	,mv_obsts_size(0),mv_data_size(0)
    ,RECEIVED_CMD_VEl(false),SET_CONFIG(false){
	ROS_INFO("constracter");
	ROS_INFO("subscriber define");
	// subscriber
	sub1=nhSub1.subscribe("classificationDataEstimateVelocity",1,&trackingAvoidance::cluster_callback,this);
	sub2=nhSub1.subscribe("/mynteye/imu/data_raw",1,&trackingAvoidance::robotIMU_callback,this);
	// sub2=nhSub1.subscribe("zed_node/odom",1,&trackingAvoidance::robotOdom_callback,this);
	sub3=nhSub1.subscribe("goalOdometry",1,&trackingAvoidance::goalOdom_callback,this);
	sub4=nhSub1.subscribe("/encoder",1,&trackingAvoidance::robotEncoder_callback,this);
	// sub5=nhSub1.subscribe("/scan",1,&trackingAvoidance::scan_callback,this);
	sub6=nhSub1.subscribe("laserMapData",1,&trackingAvoidance::scan_callback,this);
    // pub= nhPub.advertise<autonomous_mobile_robot::SensorMapData>("laserMapData", 1);
	sub=nhSub.subscribe("command_velocity",1,&trackingAvoidance::cmd_vel_callback,this);

	ROS_INFO("publisher define");
	// publisher
    pub= nhPub.advertise<geometry_msgs::Twist>("/beego/cmd_vel", 1);
	// デバッグ用
	// pubDebPcl= nhDeb.advertise<sensor_msgs::PointCloud2>("debugEstimatedVelocity", 1);
	pubDebMarkerArray= nhDeb.advertise<visualization_msgs::MarkerArray>("crossPointMarkerArray", 1);
	pubDebCross= nhDeb.advertise<visualization_msgs::MarkerArray>("crossPointCheckerResult", 1);
	pubDebHst = nhDeb.advertise<visualization_msgs::MarkerArray>("histgramChecker", 1);
	pubDebOutput = nhDeb.advertise<visualization_msgs::MarkerArray>("outputChecker", 1);
	pubDebCPVFHOutput = nhDeb.advertise<visualization_msgs::MarkerArray>("outputCPVFHChecker", 1);
	pubDebBagOutput = nhDeb.advertise<visualization_msgs::MarkerArray>("outputBagOutputChecker", 1);
	pubDebRotOutput = nhDeb.advertise<visualization_msgs::MarkerArray>("RotChecker", 1);
	pubDebOdom = nhDeb.advertise<nav_msgs::Odometry>("DeltaOdomChecker", 1);
	pubRotVel =  nhDeb.advertise<visualization_msgs::MarkerArray>("rotVelMarker", 1);
    //pub1 = nhPub_sim.advertise<autonomous_mobile_robot::ClassificationVelocityData>("classificationDataEstimateVelocity", 1);
    // pub2 = nhPub_sim.advertise<nav_msgs::Odometry>("zed_node/odom", 1);
    // pub3 = nhPub_sim.advertise<nav_msgs::Odometry>("goalOdometry", 1);
    // pub4 = nhPub_sim.advertise<beego_control::beego_encoder>("encoder", 1);
    pubDeb1 = nhPubDeb.advertise<visualization_msgs::MarkerArray>("debug_clstr", 1);
    pubDeb2 = nhPubDeb.advertise<visualization_msgs::MarkerArray>("debug_detected_clstr", 1);
    pubDeb3 = nhPubDeb.advertise<visualization_msgs::MarkerArray>("debug_odometries", 1);
    //apf debug
	// debug_image = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC3);
	// mpc_debug_image = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC3);
	pub_img=nh_pub.advertise<sensor_msgs::Image>("pot_image",1);
	pub_pan=nh_pub.advertise<dynamixel_sdk_examples::SetPosition>("set_position",1);
	pub_tilt=nh_pub.advertise<dynamixel_sdk_examples::SetPosition>("set_position",1);
	// launchファイルからパラメータの読み込み
    // read_launch_file
	setLaunchParam();
	//クロスポイントチェッカーデフォルト値入力
	setDefaultCrossPointChecker();
    // simulation
    if(SET_CONFIG){
        createObstacleData();
        createRobotData();
        createGoalOdom(goal_x_sim,goal_y_sim);
    }
	//vfhクラスのパラメータ設定
	set_histgram_param(angle_min,angle_max, angle_div);
	set_dis_threshold(dis_th);
	set_eta(eta_g, eta_curAngle, eta_prevAngle);
	if(rqt_reconfigure){
		//rqt_reconfigure
		f = boost::bind(&trackingAvoidance::configCallback, this, _1, _2);
		server.setCallback(f);
	}
	ROS_INFO("ready");
}
trackingAvoidance::~trackingAvoidance(){
    grid_map.release();
	pot_map.release();
	pot_mapt.release();
	grad_map[0].release();
	grad_map[1].release();
	debug_image.release();
    mpc_debug_image.release();
	ROS_INFO("destructor");
}
void trackingAvoidance::setLaunchParam(){
    ros::NodeHandle n("~");
	n.getParam("trackingAvoidance/SEARCH_ONLY_ANGLE",SEARCH_ONLY_ANGLE);   
	n.getParam("trackingAvoidance/rqt_reconfigure",rqt_reconfigure);    
	//ロボットパラメータ
	n.getParam("trackingAvoidance/WheelD",wheel_d);
    n.getParam("trackingAvoidance/angleMin", angle_min);
    n.getParam("trackingAvoidance/angleMax", angle_max);
    n.getParam("trackingAvoidance/angleDiv", angle_div);
    n.getParam("trackingAvoidance/maxSpeed", max_speed);
    n.getParam("trackingAvoidance/minSpeed", min_speed);
    n.getParam("trackingAvoidance/defaultSpeed", default_speed);
    //ゴール位置
    n.getParam("trackingAvoidance/goalX", goal_x);
    n.getParam("trackingAvoidance/goalY", goal_y);
    goalOdom.pose.pose.position.x = goal_x;
    goalOdom.pose.pose.position.y = goal_y;
    goalOdom.pose.pose.position.z = 0;
    RECEIVED_GOAL_ODOM = true;
    //vfh
    n.getParam("trackingAvoidance/distance_th", dis_th);
	//--k
    n.getParam("trackingAvoidance/robotRadius", robotRadius);
    n.getParam("trackingAvoidance/marginRadius", marginRadius);
	n.getParam("trackingAvoidance/Kcp",k_cp);
    n.getParam("trackingAvoidance/Kg",k_g);
    n.getParam("trackingAvoidance/KcurAngle",k_curAngle);
    n.getParam("trackingAvoidance/KprevAngle",k_prevAngle);
    //--eta
    n.getParam("trackingAvoidance/EtaCp",eta_cp);
    n.getParam("trackingAvoidance/EtaG",eta_g);
    n.getParam("trackingAvoidance/EtaCurAngle",eta_curAngle);
    n.getParam("trackingAvoidance/EtaPrevAngle",eta_prevAngle);
    //safe range
    n.getParam("trackingAvoidance/safeRange",safe_range);
    //探査範囲
    n.getParam("trackingAvoidance/searchRange_vel",dV_range);
    n.getParam("trackingAvoidance/searchDiv_vel",dV_div);
    //デバッグ
    n.getParam("trackingAvoidance/debugType",debugType);
}
//---rqt_reconfigureからの読み込み
void trackingAvoidance::configCallback(autonomous_mobile_robot::trackingAvoidanceConfig &config, uint32_t level) {
	// ROS_INFO("Reconfigure Request: %d %f %f %d", 
	// 	config.windowDivisionDegree, config.windowHeight,
	// 	config.windowWidth,config.windowMinPts
	// 	// config.str_param.c_str(), 
	// 	// config.bool_param?"True":"False", 
	// 	// config.size
	// 	);
    //評価式の重み
    k_cp = config.Kcp;
    k_g = config.Kg;
    k_curAngle = config.KcurAngle;
    k_prevAngle = config.KprevAngle;
    k_vel = config.Kv;
	//コスト関数のパラメータ
    eta_cp = config.EtaCp;
    eta_g = config.EtaG;
    eta_curAngle = config.EtaCurAngle;
    eta_prevAngle = config.EtaPrevAngle;
    eta_vel = config.EtaVel;
    safe_range = config.safeRange;
    crossWeightX = config.crossWeightX;
    crossWeightY = config.crossWeightY;
    timeBias = config.timeBias;

	// debug 以下
    debugType = config.debugType;
    //クロスポイントチェッカー
    debugFlag_crossPointChecker = config.crossPointCheckerFlag;
    // debugEncoderVel_r = config.debugEncoderVel_r;//ロボットエンコーダ速度(右車輪)
    // debugEncoderVel_l = config.debugEncoderVel_l;//ロボットエンコーダ速度(左車輪)
    debugCur_vel = config.debugCur_vel;
    debugCur_angle_steer = config.debugCur_angle*M_PI/180;
    debugCmd_vel = config.debugCmd_vel;//ロボット目標速度
    debugCmd_angle = config.debugCmd_angle*M_PI/180;//ロボット目標角度 deg -> rad
    debugIndexRef = config.debugIndexRef;//障害物番号
    debugGpRef.x = config.debugGpRef_x;//クラスタ重心x
    debugGpRef.y = config.debugGpRef_y;//クラスタ重心y
    debugGpRef.z = config.debugGpRef_z;//クラスタ重心z
    debugTwistRef.linear.x = config.debugTwistRefLinear_x;//障害物速度linear x
    debugTwistRef.linear.y = config.debugTwistRefLinear_y;//障害物速度linear y
    debugTwistRef.linear.z = 0;//config.debugTwistRefLinear_z;//障害物速度linear z
    debugTwistRef.angular.x = 0;//障害物速度anguler x
    debugTwistRef.angular.y = 0;//障害物速度anguler y
    debugTwistRef.angular.z = config.debugTwistRefAngular_theta;//障害物速度anguler z
    debugObstacleRadius = config.debugObstacleRadius;//障害物半径
    debugRobotRadius = config.debugRobotRadius;//ロボット半径
    //checkCrossPoint
    if(debugFlag_crossPointChecker){
        crossPointChecker();
    }
    //ヒストグラムチェッカー
    debugHistgramCheckerFlag = config.debugHistgramCheckerFlag;
    debugObstacleNum = config.debugObstacleNum;
    debugObstacleX1 = config.debugObstacleX1;
    debugObstacleY1 = config.debugObstacleY1;
    debugObstacleSize1 = config.debugObstacleSize1;
    debugObstacleX2 = config.debugObstacleX2;
    debugObstacleY2 = config.debugObstacleY2;
    debugObstacleSize2 = config.debugObstacleSize2;
    debugObstacleX3 = config.debugObstacleX3;
    debugObstacleY3  = config.debugObstacleY3;
    debugObstacleSize3 = config.debugObstacleSize3;
    debugThresholdDistance = config.debugThresholdDistance;
    debugMinAngle = config.debugMinAngle;
    debugMaxAngle = config.debugMaxAngle;
    debugDivAngle = config.debugDivAngle;
    debugEtaG = config.debugEtaG;
    debugEtaCurAngle = config.debugEtaCurAngle;
    debugEtaPrevAngle = config.debugEtaPrevAngle;
    debugMarginRadius = config.debugMarginRadius;
    if(debugHistgramCheckerFlag){
        histgramChecker();
    }
    //出力チェッカー
    debugOutputVFHCheckerFlag = config.debugOutputVFHCheckerFlag;
    debugEtaG = config.debugEtaG;
    debugEtaCurAngle = config.debugEtaCurAngle;
    debugEtaPrevAngle = config.debugEtaPrevAngle;
    debugKg = config.debugKg;
    debugKcurAngle = config.debugKcurAngle;
    debugKprevAngle = config.debugKprevAngle;
    // debugGoalAng = config.debugGoalAng;
    debugGoalPosX = config.debugGoalPosX;
    debugGoalPosY = config.debugGoalPosY;
    debugCurAng = config.debugCurAng;
    debugPrevTagAng = config.debugPrevTagAng;
    debugRotOmega =config.debugRotOmega;
    if(debugOutputVFHCheckerFlag){
       outputVFHChecker();
    }
    debugOutputCPVFHCheckerFlag = config.debugOutputCPVFHCheckerFlag;
    debugKcp = config.debugKcp;
    debugEtaCp = config.debugEtaCp;
    debugObstacleVx1 = config.debugObstacleVx1;
    debugObstacleVy1 = config.debugObstacleVy1;
    debugObstacleVx2 = config.debugObstacleVx2;
    debugObstacleVy2 = config.debugObstacleVy2;
    debugObstacleVx3 = config.debugObstacleVx3;
    debugObstacleVy3 = config.debugObstacleVy3;
    //位置
    debugGp1.x = debugObstacleX1;
    debugGp1.y = debugObstacleY1;
    debugGp1.z = 0;
    debugGp2.x = debugObstacleX2;
    debugGp2.y = debugObstacleY2;
    debugGp2.z = 0;
    debugGp3.x = debugObstacleX3;
    debugGp3.y = debugObstacleY3;
    debugGp3.z = 0;
    //速度
    debugTwist1.linear.x = debugObstacleVx1;
    debugTwist1.linear.y = debugObstacleVy1;
    debugTwist1.linear.z = 0;
    debugTwist2.linear.x = debugObstacleVx2;
    debugTwist2.linear.y = debugObstacleVy2;
    debugTwist2.linear.z = 0;
    debugTwist3.linear.x = debugObstacleVx3;
    debugTwist3.linear.y = debugObstacleVy3;
    debugTwist3.linear.z = 0; 
    //角速度
    debugTwist1.angular.x = 0;
    debugTwist1.angular.y = 0;
    debugTwist1.angular.z = 0;
    debugTwist2.angular.x = 0;
    debugTwist2.angular.y = 0;
    debugTwist2.angular.z = 0;
    debugTwist3.angular.x = 0;
    debugTwist3.angular.y = 0;
    debugTwist3.angular.z = 0;
    //障害物サイズ閾値
    debugObstacleSizeThreshold = config.debugObstacleSizeThreshold;
    if(debugOutputCPVFHCheckerFlag){
       outputCrossPointVFHChecker();
    }
    //交差位置の表示有無
    display_output = config.displayOutput;
	// 回転後障害物速度変化量 debug
    debugRotationVelocityCheckerFlag = config.debugRotationVelocityCheckerFlag;
    //セットフラグ
    SET_CONFIG =true;
    //障害物
    obstacleX1 = config.obstacleX1;
    obstacleY1 = config.obstacleY1;
    obstacleW1 = config.obstacleW1;
    obstacleH1 = config.obstacleH1;
    obstacleVx1 = config.obstacleVx1;
    obstacleVy1 = config.obstacleVy1;
    //ロボット
    //--パラメータ
    wheel_d = config.wheel_2d/2.0;//d 車輪間半径
    sensor_angle_min = config.sensor_angle_min*M_PI/180;
    sensor_angle_max = config.sensor_angle_max*M_PI/180;
    max_speed = config.max_speed;
    //--位置
    robot_init_x = config.robot_init_x;
    robot_init_y = config.robot_init_y;
    robot_init_angle = config.robot_init_angle;
    robot_init_v = config.robot_init_v;
    robot_init_w = config.robot_init_w;
    //ゴール
    goal_x_sim = config.goal_x;
    goal_y_sim = config.goal_y;
}
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
// debug simulation 入出力
bool trackingAvoidance::get_set_config(){
    return SET_CONFIG;
}        
bool trackingAvoidance::get_received_cmd_vel(){
    return RECEIVED_CMD_VEl;
}
void trackingAvoidance::reset_received_cmd_vel(){
    RECEIVED_CMD_VEl = false;
}
void trackingAvoidance::run(){
    //受け取りフラグのリセット
    reset_received_cmd_vel();
    //データ更新
    updateRobotEncoder();
    updateObstacleState();
    updateRobotState();
    //視野角反映
    detectObstacle();
    //出力
    publishObstacleData();
    publishRobotData();
    publishTargetData();
    bool display_output = true;
    if(display_output){
        showOdometries();
        showClusters();
        showDetectedClusters();
    }
	apf();
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
// debug simulation 入出力
//--入力
void trackingAvoidance::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    cmd_vel = *msg;
    RECEIVED_CMD_VEl =true;
}
//--出力
void trackingAvoidance::publishObstacleData(){//クラスタの出力
    pub1.publish(cur_clstr);
}
void trackingAvoidance::publishRobotData(){//ロボットオドメトリを出力
    pub2.publish(cur_robotOdom);
}
void trackingAvoidance::publishTargetData(){//目標位置(ゴール位置)を出力
    pub3.publish(goalOdom_sim);
}
//データ作成
void trackingAvoidance::createObstacleData(){
    //障害物の個数            
    //障害物の位置角度速度
    //障害物の大きさ（半径）
    //障害物のマップセル位置とセルに含まれるデータ数
    //rosヘッダ
    cur_clstr.header.frame_id= "base_link";
    cur_clstr.header.seq = 0;
    cur_clstr.header.stamp = ros::Time::now();
    //マップ実測サイズ
    cur_clstr.width.data = 8.0;
    cur_clstr.height.data = 8.0;
    cur_clstr.res.data = 0.05;
    cur_clstr.widthInt.data = cur_clstr.width.data/cur_clstr.res.data;
    cur_clstr.heightInt.data = cur_clstr.height.data/cur_clstr.res.data;
    // マップの中心位置
    cur_clstr.cp.x = cur_clstr.width.data /2.0;
    cur_clstr.cp.y = cur_clstr.height.data /2.0;
    cur_clstr.cp.z = 0;
    //クラスタデータ
    //リサイズ
    cur_clstr.size.data = 1;
    cur_clstr.data.resize(cur_clstr.size.data);
    cur_clstr.twist.resize(cur_clstr.size.data);
    //ロボット,グローバル座標系で作成
    //クラスタ1
    // ROS_INFO("set cur_clstr 1");
    cur_clstr.data[0].gc.x = obstacleX1;
    cur_clstr.data[0].gc.y = obstacleY1;
    cur_clstr.data[0].gc.z = 0;
    cur_clstr.twist[0].twist.linear.x = obstacleVx1;
    cur_clstr.twist[0].twist.linear.y = obstacleVy1;
    cur_clstr.twist[0].twist.linear.z = 0;
    double W0 = obstacleX1 - obstacleW1/2;
    double H0 = obstacleY1 - obstacleH1/2;
    double W1 = obstacleX1 + obstacleW1/2;
    double H1 = obstacleY1 + obstacleH1/2;
    cur_clstr.data[0].size.data = (int)(11*11);
    cur_clstr.data[0].pt.resize(cur_clstr.data[0].size.data);
    int count0=0;
    for(double h =H0; h <= H1; h+=obstacleH1/10.0){
        for(double w =W0; w<= W1; w+=obstacleW1/10.0){
            cur_clstr.data[0].pt[count0].x = w; 
            cur_clstr.data[0].pt[count0].y = h; 
            cur_clstr.data[0].pt[count0].z = 0;
            count0++;
        }
    }
    cur_clstr.data[0].pt.resize(count0);
    //クラスタ2なし
}
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
// debug 
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
// real experiment vfh+
void trackingAvoidance::set_histgram_param(float& angMin, float& angMax, float& angDiv){
    angle_min = angMin;
    angle_max = angMax;
    angle_div = angDiv;
    angle_center = (angle_min+angle_max)/2.0;
    clear_histgram_dis();
    resize_histgram_dis( (int)((angle_max - angle_min)/angle_div) );
}
void trackingAvoidance::set_dis_threshold(float& data){
    dis_threshold = data;    
}
void trackingAvoidance::set_eta(float& goal, float& theta, float& omega){
        eta_goal = goal;
        eta_curAngle = theta;
        eta_prevAngle = omega;
}
void trackingAvoidance::set_k(float& goal, float& theta, float& omega){
    k_goal = goal;
    k_curAngle = theta;
    k_prevAngle = omega;
}
void trackingAvoidance::get_histgram_dis(std::vector<double>& data){
    data = hst_dis;
}
void trackingAvoidance::get_histgram_bi(std::vector<bool>& data){
    data = hst_bi;
}
double trackingAvoidance::get_min_cost(){
    return min_cost;
}
float trackingAvoidance::get_selected_angle(){
    return selected_angle;
}
//transform
int trackingAvoidance::transform_angle_RobotToNum(float& angle){
    return ( (int)((angle - angle_min)/angle_div) );
}
float trackingAvoidance::transform_numToAngle(int& num){
    return ( angle_min + num * angle_div) ;
}
float trackingAvoidance::transform_angleFromCenter(float& angle){
    return (angle - angle_center);
}
//clear
void trackingAvoidance::clear_histgram_dis(){
    hst_dis.clear();
}
//resize
void trackingAvoidance::resize_histgram_dis(int size){
    initDis = -1;
    hst_dis.resize(size, initDis);
}
void trackingAvoidance::resize_histgram_dis(int size, float initValue){
    initDis = initValue;
    hst_dis.resize(size,initDis);
}
//add histgram element
void trackingAvoidance::add_histgram_dis(float& angle, float& dis){
    int num = transform_angle_RobotToNum(angle);
    //番号チェック
    if(num >= 0 && num < (int)(hst_dis.size())){
        //最小値なら格納
        if(hst_dis[num] > dis || hst_dis[num] == initDis){
            hst_dis[num] = dis;
        }
    }
}
// create binary histgram
void trackingAvoidance::create_binary_histgram(float& robotRadius, float& marginRadius){
    hst_bi.clear();
    hst_bi.resize(hst_dis.size(), true);
    for(int k=0; k < hst_dis.size(); k++){
        if(!hst_bi[k]){
            continue;
        }
        if(hst_dis[k] <= dis_threshold && hst_dis[k]!=initDis ){
            hst_bi[k] = false;
            //block width
            double blockAng = atan2((robotRadius+marginRadius), hst_dis[k])*180/M_PI;
            int blockNum = (int)(blockAng/angle_div)+1;
            // ROS_INFO("blockAng, blockNum:(%f,%d)",blockAng,blockNum);
            // ROS_INFO("robotRadius:(%f)",robotRadius);
            for(int i = k-blockNum/2; i <= k+blockNum/2; i++){
                if(i<0 || i>(int)hst_dis.size()){
                    continue;
                }
                hst_bi[i] = false;
            }
        }
    }
}
float trackingAvoidance::min_dif_angle_rad(const float& angle1, const float& angle2){
    float dif_angle_temp = angle1 - angle2;
    float dif_angle_p360 = std::abs(dif_angle_temp + 2*M_PI);
    float dif_angle_n360 = std::abs(dif_angle_temp - 2*M_PI);
    float dif_angle = std::abs(dif_angle_temp);
    float min_angle = dif_angle;
    if(min_angle > dif_angle_p360){
        min_angle = dif_angle_p360;
    }
    if(min_angle > dif_angle_n360){
        min_angle = dif_angle_n360;
    }
    return min_angle;
}
float trackingAvoidance::min_dif_angle_deg(const float& angle1, const float& angle2){
    float dif_angle_temp = angle1 - angle2;
    float dif_angle_p360 = std::abs(dif_angle_temp + 360);
    float dif_angle_n360 = std::abs(dif_angle_temp - 360);
    float dif_angle = std::abs(dif_angle_temp);
    float min_angle = dif_angle;
    if(min_angle > dif_angle_p360){
        min_angle = dif_angle_p360;
    }
    if(min_angle > dif_angle_n360){
        min_angle = dif_angle_n360;
    }
    return min_angle;
}
//cost function 
double trackingAvoidance::angleCostFunction_deg(float& eta, float value){
    return (pow(value/180.0/eta,2.0));
    // return (value/180.0/eta);
}
double trackingAvoidance::cost_goal_angle_deg(const float& angle, float goal_angle){
    float dif_angle = min_dif_angle_deg(angle,goal_angle);
    return angleCostFunction_deg(eta_goal, dif_angle);
}
double trackingAvoidance::cost_current_angle_deg(const float& angle, const float& cur_angle){
    float dif_angle = min_dif_angle_deg(angle, cur_angle);
    return angleCostFunction_deg(eta_curAngle, dif_angle);
}
double trackingAvoidance::cost_prev_select_angle_deg(const float& angle, const float& pre_target_angle){
    float dif_angle = min_dif_angle_deg(angle, pre_target_angle);
    return angleCostFunction_deg(eta_prevAngle, dif_angle);
}
double trackingAvoidance::angleCostFunction_rad(float& eta, float value){
    return (pow(value/eta,2.0));
    // return (value/180.0/eta);
}
double trackingAvoidance::cost_goal_angle_rad(const float& angle, float goal_angle){
    float dif_angle = min_dif_angle_rad(angle,goal_angle);
    return angleCostFunction_rad(eta_goal, dif_angle);
}
double trackingAvoidance::cost_current_angle_rad(const float& angle, const float& cur_angle){
    float dif_angle = min_dif_angle_rad(angle, cur_angle);
    return angleCostFunction_rad(eta_curAngle, dif_angle);
}
double trackingAvoidance::cost_prev_select_angle_rad(const float& angle, const float& pre_target_angle){
    float dif_angle = min_dif_angle_rad(angle, pre_target_angle);
    return angleCostFunction_rad(eta_prevAngle, dif_angle);
}
double trackingAvoidance::getCost_deg(float tagAng, float goalAng, float curAng, float prevTagAng){//to vfh class
    double goal_cost = cost_goal_angle_deg(tagAng, goalAng);
    double ang_cost = cost_current_angle_deg(tagAng, curAng);
    double prevAng_cost = cost_prev_select_angle_deg(tagAng, prevTagAng);
    double cost = k_goal*goal_cost + k_curAngle*ang_cost + k_prevAngle*prevAng_cost;
    return (cost);
}
double trackingAvoidance::getCost_rad(float tagAng, float goalAng, float curAng, float prevTagAng){//to vfh class
    double goal_cost = cost_goal_angle_rad(tagAng, goalAng);
    double ang_cost = cost_current_angle_rad(tagAng, curAng);
    double prevAng_cost = cost_prev_select_angle_rad(tagAng, prevTagAng);
    double cost = k_goal*goal_cost + k_curAngle*ang_cost + k_prevAngle*prevAng_cost;
    return (cost);
}
// real obstacle avoidance and tracking
// subscribe
void trackingAvoidance::cluster_callback(const autonomous_mobile_robot::ClassificationVelocityData::ConstPtr& msg){
	// ROS_INFO("cluster_callback");
    //データをコピー
	clstr = *msg;
	//move manage method
	RECEIVED_CLUSTER = true;
	// manage();
}
void trackingAvoidance::robotIMU_callback(const sensor_msgs::Imu::ConstPtr& msg){
	// ROS_INFO("robotIMU_callback");
	// ROS_INFO("robotOdom_callback");
    //データをコピー
	cur_cameraIMU = *msg;
	// 初期加速度
	// x: 9.38608398438
  	// y: 0.055029296875
  	// z: 2.40693359375
	// imu debug
	if(!PROCESS_INIT){
		// camera odometry culculation
		imu_roll = atan2(cur_cameraIMU.linear_acceleration.y , cur_cameraIMU.linear_acceleration.x);
		imu_pitch = atan2(-cur_cameraIMU.linear_acceleration.z, cur_cameraIMU.linear_acceleration.y*sin(imu_roll) + cur_cameraIMU.linear_acceleration.x*cos(imu_pitch));
		imu_roll =imu_roll + cur_cameraIMU.angular_velocity.x*(ros::Duration(cur_cameraIMU.header.stamp - pre_cameraIMU.header.stamp).toSec());
		imu_pitch =imu_pitch + cur_cameraIMU.angular_velocity.y*(ros::Duration(cur_cameraIMU.header.stamp - pre_cameraIMU.header.stamp).toSec());
		imu_yaw =imu_yaw + cur_cameraIMU.angular_velocity.z*(ros::Duration(cur_cameraIMU.header.stamp - pre_cameraIMU.header.stamp).toSec());
		tf::Quaternion quat=tf::createQuaternionFromRPY(imu_roll,imu_pitch,imu_yaw);
    	quaternionTFToMsg(quat, cur_cameraIMU.orientation);

		// 座標変換 imu->global camera x->z z->x 
		// 加速度計から重力加速度と運動加速度の分離はできない
		// double ag=9.80665;
		// robotOdom.header.stamp=cur_cameraIMU.header.stamp;
		// robotOdom.pose.pose.orientation = cur_cameraIMU.orientation;
		// robotOdom.twist.twist.angular.x = cur_cameraIMU.angular_velocity.x;
		// robotOdom.twist.twist.angular.y = cur_cameraIMU.angular_velocity.y;
		// robotOdom.twist.twist.angular.z = cur_cameraIMU.angular_velocity.z;
		// cur_cameraIMU.linear_acceleration.x = ag*cos(imu_roll);
		// cur_cameraIMU.linear_acceleration.y = ag*sin(imu_pitch);
		// cur_cameraIMU.linear_acceleration.z = ag*cos(imu_pitch);
		// robotOdom.twist.twist.linear.x = (cur_cameraIMU.linear_acceleration.x+pre_cameraIMU.linear_acceleration.x)*(ros::Duration(cur_cameraIMU.header.stamp - pre_cameraIMU.header.stamp).toSec())/2;
		// robotOdom.twist.twist.linear.y = (cur_cameraIMU.linear_acceleration.y+pre_cameraIMU.linear_acceleration.y)*(ros::Duration(cur_cameraIMU.header.stamp - pre_cameraIMU.header.stamp).toSec())/2;
		// robotOdom.twist.twist.linear.z = (cur_cameraIMU.linear_acceleration.z+pre_cameraIMU.linear_acceleration.z)*(ros::Duration(cur_cameraIMU.header.stamp - pre_cameraIMU.header.stamp).toSec())/2;
		// robotOdom.pose.pose.position.x = (robotOdom.twist.twist.linear.x+pre_robotOdom.twist.twist.linear.x)*(ros::Duration(cur_cameraIMU.header.stamp - pre_cameraIMU.header.stamp).toSec())/2;
		// robotOdom.pose.pose.position.y = (robotOdom.twist.twist.linear.y+pre_robotOdom.twist.twist.linear.y)*(ros::Duration(cur_cameraIMU.header.stamp - pre_cameraIMU.header.stamp).toSec())/2;
		// robotOdom.pose.pose.position.z = (robotOdom.twist.twist.linear.z+pre_robotOdom.twist.twist.linear.z)*(ros::Duration(cur_cameraIMU.header.stamp - pre_cameraIMU.header.stamp).toSec())/2;
		// ROS_INFO_STREAM("robotOdom_v ="<<"\n"<<robotOdom.twist.twist.linear<<"\n");
		// pre_robotOdom = robotOdom;

		pre_cameraIMU = cur_cameraIMU;	
		PROCESS_INIT = false;
		// ROS_INFO_STREAM("imu_roll ="<<"\n"<<imu_roll*180/M_1_PI<<"\n"<< "imu_pitch =" <<imu_pitch*180/M_1_PI<<"\n"<< "imu_yaw =" <<imu_yaw*180/M_1_PI);
		// ROS_INFO_STREAM("cur_cameraIMU =" <<cur_cameraIMU<<"\n");
	}
	else{
		// initialize imu
		imu_yaw = atan2(cur_cameraIMU.angular_velocity.y , cur_cameraIMU.angular_velocity.x);
		pre_cameraIMU = cur_cameraIMU;
		// ROS_INFO_STREAM("cameraIMU initialize" <<pre_cameraIMU<<"\n");
		PROCESS_INIT = false;
	}
	RECEIVED_ROBOT_IMU = true;
	// robotOdom = *msg;
	// RECEIVED_ROBOT_ODOM = true;
	//move manage method
	// manage();
}
void trackingAvoidance::robotEncoder_callback(const beego_control::beego_encoder::ConstPtr& msg){
	// ROS_INFO("robotEncoder_callback");
    //データをコピー
	robotEncoder = *msg;	
	RECEIVED_ROBOT_ENCODAR = true;
	//move manage method
	// manage();
}
void trackingAvoidance::scan_callback(const autonomous_mobile_robot::SensorMapData::ConstPtr& msg){
	// ROS_ERROR_STREAM("Could not lrfMAP."<<smd);
	// ROS_INFO("scan_callback");
	smd =*msg;
	RECEIVED_LRF_SCAN = true;
	// manage();
}
void trackingAvoidance::goalOdom_callback(const nav_msgs::Odometry::ConstPtr& msg){
	// ROS_INFO("goalOdom_callback");
    //データをコピー
	goalOdom = *msg;
	RECEIVED_GOAL_ODOM = true;
	//move manage method
	// manage();
}
void trackingAvoidance::manage(){
	// ROS_INFO("into manage");
	if(data_check()){
		data_check_reset();
		// ROS_INFO_STREAM("time_init\n");
		get_time();
		// 一回目初期値設定処理
		if(!culc_delta_time()){
			// pre_cameraIMU = cur_cameraIMU;
		    robotOdom.header.frame_id = "base_link";
		    robotOdom.header.stamp = cur_time;
		    robotOdom.header.seq = 0;
    		robotOdom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
			pre_robotOdom = robotOdom;
			total_time += delta_time;
			ROS_INFO_STREAM("cur_time =" <<cur_time<<"\n"<< "total time =" <<total_time);
			// ROS_INFO_STREAM("robot_init");
			return;
		}
		culc_delta_robotOdom(); //ロボットグローバル速度算出
		update_goal_position(); //現在地とゴール位置の位置関係更新
		create_histgram(); //ヒストグラム作成
		create_binary_histgram(robotRadius, marginRadius);
		// histgramCheckerEx();
		//探索処理
		float tagVel, tagAng;
		searchProcess(tagVel, tagAng);
		// ROS_INFO_STREAM("target vel =" <<tagVel<<"\n"<< "target angle =" <<tagAng);
		// potential
		// apf();
		// apf_mpc();
		//命令速度生成
		// ROS_INFO_STREAM("error"<<"\n");
		geometry_msgs::Twist cmd = controler(tagVel, tagAng);
		ROS_INFO_STREAM("publishData = \n" <<cmd);
		publishData(cmd);

		// ROS_INFO("debug");
		// publish_deltaRobotOdom();
		// debug();
		// if(debugRotationVelocityCheckerFlag){
			// rotationVelocityChecker(deltaRobotOdom.twist.twist.angular.z);
			// rotationVelocityChecker(cur_angVel);
		// }
	}
}
bool trackingAvoidance::data_check(){
	ROS_INFO_STREAM(
		(RECEIVED_CLUSTER ? "RECEIVED_CLUSTER" : "NOT CLUSTER") <<","<<
		(RECEIVED_GOAL_ODOM ? "RECEIVED_GOAL_ODOM" : "NOT GOAL_ODOM") <<","<<
		(RECEIVED_ROBOT_IMU ? "RECEIVED_ROBOT_IMU" : "NOT ROBOT_IMU") <<","<<
		(RECEIVED_ROBOT_ENCODAR ? "RECEIVED_ROBOT_ENCODAR" : "NOT ROBOT_ENCODAR") <<","<<
		(RECEIVED_LRF_SCAN ? "RECEIVED_LRF_SCAN" : "NOT LRF_SCAN")
	);
		// (RECEIVED_ROBOT_ODOM ? "RECEIVED_ROBOT_ODOM" : "NOT ROBOT_ODOM") <<","<<
	return(
	    RECEIVED_CLUSTER&& RECEIVED_GOAL_ODOM&& 
		RECEIVED_ROBOT_IMU&& RECEIVED_ROBOT_ENCODAR&& RECEIVED_LRF_SCAN 
		// RECEIVED_ROBOT_ODOM
	);
}
void trackingAvoidance::data_check_reset(){
	RECEIVED_CLUSTER = false;
	RECEIVED_ROBOT_IMU = false;
	RECEIVED_ROBOT_ENCODAR = false;
	RECEIVED_LRF_SCAN =false;
	// RECEIVED_ROBOT_ODOM = false;
}
void trackingAvoidance::get_time(){
	cur_time = ros::Time::now();
}
bool trackingAvoidance::culc_delta_time(){
	if(!PROCESS_ONCE){
		delta_time_ros = cur_time - pre_time;
		delta_time = delta_time_ros.toSec();//経過時間算出
		pre_time = cur_time;
		PROCESS_ONCE = false;
		return true;
	}
	else{
		// ROS_INFO_STREAM("first_process\n");
		pre_time = cur_time;
		PROCESS_ONCE = false;
		return false;
	}
}
void trackingAvoidance::culc_delta_robotOdom(){
	// deltaRobotOdom.pose.pose.position.x = robotOdom.pose.pose.position.x - pre_robotOdom.pose.pose.position.x;
	// deltaRobotOdom.pose.pose.position.y = robotOdom.pose.pose.position.y - pre_robotOdom.pose.pose.position.y;
	// deltaRobotOdom.pose.pose.position.z = robotOdom.pose.pose.position.z - pre_robotOdom.pose.pose.position.z;
	// deltaRobotOdom.twist.twist.linear.x = (robotOdom.pose.pose.position.x - pre_robotOdom.pose.pose.position.x ) / ( ros::Duration(robotOdom.header.stamp - pre_robotOdom.header.stamp).toSec() );
	// deltaRobotOdom.twist.twist.linear.y = (robotOdom.pose.pose.position.y - pre_robotOdom.pose.pose.position.y ) / ( ros::Duration(robotOdom.header.stamp - pre_robotOdom.header.stamp).toSec() );
	// deltaRobotOdom.twist.twist.linear.z = (robotOdom.pose.pose.position.z - pre_robotOdom.pose.pose.position.z ) / ( ros::Duration(robotOdom.header.stamp - pre_robotOdom.header.stamp).toSec() );
	// deltaRobotOdom = robotOdom - pre_robotOdom;
	//回転
	// tf::Quaternion quat;
	// double r_cur,p_cur,y_cur;
	// quaternionMsgToTF(robotOdom.pose.pose.orientation, quat);
	// tf::Matrix3x3(quat).getRPY(r_cur,p_cur,y_cur);
	// double r_pre,p_pre,y_pre;
	// quaternionMsgToTF(pre_robotOdom.pose.pose.orientation, quat);
	// tf::Matrix3x3(quat).getRPY(r_pre,p_pre,y_pre);
	// double delta_theta = y_cur - y_pre;
	// double omega = delta_theta/( ros::Duration(robotOdom.header.stamp - pre_robotOdom.header.stamp).toSec() );
	// double omega = delta_theta/( ros::Duration(robotOdom.header.stamp - pre_robotOdom.header.stamp).toSec() );
	// tf::Quaternion quatTheta=tf::createQuaternionFromYaw(delta_theta);//debugRobotYaw-M_PI_2);
	// quaternionTFToMsg(quatTheta, deltaRobotOdom.pose.pose.orientation);
	// deltaRobotOdom.twist.twist.angular.x = 0;
	// deltaRobotOdom.twist.twist.angular.y = 0;
	// deltaRobotOdom.twist.twist.angular.z = omega;
	// ROS_INFO("omega=%f",omega);
	
	// ロボットの位置,角度更新
    robotOdom.header.frame_id = pre_robotOdom.header.frame_id;
    robotOdom.header.stamp = cur_time;
    robotOdom.header.seq = pre_robotOdom.header.seq + 1;
    //経過時間算出
    // ros::Duration delta_time_ros = robotOdom.header.stamp - pre_robotOdom.header.stamp;
    // double delta_time = delta_time_ros.toSec();
	total_time += delta_time;
	ROS_INFO_STREAM("delta_time =" <<delta_time<<"\n"<< "total time =" <<total_time);
    // ロボットの速度算出
    cur_vel = ((-robotEncoder.vel.r) + robotEncoder.vel.l)/2;
	cur_angVel = ((-robotEncoder.vel.r) - robotEncoder.vel.l)/(2 * wheel_d);
	// ROS_INFO("cur_vel=%f",cur_vel);
    //--姿勢取得(tf座標系)
    tf::Quaternion quater;
    double r,p,y;
    quaternionMsgToTF(pre_robotOdom.pose.pose.orientation, quater);
    tf::Matrix3x3(quater).getRPY(r, p, y);
    //--ロボット速度算出(tf座標系)
    double vx_r = cur_vel*cos(cur_angVel * delta_time + y);
    double vy_r = cur_vel*sin(cur_angVel * delta_time + y);
    //位置更新(tf座標系)
    robotOdom.pose.pose.position.x = robotOdom.pose.pose.position.x + vx_r *delta_time;
    robotOdom.pose.pose.position.y = robotOdom.pose.pose.position.y + vy_r *delta_time;
    robotOdom.pose.pose.position.z = robotOdom.pose.pose.position.z;
    double yaw = cur_angVel * delta_time + y;
	deltaRobotOdom.pose.pose.orientation =  tf::createQuaternionMsgFromYaw(cur_angVel * delta_time);
    robotOdom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
	// ROS_INFO_STREAM("robotOdom_x ="<<"\n"<<robotOdom.pose.pose.position<<"\n");
	pre_robotOdom = robotOdom;
}
//自己位置姿勢とゴール位置からロボット座標軸上でのゴール座標を算出する
void trackingAvoidance::update_goal_position(){
	//robotOdom, goalOdom -> relationOdom
	//ゴール位置のフレームIDをマップに設定してgoalOdomをbase_linkに座標変化すればいいのでは
	// tf::TransformListener listener_;
	// tf::Transform cam_to_target;
	// tf::poseMsgToTF(p->pose.pose, cam_to_target);
	// tf::StampedTransform req_to_cam;
	// listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), req_to_cam);
	//面倒なので位置の差と回転行列だけで良さそう
	//位置差
	// std::cout<<"goal== "<<goalOdom.pose.pose.position.x<<","<<goalOdom.pose.pose.position.y<<","<<goalOdom.pose.pose.position.z<<std::endl;
	// std::cout<<"robot== "<<robotOdom.pose.pose.position.y<<","<<-robotOdom.pose.pose.position.x<<","<<robotOdom.pose.pose.position.z<<std::endl;
	// relationOdom.pose.pose.position.x = goalOdom.pose.pose.position.x - robotOdom.pose.pose.position.x;
	// relationOdom.pose.pose.position.y = goalOdom.pose.pose.position.y - robotOdom.pose.pose.position.y;
	// relationOdom.pose.pose.position.x = goalOdom.pose.pose.position.x - robotOdom.pose.pose.position.y;
	// relationOdom.pose.pose.position.y = goalOdom.pose.pose.position.y - (-robotOdom.pose.pose.position.x);

	relationOdom.pose.pose.position.x = goalOdom.pose.pose.position.y - robotOdom.pose.pose.position.x;
	relationOdom.pose.pose.position.y = (-goalOdom.pose.pose.position.x) - robotOdom.pose.pose.position.y;
	relationOdom.pose.pose.position.z = goalOdom.pose.pose.position.z - robotOdom.pose.pose.position.z;
	tf::Quaternion quatGoal=tf::createQuaternionFromYaw(M_PI_2);//debugRobotYaw-M_PI_2);
	// quaternionTFToMsg(quatGoal, goalOdom.pose.pose.orientation);
	//角度差はロボット姿勢角度 
	tf::Quaternion quat;
	double r,p,y;
	quaternionMsgToTF(robotOdom.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(r, p, y);
	//ロボット座標系の軸を揃える
	// y += M_PI_2;//90deg回転
	double theta_goal = atan2(relationOdom.pose.pose.position.y,relationOdom.pose.pose.position.x);
	double theta_robot = y;
	// std::cout<<"theta_goal - theta_robot = "<<theta_goal - theta_robot<<std::endl;
	double theta_relation = theta_goal - theta_robot;
	double length = std::sqrt(std::pow(relationOdom.pose.pose.position.x,2.0) + std::pow(relationOdom.pose.pose.position.y,2.0));
	//グロ-バル座標軸で算出
	// std::cout<< length <<"*"<< "cos("<<theta_relation<<");"<<std::endl;
	// std::cout<< length <<"*"<< cos(theta_relation)<<";"<<std::endl;
	relationOdom.pose.pose.position.x = length * cos(theta_relation);
	relationOdom.pose.pose.position.y = length * sin(theta_relation);
	relationOdom.pose.pose.position.z = 0;
	// std::cout<< length <<"*"<< "sin("<<theta_relation<<");"<<std::endl;
	// std::cout<< length <<"*"<< sin(theta_relation)<<";"<<std::endl;
	// std::cout<<"relationOdom.pose.pose.position.x="<< length * cos(theta_relation)<<std::endl;
	// std::cout<<"relationOdom.pose.pose.position.y="<< length * sin(theta_relation)<<std::endl;
	// std::cout<<"relationOdom.pose.pose.position.x="<< relationOdom.pose.pose.position.x<<std::endl;
	// std::cout<<"relationOdom.pose.pose.position.y="<< relationOdom.pose.pose.position.y<<std::endl;
	quat=tf::createQuaternionFromYaw(theta_relation);
	quaternionTFToMsg(quat, relationOdom.pose.pose.orientation);
	//ゴールセット
	goal_x = -relationOdom.pose.pose.position.y;
	goal_y = relationOdom.pose.pose.position.x;
	std::cout<<"goal_xy: "<<goal_x <<","<<goal_y<<std::endl;
}
// 障害物が接近障害物か判断
bool trackingAvoidance::checkSafetyObstacle(float& t, float& angle, float& x, float& y){
	//Safe Time range
	float safeTime = 100;
	//process
	//時間がマイナスor当分ぶつからない
	if( t > safeTime){
		return true;
	}
	else if( t < 0){
		return true;
	}
	//ROS_INFO("angle:%f",angle);
	if(x > 0){
		if(y > 0){//第1象限
			if(angle <= M_PI_2 && angle >= 0){
				//SAFE
				return true;
			}
			else{
				//WARNIGN
				//ROS_INFO("Num 1 field WARNIGN");
			}
		}
		else{//第4象限
			if(angle <=-M_PI_2 && angle > -M_PI){
				//SAFE
				return true;
			}
			else{
				//WARNIGN					
				//ROS_INFO("Num 4 field WARNIGN");
			}
		}
	}
	else{
		if(y > 0){//第2象限
			if(angle >= M_PI_2 && angle <= M_PI){
				//SAFE
				return true;		
			}
			else{
				//WARNIGN		
				//ROS_INFO("Num 2 field WARNIGN");	
			}
		}
		else{//第3象限
			if(angle >=-M_PI && angle < -M_PI_2){
				//SAFE
				return true;			
			}
			else{
				//WARNIGN	
				//ROS_INFO("Num 3 field WARNIGN");	
			}
		}
	}
	return false;
}
//障害物１つに対するx,y座標の交差位置を相対速度を使用して算出(交差位置を返す)
trackingAvoidance::crossPoint trackingAvoidance::getCrossPoint(int& indexRef,geometry_msgs::Point& gpRef, geometry_msgs::Twist& twistRef,float& cur_vel, float& cur_ang, float& cmd_dV, float& cmd_ang){
	// struct crossPoint{
	// 	float x;//交差位置x
	// 	float y;//交差位置y
	// 	float dis;//交差位置とロボットの距離
	// 	float t;//交差時の時間
	// 	int index;//障害物番号
	// };
	// std::vector<crossPoint> crsPts;
	//ロボット速度 
	// float Vrx = cur_vel * cos(cur_ang);//*delta_time);
	// float Vry = cur_vel * sin(cur_ang);//*delta_time);
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
	//回転速度変化
	double v_rot_x, v_rot_y;
	double x_para_x = gpRef.x;
	double x_para_y = gpRef.y;
	double x_para_vx = twistRef.linear.x;
	double x_para_vy = twistRef.linear.y;
	double delta_r,delta_p,delta_yaw;
	tf::Quaternion quat_rot;
	quaternionMsgToTF(deltaRobotOdom.pose.pose.orientation, quat_rot);
	tf::Matrix3x3(quat_rot).getRPY(delta_r,delta_p,delta_yaw);
	double x_para_theta = delta_yaw;//std::atan2(gpRef.y,gpRef.x)-M_PI_2;
	// double omega = deltaRobotOdom.twist.twist.angular.z;
	double omega = cur_angVel;
	// debug_trans_rotation_vel(v_rot_x,v_rot_y,x_para_x,x_para_y,x_para_theta,x_para_vx, x_para_vy,omega);
	// trans_rotation_vel(v_rot_x,v_rot_y,x_para_x,x_para_y,x_para_vx, x_para_vy);
	double pos_x = gpRef.x;
	double pos_y = gpRef.y;
	double vel_x = twistRef.linear.x;
	double vel_y = twistRef.linear.y;
	double vr_x = Vrx;
	double vr_y = Vry;
	double delta_pos_x = pos_x + cos(delta_yaw)*(vel_x*delta_time - pos_x + vr_x*delta_time) + sin(delta_yaw)*(vel_y*delta_time - pos_y + vr_y*delta_time);
	double delta_pos_y = pos_y - sin(delta_yaw)*(vel_x*delta_time - pos_x + vr_x*delta_time) + cos(delta_yaw)*(vel_y*delta_time - pos_y + vr_y*delta_time);
	// 速度
	// float Vox = v_rot_x + Vrx;//twistRef.linear.x + Vrx - v_rot_x;
	// float Voy = v_rot_y + Vry;//twistRef.linear.y + Vry - v_rot_y;
	float Vox = delta_pos_x/delta_time;
	float Voy = delta_pos_y/delta_time;
	//回転
	tf::Quaternion quat;
	double r_cur,p_cur,y_cur;
	quaternionMsgToTF(robotOdom.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(r_cur,p_cur,y_cur);
	double r_pre,p_pre,y_pre;
	quaternionMsgToTF(pre_robotOdom.pose.pose.orientation, quat);
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
	crossPoint crsPt;
	float Vcx = Vox - dVrx_c;
	float Vcy = Voy - dVry_c;
	crsPt.vx = Vcx;
	crsPt.vy = Vcy;
	//ROS_INFO("Vc(x,y):(%f,%f)",Vcx,Vcy);
	crsPt.safe = false;
	// 場合分け
	//相対速度ゼロ
	if(Vcx == 0&& Vcy ==0){
		crsPt.safe = true;
		return crsPt;
	}
	// 直線(y軸方向で接近)
	bool straight_y = false;	
	// 傾きが無限大に近い
	float angle = atan2(Vcy,Vcx);
	float angleThreshold = M_PI/18;//10 deg :後でrqt_reconfigureで設定できるようにする
	float frontAngle = M_PI_2;
	//ROS_INFO("angle:%f",angle/M_PI * 180);
	if(std::abs(angle + frontAngle) < angleThreshold){
		straight_y = true;
	}
	//正面方向で直線移動の障害物
	if(straight_y){
		//ROS_INFO("Xox:%f",Xox);
		crsPt.x = Xox;
		crsPt.y = 0;
		crsPt.dis = Xox;
		crsPt.t = (0-Xoy)/Vcy;
		crsPt.index = index;
	}
	//それ以外
	else{
		//直線の式
		float a = Vcy/Vcx;//X軸の右が正
		float b = Xoy - a*Xox;
		//交差位置 仮
		// x = 0
		//ROS_INFO("%f x + %f ",a,b);
		crossPoint crsPt_x0;
		crsPt_x0.x = 0;
		crsPt_x0.y = b;
		crsPt_x0.dis = crsPt_x0.y;
		crsPt_x0.t = (0-Xox)/Vcx;//
		crsPt_x0.safe = false;
		// y = 0
		crossPoint crsPt_y0;
		crsPt_y0.x = - b /a;	
		crsPt_y0.y = 0;
		crsPt_y0.dis = crsPt_y0.x;
		crsPt_y0.t = (0-Xoy)/Vcy;//
		crsPt_y0.safe = false;
		//ROS_INFO("x0,y0;(%f,%f),(%f,%f)",crsPt_x0.x,crsPt_x0.y,crsPt_y0.x,crsPt_y0.y);
		// 時間t が短い方を採用 and t > 0
		if(crsPt_x0.t < 0 && crsPt_y0.t < 0){
			//時間がどちらもマイナス -> 遠ざかっている障害物
			if(crsPt_x0.t > crsPt_y0.t){
				crsPt = crsPt_x0;
			}
			else{
				crsPt = crsPt_y0;
			}
		}
		else if(crsPt_x0.t > 0 && crsPt_y0.t > 0){
			//時間がどちらもプラス
			if(crsPt_x0.t > crsPt_y0.t){
				crsPt = crsPt_y0;
			}
			else{
				crsPt = crsPt_x0;
			}
		}
		else{
			//時間がどっちかがプラス -> プラスの値を格納
			if(crsPt_x0.t > crsPt_y0.t){
				crsPt = crsPt_x0;
			}
			else{
				crsPt = crsPt_y0;
			}
		}
	}
	//ROS_INFO("%f,crsPt:%f,%f",crsPt.t,crsPt.x,crsPt.y);
	// if(crsPt.t > safeTime){
	// 	crsPt.safe = true;
	// }
	if(checkSafetyObstacle(crsPt.t, angle,Xox,Xoy)){
		crsPt.safe = true;
	}
	else{
		crsPt.safe = false;
	}
	return crsPt;
}
void trackingAvoidance::getCrossPoints(crossPoint& crsPt_x0, crossPoint& crsPt_y0, int& indexRef, const autonomous_mobile_robot::ClassificationElement& clst_data, geometry_msgs::Twist& twistRef, float& cur_vel, float& cur_ang, float& cmd_dV, float& cmd_ang){
	// struct crossPoint{
	// 	float x;//交差位置x
	// 	float y;//交差位置y
	// 	float dis;//交差位置とロボットの距離
	// 	float t;//交差時の時間
	// 	int index;//障害物番号
	// };
	// std::vector<crossPoint> crsPts;
	//ロボット速度 
	float Vrx = cur_vel * cos(cur_ang);//*delta_time);
	float Vry = cur_vel * sin(cur_ang);//*delta_time);
	// 目標速度(探査対象)
	//cmd_dAng は水平右をx軸, 正面をy軸とする
	float dVrx_c = (cmd_dV+cur_vel) * cos(cmd_ang);//*delta_time);//(cmd_dV+cur_vel) * cos(cmd_ang);
	float dVry_c = (cmd_dV+cur_vel) * sin(cmd_ang);//*delta_time);//(cmd_dV+cur_vel) * sin(cmd_ang);
	//障害物
	// 位置
	float Xox = clst_data.gc.x;
	float Xoy = clst_data.gc.y;
	// 速度
	float Vox = twistRef.linear.x + Vrx;
	float Voy = twistRef.linear.y + Vry;
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
		// getNearestDistance(crsPt_x0, clst_data);
	}
	if(checkSafetyObstacle(crsPt_y0.t, angle,Xox,Xoy)){
		crsPt_y0.safe = true;
	}
	else{
		crsPt_y0.safe = false;
		// getNearestDistance(crsPt_y0, clst_data);
	}
}
double trackingAvoidance::getNearestDistance(crossPoint& crsPt, const autonomous_mobile_robot::ClassificationElement& clst_data){
	//点群の中で最も近い距離を交差位置.距離にセット
	for(int i=0; i<clst_data.pt.size();i++){
		double x = crsPt.x - (clst_data.pt[i].x - clst_data.gc.x);
		double y = crsPt.y - (clst_data.pt[i].y - clst_data.gc.y);
		double dis = std::sqrt(std::pow(x,2.0)+std::pow(y,2.0));
		if(crsPt.dis > dis){
			crsPt.dis = dis;
		}
	}
}
//障害物データ群に対する各x,y座標の交差位置を相対速度を使用して算出(交差位置の配列)
void trackingAvoidance::crossPointsDetect(std::vector<crossPoint>& crsPts, float& cur_vel_temp, float& cur_angle_temp, float& cmd_dV, float& cmd_dAng){
    //-before edit
	// crsPts.resize((int)clstr.data.size()*2);
	// for(int k=0; k<clstr.data.size(); k++){
	// 	// getCrossPoints(crsPts[k*2], crsPts[k*2+1], k, clstr.data[k], clstr.twist[k],cur_vel_temp,  cur_angle_temp, cmd_dV,cmd_dAng);
	// 	getCrossPoints(crsPts[k*2], crsPts[k*2+1], k, clstr.data[k].gc, clstr.twist[k],cur_vel_temp,  cur_angle_temp, cmd_dV,cmd_dAng);
	// }
	// crsPts.resize((int)clstr.data.size());
	// for(int k=0; k<clstr.data.size(); k++){
	// 	crsPts[k] = getCrossPoint(k, clstr.data[k], clstr.twist[k],cur_vel_temp,  cur_angle_temp, cmd_dV,cmd_dAng);
	// }
	//-after edit single cross
 	// crsPts.resize((int)clstr.data.size());
	// for(int k=0; k<clstr.data.size(); k++){
	// 	crsPts[k] = getCrossPoint(k,clstr.data[k].gc, clstr.twist[k],cur_vel_temp,  cur_angle_temp, cmd_dV,cmd_dAng);
	// }
	
	//-after edit double cross
 	crsPts.resize((int)clstr.data.size()*2);
	int count = 0;
	for(int k=0; k<clstr.data.size(); k++){
		// getCrossPoints(crsPts[k*2], crsPts[k*2+1], k, clstr.data[k].gc, clstr.twist[k],cur_vel_temp,  cur_angle_temp, cmd_dV,cmd_dAng);
		getCrossPoints(crsPts[k*2], crsPts[k*2+1], k, clstr.data[k].gc, clstr.twist[k].twist,cur_vel_temp,  cur_angle_temp, cmd_dV,cmd_dAng);
	}
}
//--コスト関数
double trackingAvoidance::costCrossPoint(crossPoint& crsPt, float eta_cp){
	// eta_cp : cost算出用パラメータ
	//コスト関数: あとで変更する予定
	// return (-(pow(crsPt.dis/crsPt.t/eta_cp,2.0)));
	// return ((pow(1/crsPt.dis/eta_cp,2.0)/crsPt.t));
	if(safe_range <= std::abs(crsPt.dis)){
		return (0);
	}
	// return ((pow(1/crsPt.dis/eta_cp,2.0)/(crsPt.t+1)));
	double dis =pow(crsPt.x/crossWeightX,2.0)+pow(crsPt.y/crossWeightY,2.0); 
	return ((1/(dis/eta_cp))/(crsPt.t+timeBias));
}
double trackingAvoidance::getDeltaVelCost(float& cmd_dV_temp, float& eta_vel_temp,float& cur_vel_temp){
	// return ((pow((cmd_dV_temp+cur_vel_temp - default_speed)/eta_vel_temp,2.0)));
	// return (std::abs(cmd_dV_temp)/eta_vel_temp);
	return (std::abs(cmd_dV_temp+cur_vel_temp - default_speed)/eta_vel_temp);
}
double trackingAvoidance::getCrossPointCost(std::vector<crossPoint>& crsPts, float eta_cp){
	// 衝突検出 cost算出
	float sumCost_cp=0;//交差位置に対するコスト値
	for(int k = 0; k < crsPts.size(); k++){
		if(crsPts[k].safe){
			continue;
		}
		sumCost_cp += costCrossPoint(crsPts[k], eta_cp);
	}
	return sumCost_cp;
}
geometry_msgs::Twist trackingAvoidance::controler(float& tagVel, float& tagAng){
	if(tagVel ==0){
		geometry_msgs::Twist twist;
		twist.linear.x =0; 
		twist.linear.y =0; 
		twist.linear.z =0;
		twist.angular.x =0;
		twist.angular.y =0;
		twist.angular.z =0;
		return twist;
	}	
	//p制御
	double cur_ang = 90;//正面を向いているため
	float gainP = 0.01;
	float tagAngVel = (tagAng-cur_ang)*gainP;
	geometry_msgs::Twist twist;
	twist.linear.x =tagVel; 
	twist.linear.y =0; 
	twist.linear.z =0;
	twist.angular.x =0;
	twist.angular.y =0;
	twist.angular.z =tagAngVel;
	return twist;
}
void trackingAvoidance::searchProcess(float& tagVel, float& tagAng){
	//only angle
	float dV=0;
	if(SEARCH_ONLY_ANGLE){
		// dV = debugCmd_vel;
		// cur_vel = default_speed;
		vfh_angleSearch(tagAng,cur_vel, dV);
		tagVel = default_speed;
	}
	else{
		search_vel_ang(tagAng,cur_vel, dV);
		tagVel = cur_vel + dV;
	}
	prev_tagAng = tagAng;
}
// 最適角度探索
double trackingAvoidance::vfh_angleSearch(float& target_angle_temp, float& cur_vel_temp, float& cmd_dV){//return cost
	std::vector<bool> histgram_bi;
    get_histgram_bi(histgram_bi);
	// goal_x=debugGoalPosX;
	// goal_y=debugGoalPosY;
	float goalAng = atan2(goal_y, goal_x)*180/M_PI;
	//weight正規化
    double sum_weight = k_cp + k_curAngle + k_prevAngle + k_cp;
    k_g/=sum_weight;
    k_curAngle/=sum_weight;
    k_prevAngle/=sum_weight;
    k_cp/=sum_weight;
    //コスト算出
    double min_cost = MAX_COST;
    int min_num = -1;
	std::vector<crossPoint> min_cost_crsPts;
    for(int i=0; i<histgram_bi.size();i++){
        if(!histgram_bi[i]){
            continue;
        }
        double ang = transform_numToAngle(i);//deg
        float cmd_ang = ang*M_PI/180;
		double cur_ang = 90;//正面を向いているため
		float cmd_dAng_rad = cmd_ang;//(cmd_ang-cur_ang);
		float prev_tagAng_rad = prev_tagAng*M_PI/180;
		//交差位置算出
        std::vector<crossPoint> crsPts;
		//現在の進行方向はprev_tagAngの方向であると仮定
		// ROS_INFO_STREAM(i<<":"<<cur_vel_temp<<", "<<prev_tagAng_rad<<", "<<cmd_dV<<", "<<cmd_dAng_rad);
		crossPointsDetect(crsPts, cur_vel_temp, prev_tagAng_rad, cmd_dV, cmd_dAng_rad);
		// std::cout<<i<<":"<<std::endl;
		// for(int k = 0; k < crsPts.size(); k++){
		// 	ROS_INFO_STREAM(k<<" -- "<<crsPts[k].x <<","<<crsPts[k].y<<","<<crsPts[k].t<<","<<crsPts[k].safe<<","<<crsPts[k].vx<<","<<crsPts[k].vy);
		// }
		// ROS_INFO_STREAM(i<<":"<<"for crsPts size:" << crsPts.size());		
		//コスト算出
        double goalCost = cost_goal_angle_deg(ang, goalAng);
        double angCost = cost_current_angle_deg(ang, cur_ang);
        double prevAngCost = cost_prev_select_angle_deg(ang, prev_tagAng);
        double crossCost = getCrossPointCost(crsPts,eta_cp);
        double cost = k_g * goalCost + k_curAngle * angCost + k_prevAngle * prevAngCost + k_cp*crossCost;
        // std::cout<<i<<":"<<cost<<std::endl;
        // std::cout<<i<<":"<<goalCost<<","<<angCost<<","<<prevAngCost<<","<<crossCost<<std::endl;
		if(min_cost > cost){
            min_cost = cost;
            min_num = i;
			//デバッグ用
			min_cost_crsPts = crsPts;
			// ROS_INFO_STREAM("for min crsPts size:" << min_cost_crsPts.size());
        }
    }
	//目標角度
	target_angle_temp = transform_numToAngle(min_num);
	if(min_cost == MAX_COST){
		// ROS_INFO_STREAM("Not found space");
		target_angle_temp = 90;
	}
	
	//デバッグ関数に交差位値情報を渡す
	if(display_output){
		// ROS_INFO_STREAM("min crsPts size:" << min_cost_crsPts.size());
		// ROS_INFO_STREAM("cur_vel, dV" << cur_vel_temp <<","<< cmd_dV);
		showOutPut(min_cost_crsPts, cur_vel_temp + cmd_dV, min_num);
		display_rotVel();
	}
	return min_cost;
}
// 最適角度探索(デバッグ処理なし)
double trackingAvoidance::vfh_angleSearch_nondeb(float& target_angle_temp, float& cur_vel_temp, float& cmd_dV ,std::vector<crossPoint>& min_cost_crsPts_temp ){//return cost
	std::vector<bool> histgram_bi;
    get_histgram_bi(histgram_bi);
	// goal_x=debugGoalPosX;
	// goal_y=debugGoalPosY;
	float goalAng = atan2(goal_y, goal_x)*180/M_PI;
	//weight正規化
    double sum_weight = k_cp + k_curAngle + k_prevAngle + k_cp + k_vel;
    k_g/=sum_weight;
    k_curAngle/=sum_weight;
    k_prevAngle/=sum_weight;
    k_cp/=sum_weight;
    k_vel/=sum_weight;
    //コスト算出
    double min_cost = MAX_COST;
    int min_num = -1;
	std::vector<crossPoint> min_cost_crsPts;
    for(int i=0; i<histgram_bi.size();i++){
        if(!histgram_bi[i]){
            continue;
        }
        double ang = transform_numToAngle(i);//deg
        float cmd_ang = ang*M_PI/180;
		double cur_ang = 90;//正面を向いているため
		float cmd_dAng_rad = cmd_ang;//(cmd_ang-cur_ang);
		float prev_tagAng_rad = prev_tagAng*M_PI/180;
		//交差位置算出
        std::vector<crossPoint> crsPts;
		//現在の進行方向はprev_tagAngの方向であると仮定
		// ROS_INFO_STREAM(i<<":"<<cur_vel_temp<<", "<<prev_tagAng_rad<<", "<<cmd_dV<<", "<<cmd_dAng_rad);
		crossPointsDetect(crsPts, cur_vel_temp, prev_tagAng_rad, cmd_dV, cmd_dAng_rad);
		// std::cout<<i<<":"<<std::endl;
		// for(int k = 0; k < crsPts.size(); k++){
		// 	ROS_INFO_STREAM(k<<" -- "<<crsPts[k].x <<","<<crsPts[k].y<<","<<crsPts[k].t<<","<<crsPts[k].safe<<","<<crsPts[k].vx<<","<<crsPts[k].vy);
		// }
		// ROS_INFO_STREAM(i<<":"<<"for crsPts size:" << crsPts.size());		
		//コスト算出
        double goalCost = cost_goal_angle_deg(ang, goalAng);
        double angCost = cost_current_angle_deg(ang, cur_ang);
        double prevAngCost = cost_prev_select_angle_deg(ang, prev_tagAng);
        double crossCost = getCrossPointCost(crsPts,eta_cp);
        double velCost = getDeltaVelCost(cmd_dV,eta_vel,cur_vel_temp);
        double cost = k_g * goalCost + k_curAngle * angCost + k_prevAngle * prevAngCost + k_cp*crossCost + k_vel*velCost;
        // std::cout<<i<<":"<<cost<<std::endl;
        // std::cout<<i<<":"<<goalCost<<","<<angCost<<","<<prevAngCost<<","<<crossCost<<std::endl;
		if(min_cost > cost){
            min_cost = cost;
            min_num = i;
			//デバッグ用
			min_cost_crsPts = crsPts;
			// ROS_INFO_STREAM("for min crsPts size:" << min_cost_crsPts.size());
        }
    }
	//目標角度
	target_angle_temp = transform_numToAngle(min_num);
	if(min_cost == 1){
		target_angle_temp = 90;
		// ROS_INFO_STREAM("Not found space");
	}
	min_cost_crsPts_temp.clear();
	min_cost_crsPts_temp = min_cost_crsPts;
	return min_cost;
}
void trackingAvoidance::search_vel_ang(float& target_angle, float& cur_vel_temp, float& cmd_dV){
	//探索回数(未使用)
	int count = 0;
	const int countThreshold =10;
	//最適化対象: 評価値
	double evalVal = MAX_COST;
	std::vector<crossPoint> min_cost_crsPts;
	//--全探査
	float dV = 0;//探査対象dv
	float dAng = 0;//探査対象dAng
	for(float search_dV = -dV_range; search_dV <= dV_range; search_dV +=dV_div){
		if(search_dV + cur_vel_temp > max_speed || search_dV + cur_vel_temp < min_speed){//
			continue;
		}
		float searching_angle;//探査対象angle
		std::vector<crossPoint> min_cost_crsPts_temp;
		//評価
		double evalTemp = vfh_angleSearch_nondeb(searching_angle, cur_vel_temp, search_dV,min_cost_crsPts_temp);
		if(evalTemp < evalVal){
			evalVal = evalTemp;
			dV = search_dV;
			dAng = searching_angle;
			min_cost_crsPts.clear();
			min_cost_crsPts = min_cost_crsPts_temp;
		}
	}
	if(evalVal ==MAX_COST){
		dAng=90;
	}
	cmd_dV = dV ;
	target_angle = dAng;

	//デバッグ関数に交差位値情報を渡す
	if(display_output){
		// ROS_INFO_STREAM("min crsPts size:" << min_cost_crsPts.size());
		// ROS_INFO_STREAM("cur_vel, dV :" << cur_vel_temp <<","<< cmd_dV);
		// ROS_INFO_STREAM("target_angle :" << target_angle);
		int min_num = transform_angle_RobotToNum(target_angle);
		showOutPut(min_cost_crsPts, cur_vel_temp + cmd_dV, min_num);
		display_rotVel();
	}
}
// データ送信
void trackingAvoidance::publishData(geometry_msgs::Twist& pubData){
    pub.publish(pubData);
}
//ヒストグラム配列の作成
void trackingAvoidance::create_histgram(){
	set_histgram_param(angle_min,angle_max, angle_div);
	//データ定義
	//選択角度範囲
	// float angle_min = M_PI_4;//最小センサ角度
	// float angle_max = M_PI_2 + M_PI_4;//最大センサ角度
	// float angle_div = 1.0;//解像度
	// std::vector<double> hst;//ヒストグラム配列
	// hst.resize( (int)((angle_max - angle_min)/angle_div) );
	//process
	// ROS_INFO("create_histgram");
	for(int k =0; k < clstr.data.size(); k++){//クラスタ数
		//静止障害物のみADD
		// double Vrx = cur_vel * cos( cur_angVel*delta_time*M_PI/180);
		// double Vry = cur_vel * sin( prev_tagAng*delta_time*M_PI/180);
		// double theta_o =  std::atan2(clstr.data[k].gc.y,clstr.data[k].gc.x);//robot
		// double theta_tf = theta_o +M_PI_2;
		// double dis = std::sqrt(std::pow(clstr.data[k].gc.y,2.0)+std::pow(clstr.data[k].gc.x,2.0));
		// double Vxo_rot_tf = - (dis * (-cur_angVel) * sin(theta_tf) );//*delta_time);
		// double Vyo_rot_tf = dis * (-cur_angVel) * cos(theta_tf);//*delta_time);
		// double Vxo_rot = -Vyo_rot_tf;
		// double Vyo_rot = Vxo_rot_tf;
		// double Vdif_x = Vrx + clstr.twist[k].linear.x;
		// double Vdif_y = Vry + clstr.twist[k].linear.y;
		//グローバル座標系での速度を算出
		// double vxr_r = cur_vel * sin( cur_angVel*delta_time*M_PI/180+M_PI_2);
		// double vyr_r = -cur_vel * cos( cur_angVel*delta_time*M_PI/180+M_PI_2);
		// double vxr_r = ( robotOdom.pose.pose.position.x - pre_robotOdom.pose.pose.position.x)/delta_time;
		// double vyr_r = ( robotOdom.pose.pose.position.y - pre_robotOdom.pose.pose.position.y)/delta_time;
		// double xr_o = clstr.data[k].gc.y;
		// double yr_o = -clstr.data[k].gc.x;
		// double vxr_o = clstr.twist[k].linear.y;// + vxr_r;
		// double vyr_o = -clstr.twist[k].linear.x;// + vyr_r;
		// double vxr_o = clstr.twist[k].twist.linear.y;// + vxr_r;
		// double vyr_o = -clstr.twist[k].twist.linear.x;// + vyr_r;
		// double yaw = std::atan2(robotOdom.pose.pose.position.y,robotOdom.pose.pose.position.x);
		//v rotation
		// double vx_og = -cur_angVel*sin(yaw)*xr_o + cos(yaw)*vxr_o
						// - cur_angVel*cos(yaw)*yr_o - sin(yaw)*vyr_o;
		// double vy_og = cur_angVel*cos(yaw)*xr_o + sin(yaw)*vxr_o 
						// - cur_angVel*sin(yaw)*yr_o + cos(yaw)*vyr_o;		
		//v linear
		// double vxr_g = vxr_o * cos(yaw) - vyr_o * sin(yaw);
		// double vyr_g = vxr_o * sin(yaw) + vyr_o * cos(yaw);
		// ROS_INFO("k: R= %f,%f, O = %f, %f, Rot = %f, %f",-vyr_r,vxr_r, clstr.twist[k].linear.x, clstr.twist[k].linear.y, -vyr_g,vxr_g);
		// if(clstr.twist[k].linear.x !=0 || clstr.twist[k].linear.y != 0
		if(clstr.twist[k].twist.linear.x !=0 || clstr.twist[k].twist.linear.y != 0
			// && std::sqrt(std::pow(-vyr_g,2.0)+std::pow(vxr_g,2.0)) > 0.1
			){
			// std::cout<<"("<<k<<" : moving),";
			continue;
		}
		// std::cout<<"("<<k<<" : static),";
		//各クラスタに含まれる点群を取得しヒストグラムを作成
		for(int m = 0; m < clstr.data[k].pt.size(); m++){
			float angleTemp = atan2(clstr.data[k].pt[m].y,clstr.data[k].pt[m].x)*180/M_PI;
			float disTemp = sqrt(clstr.data[k].pt[m].y*clstr.data[k].pt[m].y + clstr.data[k].pt[m].x*clstr.data[k].pt[m].x);
			add_histgram_dis(angleTemp, disTemp);
		}
	}
}
// apf
void trackingAvoidance::publish_debug_image(const cv::Mat& temp_image){
	// std::cout<<"1\n";
	cv_bridge::CvImagePtr publish_cvimage(new cv_bridge::CvImage);
	//cv::Mat temp=new cv::Mat(temp_image.rows,temp_image.cols, CV_8UC3);
	//temp=temp_image.clone();
	// std::cout<<"2\n";
	//cv::Mat temp=temp.clone();
	publish_cvimage->encoding=sensor_msgs::image_encodings::BGR8;
	// std::cout<<"3\n";
	// std::cout<<"cv::Size(temp_image.rows,temp_image.cols)"<<cv::Size(temp_image.rows,temp_image.cols)<<"\n";
	//publish_cvimage->image = cv::Mat::zeros(cv::Size(temp_image.rows,temp_image.cols), CV_8UC3);
	//cv::resize(publish_cvimage->image,publish_cvimage->image,cv::Size(temp_image.rows,temp_image.cols));
	//publish_cvimage->image=temp.clone();
	publish_cvimage->image=temp_image.clone();
	//publish_cvimage->image=temp;
	// std::cout<<"4\n";
	pub_img.publish(publish_cvimage->toImageMsg());
}
void trackingAvoidance::set_pub_debug_images(){
	int W=map_wi;
	int H=map_hi;
	for(int h0=0;h0<H;h0++){
		for(int w0=0;w0<W;w0++){
			//set potential
			float pot=pot_map.at<float>(h0,w0);//*std::abs(sum_pot);
			if(pot>0){
				debug_image.at<cv::Vec3b>(h0,w0)[2] =pot*255;
			}
			else{
				debug_image.at<cv::Vec3b>(h0,w0)[0] =(-pot)*255;
				debug_image.at<cv::Vec3b>(h0,w0)[1] =(-pot)*255;
			}
			//set path
			// ROS_INFO_STREAM("path"<<"\n");
		}	
	}
	debug_image.at<cv::Vec3b>(xri.y,xri.x)[0] =255;
	debug_image.at<cv::Vec3b>(xri.y,xri.x)[1] =255;
	debug_image.at<cv::Vec3b>(xri.y,xri.x)[2] =255;
	publish_debug_image(debug_image);
}
void trackingAvoidance::set_command_vel(cv::Point2i& xri0,float& v,float& w){
		float vx = grad_map[0].at<float>(xri0.y,xri0.x);
		float vy = grad_map[1].at<float>(xri0.y,xri0.x);
		float v0 = std::sqrt(vx*vx+vy*vy);
		vx /= v0;
		vy /= v0;
		//反時計回りを正(0~360)
		float th = std::atan2(vy,vx);
		float delta_th;
		//-180<th_t<180
		if(th_t>M_PI){
			th_t-=2*M_PI;
		}
		else if(th_t<-M_PI){
			th_t+=2*M_PI;
		}
		// 2*M_PI > th > M_PI && 0 < th_t < M_PI
		if(th<0&&th_t>0){
			delta_th= th + 2*M_PI - th_t;
		}
		else if(th>0&&th_t<0){
			delta_th= th - (th_t+2*M_PI);
		}
		else{
			delta_th= th -th_t;
		}
		delta_th=th-th_t;
		if(delta_th<-M_PI){
			delta_th+=2*M_PI;
		}
		else if(delta_th>M_PI){
			delta_th-=2*M_PI;
		}
		//角速度(P制御)
		float Kp=1;
		w=Kp*delta_th;
		if(w>max_w){
			w=max_w;
		}
		else if(w<-max_w){
			w=-max_w;
		}
		//速度可変
		//float d=0.138;
		//float dif_v = w*2*d;
		//v=vrt-std::abs(dif_v)/2;
		//速度は一定
		v=vrt;
	// dynamixel_sdk_examples::SetPosition pubPanData;
	// dynamixel_sdk_examples::SetPosition pubTiltData;
	// pan id 2 center 180 2048 right 117.95 1400 left 241.69 2800 
	// tilt id 1 center 180 2048 right 146.25 1650 left 212.17 2450 
	pubPanData.id = 2;
	pubPanData.position = 2048 + w*180/M_PI;
	pubTiltData.id = 1;
	pubTiltData.position = 2048;
	pub_pan.publish(pubPanData);
	// ROS_INFO("published_debug_pantilt\n");
	pub_tilt.publish(pubTiltData);
}
void trackingAvoidance::trans_point_f_to_i(const cv::Point2f& ptf,cv::Point2i& pti){
	float map_ptx = map_wf/2 + ptf.x;
	float map_pty = map_hf/2 - ptf.y;
	pti.x = (int)(map_ptx/reso);
	pti.y = (int)(map_pty/reso);
}
void trackingAvoidance::draw_path_mat(){
	// ROS_INFO_STREAM("ros::ok() =" <<ros::ok()<<"\n");
	ros::NodeHandle n;
	ros::Rate rate(100);
	while(ros::ok()){
		//float to int
		trans_point_f_to_i(xrf,xri);
		// std::cout<<"xrf,xgf:"<<xrf<<"-->"<<xgf<<"\n";
		//ゴールセルに到達したら終了
		if(xri.x==xgi.x && xri.y==xgi.y){
			// std::cout<<"Goal\n";
			break;
		}
		//ロボットの命令速度算出
		float w,v;
		set_command_vel(xri,v,w);
		// std::cout<<"v,w,th_t:"<<v<<","<<w<<","<<th_t<<"\n";
		//ロボットの移動
		//mv_t:移動時間
		float l=v*mv_t;
		th_t=th_t+w*mv_t;
		xrf.x=xrf.x + l*cos(th_t);
		xrf.y=xrf.y + l*sin(th_t);
		//debug
		// ROS_INFO_STREAM("test1"<<"\n");
		// set_pub_debug_images();重い
		// ROS_INFO_STREAM("test2"<<"\n");
		rate.sleep();				
	}
}
float trackingAvoidance::get_grad_1(float& x0,float& x1,float& delta){
	return (x1-x0)/delta;
}
float trackingAvoidance::get_grad_2(float& x0,float& x1,float& delta){
	return (x1-x0)/(2*delta);
}
void trackingAvoidance::create_grad_map(){
	//map size
	int W=map_wi;
	int H=map_hi;
	//gradient
	float grad_x;
	float grad_y;
	int delta=1;
	float delta_cell=delta*reso;
	int ch_p = pot_map.channels();
	int ch_gr0 = grad_map[0].channels();
	int ch_gr1 = grad_map[1].channels();
	for(int h=0;h<H;h++){
		float *ppot = pot_map.ptr<float>(h);
		float *ppot_pd = pot_map.ptr<float>(h+delta);
		float *ppot_md = pot_map.ptr<float>(h-delta);
		float *pgrad0 = grad_map[0].ptr<float>(h);
		float *pgrad1 = grad_map[1].ptr<float>(h);
		for(int w=0;w<W;w++){
			if(h==0){
				grad_y=	-get_grad_1(ppot_pd[w*ch_p],ppot[w*ch_p],delta_cell);
			}
			else if(h==H-delta){
				grad_y=	-get_grad_1(ppot[w*ch_p],ppot_md[w*ch_p],delta_cell);
			}
			else{
				grad_y=	-get_grad_2(ppot_pd[w*ch_p],ppot_md[w*ch_p],delta_cell);
			}
			if(w==0){
				grad_x=	get_grad_1(ppot[(w+delta)*ch_p],ppot[w*ch_p],delta_cell);
			}
			else if(w==W-delta){
				grad_x=get_grad_1(ppot[w*ch_p],ppot[(w-delta)*ch_p],delta_cell);
			}
			else{
				grad_x=get_grad_2(ppot[(w+delta)*ch_p],ppot[(w-delta)*ch_p],delta_cell);
			}
			pgrad0[(w)*ch_gr0]=grad_x;
			pgrad1[(w)*ch_gr1]=grad_y;	
		}
	}
}
float trackingAvoidance::culc_fa(const int& w0,const int& h0,const int& w1,const int& h1){
	float del_w=(float)(w0-w1)*reso;
	float del_h=(float)(h0-h1)*reso;
	float dis=std::sqrt( del_w*del_w + del_h*del_h );
	float weight=1;
	float eps=0.1;
	if(1/(dis+eps)>max_pot){ 
		return (-max_pot);
	}
	else{
		return (-1/(dis+eps) )*weight;	
	}
}
float trackingAvoidance::culc_fa(float dis){
	float weight=1;
	float eps=0.1;
	if(1/(dis+eps)>max_pot){ 
		return (-max_pot);
	}
	else{
		return (-1/((dis+eps)) )*weight;	
	}
}
float trackingAvoidance::culc_fr(const float& dis,const int& obstNum){
	float weight=0.01;
	float safe_rate=3;
	float margin=0.1;
	float eps=0.1;
	for(int i=0;i<safe_rate;i++){	
		if(dis>(rr+cr+margin)*(safe_rate-i)){
			return (1/ (dis+eps))*weight*i;
		}
	}
	if(dis>(rr+cr)){
		return (1/ (dis+eps))*max_pot/2*obstNum;
	}
	/*if(dis>(rr+cr+margin)){
		return (1/ (dis+eps))*weight;
	}*/
	return ((1/ (dis+eps))*max_pot*obstNum);
}
float trackingAvoidance::culc_dis(const int& w0,const int& h0,const int& w1,const int& h1){
	float del_w=(float)(w0-w1)*reso;
	float del_h=(float)(h0-h1)*reso;
	return std::sqrt( del_w*del_w + del_h*del_h );
}
void trackingAvoidance::search_obst_pt(){
	int W=map_wi;
	int H=map_hi;
	obst_pti.resize(W*H);
	int ch_g = grid_map.channels();
	int vsize=0;
	for(int h=0;h<H;h++){
		uint8_t *pgrid = grid_map.ptr<uint8_t>(h);
		for(int w=0;w<W&&ros::ok();w++){
			if(pgrid[w * ch_g]>=1)
			{
				obst_pti[vsize++]=cv::Point2i(w,h);
			}
		}	
	}
	obst_pti.resize(vsize);
}
void trackingAvoidance::create_pot_map(){
	int W=map_wi;
	int H=map_hi;
	float sum_potf=0;
	float sum_pota=0;
	int ch_g = grid_map.channels();
	int ch_p = pot_map.channels();
	search_obst_pt();
	int obst_num=(int)obst_pti.size();
	for(int h0=0;h0<H;h0++){
		float *ppot = pot_map.ptr<float>(h0);
		for(int w0=0;w0<W;w0++){
			//斥力算出
			bool break_flag=false;
			/*for(int h=0;h<H;h++){
				uint8_t *pgrid = grid_map.ptr<uint8_t>(h);
				for(int w=0;w<W&&ros::ok();w++){
					if(h0==h&&w0==w){
						continue;
					}
					//if(grid_map.at<uint8_t>(h,w)>=1)
					if(pgrid[w * ch_g]>=1){
						//L2
						float dis=culc_dis(w0,h0,w,h);
						//pot_map.at<float>(h0,w0)+=culc_fr(dis,obst_num);
						ppot[w0 * ch_p]+=culc_fr(dis,obst_num);
						if(dis<(rr+cr)){
							break_flag=true;
							break;
						}
					}					
				}
				if(break_flag)
					break;
			}*/
			for(int k=0;k<obst_pti.size();k++){
				float dis=culc_dis(w0,h0,obst_pti[k].x,obst_pti[k].y);
				//pot_map.at<float>(h0,w0)+=culc_fr(dis,obst_num);
				if(dis<(rr+cr)){
					ppot[w0 * ch_p]=culc_fr(dis,obst_num);
					break_flag=true;
					break;
				}
				ppot[w0 * ch_p]+=culc_fr(dis,obst_num);
			}
			if(break_flag)
				continue;
			if(obst_num>0){
				//pot_map.at<float>(h0,w0)/=obst_num;
				ppot[w0 * ch_p]/=obst_num;
			}
			//引力算出
			//std::cout<<"fa:"<<-culc_fa(w0,h0,xgi.x,xgi.y)<<"\n";
			//pot_map.at<float>(h0,w0)+=culc_fa(culc_dis(w0,h0,xgi.x,xgi.y));
			ppot[w0 * ch_p]+=culc_fa(culc_dis(w0,h0,xgi.x,xgi.y));
			//sum_pot+=pot_map.at<float>(h0,w0);
			//std::cout<<"pot_map,sum_pot:"<<pot_map.at<float>(h0,w0)<<","<<sum_pot<<"\n";
		}
	}
}
// global point-> grid point
bool trackingAvoidance::trans_point(const cv::Point2f& pt,cv::Point2i& pti){
	float map_ptx = map_wf/2 + (pt.x - cx);
	float map_pty = map_hf/2 + ( -(pt.y - cy) );
	// float map_pty = map_hf/2 + ( -(pt.y - cy) );
	if(map_ptx<0 || map_ptx>map_wf)
		return false;	
	if(map_pty<0 || map_pty>map_hf)
		return false;	
	pti.x =	(int)(map_ptx/reso);
	pti.y =	(int)(map_pty/reso);
	return true;
}
void trackingAvoidance::set_obstacle_data(const cv::Point2f& data){
	cv::Point2i data_gp;
	if(trans_point(data,data_gp)){
		int ch_g = grid_map.channels();
		uint8_t *pgrid = grid_map.ptr<uint8_t>(data_gp.y);
		pgrid[data_gp.x * ch_g]++;
	}
	else{
		// std::cout<<"ObstPoint is not in grid map\n";
	}
}
void trackingAvoidance::set_mov_time(float time){
	mv_t=time;
}
void trackingAvoidance::set_command_limit(float dif_vel){
	// float d=0.138;
	// float d=0.314;
	float wheel_d=0.310;
	max_w=dif_vel/(2*wheel_d);//dif_vel=vr-vl
}
bool trackingAvoidance::set_robot_param(float x,float y, float r,float vt0,float th_t0){
	xr.x=x;
	xr.y=y;
	rr=r;
	vrt=vt0;
	th_t=th_t0;//反時計回り(0~360)
	if(trans_point(xr,xri,xrf)){
		//std::cout<<"xr,xri,xrf:"<<xr<<","<<xri<<","<<xrf<<"\n";
		return true;
	}
	else{
		// std::cout<<"RobotPoint is not in grid map\n";
		return false;
	}	
}
//global point-> grid point float and int
bool trackingAvoidance::trans_point(const cv::Point2f& pt,cv::Point2i& pti,cv::Point2f& ptf){
	ptf.x = pt.x - cx;
	ptf.y = pt.y - cy;
	float map_ptx = map_wf/2 + ptf.x;
	float map_pty = map_hf/2 - ptf.y;
	if(map_ptx<0 || map_ptx>map_wf)
		return false;	
	if(map_pty<0 || map_pty>map_hf)
		return false;	
	pti.x =	(int)(map_ptx/reso);
	pti.y =	(int)(map_pty/reso);
	return true;
}
void trackingAvoidance::set_goal(cv::Point2f& goal_2f){
	if(trans_point(goal_2f,xgi,xgf)){}
	else{
		// std::cout<<"GoalPoint is not in grid map\n";
	}
}
//グリッドマップの中心座標（クローバル座標）
void trackingAvoidance::set_center_point(float cpx,float cpy){
	cx=cpx;
	cy=cpy;
}
void trackingAvoidance::set_grid_param(float width,float height,float resolution){
    //マップデータの設定と初期化
	map_wf=width;
	map_hf=height;
	reso=resolution;
	map_wi=(int)(map_wf/resolution);
	map_hi=(int)(map_hf/resolution);
	cr=std::sqrt(2)*reso/2;
	//cv::Mat
	//int
	cv::Mat m_temp = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC1);
	grid_map=m_temp.clone();
	//float
	m_temp = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_32FC1);
	pot_map=m_temp.clone();
	pot_mapt=m_temp.clone();
	grad_map[0]=m_temp.clone();
	grad_map[1]=m_temp.clone();
	debug_image = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC3);
}
void trackingAvoidance::apf(){
   	float H=6;
	float W=6;
	float reso=0.1;
    // apf
	set_grid_param(W,H,reso);
	// std::cout<<"wait...\n";
	//center point
	cv::Point2f cpt=cv::Point(0.0,0.0);
	//goal point
	// cv::Point2f goal_pt=cv::Point2f(7.5-5,7.5-5);
	cv::Point2f goal_pt=cv::Point2f(0,3.0);
	//set_param
	// std::cout<<"set_param...\n";
	set_center_point(cpt.x,cpt.y);
	set_goal(goal_pt);
	//--set_robot_param(float x,float y, float r,float vt0,float th_t0)
	// if(!set_robot_param(-2.5,-2.5,0.3,0.2,0.0))//-M_PI/2))
	if(!set_robot_param(0.0,0.0,0.3,0.2,0.0)){
		// std::cout<<"Error: robot param\n";
		// return -1; 
    }
	//--set_command_limit(float dif_vel)
	set_command_limit(0.2);
	set_mov_time(0.05);
	/*//grid_map
	std::cout<<"grid_map...\n";
	cv::Point2f obst_data=cv::Point2f(0.0,0.75);
	cv::Point2f obst_data2=cv::Point2f(2.3,1.5);
	cv::Point2f obst_data3=cv::Point2f(-2.3,1.5);
	// cv::Point2f obst_data=cv::Point2f(0.0,0.0);
	// cv::Point2f obst_data2=cv::Point2f(2.0,1.8);
	// cv::Point2f obst_data3=cv::Point2f(-1.0,1.0);
	cv::Point2f obst_data4=cv::Point2f(1.0,-0.5);
	cv::Point2f obst_data5=cv::Point2f(2.0,-1.0);
	cv::Point2f obst_data6=cv::Point2f(-2.0,1.0);
	cv::Point2f obst_data7=cv::Point2f(3.5,-2.0);
	// int obst_num=6;*/
	// grid_map
	// std::cout<<"grid_map...\n";
	cv::Point2f obst_data=cv::Point2f(0.0,0.75);
	cv::Point2f obst_data2=cv::Point2f(2.3,1.5);
	cv::Point2f obst_data3=cv::Point2f(-2.3,1.5);
	// cv::Point2f obst_data=cv::Point2f(0.0,0.0);
	// cv::Point2f obst_data2=cv::Point2f(2.0,1.8);
	// cv::Point2f obst_data3=cv::Point2f(-1.0,1.0);
	cv::Point2f obst_data4=cv::Point2f(1.0,-0.5);
	cv::Point2f obst_data5=cv::Point2f(2.0,-1.0);
	cv::Point2f obst_data6=cv::Point2f(-2.0,1.0);
	cv::Point2f obst_data7=cv::Point2f(3.5,-2.0);
	// int obst_num=6;
	//--set_obstacle_data(cv::Point2f& data)
	set_obstacle_data(obst_data);
	set_obstacle_data(obst_data2);
	set_obstacle_data(obst_data3);
	// set_obstacle_data(obst_data4);
	// set_obstacle_data(obst_data5);
	// set_obstacle_data(obst_data6);
	// set_obstacle_data(obst_data7);
	//potential_map
	// std::cout<<"potential_map...\n";
	create_pot_map();
	//gradient_map
	// std::cout<<"gradient_map...\n";
	create_grad_map();
	//path_planning
	// std::cout<<"path_planning...\n";
	draw_path_mat();	
	// std::cout<<"Done\n";
}
void trackingAvoidance::draw_mv_obst(){
	//std::cout<<"void trackingAvoidance::draw_mv_obst(){\n";
	//std::cout<<"mv_obsts.size():"<<mv_obsts.size()<<"\n";
	for(int n=0;n<mv_obsts.size();n++){
		cv::Point2i pti;
		//std::cout<<"mv_obsts[n].data.size():"<<mv_obsts[n].data.size()<<"\n";
		for(int k=0;k<mv_obsts[n].data.size();k++){
			cv::Point2f pt=mv_obsts[n].data[k];
			//std::cout<<"pt0:"<<pt<<"\n";
			pt.x+=mv_obsts[n].mvx;
			pt.y+=mv_obsts[n].mvy;
			pt.x+=mv_obsts[n].mvxt;
			pt.y+=mv_obsts[n].mvyt;
			//std::cout<<"pt:"<<pt<<"\n";
			if(trans_point(pt,pti)){
				//std::cout<<"pti:"<<pti<<"\n";
				mpc_debug_image.at<cv::Vec3b>(pti.y,pti.x)[1] =255;
			}
		}	
	}
}
void trackingAvoidance::set_pub_mpc_debug_images(const cv::Point2i& xrit0){
	int W=map_wi;
	int H=map_hi;
	//mpc_debug_image = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC3);
	for(int h0=0;h0<H;h0++){
		for(int w0=0;w0<W;w0++){
			//set potential
			//float pot=pot_mapt.at<float>(h0,w0);//*std::abs(sum_pot);
			float pot=pot_map.at<float>(h0,w0);//*std::abs(sum_pot);
			//std::cout<<"pot:"<<pot<<"\n";
			if(pot>0){
				mpc_debug_image.at<cv::Vec3b>(h0,w0)[2] =pot*255;
			}
			else{
				mpc_debug_image.at<cv::Vec3b>(h0,w0)[0] =(-pot)*255;
				mpc_debug_image.at<cv::Vec3b>(h0,w0)[1] =(-pot)*255;
			}
			//set path
		}	
	}
	mpc_debug_image.at<cv::Vec3b>(xrit0.y,xrit0.x)[0] =255;
	mpc_debug_image.at<cv::Vec3b>(xrit0.y,xrit0.x)[1] =255;
	mpc_debug_image.at<cv::Vec3b>(xrit0.y,xrit0.x)[2] =255;
	draw_mv_obst();
	publish_debug_image(mpc_debug_image);
	// ROS_INFO("published_debug_image\n");	
}
void trackingAvoidance::past_time(const float& time){
	//std::cout<<"void trackingAvoidance::past_time(const float& time){\n";
	clear_move_data();
	for(int n=0;n<mv_obsts.size();n++){
		/*float mvx=mv_obsts[n].vx*time;
		float mvy=mv_obsts[n].vy*time;
		for(int k=0;k<mv_obsts[n].data.size();k++)
		{
			mv_obsts[n].data[k].x+=mvx;
			mv_obsts[n].data[k].y+=mvy;
		}*/
		mv_obsts[n].mvxt+=mv_obsts[n].vx*time;
		mv_obsts[n].mvyt+=mv_obsts[n].vy*time;
	}
	//std::cout<<"void trackingAvoidance::past_time(const float& time){\n";
}
bool trackingAvoidance::check_collision(const cv::Point2f xrf00){
	for(int n=0;n<mv_obsts.size();n++){
		cv::Point2i pti;
		//std::cout<<"mv_obsts[n].data.size():"<<mv_obsts[n].data.size()<<"\n";
		for(int k=0;k<mv_obsts[n].data.size();k++){
			cv::Point2f pt=mv_obsts[n].data[k];
			//std::cout<<"pt0:"<<pt<<"\n";
			pt.x-=cx;
			pt.y-=cy;
			pt.x+=mv_obsts[n].mvx;
			pt.y+=mv_obsts[n].mvy;
			pt.x+=mv_obsts[n].mvxt;
			pt.y+=mv_obsts[n].mvyt;
			//std::cout<<"pt:"<<pt<<"\n";
			float dis=std::sqrt((xrf00.x-pt.x)*(xrf00.x-pt.x)+(xrf00.y-pt.y)*(xrf00.y-pt.y));
			if(dis<=rr+cr){
				return true;
			}
		}	
	}
	for(int k=0;k<obst_pti.size();k++){
		float dis=std::sqrt((xrf00.x-obst_pti[k].x)*(xrf00.x-obst_pti[k].x)+(xrf00.y-obst_pti[k].y)*(xrf00.y-obst_pti[k].y));
		if(dis<rr+cr){
			return true;
		}
	}
	return false;
}
bool trackingAvoidance::set_grad(const cv::Point2i& xti){
	//map size
	int W=map_wi;
	int H=map_hi;
	int delta=1;
	float delta_cell=delta*reso;
	cv::Point2i yti_0 =xti;
	cv::Point2i yti_1 =xti;
	//pot_xt0=pot_mapt.at<float>(xti.y,xti.x);
	if(xti.y==0){
		yti_1.y+=delta;
		pot_y1=pot_mapt.at<float>(xti.y+delta,xti.x);
		pot_y0=pot_mapt.at<float>(xti.y,xti.x);
		grad_yt=-get_grad_1(pot_y1,pot_y0,delta_cell);
	}
	else if(xti.y==H-delta){
		yti_0.y-=delta;
		pot_y1=pot_mapt.at<float>(xti.y,xti.x);
		pot_y0=pot_mapt.at<float>(xti.y-delta,xti.x);
		grad_yt=-get_grad_1(pot_y1,pot_y0,delta_cell);
	}
	else{
		yti_1.y+=delta;
		yti_0.y-=delta;	
		pot_y1=pot_mapt.at<float>(xti.y+delta,xti.x);
		pot_y0=pot_mapt.at<float>(xti.y-delta,xti.x);
		grad_yt=-get_grad_2(pot_y1,pot_y0,delta_cell);
	}
	cv::Point2i xti_0 =xti;
	cv::Point2i xti_1 =xti;
	if(xti.x==0){
		xti_1.x+=delta;	
		pot_x1=pot_mapt.at<float>(xti.y,xti.x+delta);
		pot_x0=pot_mapt.at<float>(xti.y,xti.x);
		grad_xt=get_grad_1(pot_x1,pot_x0,delta_cell);
	}
	else if(xti.x==W-delta){
		xti_0.x-=delta;
		pot_x1=pot_mapt.at<float>(xti.y,xti.x);
		pot_x0=pot_mapt.at<float>(xti.y,xti.x-delta);
		grad_xt=get_grad_1(pot_x1,pot_x0,delta_cell);
	}
	else{
		xti_1.x+=delta;	
		xti_0.x-=delta;
		pot_x1=pot_mapt.at<float>(xti.y,xti.x+delta);
		pot_x0=pot_mapt.at<float>(xti.y,xti.x-delta);
		grad_xt=get_grad_2(pot_x1,pot_x0,delta_cell);
	}
	if(std::isinf(grad_xt)){
		if(grad_xt>0)
			grad_xt=FLT_MAX;
		else
			grad_xt=-FLT_MAX;
	}
	if(std::isinf(grad_yt)){
		if(grad_yt>0)
			grad_yt=FLT_MAX;
		else
			grad_yt=-FLT_MAX;
	}
	double th_pot=max_pot*((int)obst_pti.size()+mv_data_size);
	if(pot_x0>th_pot||pot_x1>th_pot||pot_y0>th_pot||pot_y1>th_pot){
		return true;
	}
	return false;
}
void trackingAvoidance::move_obstacle_data(float& time){
	for(int k=0;k<mv_obsts.size();k++){
		mv_obsts[k].mvx+=mv_obsts[k].vx*time;
		mv_obsts[k].mvy+=mv_obsts[k].vy*time;
	}
}
void trackingAvoidance::set_command_vel(const cv::Point2i& xri0,const float& v0,float& v,float& w,float& th_t0){
	float vx = grad_xt;
	float vy = grad_yt;
	float v00 = std::sqrt(vx*vx+vy*vy);
	vx /= v00;
	vy /= v00;
	//反時計回りを正(0~360)
	//std::cout<<"grad_xt,yt:"<<grad_xt<<","<<grad_yt<<"\n";
	float th = std::atan2(vy,vx);
	float delta_th;
	//-180<th_t<180
	if(th_t0>M_PI){
		th_t0-=2*M_PI;
	}
	else if(th_t0<-M_PI){
		th_t0+=2*M_PI;
	}
	// 2*M_PI > th > M_PI && 0 < th_t < M_PI
	if(th<0&&th_t0>0){
		delta_th= th + 2*M_PI - th_t0;
	}
	else if(th>0&&th_t0<0){
		delta_th= th - (th_t0+2*M_PI);
	}
	else{
		delta_th= th -th_t0;
	}
	delta_th=th-th_t0;
	if(delta_th<-M_PI){
		delta_th+=2*M_PI;
	}
	else if(delta_th>M_PI){
		delta_th-=2*M_PI;
	}
	//limitを速度の大きさと設定
	//set_command_limit(v0/2);
	//角速度(P制御)
	float Kp=1;
	w=Kp*delta_th;
	//std::cout<<"th,th_t0:"<<th<<","<<th_t0<<"\n";
	//std::cout<<"delta_th,w:"<<delta_th<<","<<w<<"\n";
	if(w>max_w){
		w=max_w;
	}
	else if(w<-max_w){
		w=-max_w;
	}
	//速度可変
	float d=0.150;
	// float d=0.138;
	float dif_v = w*2*d;
	//v=v0-std::abs(dif_v)/2;
	//速度可変
	v=v0;	
}
float& trackingAvoidance::get_pot_xt(const cv::Point2i& xti){
	return pot_mapt.at<float>(xti.y,xti.x);
}
//obstacle position:pt,robot position:xti
float trackingAvoidance::culc_mv_obstacle_fr(const cv::Point2i xti,const int& obstNum){
	float sum_mvfr=0;
	bool break_flag=false;
	for(int n=0;n<mv_obsts.size();n++){
		cv::Point2i pti;
		for(int k=0;k<mv_obsts[n].data.size();k++){
			cv::Point2f pt=mv_obsts[n].data[k];
			pt.x+=mv_obsts[n].mvx;
			pt.y+=mv_obsts[n].mvy;
			pt.x+=mv_obsts[n].mvxt;
			pt.y+=mv_obsts[n].mvyt;
			if(trans_point(pt,pti)){
				float dis=culc_dis(xti.x,xti.y,pti.x,pti.y);
				sum_mvfr+=culc_fr(dis,obstNum);
				if(dis<(rr+cr)){
					break_flag=true;
					break;
				}
			}
			else{
				sum_mvfr+=0;
			}
		}
		if(break_flag){
			break;
		}
	}
	if(mv_data_size>0)
		sum_mvfr/=obstNum;
	return sum_mvfr;	
}
void trackingAvoidance::add_mv_pot(const cv::Point2i xti,const int& obstNum){
	int W=map_wi;
	int H=map_hi;
	int delta=1;
	cv::Point2i yti0=xti;
	cv::Point2i xti0=xti;
	//ROS_INFO("culc_mv_obstacle_fr...add_mv_pot\n");
	//std::cout<<"xti:"<<xti<<"\n";
	pot_mapt.at<float>(xti.y,xti.x)+=culc_mv_obstacle_fr(xti,obstNum);
	//ROS_INFO("if(xti.y==0)...add_mv_pot\n");
	if(xti.y==0){
		yti0.y+=delta;
		pot_mapt.at<float>(yti0.y,yti0.x)+=culc_mv_obstacle_fr(yti0,obstNum);	
	}
	else if(xti.y==H-delta){
		yti0.y-=delta;
		pot_mapt.at<float>(yti0.y,yti0.x)+=culc_mv_obstacle_fr(yti0,obstNum);		
	}
	else{
		yti0.y=xti.y+delta;
		pot_mapt.at<float>(yti0.y,yti0.x)+=culc_mv_obstacle_fr(yti0,obstNum);	
		yti0.y=xti.y-delta;
		pot_mapt.at<float>(yti0.y,yti0.x)+=culc_mv_obstacle_fr(yti0,obstNum);			
	}
	//ROS_INFO("if(xti.x==0)...add_mv_pot\n");
	if(xti.x==0){
		xti0.x+=delta;
		pot_mapt.at<float>(xti0.y,xti0.x)+=culc_mv_obstacle_fr(xti0,obstNum);	
	}
	else if(xti.x==W-delta){
		xti0.x-=delta;
		pot_mapt.at<float>(xti0.y,xti0.x)+=culc_mv_obstacle_fr(xti0,obstNum);	
	}
	else{
		xti0.x=xti.x+delta;
		pot_mapt.at<float>(xti0.y,xti0.x)+=culc_mv_obstacle_fr(xti0,obstNum);	
		xti0.x=xti.x-delta;
		pot_mapt.at<float>(xti0.y,xti0.x)+=culc_mv_obstacle_fr(xti0,obstNum);	
	}
}
double trackingAvoidance::culc_cost(cv::Point2f& xrft0,const float v0,const float& time_range){
	ros::NodeHandle n;
	ros::Rate rate(1000);
	double sum_cost=0;
	float tr=time_range;
	cv::Point2f xrft=xrft0;
	cv::Point2i xrit=xri;
	float th_t0=th_t;
	// init mv data
	clear_move_data();
	int obst_num=(int)obst_pti.size()+mv_data_size;
	//ROS_INFO("culc_cost...while\n");
	//std::cout<<"xrft0,xrft:"<<xrft0<<","<<xrft<<"\n";
	//std::cout<<"xrit,xri:"<<xrft0<<","<<xrft<<"\n";
	ROS_INFO_STREAM("ros::ok()3 =" <<ros::ok()<<"\n");
	float vrate=2;
	while(ros::ok()&&tr>0){
		//float to int
		trans_point_f_to_i(xrft,xrit);
		//std::cout<<"xrft,xgf:"<<xrft<<"-->"<<xgf<<"\n";
		//std::cout<<"xrit:"<<xrit<<"\n";
		// std::cout<<"i:"<<i++<<"\n";
		//set pot map(t)
		pot_mapt=pot_map.clone();
		// ROS_INFO("add_mv_pot...while\n");
		add_mv_pot(xrit,obst_num);
		//add cost
		//ROS_INFO("sum_cost...while\n");
		//std::cout<<"sum_cost:"<<sum_cost<<"\n";
		ROS_INFO_STREAM("sum_cost "<<sum_cost<<"\n");
		sum_cost+=get_pot_xt(xrit);
		//ゴールセルに到達したら終了
		if(xrit.x==xgi.x && xrit.y==xgi.y){
			// std::cout<<"Goal\n";
			if(sum_cost>0){
				sum_cost/=vrate;
			}
			else{
				sum_cost*=vrate;
			}
			return -DBL_MAX*(tr/time_range);
		}
		// ROS_INFO("set_grad...while\n");
		//collision
		if(set_grad(xrit)){
			// ROS_INFO("collision...MPC\n");
			return DBL_MAX-(time_range-tr);
		}
		//std::cout<<"FLT_MAX,isninf x,y:"<<FLT_MAX<<","<<std::isinf(grad_xt)<<","<<std::isinf(grad_yt)<<"\n";
		//ロボットの命令速度算出
		float w,v;
		set_command_vel(xrit,v0,v,w,th_t0);
		//std::cout<<"v,w,th_t0:"<<v<<","<<w<<","<<th_t0<<"\n";
		//ロボットの移動
		//mv_t:移動時間
		float l=v*mv_t;
		th_t0=th_t0+w*mv_t;
		xrft.x=xrft.x + l*cos(th_t0);
		xrft.y=xrft.y + l*sin(th_t0);
		//障害物の移動
		//ROS_INFO("move_obstacle_data...while\n");
		move_obstacle_data(mv_t);
		//debug
		//ROS_INFO("set_pub_mpc_debug_images()\n");
		//set_pub_mpc_debug_images(xrit);
		/*if(tr==time_range){
			std::cout<<"in cost:grad(x,y,w):"<<grad_xt<<","<<grad_yt<<","<<w<<"\n";
			std::cout<<"xrit:"<<xrit<<"\n";
			std::cout<<"pot_xrit:"<<get_pot_xt(xrit)<<"\n";
			std::cout<<"pot(x0,x1),(y0,y1):("<<pot_x0<<","<<pot_x1<<"),("<<pot_y0<<","<<pot_y1<<")\n";
			pot_maptt=pot_mapt.clone();
			std::cout<<"w in mpc:"<<w<<"\n";
		}*/
		//rate.sleep();
		tr-=mv_t;
		ROS_INFO_STREAM("time_range"<<tr<<"\n");
	}
	/*cv::Point2f del=xrft-xrft0;
	vrate=std::abs(del.x)+std::abs(del.y);
	if(sum_cost>0){
		sum_cost/=vrate;
	}
	else{
		sum_cost*=vrate;
	}*/
	// ROS_INFO_STREAM("sum_cost"<<sum_cost<<"\n");
	return sum_cost;
}
float trackingAvoidance::get_speed(const cv::Point2f& xrft0,const float& vrt00){
	cv::Point2f xrft=xrft0;
	cv::Point2i xrit;
	float vrt0=vrt00;//
	float delta_v=0.01;
	float vrt1=vrt00+delta_v;
	float max_v=0.3;
	float min_v=0.1;
	double cost0=0;
	double cost1=0;
	float time_range=1;
	// float time_range=10;
	float opt_v;
	//std::cout<<"vrt0:"<<vrt0<<"\n";
	if(vrt1>=max_v){
		vrt0=vrt00-delta_v;
		vrt1=vrt00;//+delta_v;
	}
	// Process Once
	/*trans_point_f_to_i(xrft,xrit);
	if(xrit.x==xgi.x && xrit.y==xgi.y)
	{
		std::cout<<"Goal\n";
		return 0;
	}*/
	//std::cout<<"xrft0,xrft:"<<xrft0<<","<<xrft<<"\n";
	// ROS_INFO("culc_cost0...\n");
	cost0=culc_cost(xrft,vrt0,time_range);
	// ROS_INFO("culc_cost1...\n");
	cost1=culc_cost(xrft,vrt1,time_range);
	//predict param
	// int search_num=20;
	int search_num=1;
	// float pot_th=0.10;//10%
	// float pot_rate;
	//gradient v
	// float grad_v;
	bool flag01=false;
	bool flag10=false;
	ROS_INFO_STREAM("ros::ok()2 =" <<ros::ok()<<"\n");
	//ROS_INFO("while...\n");
	while(search_num>0&&ros::ok()){
		search_num--;
		// grad_v=(cost1-cost0)/delta_v;
		/*pot_rate=std::abs((cost1-cost0)/cost0);
		std::cout<<"pot_rate:"<<pot_rate<<"\n";
		if(pot_rate<pot_th){
			break;
		}*/
		//保留
		//if(grad_v>0){//cost0<cost1
		//std::cout<<"vrt("<<vrt0<<","<<vrt1<<")\n";
		//std::cout<<"cost("<<cost0<<","<<cost1<<")\n";
		std::cout<<"opt_v("<<opt_v<<")"<<"search_num("<<search_num<<"\n";
		if(cost0<=cost1){
			if(vrt0>=max_v){
				return max_v;
			}
			if(vrt0<=min_v){
				return min_v;
			}
			//ROS_INFO("vrt0<vrt1:(%f,%f)\n",vrt0,vrt1);
			opt_v=vrt0;
			vrt1=vrt0;
			cost1=cost0;
			vrt0=vrt0-delta_v;
			cost0=culc_cost(xrft,vrt0,time_range);
			flag01=true;
		}
		else{//cost1<cost0:vrt1<vrt0
			if(vrt1>=max_v){
				return max_v;
			}
			if(vrt1<=min_v){
				return min_v;
			}
			//ROS_INFO("vrt0>vrt1:(%f,%f)\n",vrt0,vrt1);
			opt_v=vrt1;
			vrt0=vrt1;
			cost0=cost1;
			vrt1=vrt0+delta_v;
			cost1=culc_cost(xrft,vrt1,time_range);	
			flag10=true;
		}
		if(flag01&&flag10)
			break;			
	}
	return opt_v;
}
void trackingAvoidance::clear_move_data(){
	for(int k=0;k<mv_obsts.size();k++){
		mv_obsts[k].mvx=0;
		mv_obsts[k].mvy=0;
	}
}
void trackingAvoidance::draw_mpc_path_mat(){
	ros::NodeHandle n;
	ros::Rate rate(100);
	int obst_num=(int)obst_pti.size()+mv_data_size;
	clear_move_data();
	float v0=vrt;
	//float dt=(float)1/100;
	float goal_time=0;
	ROS_INFO_STREAM("ros::ok()1 =" <<ros::ok()<<"\n");
	// while(ros::ok()){
		//float to int
		trans_point_f_to_i(xrf,xri);
		// std::cout<<"xrf,xgf:"<<xrf<<"-->"<<xgf<<"\n";
		//ゴールセルに到達したら終了
		if(xri.x==xgi.x && xri.y==xgi.y){
			std::cout<<"Goal\n";
			// break;
		}
		//MPC
		//ROS_INFO("get_speed...\n");
		//std::cout<<"vrt:"<<vrt<<"\n";
		v0=get_speed(xrf,vrt);
		//pot_mapt=pot_map.clone();
		// std::cout<<"v0:"<<v0<<"\n";
		/*int kk=0;
		while(ros::ok()&&kk++<100){
			set_pub_mpc_debug_images(xri);
			rate.sleep();
		}
		clear_move_data();
		kk=0;
		while(ros::ok()&&kk++<100){
			set_pub_mpc_debug_images(xri);
			rate.sleep();
		}*/
		clear_move_data();
		//pot_mapt=pot_map.clone();
		//ROS_INFO("add_mv_pot...\n");
		add_mv_pot(xri,obst_num);
		//ROS_INFO("set_grad...\n");
		set_grad(xri);
		//collision
		if(check_collision(xrf)){
			ROS_INFO("collision...\n");
			// break;
		}
		//ロボットの命令速度算出
		float w,v;
		// ROS_INFO("set_command_vel...\n");
		set_command_vel(xri,v0,v,w,th_t);
		/*std::cout<<"xri:"<<xri<<"\n";
		std::cout<<"pot_xri:"<<get_pot_xt(xri)<<"\n";
		std::cout<<"pot(x0,x1),(y0,y1):("<<pot_x0<<","<<pot_x1<<"),("<<pot_y0<<","<<pot_y1<<")\n";
		std::cout<<"grad(x,y):"<<grad_xt<<","<<grad_yt<<"\n";//<--gradに問題:解決
		// std::cout<<"v0,vrt,w,th_t:"<<v0<<","<<vrt<<","<<w<<","<<th_t<<"\n";
		// ofss<<goal_time<<","<<xrf.x+cx<<","<<xrf.y+cy<<","<<v<<","<<w<<","<<th_t<<","<<std::endl;*/
		//ロボットの移動
		//mv_t:移動時間
		float l=v*mv_t;
		th_t=th_t+w*mv_t;
		xrf.x=xrf.x + l*cos(th_t);
		xrf.y=xrf.y + l*sin(th_t);
		//速度変化
		//ロボットの速度は1時遅れ系で変化すると仮定
		//目標値v0,現在速度vrt
		float Tr=0.25/1000;//マブチ3Vモータを参考
		//vrt=vrt+(v0-vrt)*(1-exp(-Tr*dt));
		vrt=vrt+(v0-vrt)*(exp(-Tr*mv_t));
		// std::cout<<"(1-exp(-Tr*dt)):"<<(1-exp(-Tr*mv_t))<<"\n";
		//debug
		//ROS_INFO("set_pub_mpc_debug_images...\n");
		past_time(mv_t);
		// set_pub_mpc_debug_images(xri);重い
		//move obstacles
		rate.sleep();
		goal_time+=mv_t;
		std::cout<<"goal_time:"<<goal_time<<"\n";
	// }
}
void trackingAvoidance::set_mv_obstacle_data(const std::vector<cv::Point2f>& pts,const float vx0,const float vy0){
	mv_obst obst_temp;
	obst_temp.data=pts;
	obst_temp.vx=vx0;
	obst_temp.vy=vy0;
	obst_temp.mvx=0;
	obst_temp.mvy=0;
	obst_temp.mvxt=0;
	obst_temp.mvyt=0;
	mv_obsts[mv_obsts_size++]=obst_temp;
	mv_data_size+=(int)pts.size();
}
void trackingAvoidance::end_set_mv_obstacle_data(){
	mv_obsts.resize(mv_obsts_size);
}
// 使わない
void trackingAvoidance::condition1(float reso){
	// ROS_INFO("seting condition1...\n");
	//--def mvObst
	float ro=0.2;
	float wo=ro/std::sqrt(2);
	float ho=ro/std::sqrt(2);
	int obst_size=(int)(wo/reso*2);
	float v=0.2;
	float th=M_PI;
	float vx=v*cos(th);
	float vy=v*sin(th);
	//std::cout<<"vx,vy:"<<vx<<","<<vy<<"\n";
	cv::Point2f x1=cv::Point2f(10-wo,5-ho);
	//std::cout<<"x1:"<<x1<<"\n";
	std::vector<cv::Point2f> mvObst1;
	mvObst1.resize(673*376);
	int k=0;
	for(int i=0;i<obst_size;i++){
		for(int j=0;j<obst_size;j++){
			if(i==0||j==0||i==obst_size-1||j==obst_size-1){
				cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
				mvObst1[k++]=temp;
			}
		}
	}
	mvObst1.resize(k);
	set_mv_obstacle_data(mvObst1,vx,vy);
}
void trackingAvoidance::condition2(float reso){
	// ROS_INFO("seting condition2...\n");
	//--def mvObst
	float ro=0.2;
	float wo=ro/std::sqrt(2);
	float ho=ro/std::sqrt(2);
	int obst_size=(int)(wo/reso*2);
	float v=0.2;
	float th=-M_PI*3.0/4.0;
	float vx=v*cos(th);
	float vy=v*sin(th);
	cv::Point2f x1=cv::Point2f(5.0+5/std::sqrt(2)-wo,5.0+5/std::sqrt(2)-ho);
	std::vector<cv::Point2f> mvObst1;
	mvObst1.resize(673*376);
	int k=0;
	for(int i=0;i<obst_size;i++){
		for(int j=0;j<obst_size;j++){
			if(i==0||j==0||i==obst_size-1||j==obst_size-1){
				cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
				mvObst1[k++]=temp;
			}
		}
	}
	mvObst1.resize(k);
	set_mv_obstacle_data(mvObst1,vx,vy);
}
void trackingAvoidance::condition3(float reso){
	// ROS_INFO("seting condition3...\n");
	//--def mvObst
	float ro=0.2;
	float wo=ro/std::sqrt(2);
	float ho=ro/std::sqrt(2);
	int obst_size=(int)(wo/reso*2);
	float v=0.2;
	float th=-M_PI/2;
	float vx=v*cos(th);
	float vy=v*sin(th);
	cv::Point2f x1=cv::Point2f(5.0-wo,10.0-ho);//5.0-wo,10.0-ho);
	std::vector<cv::Point2f> mvObst1;
	mvObst1.resize(673*376);
	int k=0;
	for(int i=0;i<obst_size;i++){
		for(int j=0;j<obst_size;j++){
			if(i==0||j==0||i==obst_size-1||j==obst_size-1){
				cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
				mvObst1[k++]=temp;
			}
		}
	}
	mvObst1.resize(k);
	set_mv_obstacle_data(mvObst1,vx,vy);
}
void trackingAvoidance::condition4(float reso){
	// ROS_INFO("seting condition4...\n");
	//--def mvObst
	float ro=0.2;
	float wo=ro/std::sqrt(2);
	float ho=ro/std::sqrt(2);
	int obst_size=(int)(wo/reso*2);
	float v=0.2;
	float th=-M_PI/4.0;
	float vx=v*cos(th);
	float vy=v*sin(th);
	cv::Point2f x1=cv::Point2f(5.0-5/std::sqrt(2)-wo,5.0+5/std::sqrt(2)-ho);
	std::vector<cv::Point2f> mvObst1;
	mvObst1.resize(673*376);
	int k=0;
	for(int i=0;i<obst_size;i++){
		for(int j=0;j<obst_size;j++){
			if(i==0||j==0||i==obst_size-1||j==obst_size-1){
				cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
				mvObst1[k++]=temp;
			}
		}
	}
	mvObst1.resize(k);
	set_mv_obstacle_data(mvObst1,vx,vy);
}
void trackingAvoidance::condition5(float reso){
	// ROS_INFO("seting condition5...\n");
	//--def mvObst
	float ro=0.2;
	float wo=ro/std::sqrt(2);
	float ho=ro/std::sqrt(2);
	int obst_size=(int)(wo/reso*2);
	float v=0.2;
	float th=0;
	float vx=v*cos(th);
	float vy=v*sin(th);
	cv::Point2f x1=cv::Point2f(0.0-wo,5.0-ho);
	std::vector<cv::Point2f> mvObst1;
	mvObst1.resize(673*376);
	int k=0;
	for(int i=0;i<obst_size;i++){
		for(int j=0;j<obst_size;j++){
			if(i==0||j==0||i==obst_size-1||j==obst_size-1){
				cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
				mvObst1[k++]=temp;
			}
		}
	}
	mvObst1.resize(k);
	set_mv_obstacle_data(mvObst1,vx,vy);
}
// 使わない
void trackingAvoidance::clear_mv_obstacle_data(){
	mv_obsts.clear();
	mv_obsts.resize(map_wi*map_hi/reso/reso);
	mv_obsts_size=0;
	mv_data_size=0;
}
// 使わない experiment condition
void trackingAvoidance::setting_condition(float reso,int num){
	// ROS_INFO("chosing condition...\n");	
	switch(num){
		case 1: 
			condition1(reso);
			break;
		case 2:
			condition2(reso);
			break;
		case 3:
			condition3(reso);
			break;
		case 4:
			condition4(reso);
			break;
		case 5:
			condition5(reso);
			break;
	}
}
void trackingAvoidance::set_static_obstacle_data(const cv::Point2f& data){
	set_obstacle_data(data);
}
void trackingAvoidance::setting_testcondition(float reso){
	// grid_map
	// ROS_INFO("grid_map...\n");
	bool floor=false;
	int obst_num=0;
	if(!floor){
		cv::Point2f obst_data=cv::Point2f(-2.3,1.5);
		cv::Point2f obst_data2=cv::Point2f(2.3,1.5);
		// cv::Point2f obst_data=cv::Point2f(0.0,0.0);
		// cv::Point2f obst_data2=cv::Point2f(2.0,1.8);
		cv::Point2f obst_data3=cv::Point2f(-1.0,1.0);
		cv::Point2f obst_data4=cv::Point2f(1.0,-0.5);
		cv::Point2f obst_data5=cv::Point2f(2.0,-1.0);
		cv::Point2f obst_data6=cv::Point2f(-2.0,1.0);
		cv::Point2f obst_data7=cv::Point2f(3.5,-2.0);
		//--set_static_obstacle_data(cv::Point2f& data)
		// set_static_obstacle_data(obst_data);
		// set_static_obstacle_data(obst_data2);
		// set_static_obstacle_data(obst_data3);
		// set_static_obstacle_data(obst_data4);
		// set_static_obstacle_data(obst_data5);
		//set_static_obstacle_data(obst_data6);
		// set_static_obstacle_data(obst_data7);
		obst_num=0;
		// obst_num=6;
	}
	// else{
	// 	for(int j=0;j<100;j++){
	// 		for(int i=0;i<100;i++){
	// 			int ti=20;
	// 			//std::cout<<"i,j:"<<i<<","<<j<<"\n";
	// 			if(i==ti||i==100-ti){	
	// 				// std::cout<<"i,j:"<<i<<","<<j<<"\n";
	// 				cv::Point2f obst_data=cv::Point2f((float)i/10.0,(float)j/10.0);
	// 				set_static_obstacle_data(obst_data);
	// 				obst_num++;
	// 			}
	// 		}
	// 	}
	// }
	// static potential_map
	// ROS_INFO("potential_map...\n");
	create_pot_map();
	// set movin obstacle data
	// ROS_INFO("set movin obstacle data...\n");
	if(!floor){
		//--def mvObst
		float wo=0.20;
		float ho=0.2;
		int obst_size=(int)(wo/reso*2);
		float vx=-0.0;
		float vy=-0.20;
		cv::Point2f x1=cv::Point2f(0.0-wo,2.0-ho);
		// cv::Point2f x1=cv::Point2f(-2.0-wo,1.0-ho);
		std::vector<cv::Point2f> mvObst1;
		mvObst1.resize(673*376);
		int k=0;
		for(int i=0;i<obst_size;i++){
			for(int j=0;j<obst_size;j++){
				if(i==0||j==0||i==obst_size-1||j==obst_size-1){
					cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
					mvObst1[k++]=temp;
				}
			}
		}
		mvObst1.resize(k);
		//----obst2
		float wo2=0.2;
		float ho2=0.2;
		int obst_size2=(int)(wo2/reso*2);
		float vx2=-0.1;
		float vy2=0.0;
		cv::Point2f x2=cv::Point2f(1.0-wo2,-2.0-ho2);
		// cv::Point2f x2=cv::Point2f(1.0-wo2,-2.0-ho2);
		std::vector<cv::Point2f> mvObst2;
		mvObst2.resize(673*376);
		int k2=0;
		for(int i=0;i<obst_size2;i++){
			for(int j=0;j<obst_size2;j++){
				if(i==0||j==0||i==obst_size2-1||j==obst_size2-1){
					cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x2;
					mvObst2[k2++]=temp;
				}
			}
		}
		mvObst2.resize(k2);
		//--set mvObst
		// ROS_INFO("mvObst...\n");
		set_mv_obstacle_data(mvObst1,vx,vy);
		set_mv_obstacle_data(mvObst2,vx2,vy2);
	}
	else{
		//--def mvObst
		float ro=0.2;
		float wo=ro/std::sqrt(2);
		float ho=ro/std::sqrt(2);
		int obst_size=(int)(wo/reso*2);
		float vx=-0.0;
		float vy=-0.20;
		cv::Point2f x1=cv::Point2f(5.4-wo,10.0-ho);
		std::vector<cv::Point2f> mvObst1;
		mvObst1.resize(673*376);
		int k=0;
		for(int i=0;i<obst_size;i++){
			for(int j=0;j<obst_size;j++){
				if(i==0||j==0||i==obst_size-1||j==obst_size-1){
					cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
					mvObst1[k++]=temp;
				}
			}
		}
		mvObst1.resize(k);
		set_mv_obstacle_data(mvObst1,vx,vy);
		
		std::vector<cv::Point2f> mvObst2;
		k=0;
		x1=cv::Point2f(4.6-wo,8.0-ho);		
		mvObst2.resize(673*376);
		for(int i=0;i<obst_size;i++){
			for(int j=0;j<obst_size;j++){
				if(i==0||j==0||i==obst_size-1||j==obst_size-1){
					cv::Point2f temp=cv::Point2f(i*reso,j*reso)+x1;
					mvObst2[k++]=temp;
				}
			}
		}
		mvObst2.resize(k);
		set_mv_obstacle_data(mvObst2,vx,vy);
	}	
}
// 使わない
bool trackingAvoidance::setting_RobotTestCondition(float reso){
	// setting
	// std::cout<<"wait...\n";
	// center point
	cv::Point2f cpt=cv::Point(0.0,0.0);
	// goal point
	cv::Point2f goal_pt=cv::Point2f(7.5-5,7.5-5);
	// set_param
	// std::cout<<"set_param...\n";
	set_center_point(cpt.x,cpt.y);
	set_goal(goal_pt);
	//--set_robot_param(float x,float y, float r,float vt0,float th_t0)
	// if(!set_robot_param(-2.5,-2.5,0.2,0.2,0.0))//-M_PI/2))
	if(!set_robot_param(0.0,0.0,0.3,0.2,0.0)){
		// std::cout<<"Error: robot param\n";
		return false; 
	}
	//--set_command_limit(float dif_vel)
	set_command_limit(0.2);
	set_mov_time(0.05);
	return true;
}
bool trackingAvoidance::setting_RobotExpCondition(float reso){
	// setting
	// std::cout<<"wait...\n";
	// center point
	// cv::Point2f cpt=cv::Point(5.0,5.0);
	cv::Point2f cpt=cv::Point(0,0);
	// goal point
	// cv::Point2f goal_pt=cv::Point2f(5.0,10.0);
	cv::Point2f goal_pt=cv::Point2f(0.0,3.0);
	// set_param
	// std::cout<<"set_param...\n";
	set_center_point(cpt.x,cpt.y);
	set_goal(goal_pt);
	//--set_robot_param(float x,float y, float r,float vt0,float th_t0)
	// if(!set_robot_param(5.0,0.0,0.2,0.2,M_PI/2))//)
	if(!set_robot_param(0.0,-3.0,0.2,0.2,M_PI/2)){
		// std::cout<<"Error: robot param\n";
		return false; 
	}
	//--set_command_limit(float dif_vel)
	set_command_limit(0.2);
	set_mov_time(0.05);
	return true;	
}
void trackingAvoidance::set_apf_mpc(float width,float height,float resolution){
	set_grid_param(width,height,resolution);
	mv_obsts.resize((int)(width/resolution*height/resolution));
	mpc_debug_image = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC3);
}
void trackingAvoidance::apf_mpc(){
	float H=6;
	float W=6;
	float reso=0.1;
	set_apf_mpc(W,H,reso);
	setting_RobotExpCondition(reso);
	//setting_RobotTestCondition(reso);
	setting_testcondition(reso);
	create_pot_map();
	//setting_condition(reso,1);
	end_set_mv_obstacle_data();
	/*ROS_INFO("get_speed...\n");
	cv::Point2f xrft=cv::Point2f(-2.5,-2.5);
	float vrt0=0.3;
	float v=get_speed(xrft,vrt0);
	std::cout<<"v:"<<v<<"\n";
	for(int i=1;i<=5;i++){
		setting_RobotExpCondition(reso);
		//create_pot_map();
		clear_mv_obstacle_data();
		setting_condition(reso,i);
		end_set_mv_obstacle_data();
	}*/
	draw_mpc_path_mat();
	ROS_INFO("Done...\n");
}
// switch変数
bool PROCESS_START;
//switch関数
void callback_function(const std_msgs::Empty::ConstPtr& msg){
	ROS_INFO("Turn On");
	PROCESS_START =true;
}
int main(int argc,char **argv){
	ros::init(argc,argv,"autonomous_mobile_robot_aa_sim");
	ROS_INFO("trackingAvoidance constructer define");
    trackingAvoidance aa_sim; //
	//idle process
	std_msgs::Empty empty_msg;
	ros::NodeHandle nh_s;
	ros::Publisher pub_s;
	pub_s = nh_s.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1);
	ros::Subscriber sub_s;
	ros::CallbackQueue queue;
	nh_s.setCallbackQueue(&queue);
	sub_s=nh_s.subscribe("start_swich",1,callback_function);
	PROCESS_START=false;
    //--process
	while(ros::ok()&&!PROCESS_START){
        // ros::spinOnce();
		ROS_INFO("Waiting switch msg");
		pub_s.publish(empty_msg);
		queue.callOne(ros::WallDuration(0.1));
        if(aa_sim.get_received_cmd_vel() && aa_sim.get_set_config()){
        aa_sim.run();
	    }
    }
	ros::spin();
	ROS_INFO("start");
	return 0;
}
