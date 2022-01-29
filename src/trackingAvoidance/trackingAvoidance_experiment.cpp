#include<autonomous_mobile_robot/trackingAvoidance.h>
trackingAvoidance::trackingAvoidance()
	:wheel_d(0.155),angle_min(45),angle_max(135), angle_div(1.0)
	,RECEIVED_CLUSTER(false),RECEIVED_GOAL_ODOM(false),RECEIVED_ROBOT_ODOM(false),RECEIVED_LRF_SCAN(false)
    ,SEARCH_ONLY_ANGLE(false),MAX_COST(900000),PROCESS_INIT(true),PROCESS_ONCE(true)
	,max_pot(100),min_pot(-100),stop_flag(true)
	,mv_obsts_size(0),mv_data_size(0){
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
    // pub6= nhPub.advertise<autonomous_mobile_robot::SensorMapData>("laserMapData", 1);
	ROS_INFO("publisher define");
	// publisher
    pub= nhPub.advertise<geometry_msgs::Twist>("/beego/cmd_vel", 1);
	pub_pan=nh_pub.advertise<dynamixel_sdk_examples::SetPosition>("set_position",1);
	pub_tilt=nh_pub.advertise<dynamixel_sdk_examples::SetPosition>("set_position",1);
	pub_Trajectory=nh_pub.advertise<autonomous_mobile_robot::ClassificationState>("Trajectory",1);
	// デバッグ用
	pubDebMarkerArray= nhDeb.advertise<visualization_msgs::MarkerArray>("crossPointMarkerArray", 1);
	pubDebCross= nhDeb.advertise<visualization_msgs::MarkerArray>("crossPointCheckerResult", 1);
	pubDebHst = nhDeb.advertise<visualization_msgs::MarkerArray>("histgramChecker", 1);
	pubDebOutput = nhDeb.advertise<visualization_msgs::MarkerArray>("outputChecker", 1);
	pubDebCPVFHOutput = nhDeb.advertise<visualization_msgs::MarkerArray>("outputCPVFHChecker", 1);
	pubDebBagOutput = nhDeb.advertise<visualization_msgs::MarkerArray>("outputBagOutputChecker", 1);
	pubDebRotOutput = nhDeb.advertise<visualization_msgs::MarkerArray>("RotChecker", 1);
	pubRotVel =  nhDeb.advertise<visualization_msgs::MarkerArray>("rotVelMarker", 1);
	pubDebOdom = nhDeb.advertise<nav_msgs::Odometry>("DeltaOdomChecker", 1);
    pubDeb1 = nhPubDeb.advertise<visualization_msgs::MarkerArray>("debug_clstr", 1);
    pubDeb2 = nhPubDeb.advertise<visualization_msgs::MarkerArray>("debug_detected_clstr", 1);
    pubDeb3 = nhPubDeb.advertise<visualization_msgs::MarkerArray>("debug_odometries", 1);
    //apf debug
	pub_img=nh_pub.advertise<sensor_msgs::Image>("pot_image",1);
	// launchファイルからパラメータの読み込み
	setLaunchParam();
	//クロスポイントチェッカーデフォルト値入力
	setDefaultCrossPointChecker();
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
	ROS_INFO("destructor");
}
//---launch fileからの読み込み
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
// real obstacle avoidance and tracking
// subscribe
void trackingAvoidance::cluster_callback(const autonomous_mobile_robot::ClassificationVelocityData::ConstPtr& msg){
	// ROS_INFO("cluster_callback");
    //データをコピー
	clstr = *msg;
	//move manage method
	RECEIVED_CLUSTER = true;
	manage();
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
		// ROS_INFO_STREAM("imu_yaw =" <<imu_yaw*180/M_PI);
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
	manage();
}
void trackingAvoidance::robotEncoder_callback(const beego_control::beego_encoder::ConstPtr& msg){
	// ROS_INFO("robotEncoder_callback");
    //データをコピー
	robotEncoder = *msg;	
	RECEIVED_ROBOT_ENCODAR = true;
	//move manage method
	manage();
}
void trackingAvoidance::scan_callback(const autonomous_mobile_robot::SensorMapData::ConstPtr& msg){
	// ROS_ERROR_STREAM("Could not lrfMAP."<<smd);
	// ROS_INFO("scan_callback");
	smd =*msg;
	RECEIVED_LRF_SCAN = true;
	manage();
}
void trackingAvoidance::goalOdom_callback(const nav_msgs::Odometry::ConstPtr& msg){
	// ROS_INFO("goalOdom_callback");
    //データをコピー
	goalOdom = *msg;
	RECEIVED_GOAL_ODOM = true;
	//move manage method
	manage();
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
        // 2回目以降提案法と比較法
        // cross point
        vfh();
		// potential
		apf();
		// ROS_INFO_STREAM("error"<<"\n");
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
    stop_flag=true;
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
// switch変数
bool PROCESS_START;
//switch関数
void callback_function(const std_msgs::Empty::ConstPtr& msg){
	ROS_INFO("Turn On");
	PROCESS_START =true;
}
int main(int argc,char **argv){
	ros::init(argc,argv,"autonomous_mobile_robot_aa");
	ROS_INFO("trackingAvoidance constructer define");
    trackingAvoidance aa; //
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
    }
	ros::spin();
	ROS_INFO("start");
	return 0;
}