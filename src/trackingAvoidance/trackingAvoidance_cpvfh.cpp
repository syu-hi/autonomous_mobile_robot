#include<autonomous_mobile_robot/trackingAvoidance.h>
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
    robotOdom.twist.twist.linear.x = cur_vel;
	robotOdom.twist.twist.angular.z = cur_angVel;
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
// cp cost avoidance
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
	// std::cout<<"goal_xy: "<<goal_x <<","<<goal_y<<std::endl;
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
// データ送信
void trackingAvoidance::publishData(geometry_msgs::Twist& pubData){
    pub.publish(pubData);
}
void trackingAvoidance::vfh(){
	culc_delta_robotOdom(); //ロボットグローバル速度算出
	update_goal_position(); //現在地とゴール位置の位置関係更新
	create_histgram(); //ヒストグラム作成
	create_binary_histgram(robotRadius, marginRadius);
	// histgramCheckerEx();
	//探索処理
	float tagVel, tagAng;
	searchProcess(tagVel, tagAng);
	// ROS_INFO_STREAM("target vel =" <<tagVel<<"\n"<< "target angle =" <<tagAng);
	//命令速度生成
	// geometry_msgs::Twist cmd = controler(tagVel, tagAng);
	// ROS_INFO_STREAM("publishData = \n" <<cmd);
	// publishData(cmd);
}