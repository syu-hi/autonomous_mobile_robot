//多重インクルード防止
#ifndef INCLUDE_TRACKING_AVOIDANCE_SIMULATION_CLASS
#define INCLUDE_TRACKING_AVOIDANCE_SIMULATION_CLASS
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
// msg
#include <autonomous_mobile_robot/ClassificationVelocityData.h>
#include <autonomous_mobile_robot/SensorMapData.h>
#include <std_msgs/String.h>
#include <dynamixel_sdk_examples/GetPosition.h>
#include <dynamixel_sdk_examples/SetPosition.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <beego_control/beego_encoder.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
// opencv
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/MarkerArray.h>
// rqt_reconfige
#include <dynamic_reconfigure/server.h>
#include <autonomous_mobile_robot/trackingAvoidanceConfig.h>
// c++
#include <stdio.h>
#include <iostream>
#include <cmath>
//file input output
#include <fstream>
#include <omp.h>
//クラスの定義
class trackingAvoidance{
    private:
        // apf_mpc 処理データ
        struct mv_obst{
			float vx;
			float vy;
			float mvx;
			float mvy;
			//debug
			float mvxt;
			float mvyt;
			std::vector<cv::Point2f> data;
        };
        //obstacle avoidance 処理データ
        struct crossPoint{
            float x;//交差位置x
            float y;//交差位置y
            float dis;//交差位置とロボットの距離
            float vx;//相対速度Vx
            float vy;//相対速度Vy
            float t;//交差時の時間
            int index;//障害物番号
            bool safe;//安全フラグ
        };
        // simulation
        //受信データ
		ros::NodeHandle nhSub;
		ros::Subscriber sub;
        geometry_msgs::Twist cmd_vel;
        //送信データ
        ros::NodeHandle nhPub_sim;
        ros::Publisher pub1,pub2,pub3,pub4;
    	autonomous_mobile_robot::ClassificationVelocityData cur_clstr,pre_clstr;//速度データ付きのクラスタデータ
    	autonomous_mobile_robot::ClassificationVelocityData detected_clstr;//視認可能 
        nav_msgs::Odometry enc_robotOdom,cur_robotOdom,goalOdom_sim;
        beego_control::beego_encoder robotEncoder_sim;
        //処理用
        bool RECEIVED_CMD_VEl;
        bool SET_CONFIG;
        //ロボットデータ
        double sensor_angle_min,sensor_angle_max;
        double t;//シミュレータ内の時間
        //障害物データ
        double obstacleX1,obstacleY1;
        double obstacleW1,obstacleH1;
        double obstacleVx1,obstacleVy1;
        //ロボット初期位置
        double robot_init_x;
        double robot_init_y;
        double robot_init_angle;
        double robot_init_v;
        double robot_init_w;
        //ゴールデータ
        double goal_x_sim,goal_y_sim;
        // simulater_run.cpp
        ros::NodeHandle nhPubDeb;
        ros::Publisher pubDeb1,pubDeb2,pubDeb3;
        //apf.cpp
		float map_wf,map_hf;//map size float
		float reso;//resolution
		int map_wi,map_hi;//map size int
		float cr;//cell radius
		float max_pot;
		float min_pot;
		//grid_map.cpp
		cv::Mat grid_map;//グリッドマップ
		//potential_map.cpp
		cv::Mat pot_map;//コストマップ
		//gradient_map.cpp
		cv::Mat grad_map[2];//勾配マップ
		//debug
		cv::Mat debug_image;
		cv_bridge::CvImagePtr cvbridge_image;
		ros::NodeHandle nh_pub;
		ros::Publisher pub;
		//set_param.cpp
		float cx,cy;//center point x,y
		cv::Point2f xgf;//ゴール座標 float
		cv::Point2i xgi;//ゴール座標 int
		cv::Point2f xr;//ロボットの位置 float global coordinate
		cv::Point2f xrf;//ロボットの位置 float gridmap coordinate
		cv::Point2i xri;//ロボットの位置 int
		float rr;//robot radiu
		float vrt;//robot speed
		float th_t;//robot angular
		float max_w;//robot angular speed (max)
		float mv_t;//movement time(dt)
		float sum_pot;//sum_potential
		std::vector<cv::Point2i> obst_pti;
        // apf_mpc
        float pot_x0,pot_x1;
		float pot_y0,pot_y1;
		float grad_xt,grad_yt;
		float pot_xt0;
		cv::Point2f xrf0;
		cv::Mat pot_mapt;
		float time_range;
        std::vector<mv_obst> mv_obsts;
		int mv_obsts_size;
		int mv_data_size;
		cv::Mat mpc_debug_image;//debug
		//cv::Mat pot_maptt;
        // obstacle avoidance
        //受信データ
		ros::NodeHandle nhSub1;
		ros::Subscriber sub1,sub2,sub3,sub4,sub6;
    	autonomous_mobile_robot::ClassificationVelocityData clstr;//速度データ付きのクラスタデータ
    	autonomous_mobile_robot::ClassificationVelocityData rotClstr;
        nav_msgs::Odometry robotOdom,pre_robotOdom,deltaRobotOdom,goalOdom,relationOdom;
        sensor_msgs::Imu cur_cameraIMU, pre_cameraIMU;
        beego_control::beego_encoder robotEncoder;
        sensor_msgs::LaserScan lrfScan;
        autonomous_mobile_robot::SensorMapData smd;
       	dynamixel_sdk_examples::SetPosition pubPanData;
    	dynamixel_sdk_examples::SetPosition pubTiltData;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub_img,pub_img_mpc,pub_pan,pub_tilt;
        // 処理
		bool RECEIVED_CLUSTER;
        bool RECEIVED_GOAL_ODOM;
        bool RECEIVED_ROBOT_IMU;
        bool RECEIVED_ROBOT_ODOM;
		bool RECEIVED_ROBOT_ENCODAR;
        bool RECEIVED_LRF_SCAN ;
        bool SEARCH_ONLY_ANGLE;
        double MAX_COST;
        //時間
        ros::Time cur_time,pre_time;
        ros::Duration delta_time_ros;
        double delta_time;
        double total_time;
        bool PROCESS_ONCE;
        bool PROCESS_INIT;
        // imu parameter
		double imu_roll,imu_pitch,imu_yaw;
        //launch ファイル
        //--ロボットパラメータ
        float wheel_d;//車輪間隔の半分
        float robotRadius;//ロボット半径
        float max_speed,min_speed;
        float default_speed;
        //--vfh+
        float angle_center;//センサ中心角度
        double min_cost;//最小コスト
        float selected_angle;//最小コスト角度
        float dis_threshold;//距離閾値: バイナリ作成用
        float initDis;
        float marginRadius;//マージン半径
        float dis_th;//距離ヒストグラムの閾値
        float k_cp, eta_cp;//交差位置に対する重み
        float k_g, eta_g;//ゴール位置への角度と目標角度に対する重み
        //cost 
        float eta_goal, k_goal;
        float k_curAngle, eta_curAngle;//現在の角度と目標角度に対する重み
        float k_prevAngle, eta_prevAngle;//現在の角速度と目標角速度に対する重み
        float k_vel,eta_vel;//速度加減速に対する重み
        float safe_range;
        float crossWeightX;
        float crossWeightY;
        float timeBias;
    	std::vector<crossPoint> crsPts;
        float goal_x, goal_y;
        float cur_angle;
        float prev_tagAng;
        float angle_min, angle_max;//センサ測定可能角度
        float angle_div;//ヒストグラム解像度
        float cur_vel,cur_angVel;
        float dV_range, dV_div;
        std::vector<double> hst_dis;//ヒストグラム配列(距離) 距離ヒストグラム
        std::vector<bool> hst_bi;//ヒストグラム配列(２値化後) バイナリヒストグラム
        // デバッグ用
		ros::NodeHandle nhDeb;
        ros::Publisher pubDebPcl,pubDebCross,pubDebMarkerArray, pubDebHst,pubDebOutput,pubDebCPVFHOutput,pubDebBagOutput;
        ros::Publisher pubDebRotOutput,pubDebOdom,pubRotVel;
        int debugType;
        //カラーリスト
        float colors[12][3] ={{1.0,0,1.0},{1.0,1.0,0},{0,1.0,1.0},{1.0,0,0},{0,1.0,0},{0,0,1.0},{0.5,1.0,0},{0,0.5,1.0},{0.5,0,1.0},{1.0,0.5,0},{0,1.0,0.5},{1.0,0,0.5}};//色リスト
        //クロスポイントチェッカー入力
        bool debugFlag_crossPointChecker;
        float debugEncoderVel_r;//ロボットエンコーダ速度(右車輪)
        float debugEncoderVel_l;//ロボットエンコーダ速度(左車輪)
        float debugCur_vel;//ロボット速度
        float debugCur_angle_steer;//現在のロボットステアリング角度（進行方向）
        float debugCmd_vel;//ロボット目標速度
        float debugCmd_angle;//ロボット目標角度
        int debugIndexRef;//障害物番号
        geometry_msgs::Point debugGpRef;//クラスタ重心
        geometry_msgs::Twist debugTwistRef;//障害物速度
        float debugObstacleRadius;//障害物半径
        float debugRobotRadius;//ロボット半径（ヒストグラムチェッカーでも使用）
        //ヒストグラムチェッカー入力
        bool debugHistgramCheckerFlag;
        int debugObstacleNum;
        float debugObstacleX1;
        float debugObstacleY1;
        float debugObstacleSize1;
        float debugObstacleX2;
        float debugObstacleY2;
        float debugObstacleSize2;
        float debugObstacleX3;
        float debugObstacleY3;
        float debugObstacleSize3;
        float debugThresholdDistance;
        float debugMinAngle;
        float debugMaxAngle;
        float debugDivAngle;
        float debugMarginRadius;//マージン半径
        //VFH出力チェッカー
        bool debugOutputVFHCheckerFlag;
        float debugKg, debugEtaG;//ゴール位置への角度と目標角度に対する重み
        float debugKcurAngle, debugEtaCurAngle;//現在の角度と目標角度に対する重み
        float debugKprevAngle, debugEtaPrevAngle;//現在の角速度と目標角速度に対する重み
        float debugGoalAng;//目標角度
        float debugGoalPosX;//目標位置X
        float debugGoalPosY;//目標位置Y 
        float debugCurAng;//現在の角度
        float debugPrevTagAng;//角速度を取得
        //CP-VFH出力チェッカー
        bool debugOutputCPVFHCheckerFlag;
        float debugKcp, debugEtaCp;//交差位置に対する重み
        float debugObstacleVx1;
        float debugObstacleVy1;
        float debugObstacleVx2;
        float debugObstacleVy2;
        float debugObstacleVx3;
        float debugObstacleVy3;
        float debugObstacleSizeThreshold;
        geometry_msgs::Point debugGp1,debugGp2,debugGp3;//クラスタ重心
        geometry_msgs::Twist debugTwist1,debugTwist2,debugTwist3;//障害物速度
        bool debugRotationVelocityCheckerFlag;
        double debugRotOmega;
        //交差位値表示
        bool display_output;
        //--rqt_reconfigure
        bool rqt_reconfigure;//rqt_reconfigureを使用するか
        dynamic_reconfigure::Server<autonomous_mobile_robot::trackingAvoidanceConfig> server;
        dynamic_reconfigure::Server<autonomous_mobile_robot::trackingAvoidanceConfig>::CallbackType f;
    public:
        //in constracter.cpp
        trackingAvoidance();//コンストラクタ：クラス定義に呼び出されるメソッド
        ~trackingAvoidance();//デストラクタ：クラスが消滅するときに呼びだされるメソッド
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルからの探索窓パラメータ書き込み
        // debug simulation
        void setDefaultCrossPointChecker();//デバッグ用のパラメータ初期化
        bool get_set_config();
        bool get_received_cmd_vel();
        void reset_received_cmd_vel();
        void run();
        void showOdometries();
        void showClusters();
        void showDetectedClusters();
        // void callback_function(const std_msgs::Empty::ConstPtr& msg);
        void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
        void publishObstacleData();//クラスタの出力
        void publishRobotData();//ロボットオドメトリを出力
        void publishTargetData();//目標位置(ゴール位置)を出力
        void createObstacleData();
        void createRobotData();
        void createGoalOdom(double goal_x_temp,double goal_y_temp);
        void updateRobotEncoder();//ロボットオドメトリ, 命令速度依存
        void updateObstacleState();//一つ前のロボット姿勢, 現時点のエンコーダ使用
        void updateRobotState();//ロボット速度使用
        void detectObstacle();//ロボットから視認できる障害物を抽出
        //obstacle avoidance debug デバッグ用のメソッド
        void debug();
        void showCrossPoints();
        void showOutPut(std::vector<crossPoint>& crsPts, float v, int num);
        void showCostMap();
        void crossPointChecker();
        void histgramChecker();
        void outputVFHChecker();
        void outputCrossPointVFHChecker();
        void histgramCheckerEx();
        void publish_deltaRobotOdom();
        void rotationVelocityChecker(double omega);
        void debug_trans_rotation_vel(double& v_rot_x, double& v_rot_y, const double& x_para_x,const double& x_para_y,const double& x_para_theta,const double& x_para_vx,const double& x_para_vy, const double& omega_base);
        void display_rotVel();
        //vfh+
        void set_histgram_param(float& angMin, float& angMax, float& angDiv);
        void set_dis_threshold(float& data);
        void set_eta(float& goal, float& theta, float& omega);
        void set_k(float& goal, float& theta, float& omega);
        void get_histgram_dis(std::vector<double>& data);
        void get_histgram_bi(std::vector<bool>& data);
        double get_min_cost();
        float get_selected_angle();
        //transform
        int transform_angle_RobotToNum(float& angle);
        float transform_numToAngle(int& num);
        float transform_angleFromCenter(float& angle);
        //clear
        void clear_histgram_dis();
        //resize
        void resize_histgram_dis(int size);
        void resize_histgram_dis(int size, float initValue);
        //add histgram element
        void add_histgram_dis(float& angle, float& dis);
        // create binary histgram
        void create_binary_histgram(float& robotRadius, float& marginRadius);
        float min_dif_angle_rad(const float& angle1, const float& angle2);
        float min_dif_angle_deg(const float& angle1, const float& angle2);
        //cost function 
        double angleCostFunction_deg(float& eta, float value);
        double cost_goal_angle_deg(const float& angle, float goal_angle);
        double cost_current_angle_deg(const float& angle, const float& cur_angle);
        double cost_prev_select_angle_deg(const float& angle, const float& pre_target_angle);
        double angleCostFunction_rad(float& eta, float value);
        double cost_goal_angle_rad(const float& angle, float goal_angle);
        double cost_current_angle_rad(const float& angle, const float& cur_angle);
        double cost_prev_select_angle_rad(const float& angle, const float& pre_target_angle);
        double getCost_deg(float tagAng, float goalAng, float curAng, float prevTagAng);
        double getCost_rad(float tagAng, float goalAng, float curAng, float prevTagAng);
        //obstacle avoidance
        //--メソッド管理
        void manage();
        //--センサーデータ受信
        void cluster_callback(const autonomous_mobile_robot::ClassificationVelocityData::ConstPtr& msg);
        void robotIMU_callback(const sensor_msgs::Imu::ConstPtr& msg);
        void robotEncoder_callback(const beego_control::beego_encoder::ConstPtr& msg);
        void scan_callback(const autonomous_mobile_robot::SensorMapData::ConstPtr& msg);
        void goalOdom_callback(const nav_msgs::Odometry::ConstPtr& msg);
        //--rqt_reconfigureからの読み込み
        void configCallback(autonomous_mobile_robot::trackingAvoidanceConfig &config, uint32_t level);
        void update_goal_position();
        //処理
        bool data_check();
        void data_check_reset();
        void get_time();
        bool culc_delta_time();
        void culc_delta_robotOdom();
        void trans_rotation_vel(double& v_rot_x, double& v_rot_y, const double& x_para_x,const double& x_para_y,const double& x_para_vx,const double& x_para_vy);
        crossPoint getCrossPoint(int& indexRef,geometry_msgs::Point& gpRef, geometry_msgs::Twist& twistRef, float& cur_vel, float& cur_ang, float& cmd_dV, float& cmd_ang);
        crossPoint getCrossPoint(int& cp_num, std::vector<crossPoint>& crsPts, int& indexRef,geometry_msgs::Point& gpRef, geometry_msgs::Twist& twistRef, float& cur_vel, float& cmd_dV, float& cmd_dAng);
        void getCrossPoints(crossPoint& crsPt_x0, crossPoint& crsPt_y0, int& indexRef,geometry_msgs::Point& gpRef, geometry_msgs::Twist& twistRef, float& cur_vel, float& cur_ang, float& cmd_dV, float& cmd_ang);
        void getCrossPoints(crossPoint& crsPt_x0, crossPoint& crsPt_y0, int& indexRef, const autonomous_mobile_robot::ClassificationElement& clst_data, geometry_msgs::Twist& twistRef, float& cur_vel, float& cur_ang, float& cmd_dV, float& cmd_ang);
        double getNearestDistance(crossPoint& crsPt, const autonomous_mobile_robot::ClassificationElement& clst_data);
        double getDeltaVelCost(float& cmd_dV_temp, float& eta_vel_temp,float& cur_vel_temp);
        void crossPointsDetect(float& cmd_vel, float& cmd_angle);
        void crossPointsDetect(std::vector<crossPoint>& crsPts, float& cur_vel_temp, float& cur_angle_temp, float& cmd_dV, float& cmd_dAng);
        float generalCostFunction(float& eta, float& value);
        double costCrossPoint(crossPoint& crsPt, float eta_cp);
        double getCrossPointCost(std::vector<crossPoint>& crsPts, float eta_cp);//交差位置コスト
        bool checkSafetyObstacle(float& t, float& angle, float& x, float& y);
        void searchProcess(float& tagVel, float& tagAng);
        void search_vel_ang(float& target_angle, float& cur_vel_temp, float& cmd_dV);        
        double vfh_angleSearch(float& target_angle_temp, float& cur_vel_temp, float& cmd_dV);//return cost
        double vfh_angleSearch_nondeb(float& target_angle_temp, float& cur_vel_temp, float& cmd_dV ,std::vector<crossPoint>& min_cost_crsPts_temp );//return cost
        geometry_msgs::Twist controler(float& tagVel, float& tagAng);
        //vfh+
        void create_histgram();
        // データ送信
        void publishData(geometry_msgs::Twist& pubData);//データ送信
        //apf.cpp
		void apf();
		//set_param.cpp
		void set_grid_param(float width,float height,float resolution);
		void set_center_point(float cpx,float cpy);
		void set_goal(cv::Point2f& goal_2f);
		bool set_robot_param(float x,float y, float r,float vt0,float th_t0);
		void set_command_limit(float max_dif_vel);
		void set_mov_time(float time);
		//coordinate_transform.cpp
		bool trans_point(const cv::Point2f& pt,cv::Point2i& pti);
		bool trans_point(const cv::Point2f& pt,cv::Point2i& pti,cv::Point2f& ptf);
		void trans_point_f_to_i(const cv::Point2f& ptf,cv::Point2i& pti);
		//grid_map.cpp
		// bool check_gridmap_format(cv::Mat& map);
		// void clear_grid_map();
		// void set_grid_map(cv::Mat& map);
		void set_obstacle_data(const cv::Point2f& data);
		//potential_map.cpp
		float culc_dis(const int& w0,const int& h0,const int& w1,const int& h1);
		float culc_fr(const float& dis,const int& obstNum);
		float culc_fr(const int& w0,const int& h0,const int& w1,const int& h1,const int& obstNum);
		// float culc_L1dis(const int& w0,const int& h0,const int& w1,const int& h1);
		// float culc_L1fr(float& dis,const int& obstNum);
		// float culc_chvdis(const int& w0,const int& h0,const int& w1,const int& h1);
		// float culc_chvfr(const int& w0,const int& h0,const int& w1,const int& h1,const int& obstNum);
		float culc_fa(float dis);
		float culc_fa(const int& w0,const int& h0,const int& w1,const int& h1);
		void search_obst_pt();
		void create_pot_map();
		//gradient_map.cpp
		float get_grad_1(float& x0,float& x1,float& delta);
		float get_grad_2(float& x0,float& x1,float& delta);
		void create_grad_map();
		//path_planning.cpp
		void set_command_vel(cv::Point2i& xri0,float& v,float& w);
		void draw_path_mat();
		//debug apf
		void set_pub_debug_images();
		void publish_debug_image(const cv::Mat& temp_image);
        //apf_mpc.cpp
		void set_apf_mpc(float width,float height,float resolution);
		void apf_mpc();
		//set_data.cpp
		void set_static_obstacle_data(const cv::Point2f& data);
		void set_mv_obstacle_data(const std::vector<cv::Point2f>& pts,const float vx0,const float vy0);
		void end_set_mv_obstacle_data();
		void clear_mv_obstacle_data();
		void move_obstacle_data(float& time);
		void clear_move_data();
		//grid_map.cpp in apf class sources
		////void set_command_vel(cv::Point2i& xri0,float& w);
		void set_command_vel(const cv::Point2i& xri0,const float& v0,float& v,float& w,float& th_t0);
		//mv_pot_map.cpp
		float culc_mv_obstacle_fr(const cv::Point2i xti,const int& obstNum);
		void add_mv_pot(const cv::Point2i xti,const int& obstNum);
		//mpc_func.cpp
		float& get_pot_xt(const cv::Point2i& xti);
		bool set_grad(const cv::Point2i& xti);
		double culc_cost(cv::Point2f& xrft0,const float v0,const float& time_range);
		float get_speed(const cv::Point2f& xrft0,const float& vrt00);
		//debug.cpp
		void set_pub_mpc_debug_images(const cv::Point2i& xrit0);
		void draw_mv_obst();
		void draw_mpc_path_mat();
		bool check_collision(const cv::Point2f xrf00);
		void past_time(const float& time);
        bool setting_RobotTestCondition(float reso);
        bool setting_RobotExpCondition(float reso);
        void setting_testcondition(float reso);
        void setting_condition(float reso,int num);
        void condition1(float reso);
        void condition2(float reso);
        void condition3(float reso);
        void condition4(float reso);
        void condition5(float reso);
};
#endif