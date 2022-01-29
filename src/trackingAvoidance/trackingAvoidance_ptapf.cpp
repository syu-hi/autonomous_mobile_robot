#include<autonomous_mobile_robot/trackingAvoidance.h>
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
void trackingAvoidance::draw_mv_obst(){
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
				debug_image.at<cv::Vec3b>(pti.y,pti.x)[1] =255;
			}
		}	
	}
}
void trackingAvoidance::set_pub_debug_images(){
	int W=map_wi;
	int H=map_hi;
	for(int h0=0;h0<H;h0++){
		for(int w0=0;w0<W;w0++){
			//set potential
			//float pot=pot_mapt.at<float>(h0,w0);//*std::abs(sum_pot);
			float pot=pot_map.at<float>(h0,w0);//*std::abs(sum_pot);
			//std::cout<<"pot:"<<pot<<"\n";
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
	draw_mv_obst();
	publish_debug_image(debug_image);
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
void trackingAvoidance::set_command_vel(const cv::Point2i& xri0,const float& v0,float& v,float& w,float& th_t0){
	// float vx = grad_map[0].at<float>(xri0.y,xri0.x);
	// float vy = grad_map[1].at<float>(xri0.y,xri0.x);
	float vx = grad_xt;
	float vy = grad_yt;
	float v00 = std::sqrt(vx*vx+vy*vy);
	vx /= v00;
	vy /= v00;
	//反時計回りを正(0~360)
	// std::cout<<"grad_xt,yt:"<<grad_xt<<","<<grad_yt<<"\n";
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
	// std::cout<<"th,th_t0:"<<th<<","<<th_t0<<"\n";
	// std::cout<<"delta_th,w:"<<delta_th<<","<<w<<"\n";
	if(w>max_w){
		w=max_w;
	}
	else if(w<-max_w){
		w=-max_w;
	}
	//速度は一定
	v=vrt;
	//速度可変
	// float d=0.150;
	// float d=0.138;
	// float dif_v = w*2*d;
	//v=v0-std::abs(dif_v)/2;
	//速度可変
	v=v0;
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
	// ROS_INFO_STREAM("ros::ok()3 =" <<ros::ok()<<"\n");
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
		// ROS_INFO_STREAM("sum_cost "<<sum_cost<<"\n");
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
		//ROS_INFO("set_pub_debug_images()\n");
		//set_pub_debug_images(xrit);
		/*if(tr==time_range){
			std::cout<<"in cost:grad(x,y,w):"<<grad_xt<<","<<grad_yt<<","<<w<<"\n";
			std::cout<<"xrit:"<<xrit<<"\n";
			std::cout<<"pot_xrit:"<<get_pot_xt(xrit)<<"\n";
			std::cout<<"pot(x0,x1),(y0,y1):("<<pot_x0<<","<<pot_x1<<"),("<<pot_y0<<","<<pot_y1<<")\n";
			pot_maptt=pot_mapt.clone();
			std::cout<<"w :"<<w<<"\n";
		}*/
		//rate.sleep();
		tr-=mv_t;
		// ROS_INFO_STREAM("time_range"<<tr<<"\n");
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
	float time_range=0.1;
	// float time_range=10;
	float opt_v;
	// std::cout<<"vrt0:"<<vrt0<<"\n";
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
	cost0=culc_cost(xrft,vrt0,time_range);
	// ROS_INFO("culc_cost0...\n");
	cost1=culc_cost(xrft,vrt1,time_range);
	// ROS_INFO("culc_cost1...\n");
	//predict param
	int search_num=1;
	// float pot_th=0.10;//10%
	// float pot_rate;
	//gradient v
	// float grad_v;
	bool flag01=false;
	bool flag10=false;
	// ROS_INFO_STREAM("ros::ok()2 =" <<ros::ok()<<"\n");
	// ROS_INFO("while...\n");
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
		// std::cout<<"cost("<<cost0<<","<<cost1<<")\n";
		// std::cout<<"opt_v("<<opt_v<<")"<<"search_num("<<search_num<<"\n";
		if(cost0<=cost1){
			if(vrt0>=max_v){
				return max_v;
			}
			if(vrt0<=min_v){
				return min_v;
			}
			// ROS_INFO("vrt0<vrt1:(%f,%f)\n",vrt0,vrt1);
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
			// ROS_INFO("vrt0>vrt1:(%f,%f)\n",vrt0,vrt1);
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
void trackingAvoidance::trans_point_f_to_i(const cv::Point2f& ptf,cv::Point2i& pti){
	float map_ptx = map_wf/2 + ptf.x;
	float map_pty = map_hf/2 - ptf.y;
	pti.x = (int)(map_ptx/reso);
	pti.y = (int)(map_pty/reso);
}
void trackingAvoidance::clear_move_data(){
	for(int k=0;k<mv_obsts.size();k++){
		mv_obsts[k].mvx=0;
		mv_obsts[k].mvy=0;
	}
}
void trackingAvoidance::draw_path_mat(){
	// ROS_INFO_STREAM("ros::ok() =" <<ros::ok()<<"\n");
	ros::NodeHandle n;
	ros::Rate rate(100);
	int obst_num=(int)obst_pti.size()+mv_data_size;
	clear_move_data();
	float v0=vrt;
	//float dt=(float)1/100;
	float goal_time=0;
	int i=0;
	// ROS_INFO_STREAM("ros::ok()1 =" <<ros::ok()<<"\n");
	while(ros::ok()&&i>=0){
		//float to int
		trans_point_f_to_i(xrf,xri);
		// std::cout<<"xrf,xgf:"<<xrf<<"-->"<<xgf<<"\n";
		//ゴールセルに到達したら終了
		if(xri.x==xgi.x && xri.y==xgi.y){
			std::cout<<"Goal_pre_draw\n";
			break;
		}
		//ROS_INFO("get_speed...\n");
		//std::cout<<"vrt:"<<vrt<<"\n";
		v0=get_speed(xrf,vrt);
		//pot_mapt=pot_map.clone();
		// std::cout<<"v0:"<<v0<<"\n";
		clear_move_data();
		//pot_mapt=pot_map.clone();
		// ROS_INFO("add_mv_pot...\n");
		add_mv_pot(xri,obst_num);
		// ROS_INFO("set_grad...\n");
		set_grad(xri);
		//collision
		if(check_collision(xrf)){
			ROS_INFO("collision...\n");
			stop_flag = false;
			break;
		}
		//ロボットの命令速度算出
		float w,v;
		set_command_vel(xri,v0,v,w,th_t);
		/*std::cout<<"xri:"<<xri<<"\n";
		std::cout<<"pot_xri:"<<get_pot_xt(xri)<<"\n";
		std::cout<<"pot(x0,x1),(y0,y1):("<<pot_x0<<","<<pot_x1<<"),("<<pot_y0<<","<<pot_y1<<")\n";
		std::cout<<"grad(x,y):"<<grad_xt<<","<<grad_yt<<"\n";
		// std::cout<<"v0,vrt,w,th_t:"<<v0<<","<<vrt<<","<<w<<","<<th_t<<"\n";
		// ofss<<goal_time<<","<<xrf.x+cx<<","<<xrf.y+cy<<","<<v<<","<<w<<","<<th_t<<","<<std::endl;*/
		// std::cout<<"v,w,th_t:"<<v<<","<<w<<","<<th_t<<"\n";
		//ロボットの移動
		//mv_t:移動時間
		float l=v*mv_t;
		th_t=th_t+w*mv_t;
		xrf.x=xrf.x + l*cos(th_t);
		xrf.y=xrf.y + l*sin(th_t);
		float robot_x_tr,robot_y_tr;
		robot_x_tr = xrf.x;
		robot_y_tr = xrf.y;
		// ROS_INFO_STREAM("robot_x_tr =" <<robot_x_tr<<"\n");
		// ROS_INFO_STREAM("robot_y_tr =" <<robot_y_tr<<"\n");
		Trajectory.position.resize(i+1);
		Trajectory.header.stamp = ros::Time::now();
		Trajectory.header.seq = i;
		Trajectory.header.frame_id ="base_link";
		Trajectory.position[i].x=robot_x_tr;
		Trajectory.position[i].y=robot_y_tr;
		Trajectory.position[i].z=th_t;//theta
		// ROS_INFO_STREAM("Trajectory =" <<Trajectory<<"\n");
		i++;
		Trajectory.position.resize(i+1);
		//速度変化
		//ロボットの速度は1時遅れ系で変化すると仮定
		//目標値v0,現在速度vrt
		float Tr=0.25/1000;//マブチ3Vモータを参考
		//vrt=vrt+(v0-vrt)*(1-exp(-Tr*dt));
		vrt=vrt+(v0-vrt)*(exp(-Tr*mv_t));
		// std::cout<<"(1-exp(-Tr*dt)):"<<(1-exp(-Tr*mv_t))<<"\n";
		past_time(mv_t);		
		//debug
		// ROS_INFO_STREAM("test1"<<"\n");
		// set_pub_debug_images();//重い
		// ROS_INFO_STREAM("test2"<<"\n");
		rate.sleep();
		goal_time+=mv_t;
		// std::cout<<"goal_time:"<<goal_time<<"\n";
	}
	Trajectory.position.resize(i+1);
	pub_Trajectory.publish(Trajectory);
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
void trackingAvoidance::clear_mv_obstacle_data(){
	mv_obsts.clear();
	mv_obsts.resize(map_wi*map_hi/reso/reso);
	mv_obsts_size=0;
	mv_data_size=0;
}
void trackingAvoidance::end_set_mv_obstacle_data(){
	mv_obsts.resize(mv_obsts_size);
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
void trackingAvoidance::set_static_obstacle_data(const cv::Point2f& data){
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
// void trackingAvoidance::setting_testcondition(float reso){
void trackingAvoidance::setting_testcondition(){
	// grid_map
	// ROS_INFO("grid_map...\n");
	bool floor=false;
	int obst_num=0;
	int fast=0;
	for(int k =0; k < clstr.data.size(); k++){//クラスタ数
		if(clstr.twist[k].twist.linear.x !=0 || clstr.twist[k].twist.linear.y != 0
		&& sqrt(std::pow(clstr.twist[k].twist.linear.x,2.0)+std::pow(clstr.twist[k].twist.linear.y,2.0)) > 0.1)
			{
				std::cout<<"("<<k<<" : moving),\n";
				if( sqrt(std::pow(clstr.twist[k].twist.linear.x,2.0)+std::pow(clstr.twist[k].twist.linear.y,2.0))
			 	<= sqrt(std::pow(clstr.twist[fast].twist.linear.x,2.0)+std::pow(clstr.twist[fast].twist.linear.y,2.0)))
					{
						fast = k;
					}
					continue;
			}
		std::cout<<"("<<k<<" : static),\n";
	}
	obstacle_pt_x = clstr.data[fast].gc.x;
	obstacle_pt_y = clstr.data[fast].gc.y;
	if(!floor){
		cv::Point2f obst_data=cv::Point2f(-2.3,2.0);
		cv::Point2f obst_data2=cv::Point2f(2.3,2.0);
		// cv::Point2f obst_data=cv::Point2f(0.0,0.0);
		// cv::Point2f obst_data2=cv::Point2f(2.0,1.8);
		// cv::Point2f obst_data3=cv::Point2f(-1.0,1.0);
		// cv::Point2f obst_data4=cv::Point2f(1.0,-0.5);
		// cv::Point2f obst_data5=cv::Point2f(2.0,-1.0);
		// cv::Point2f obst_data6=cv::Point2f(-2.0,1.0);
		// cv::Point2f obst_data7=cv::Point2f(3.5,-2.0);
		// cv::Point2f obst_data=cv::Point2f(0.0,0.75);
		// cv::Point2f obst_data2=cv::Point2f(2.3,1.5);
		// cv::Point2f obst_data3=cv::Point2f(-2.3,1.5);
		// cv::Point2f obst_data=cv::Point2f(0.0,0.0);
		// cv::Point2f obst_data2=cv::Point2f(2.0,1.8);
		// cv::Point2f obst_data3=cv::Point2f(-1.0,1.0);
		// cv::Point2f obst_data4=cv::Point2f(1.0,-0.5);
		// cv::Point2f obst_data5=cv::Point2f(2.0,-1.0);
		// cv::Point2f obst_data6=cv::Point2f(-2.0,1.0);
		// cv::Point2f obst_data7=cv::Point2f(3.5,-2.0);
		// cv::Point2f obst_data=cv::Point2f(0.0,0.75);
		// cv::Point2f obst_data2=cv::Point2f(2.3,1.5);
		// cv::Point2f obst_data3=cv::Point2f(-2.3,1.5);
		// cv::Point2f obst_data=cv::Point2f(0.0,0.0);
		// cv::Point2f obst_data2=cv::Point2f(2.0,1.8);
		// cv::Point2f obst_data3=cv::Point2f(-1.0,1.0);
		// cv::Point2f obst_data4=cv::Point2f(1.0,-0.5);
		// cv::Point2f obst_data5=cv::Point2f(2.0,-1.0);
		// cv::Point2f obst_data6=cv::Point2f(-2.0,1.0);
		// cv::Point2f obst_data7=cv::Point2f(3.5,-2.0);
		// obst_num=6;
		// obst_num=6;
		//--set_static_obstacle_data(cv::Point2f& data)
		// set_static_obstacle_data(obst_data);
		// set_static_obstacle_data(obst_data2);
		// set_static_obstacle_data(obst_data3);
		// set_static_obstacle_data(obst_data4);
		// set_static_obstacle_data(obst_data5);
		// set_static_obstacle_data(obst_data6);
		// set_static_obstacle_data(obst_data7);
	}
	else{
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
		// }
	}
	// static potential_map
	// ROS_INFO("potential_map...\n");
	// create_pot_map();
	// set movin obstacle data
	// ROS_INFO("set moving obstacle data...\n");
	if(!floor){
		//--def mvObst
		float wo=0.20;
		float ho=0.20;
		int obst_size=(int)(wo/reso*2);
		float vx=-0.0;
		float vy=-0.10;
		obstacle_pt_theta = clstr.twist[fast].twist.angular.z;
		cv::Point2f x1=cv::Point2f(obstacle_pt_x-wo,obstacle_pt_y-ho);
		// cv::Point2f x1=cv::Point2f(0.0-wo,6.0-ho);
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
		ROS_INFO_STREAM("obstacle_pt_x =" <<obstacle_pt_x<<"\n");
		ROS_INFO_STREAM("obstacle_pt_y =" <<obstacle_pt_y<<"\n");
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
		// set_mv_obstacle_data(mvObst2,vx2,vy2);
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
void trackingAvoidance::set_mov_time(float time){
	mv_t=time;
}
void trackingAvoidance::set_command_limit(float dif_vel){
	// float d=0.138;
	// float d=0.310;
	float wheel_d=0.155;//車間半径
	max_w=dif_vel/(2*wheel_d);//dif_vel=vr-vl
}
bool trackingAvoidance::set_robot_param(float x,float y, float r,float vt0,float th_t0){
	xr.x=x;
	xr.y=y;
	rr=r;
	vrt=vt0;//初期速度
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
	cv::Mat n_temp = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_32FC1);
	pot_map=n_temp.clone();
	pot_mapt=n_temp.clone();
	grad_map[0]=n_temp.clone();
	grad_map[1]=n_temp.clone();
	debug_image = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC3);	
	mv_obsts.resize((int)(width/resolution*height/resolution));
}
void trackingAvoidance::apf(){
	// float H=clstr.widthInt;
	// float W=clstr.heightInt;
	// float reso=clstr.res;
	float H=6;//偶数正方形以外エラー
	float W=6;
	float reso=0.05;
    // apf
	// setting
	set_grid_param(W,H,reso);
	// std::cout<<"wait...\n";
	//center point
	float center_x = clstr.cp.x;
	float center_y = clstr.cp.y;
	cv::Point2f cpt=cv::Point2f(0.0,3.0);//縦軸-y 横軸-x
	//goal point
	// cv::Point2f goal_pt=cv::Point2f(7.5-5,7.5-5);
	float goal_x = goalOdom.pose.pose.position.x;
	float goal_y = goalOdom.pose.pose.position.y;
	// vfh cp
	float robot_vel,robot_angle;
	searchProcess(robot_vel,robot_angle);
	double robot_theta_position_y = dis_threshold*sin(robot_angle*M_PI/180);
	double robot_theta_position_x = dis_threshold*cos(robot_angle);
	ROS_INFO_STREAM("robot_angle =" <<robot_angle<<"\n");
	ROS_INFO_STREAM("robot_theta_x_1 =" <<robot_theta_position_x<<"\n");
	ROS_INFO_STREAM("robot_theta_y_1 =" <<robot_theta_position_y<<"\n");
	cv::Point2f robot_theta_pt=cv::Point2f(robot_theta_position_x,robot_theta_position_y);//cptが基準	
	// double robot_x_1=delta_time*robot_vel;
	cv::Point2f goal_pt=cv::Point2f(goal_x,goal_y);//cptが基準
	ROS_INFO_STREAM("goal_x =" <<goal_x<<"\n");
	ROS_INFO_STREAM("goal_y =" <<goal_y<<"\n");
	//set_param
	// std::cout<<"set_param...\n";
	set_center_point(cpt.x,cpt.y);
	// std::cout<<"set_param...\n";
	set_goal(robot_theta_pt);	
	// set_goal(goal_pt);
	//--set_robot_param(float x,float y, float r,float vt0,float th_t0)
	// if(!set_robot_param(-2.5,-2.5,0.3,0.2,0.0))//-M_PI/2))
	// if(!set_robot_param(5.0,0.0,0.2,0.2,M_PI/2))//)
	// if(!set_robot_param(0.0,-3.0,0.2,0.2,M_PI/2)){
	// if(!set_robot_param(0.0,0.0,0.16,0.1,0.0)){//ビーゴ余剰分含めて32✕32
	// if(!set_robot_param(0.0,0.0,0.16,0.1,M_PI/2)){//ビーゴ余剰分含めて32✕32x軸0半時計回り
	float robot_x = robotOdom.pose.pose.position.x;
	float robot_y = robotOdom.pose.pose.position.y;
	// ROS_INFO_STREAM("robot_x =" <<robot_x<<"\n");
	// ROS_INFO_STREAM("robot_y =" <<robot_y<<"\n");
    tf::Quaternion quater;
	double roll,pitch,yaw;
    quaternionMsgToTF(pre_robotOdom.pose.pose.orientation, quater);
    tf::Matrix3x3(quater).getRPY(roll,pitch,yaw);
    double robot_yaw = M_PI/2+ cur_angVel * delta_time + yaw;
	// ROS_INFO_STREAM("cur_vel =" <<cur_vel<<"\n");
	// ROS_INFO_STREAM("robot_yaw =" <<robot_yaw<<"\n");
	if(!set_robot_param(0.0,0.0,0.16,robot_vel,robot_yaw)){//ビーゴ余剰分含めて32✕32
		std::cout<<"Error: robot param\n";
		return ; 
    }
	//--set_command_limit(float dif_vel)
	set_command_limit(0.1);
	set_mov_time(0.05);
	
	// grid_map
	/*std::cout<<"grid_map...\n";*/
	setting_testcondition();
	// setting_testcondition(reso);
	//potential_map
	// std::cout<<"potential_map...\n";
	create_pot_map();//2回目
	end_set_mv_obstacle_data();
	//gradient_map
	// std::cout<<"gradient_map...\n";
	create_grad_map();
	//path_planning
	// std::cout<<"path_planning...\n";
	draw_path_mat();
	/*ROS_INFO("get_speed...\n");
	cv::Point2f xrft=cv::Point2f(-2.5,-2.5);
	float vrt0=0.3;
	float v=get_speed(xrft,vrt0);
	std::cout<<"v:"<<v<<"\n";
	for(int i=1;i<=5;i++){
		setting_RobotExpCondition(reso);
		create_pot_map();
		clear_mv_obstacle_data();
		setting_testcondition(reso);
		end_set_mv_obstacle_data();
	}*/
		// dynamixel_sdk_examples::SetPosition pubPanData;
	// dynamixel_sdk_examples::SetPosition pubTiltData;
	// pan id 2 center 180 2048 right 117.95 1400 left 241.69 2800 
	// tilt id 1 center 180 2048 right 146.25 1650 left 212.17 2450 
	// 一度あたり
	// 現角度
	double lamda_p=0.1,lamda_d=0.01;
	float robot_rotation = lamda_p*(robot_angle*M_PI/180 - imu_yaw - (robot_yaw-M_PI/2))+lamda_d*(-cur_cameraIMU.angular_velocity.z)/delta_time;
	ROS_INFO_STREAM("robot_angle =" <<robot_angle*M_PI/180<<"\n");
	ROS_INFO_STREAM("imu_yaw =" <<imu_yaw<<"\n");
	ROS_INFO_STREAM("robot_yaw =" <<robot_yaw<<"\n");
	ROS_INFO_STREAM("robot_rotation =" <<robot_rotation<<"\n");	
	double pan_rotation;
	// パンチルト目標姿勢
	ROS_INFO_STREAM("delta_time =" <<delta_time<<"\n");
	ROS_INFO_STREAM("Trajectory.header.seq =" <<Trajectory.header.seq<<"\n");
		if(Trajectory.position[Trajectory.position[(int)(delta_time/mv_t)].z].z-M_PI<=0){//右
		pan_rotation = (2048-1400)/(180-118)*((int)(obstacle_pt_theta+2*M_PI) - (Trajectory.position[(int)(delta_time/mv_t)].z-M_PI))*180/M_PI;//deg
		}
		else{//左
		pan_rotation = (2800-2048)/(242-180)*((int)(obstacle_pt_theta+2*M_PI) - (Trajectory.position[(int)(delta_time/mv_t)].z-M_PI))*180/M_PI;
		}
	robot_x = -lamda_p*((obstacle_pt_x-Trajectory.position[(int)(delta_time/mv_t)].x - robot_x)
			+(-obstacle_pt_y*((obstacle_pt_theta+2*M_PI) - (Trajectory.position[(int)(delta_time/mv_t)].z-M_PI))))-lamda_d*(robot_vel/delta_time);//-ty
	robot_y = -lamda_p*((obstacle_pt_y-Trajectory.position[(int)(delta_time/mv_t)].y - robot_y)
			+(-obstacle_pt_x*((obstacle_pt_theta+2*M_PI) - (Trajectory.position[(int)(delta_time/mv_t)].z-M_PI))))-lamda_d*(robot_vel/delta_time);//-tx
	ROS_INFO_STREAM("Trajectory.position[i].z =" <<Trajectory.position[(int)(delta_time/mv_t)].z<<"\n");
	ROS_INFO_STREAM("obstacle_pt_theta =" <<obstacle_pt_theta<<"\n");
	geometry_msgs::Twist robot_controler = controler(robot_vel, robot_rotation);
	publishData(robot_controler);
	ROS_INFO_STREAM("pan_rotation =" <<pan_rotation<<"\n");
	if(pan_rotation<2800||pan_rotation>1400)
	{pubPanData.id = 2;
	pubPanData.position = 2048 + (int)pan_rotation;
	pubTiltData.id = 1;
	pubTiltData.position = 2048;
	pub_pan.publish(pubPanData);
	// ROS_INFO("published_debug_pantilt\n");
	pub_tilt.publish(pubTiltData);
	}
	clear_mv_obstacle_data();
	std::cout<<"Done\n";
}