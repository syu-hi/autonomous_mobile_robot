#include<autonomous_mobile_robot/apf.h>

// 使っていない
bool APF::check_gridmap_format(cv::Mat& map){
	
	//check map size
	if(map.cols!=map_wi || map.rows!=map_hi)
		return false;
		
	//check map format
	if(map.depth()!=CV_8U || map.channels()!=1)
		return false;
		 
	return true;
}

// 使っていない
void APF::set_grid_map(cv::Mat& map){
	grid_map=map.clone();
}

void APF::set_obstacle_data(const cv::Point2f& data)
{
	cv::Point2i data_gp;

	if(trans_point(data,data_gp))
	{
		int ch_g = grid_map.channels();
		uint8_t *pgrid = grid_map.ptr<uint8_t>(data_gp.y);
		pgrid[data_gp.x * ch_g]++;
	}
	else
	{
		std::cout<<"ObstPoint is not in grid map\n";
	}
}

// 使っていない
void APF::clear_grid_map(void)
{
	cv::Mat m_temp = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC1);
	grid_map=m_temp.clone(); 
}


