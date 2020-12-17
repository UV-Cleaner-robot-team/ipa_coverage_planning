#pragma once

#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>

class Segment{
	public:
	
	Segment(cv::Point c, cv::Point e);
	void get_line_points(cv::Point c, cv::Point e);
	
	std::vector<cv::Point> point_array;
};

class Zone_Center{
	public:
	
	Zone_Center(cv::Point c, unsigned int index);
	
	unsigned int index;
	cv::Point center;
	std::vector<cv::Point> exposed_points;
};

class Obstacle_Point{
	public:
	
	Obstacle_Point(cv::Point p);
	
	cv::Point point;
	std::vector<unsigned int> exposed_to;
};

class Zone_Filter{
	public:
	
	Zone_Filter();
	void get_edges(cv::Mat room_map);
	void get_obstacle_edge_points();
	void get_center_points(std::vector<cv::Point> cell_centers);
	void fill_points();
	void draw(unsigned int index);
	void fill_draw_debug(unsigned int index);
	void vote_out();
	void erase_tail();
	
	cv::Mat map;
	cv::Mat map_edges;
	std::vector<Obstacle_Point> obstacle_points_array;
	std::vector<Zone_Center> zone_centers_array;
};
