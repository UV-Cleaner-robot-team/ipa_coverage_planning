#pragma once

#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <math.h>


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Description: Line segment between two points on a grid map.		   *
 * Constructor: Takes 2 pts in paramaters and assign the class members *
 * by calculating the distance and the line points on the grid.		   *
 * Members:															   *
 * 		Distance: Length of the segment in pixels.					   *
 * 		pointçarray: The list of points that the segment includes.	   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
class Segment{
	public:
	
	Segment(cv::Point c, cv::Point e);
	void get_line_points(cv::Point c, cv::Point e);
	float distanceCalculate(float x1, float y1, float x2, float y2);
	

	float distance;
	std::vector<cv::Point> point_array;
};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Description: Center point of the map zones				 		   *
 * Constructor: Takes the index of the zone and coordinates of the grid*
 * and assign the class members.									   *
 * Members:															   *
 * 		index: Zone index according to the decomposition output		   *
 * 		Center: Zone cebnter point.									   * 
 * 		exposed_points: List of the obstacle points exposed to the UV of*
 * 			the robot from this center.								    *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
class Zone_Center{
	public:
	
	Zone_Center(cv::Point c, unsigned int index);
	
	unsigned int index;
	cv::Point center;
	std::vector<cv::Point> exposed_points;
};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Description: Point on the obstacles edges.				 		   *
 * Constructor: Assign the point cordinates							   *
 * Members:															   *
 * 		point: Obstacle point	.									   * 
 * 		exposed_to: List of the zone indecies the obstcle point is	   *
 * 			exposed to.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
class Obstacle_Point{
	public:
	
	Obstacle_Point(cv::Point p);
	
	cv::Point point;
	std::vector<unsigned int> exposed_to;
};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Description: Main class. It 	elliminate zones that covers obstacle  *
 * 		edges that are already covered by UV from other zones and test *
 * 		the coverage rate.			 		   						   *
 * Constructor: Assign robot radius variable.						   *
 * Members:															   *
 * 		robot_radius: the robot radius.								   * 
 * 		map: The map grid; the obstacles and the unreacheable zones are*
 * 			in black rest is to be covered. 						   *
 * 		map_edges: Obstacles lines									   *
 * 		obstacle_points_array: The list of all the obstacle points in  *
 *			the map.												   *
 * 		zone_centers_array: The zone centers in the map.			   *
 * 		result_zone_list: The zone centers after eliminating non	   *
 * 			favorable zones.										   *
 * 		eliminated: Eliminated zone indecies.						   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
class Zone_Filter{
	public:
	
	Zone_Filter(){};
	Zone_Filter(unsigned int rr, cv::Mat room_map);
	void get_edges();
	void get_obstacle_edge_points();
	void get_center_points(std::vector<cv::Point> cell_centers);
	void fill_points(float d_max, float resolution);
	void draw(unsigned int index);
	void fill_draw_debug(unsigned int index);
	void test_coverage();
	void vote_out();
	void correct_pose_cordinates(unsigned int index);
	
	unsigned int robot_radius;
	cv::Mat map;
	cv::Mat map_edges;
	std::vector<Obstacle_Point> obstacle_points_array;
	std::vector<Zone_Center> zone_centers_array;
	std::vector<cv::Point> result_zone_list;
	std::set<unsigned int, std::greater<unsigned int>> eliminated;
};
