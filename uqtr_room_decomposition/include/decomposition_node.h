#pragma once

// Ros specific
#include <ros/ros.h>
// OpenCV specific
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
// standard c++ libraries
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
// messages
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
// specific from this package
#include <ipa_room_exploration/boustrophedon_explorator.h>
#include <zone_filter.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// remove unconnected, i.e. inaccessible, parts of the room (i.e. obstructed by furniture), only keep the room with the largest area
bool removeUnconnectedRoomParts(cv::Mat& room_map);
//Write the zone centers array real cordinates in a csv file.
void write_csv(std::vector<cv::Point> cell_centers, std::string csv_file_path, cv::Point2d map_origin, double map_resolution);
//Move the robot to the x,y location using the move base actionS erver.
void move_to(MoveBaseClient& ac, float x, float y, unsigned int& seq, unsigned int i);
//Calculate euclidian distance between two opencv points.
float euclidian_distance(cv::Point x, cv::Point y);
