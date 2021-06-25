#include <decomposition_node.h>

#include "uqtr_local_coverage/lc_service.h"


int main(int argc, char **argv){
	//Initialization.
	ros::init(argc, argv, "uqtr_room_decomposition");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");

	//Read ROS Pram configurations from the launch file.
	std::string map_image_path;
	p_nh.param<std::string>("map_image_path", map_image_path,"");
	std::string csv_file_path;
	p_nh.param<std::string>("csv_file_path", csv_file_path,"~/");
	double min_cell_area;
	p_nh.param("min_cell_area", min_cell_area, 1.0);
	double min_cell_width;
	p_nh.param("min_cell_width", min_cell_width, 5.0);
	double rotation_offset;
	p_nh.param("rotation_offset", rotation_offset, 0.0);
	int map_correction_closing_neighborhood_size;
	p_nh.param("map_correction_closing_neighborhood_size", map_correction_closing_neighborhood_size, 2);
	double map_resolution;
	p_nh.param("resolution", map_resolution, 0.05);
	std::vector<double> origin (3,0);
	p_nh.param("origin", origin, origin);
	double max_uv_distance_range;
	p_nh.param("max_uv_distance_range", max_uv_distance_range, 50.0);
	double robot_radius_m;unsigned int robot_radius;
	p_nh.param("robot_radius", robot_radius_m, 1.0);
	robot_radius = (unsigned int)(robot_radius_m / map_resolution);

	//Decoposition algoritme pre-calculations.
	const cv::Point2d map_origin(origin[0], origin[1]);
	cv::Mat room_map = cv::imread(map_image_path,0);
	room_map.convertTo(room_map,CV_8UC1);
	cv::Mat map;
	cv::flip(room_map, map, 0);
	
	//make non-white pixels black
	for (int y = 0; y < map.rows; y++)
	{
		for (int x = 0; x < map.cols; x++)
		{
			//find not reachable regions and make them black
			if (map.at<unsigned char>(y, x) < 250)
			{
				map.at<unsigned char>(y, x) = 0;
			}
			//else make it white
			else
			{
				map.at<unsigned char>(y, x) = 255;
			}
		}
	}
	ROS_INFO("%s%d%s%d","map-size: ", map.rows, "x", map.cols);
	// closing operation to neglect inaccessible areas and map errors/artifacts
	cv::Mat temp;
	cv::erode(map, temp, cv::Mat(), cv::Point(-1, -1), map_correction_closing_neighborhood_size);
	cv::dilate(temp, map, cv::Mat(), cv::Point(-1, -1), map_correction_closing_neighborhood_size);
	const bool room_not_empty = removeUnconnectedRoomParts(map);
	if (room_not_empty == false)
		ROS_WARN("%s","the requested room is too small for generating exploration trajectories.");
	
	//Decompose the map into zones unsing the boustrophedon decomposition algorithm.
	cv::Mat R;
	cv::Rect bbox;
	cv::Mat rotated_room_map;
	std::vector<GeneralizedPolygon> cell_polygons;
	std::vector<cv::Point> polygon_centers;
	BoustrophedonExplorer boustrophedon_explorer;
	boustrophedon_explorer.computeCellDecompositionWithRotation(map, map_resolution, min_cell_area, ((int)(min_cell_width/map_resolution)), 0., R, bbox, rotated_room_map, cell_polygons, polygon_centers, map_origin);
	
	
	//Eliminate useless zones and improve the decomposition.
	Zone_Filter zf(robot_radius, map);
	
	zf.get_edges();
	zf.get_obstacle_edge_points();
	ROS_INFO("%lu%s", zf.obstacle_points_array.size(),  " obstacle points detected.");
	zf.get_center_points(polygon_centers);
	zf.fill_points(max_uv_distance_range, map_resolution);
	float rate = zf.test_coverage();
	ROS_INFO("2D coverage rate:  %.3f ", rate);
	std::set<unsigned int, std::greater<unsigned int>> eliminated;
	eliminated = zf.vote_out();
	for(unsigned int x: eliminated)
		ROS_INFO("Zone %u is eliminated.", x);
	
	//Save the zone centers cordinates in csv file.
	write_csv(zf.result_zone_list,csv_file_path,map_origin,map_resolution);
	
	//Initialization of move_base.
	MoveBaseClient ac("/move_base", true);
	while(!ac.waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for the move_base action server to come up");
	
	//Drive the robot to zone centers and simultaneously visulize the 
	//zone center and the covered obstacles.
	unsigned int seq = 0;//The move base message header sequence.
	
	//initial position of the robot.
	int position_x = (int)map_origin.x * (int)(-1/ map_resolution);
	int position_y = (int)map_origin.y * (int)(-1/ map_resolution);
	cv::Point pixel_origin(position_x, position_y);
	
	std::vector<float> distances;
	std::vector<unsigned int> sorted_indecies;
	std::vector<unsigned int> remaining_indecies;
	
	//Get all the zone centers respective distances to the origin.
	for(int i=0;i<zf.zone_centers_array.size();i++){	
		distances.push_back(euclidian_distance(zf.zone_centers_array[i].center, pixel_origin));
		remaining_indecies.push_back(i+1);
	}
	
	//Find the closest zone center to the origin & marked as the first 
	//zone to move to.
	unsigned int index = remaining_indecies[std::min_element(distances.begin(), distances.end()) - distances.begin()];
	sorted_indecies.push_back(index);
	std::vector<unsigned int>::iterator it = std::find(remaining_indecies.begin(), remaining_indecies.end(), index);
	remaining_indecies.erase(it);
	
	//Find the sorted set of the center zones by the nearest
	for(int i=1;i<=zf.zone_centers_array.size();i++){
		distances.erase (distances.begin(), distances.end());
		unsigned int last_sorted = sorted_indecies.back();
		for(unsigned int x: remaining_indecies){
			distances.push_back(euclidian_distance(zf.zone_centers_array[last_sorted-1].center, 
							zf.zone_centers_array[x-1].center));
		}
		
		index = remaining_indecies[std::min_element(distances.begin(), distances.end()) - distances.begin()];
		sorted_indecies.push_back(index);
		const std::vector<unsigned int>::iterator it1 = std::find(remaining_indecies.begin(), remaining_indecies.end(), index);
		if(it1 != remaining_indecies.end())
			remaining_indecies.erase(it1);
	}
	
	//Move the robot between zones.
	for(unsigned int i : sorted_indecies){
		zf.correct_pose_cordinates(i);
		//zf.draw(i);
		float x = (float)(zf.zone_centers_array[i-1].center.x)*map_resolution+map_origin.x;
		float y = (float)(zf.zone_centers_array[i-1].center.y)*map_resolution+map_origin.y;
		move_to(ac, x, y, seq, i);
		
		uqtr_local_coverage::lc_service srv_object;
		srv_object.request.x = x;
		srv_object.request.y = y;
		if (ros::service::call("/uqtr_local_coverage/lc_service", srv_object)){
			ROS_INFO("3D coverage rate in zone %d:  %.3f %%", i, rate);
		}
	}
	//Show remaining non covered obstacles.
	zf.show_non_covered();
	return 0;
}

float euclidian_distance(cv::Point x, cv::Point y){
	return sqrt(pow(x.x - y.x, 2) + pow(x.y - y.y, 2));
}

void move_to(MoveBaseClient& ac, float x, float y, unsigned int& seq, unsigned int i){
		move_base_msgs::MoveBaseGoal goal;
		
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.header.seq = seq++;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.pose.position.x = x;
		goal.target_pose.pose.position.y = y;
		goal.target_pose.pose.position.z = 0;
		goal.target_pose.pose.orientation.w = 1.0;
		goal.target_pose.pose.orientation.z = 0;
		goal.target_pose.pose.orientation.x = 0;
		goal.target_pose.pose.orientation.y = 0;
		ROS_INFO("Going to zone %d [x = %f,\ty = %f]", i, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
		ac.sendGoal(goal);
		ac.waitForResult();
}

// remove unconnected, i.e. inaccessible, parts of the room (i.e. obstructed by furniture), only keep the room with the largest area
bool removeUnconnectedRoomParts(cv::Mat& room_map){
	
	// create new map with segments labeled by increasing labels from 1,2,3,...
	cv::Mat room_map_int(room_map.rows, room_map.cols, CV_32SC1);
	for (int v=0; v<room_map.rows; ++v){
		for (int u=0; u<room_map.cols; ++u){
			if (room_map.at<uchar>(v,u) == 255)
				room_map_int.at<int32_t>(v,u) = -100;
			else
				room_map_int.at<int32_t>(v,u) = 0;
		}
	}

	std::map<int, int> area_to_label_map;	// maps area=number of segment pixels (keys) to the respective label (value)
	int label = 1;
	for (int v=0; v<room_map_int.rows; ++v)
	{
		for (int u=0; u<room_map_int.cols; ++u)
		{
			if (room_map_int.at<int32_t>(v,u) == -100)
			{
				const int area = cv::floodFill(room_map_int, cv::Point(u,v), cv::Scalar(label), 0, 0, 0, 8 | cv::FLOODFILL_FIXED_RANGE);
				area_to_label_map[area] = label;
				++label;
			}
		}
	}
	// abort if area_to_label_map.size() is empty
	if (area_to_label_map.size() == 0)
		return false;

	// remove all room pixels from room_map which are not accessible
	const int label_of_biggest_room = area_to_label_map.rbegin()->second;
	for (int v=0; v<room_map.rows; ++v)
		for (int u=0; u<room_map.cols; ++u)
			if (room_map_int.at<int32_t>(v,u) != label_of_biggest_room)
				room_map.at<uchar>(v,u) = 0;

	return true;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Write zone centers cordinates in a csv file.			   *
 * Paramaters: 														   *
 * 		cell_centers: Center point.									   *
 * 		csv_file_path: Edge point. (They could be inverted. Implemented*
 * 		like this.													   *
 * 			for future reasons.) 									   *
 * 		map_origin: Map origin in meters(In .yaml map file).		   *
 * 		map_resolution: Map resolution(In .yaml map file).			   *
 * Return: No return.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void write_csv(std::vector<cv::Point> cell_centers, std::string csv_file_path, cv::Point2d map_origin, double map_resolution){
	std::ofstream csv(csv_file_path);
	for(cv::Point2d p: cell_centers)
		csv<<map_resolution*p.x+map_origin.x<<","<<map_resolution*p.y+map_origin.y<<"\n";
	csv.close();
}

