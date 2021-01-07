#include <zone_filter.h>


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Class constructor.										   *
 * Paramaters: 														   *
 * 		c: Center point.											   *
 * 		e: Edge point. (They could be inverted. Implemented like this  *
 * 			for future reasons.) 									   *
 * Return: No return.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
Segment::Segment(cv::Point c, cv::Point e){
	get_line_points(c, e);
	distance = distanceCalculate(c.x, c.y, e.x, e.y);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Compute a set  of points forming a straight line from	   *
 * 		point c to point e.											   *
 * Paramaters: 														   *
 * 		c: Center point.											   *
 * 		e: Edge point. (They could be inverted. Implemented like this  *
 * 			for future reasons.) 									   *
 * Return: No return.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void Segment::get_line_points(cv::Point c, cv::Point e){
	int lx, ly, sx, sy;float ay, ax, by, bx;
	//Calculate the segment start point and the length of the longest 
	//X,Y decomposition.
	if(c.x > e.x){
		lx = c.x - e.x;
		sx = e.x;
	}
	else{
		lx = e.x - c.x;
		sx = c.x;
	}
	
	if(c.y > e.y){
		ly = c.y - e.y;
		sy = e.y;
	}
	else{
		ly = e.y - c.y;
		sy = c.y;
	}
	
	//Calculate the segment equation and the approximation of the points.
	if(ly == 0){
		for(int u=0;u<lx;u++){
			point_array.push_back(cv::Point(sx+u,sy));
		}
	}
	else if(lx == 0){
		for(int v=0;v<ly;v++){
			point_array.push_back(cv::Point(sx,sy+v));
		}
	}
	else if(lx >= ly){
		ax = ((float)(c.y-e.y) / (float)(c.x-e.x));
		bx = c.y - ax*c.x;
		for(int u=0;u<lx;u++){
			int x = sx+u;
			int y = std::round(ax*x+bx);
			point_array.push_back(cv::Point(x,y));
		}
	}
	else if(lx < ly){
		ay = ((float)(c.x-e.x) / (float)(c.y-e.y));
		by = c.x - ay*c.y;
		for(int v=0;v<ly;v++){
			int y = sy+v;
			int x = std::round(ay*y+by);
			point_array.push_back(cv::Point(x,y));
		}
	}

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Compute the distance from point c to point e.			   *
 * Paramaters: 														   *
 * 		c: Center point.											   *
 * 		e: Edge point. (They could be inverted. Implemented like this  *
 * 			for future reasons.) 									   *
 * Return: No return.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
float Segment::distanceCalculate(float x1, float y1, float x2, float y2){
	//Euclidian distance.
	double x = x1 - x2;
	double y = y1 - y2;
	double dist;

	dist = pow(x, 2) + pow(y, 2);
	dist = sqrt(dist);                  

	return dist;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Class constructor.										   *
 * Paramaters: 														   *
 * 		c: Center point of the zone.								   *
 * 		i: The zone index.											   *
 * Return: No return.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
Zone_Center::Zone_Center(cv::Point c, unsigned int i){
	center = c;
	index = i;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Class constructor.										   *
 * Paramaters: 														   *
 * 		p: Obstacle point.											   *
 * Return: No return.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
Obstacle_Point::Obstacle_Point(cv::Point p){
	point = p;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Class constructor.										   *
 * Paramaters: 														   *
 * 		rr: Robot radius.											   *
 * Return: No return.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
Zone_Filter::Zone_Filter(unsigned int rr, cv::Mat room_map){
	robot_radius = rr;
	map = room_map.clone();
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Assign Extract contours from "map" & save them in 		   *
 * 		"map_edges".												   *
 * Paramaters: 														   *
 * 		room_map: 													   *
 * Return: No return.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void Zone_Filter::get_edges(){
	map.convertTo(map,CV_8U);
	normalize(map, map, 255, 0, cv::NORM_MINMAX, CV_8U);
	cv::Canny(map, map_edges, 0, 1, 3);
	normalize(map_edges, map_edges, 255, 0, cv::NORM_MINMAX, CV_8U);
	/*cv::imshow("map_edges",map_edges);
	cv::waitKey(0);*/
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Fill the "obstacle_points_array" with points from 		   *
 * 		"map_edges".													   *
 * Paramaters: 	 													   *
 * Return: No return.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void Zone_Filter::get_obstacle_edge_points(){
	//Create an "Obstacle_Point" object for every point on the map edges
	//and push them to the array.
	for(int u=0; u<map_edges.rows;u++)
		for(int v=0; v<map_edges.cols;v++)
			if(map_edges.at<uchar>(u,v) > 250){
				obstacle_points_array.push_back(Obstacle_Point(cv::Point(v,u)));
			}
	/*cv::Mat tmp = cv::Mat(map_edges.size(), CV_8U, cv::Scalar(0,0,0));
	for(Obstacle_Point o : obstacle_points_array)
		cv::circle(tmp, o.point, 1, cv::Scalar(150,150,50), cv::FILLED, cv::LINE_8);
	cv::imshow("tmp",tmp);
	cv::waitKey(0);*/
	
	
	/*cv::Mat out = map.clone();
	Segment s(cv::Point(600,300), cv::Point(400,300));
	for(cv::Point p : s.point_array){
		out.at<uchar>(p.x, p.y) = 100;
	}
	cv::imshow("out",out);
	cv::waitKey(0);*/
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Fill the "zone_centers_array" with points from the 	   *
 * 		decomposition algorithme output.							   *
 * Paramaters: 	 													   *
 * Return: No return.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void Zone_Filter::get_center_points(std::vector<cv::Point> cell_centers){
	//Create an "Zone_Center" object for every zone center and push them
	//to the array.
	unsigned int index = 0;
	for(cv::Point c : cell_centers)
		zone_centers_array.push_back(Zone_Center(c,++index));
	
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Fill the "zone_centers_array" with points from the 	   *
 * 		decomposition algorithme output.							   *
 * Paramaters: 	 													   *
 * 		d_max: La portée des rayons UV.								   *
 * 		resolution: La resolution du map.							   *
 * Return: No return.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void Zone_Filter::fill_points(float d_max, float resolution){
	float d_max_p = d_max / resolution;
	for(int m=0; m<zone_centers_array.size();m++){
		for(int q=0; q<obstacle_points_array.size();q++){
			Segment s(zone_centers_array[m].center, obstacle_points_array[q].point);
			bool obstacle_detected = false;
			if(d_max_p < s.distance)
				obstacle_detected = true;
			else if(s.point_array.size() > 2){
				for(int n=1; n<s.point_array.size()-1;n++){
					cv::Point p = s.point_array[n];
					if(map.at<uchar>(p.y, p.x) < 200){
						obstacle_detected = true;
						break;
					}
				}
			}
			
			if(!obstacle_detected){
				zone_centers_array[m].exposed_points.push_back(obstacle_points_array[q].point);
				obstacle_points_array[q].exposed_to.push_back(zone_centers_array[m].index);
			}
		}
		//std::cout<<zone_centers_array[m].exposed_points.size()<<std::endl;
	}
	
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Visualisation of zone centers and their respective covered*
 *  		obstacle edges.											   *
 * Paramaters: 	 													   *
 * 		index: Zone index to visualize.								   *
 * Return: No return.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void Zone_Filter::draw(unsigned int index){
	cv::Mat out = map.clone();
	cv::circle(out, zone_centers_array[index-1].center, 3, cv::Scalar(150,150,50), cv::FILLED, cv::LINE_8);
	for(cv::Point p : zone_centers_array[index-1].exposed_points){
		out.at<uchar>(p) = 130;
	}
	/*Segment s(cv::Point(0,50), cv::Point(400,300));
	std::cout<<s.point_array.size()<<std::endl;
	for(cv::Point p : s.point_array){
		out.at<uchar>(p.x, p.y) = 100;
	}*/
	cv::imshow("Zone " + std::to_string(index) ,out);
	cv::waitKey(0);
	cv::destroyAllWindows();
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Debugging the fill_points function.					   *
 * Paramaters: 	 													   *
 * 		index: Zone index to visualize.								   *
 * Return: No return.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void Zone_Filter::fill_draw_debug(unsigned int index){
	cv::Point c(100,100);
	cv::Mat out = map_edges.clone();
	for(int q=0; q<obstacle_points_array.size();q++){
		out = map_edges.clone();
		cv::circle(out, c, 3, cv::Scalar(150,150,50), cv::FILLED, cv::LINE_8);
		cv::circle(out, obstacle_points_array[q].point, 3, cv::Scalar(150,150,50), cv::FILLED, cv::LINE_8);
		Segment s(obstacle_points_array[q].point, c);
		for(cv::Point p : s.point_array){
			out.at<uchar>(p) = 150;
		}
		cv::imshow("out",out);
		cv::waitKey(0);
	}
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Output the rate of coverage of obstacle edges.			   *
 * Paramaters: 	 													   *
 * Return: No return.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void Zone_Filter::test_coverage(){
	unsigned long int not_covered = 0;
	for(Obstacle_Point o : obstacle_points_array){
		if(o.exposed_to.size() == 0) not_covered++;
	}
	
	std::cout<<"Coverage rate: "<<100.0*(1.0-(float)((float)not_covered/obstacle_points_array.size()))<<std::endl;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Elliminate zone centers that covers already covered 	   *
 * obstacles.														   *
 * Paramaters: 	 													   *
 * 		index: Zone index to visualize.								   *
 * Return: No return.												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void Zone_Filter::vote_out(){
	std::set<unsigned int, std::greater<unsigned int>> voted;
	while(1){
		for(Obstacle_Point o : obstacle_points_array){
			//count non voted obstacles
			unsigned int tmp;int n = 0;
			for(unsigned int u : o.exposed_to){
				if((voted.find(u) == voted.end()) || (!voted.empty())){
					tmp = u;n++;
				}
			}
			if(n == 1){
				voted.insert(tmp);
			}
			//std::cout<<tmp.size()<<" ";
		}
		//std::cout<<"There are "<<zone_centers_array.size()-voted.size()<<" zones could be elliminated."<<std::endl;
		//for(unsigned int p : voted)std::cout<<p<<" ";
		//std::cout<<std::endl;
		//Get minimum exposed points zone to elliminate.
		long int min_v = INT_MAX;int min_p = -1;
		for(int m=0; m<zone_centers_array.size();m++){
			if((zone_centers_array[m].exposed_points.size() < min_v) && (voted.find(zone_centers_array[m].index) == voted.end())){
				min_v = zone_centers_array[m].exposed_points.size();
				min_p = zone_centers_array[m].index;
			}
		}
		if(min_p == -1)break;
		voted.insert(min_p);
		//std::cout<<"Zone "<<min_p<<" elliminated."<<std::endl;
		eliminated.insert(min_p);
	}
	
	std::cout<<"Eliminated zones: ";
	for(unsigned int x : eliminated){
		std::cout<<x<<" ";
		zone_centers_array.erase(zone_centers_array.begin()+x-1);
	}
	std::cout<<std::endl;
	for(Zone_Center c : zone_centers_array)
		if(eliminated.find(c.index) == eliminated.end())result_zone_list.push_back(c.center);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Function: Correct poses of unreacheable zone centers.               *
 * Paramaters:                                                         *
 * 		index: Zone index to correct.                                  *
 * Return: No return.                                                  *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void Zone_Filter::correct_pose_cordinates(unsigned int index){
	unsigned int robot_footprint_width = (unsigned int)((double)robot_radius * 0.6);
	cv::Rect roi(cv::Point(zone_centers_array[index-1].center.x-robot_footprint_width,zone_centers_array[index-1].center.y-robot_footprint_width),
								cv::Point(zone_centers_array[index-1].center.x+robot_footprint_width,zone_centers_array[index-1].center.y+robot_footprint_width));
	cv::Mat tmp = map(roi);
	long int x=0,y=0;unsigned int c=0;
	for(int i = 0;i<tmp.cols;i++)
		for(int j = 0;j<tmp.rows;j++){
			if(tmp.at<uchar>(i, j) == 0){
				c++;
				x = x + j;
				y = y + i;
			}
		}
	cv::imshow("tmp",tmp);
	std::cout<<"x = "<<x/c-robot_footprint_width<<" y = "<<y/c-robot_footprint_width<<std::endl;
	x = x/c + robot_footprint_width;
	y = y/c + robot_footprint_width;
	
	std::cout<<"x = "<<zone_centers_array[index-1].center.x<<" y = "<<zone_centers_array[index-1].center.y<<std::endl;
	zone_centers_array[index-1].center.x = zone_centers_array[index-1].center.x + x;
	zone_centers_array[index-1].center.y = zone_centers_array[index-1].center.y + y;
	std::cout<<"x = "<<zone_centers_array[index-1].center.x<<" y = "<<zone_centers_array[index-1].center.y<<std::endl;
}
