#include <zone_filter.h>
Obstacle_Point::Obstacle_Point(cv::Point p){
	point = p;
}

Zone_Center::Zone_Center(cv::Point c, unsigned int i){
	center = c;
	index = i;
}

Segment::Segment(cv::Point c, cv::Point e){
	get_line_points(c, e);
}

void Segment::get_line_points(cv::Point c, cv::Point e){
	std::vector<cv::Point> line_points;
	int lx, ly, sx, sy;float ay, ax, by, bx;
	
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

Zone_Filter::Zone_Filter(){}

void Zone_Filter::get_edges(cv::Mat room_map){
	map = room_map.clone();
	map.convertTo(map,CV_8U);
	cv::Canny(map, map_edges, 0, 1, 3);
	normalize(map_edges, map_edges, 255, 0, cv::NORM_MINMAX, CV_8U);
	normalize(map, map, 255, 0, cv::NORM_MINMAX, CV_8U);
	/*cv::imshow("map_edges",map_edges);
	cv::waitKey(0);*/
}

void Zone_Filter::get_obstacle_edge_points(){
	
	//cv::imshow("map_edges",map_edges);
	cv::waitKey(0);
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

void Zone_Filter::get_center_points(std::vector<cv::Point> cell_centers){
	unsigned int index = 0;
	for(cv::Point c : cell_centers)
		zone_centers_array.push_back(Zone_Center(c,++index));
	
}

void Zone_Filter::fill_points(){
	
	for(int m=0; m<zone_centers_array.size();m++){
		//std::cout<<std::endl<<"Center index: "<<zone_centers_array[m].index<<std::endl;
		for(int q=0; q<obstacle_points_array.size();q++){
			Segment s(zone_centers_array[m].center, obstacle_points_array[q].point);
			bool obstacle_detected = false;
			if(s.point_array.size() > 2){
				for(int n=1; n<s.point_array.size()-1;n++){
					cv::Point* p;
					*p = s.point_array[n];
					if(map.at<uchar>(p->y, p->x) < 200){
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
	cv::imshow("out",out);
	cv::waitKey(0);
}

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

void Zone_Filter::vote_out(){
	std::set<unsigned int, std::greater<unsigned int> > voting;
	std::set<unsigned int, std::greater<unsigned int> > voted;
	while(1){
		for(Obstacle_Point o : obstacle_points_array){
			//count non voted obstacles
			std::vector<unsigned int> tmp;
			for(unsigned int u : o.exposed_to){
				if((voted.find(u) == voted.end()) || (!voted.empty())){
					tmp.push_back(u);
				}
			}
			if(tmp.size() > 1){
				for(unsigned int i : tmp)
					voting.insert(i);
			}
			else if(tmp.size() == 1){
				voted.insert(tmp[0]);
				voting.erase(tmp[0]);
			}
			//std::cout<<tmp.size()<<" ";
		}
		std::cout<<"There are "<<zone_centers_array.size()-voted.size()<<" zones could be elliminated."<<std::endl;
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
		voted.insert(min_p);
		voting.erase(min_p);
		if(min_p == -1)break;
		std::cout<<"Zone "<<min_p<<" elliminated."<<std::endl;
	}
}

void Zone_Filter::erase_tail(){

}
