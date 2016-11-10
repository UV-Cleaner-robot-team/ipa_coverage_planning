#include <ipa_room_exploration/boustrophedon_explorator.h>

// Constructor
boustrophedonExplorer::boustrophedonExplorer()
{

}

// Function that creates a room exploration path for the given map, by using the morse cellular decomposition method proposed in
//
// "H. Choset, E. Acar, A. A. Rizzi and J. Luntz,
// "Exact cellular decompositions in terms of critical points of Morse functions," Robotics and Automation, 2000. Proceedings.
// ICRA '00. IEEE International Conference on, San Francisco, CA, 2000, pp. 2270-2277 vol.3."
//
// This method takes the given map and separates it into several cells. Each cell is obstacle free and so allows an
// easier path planning. For each cell then a boustrophedon path is planned, which goes up, down and parallel to the
// upper and lower boundaries of the cell, see the referenced paper for details. This function does the following steps:
//	I.	Sweep a slice (a morse function) trough the given map and check for connectivity of this line,
//		i.e. how many connected segments there are. If the connectivity increases, i.e. more segments appear,
//		an IN event occurs that opens new separate cells, if it decreases, i.e. segments merge, an OUT event occurs that
//		merges two cells together. If an event occurs, the algorithm checks along the current line for critical points,
//		that are points that trigger the events. From these the boundary of the cells are drawn, starting from the CP
//		and going left/right until a black pixel is hit.
//	II.	After all cells have been determined by the sweeping slice, the algorithm finds these by using cv::findContours().
//		This gives a set of points for each cell, that are used to create a generalizedPolygon out of each cell.
//	III. After all polygons have been created, plan the path trough all of them for the field of view s.t. the whole area
//		 is covered. To do so, first a global path trough all cells is generated, using the traveling salesmen problem
//		 formulation. This produces an optimal visiting order of the cells. Next for each cell a boustrophedon path is
//		 determined, which goes back and forth trough the cell and between the horizontal paths along the boundaries of
//		 the cell, what ensures that the whole area of the cell is covered. The startpoint of the cell-path is determined
//		 by the endpoint of the previous cell, s.t. the distance between two cell-paths is minimized.
void boustrophedonExplorer::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float robot_radius,
		const float map_resolution, const geometry_msgs::Pose2D starting_position,
		const geometry_msgs::Polygon room_min_max_coordinates, const cv::Point2d map_origin,
		const float fow_fitting_circle_radius, const int path_eps)
{
	// *********************** I. Sweep a slice trough the map and mark the found cell boundaries. ***********************
	// create a map copy to mark the cell boundaries
	cv::Mat cell_map = room_map.clone();

	// find smallest y-value for that a white pixel occurs, to set initial y value and find initial number of segments
	size_t y_start = 0;
	int n_start = 0;
	bool found = false, obstacle = false;
	for(size_t y=0; y<room_map.rows; ++y)
	{
		for(size_t x=0; x<room_map.cols; ++x)
		{
			if(room_map.at<uchar>(y,x) == 255 && found == false)
			{
				y_start = y;
				found = true;
			}
			else if(found == true && obstacle == false && room_map.at<uchar>(y,x) == 0)
			{
				++n_start;
				obstacle = true;
			}
			else if(found == true && obstacle == true && room_map.at<uchar>(y,x) == 255)
			{
				obstacle = false;
			}
		}

		if(found == true)
			break;
	}

	// swipe trough the map and detect critical points
	int previous_number_of_segments = n_start;
	for(size_t y=y_start+1; y<room_map.rows; ++y) // start at y_start+1 because we know number of segments at y_start
	{
		int number_of_segments = 0; // int to count how many segments at the current slice are
		bool obstacle_hit = false; // bool to check if the line currently hit an obstacle, s.t. not all black pixels trigger an event
		bool hit_white_pixel = false; // bool to check if a white pixel has been hit at the current slice, to start the slice at the first white pixel

		for(size_t x=0; x<room_map.cols; ++x)
		{
			if(room_map.at<uchar>(y,x) == 255 && hit_white_pixel == false)
				hit_white_pixel = true;

			else if(hit_white_pixel == true)
			{
				if(obstacle_hit == false && room_map.at<uchar>(y,x) == 0) // check for obstacle
				{
					++number_of_segments;
					obstacle_hit = true;
				}
				else if(obstacle_hit == true && room_map.at<uchar>(y,x) == 255) // check for leaving obstacle
				{
					obstacle_hit = false;
				}
			}
		}

		// reset hit_white_pixel to use this Boolean later
		hit_white_pixel = false;

		// check if number of segments has changed --> event occurred
		if(previous_number_of_segments < number_of_segments) // IN event
		{
			// check the current slice again for critical points
			for(int x=0; x<room_map.cols; ++x)
			{
				if(room_map.at<uchar>(y,x) == 255 && hit_white_pixel == false)
					hit_white_pixel = true;

				else if(hit_white_pixel == true && room_map.at<uchar>(y,x) == 0)
				{
					// check over black pixel for other black pixels, if none occur a critical point is found
					bool critical_point = true;
					for(int dx=-1; dx<=1; ++dx)
						if(room_map.at<uchar>(y-1,x+dx) == 0)
							critical_point = false;

					// if a critical point is found mark the separation, note that this algorithm goes left and right
					// starting at the critical point until an obstacle is hit, because this prevents unnecessary cells
					// behind other obstacles on the same y-value as the critical point
					if(critical_point == true)
					{
						// to the left until a black pixel is hit
						for(int dx=-1; x+dx>0; --dx)
						{
							if(cell_map.at<uchar>(y,x+dx) == 255)
								cell_map.at<uchar>(y,x+dx) = 0;
							else if(cell_map.at<uchar>(y,x+dx) == 0)
								break;
						}

						// to the right until a black pixel is hit
						for(int dx=1; x+dx<room_map.cols; ++dx)
						{
							if(cell_map.at<uchar>(y,x+dx) == 255)
								cell_map.at<uchar>(y,x+dx) = 0;
							else if(cell_map.at<uchar>(y,x+dx) == 0)
								break;
						}
					}
				}
			}
		}
		else if(previous_number_of_segments > number_of_segments) // OUT event
		{
			// check the previous slice again for critical points --> y-1
			for(int x=0; x<room_map.cols; ++x)
			{
				if(room_map.at<uchar>(y-1,x) == 255 && hit_white_pixel == false)
					hit_white_pixel = true;

				else if(hit_white_pixel == true && room_map.at<uchar>(y-1,x) == 0)
				{
					// check over black pixel for other black pixels, if none occur a critical point is found
					bool critical_point = true;
					for(int dx=-1; dx<=1; ++dx)
						if(room_map.at<uchar>(y,x+dx) == 0) // check at side after obstacle
							critical_point = false;

					// if a critical point is found mark the separation, note that this algorithm goes left and right
					// starting at the critical point until an obstacle is hit, because this prevents unnecessary cells
					// behind other obstacles on the same y-value as the critical point
					if(critical_point == true)
					{
						// to the left until a black pixel is hit
						for(int dx=-1; x+dx>0; --dx)
						{
							if(cell_map.at<uchar>(y-1,x+dx) == 255)
								cell_map.at<uchar>(y-1,x+dx) = 0;
							else if(cell_map.at<uchar>(y-1,x+dx) == 0)
								break;
						}

						// to the right until a black pixel is hit
						for(int dx=1; x+dx<room_map.cols; ++dx)
						{
							if(cell_map.at<uchar>(y-1,x+dx) == 255)
								cell_map.at<uchar>(y-1,x+dx) = 0;
							else if(cell_map.at<uchar>(y-1,x+dx) == 0)
								break;
						}
					}
				}
			}
		}

		// save the found number of segments
		previous_number_of_segments = number_of_segments;
	}
	cv::imshow("cells", cell_map);
	cv::waitKey();

	// *********************** II. Find the separated cells. ***********************
	std::vector<std::vector<cv::Point> > cells;
	cv::findContours(cell_map, cells, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//	 testing
//	cv::Mat black_map = cv::Mat(cell_map.rows, cell_map.cols, cell_map.type(), cv::Scalar(0));
//	for(size_t i=0; i<cells.size(); ++i)
//	{
//		for(size_t j=0; j<cells[i].size(); ++j)
//		{
//			cv::circle(black_map, cells[i][j], 2, cv::Scalar(127), CV_FILLED);
//			cv::imshow("contours", black_map);
//			cv::waitKey();
//		}
//	}

	// create generalized Polygons out of the contours to handle the cells
	std::vector<generalizedPolygon> cell_polygons;
	std::vector<cv::Point> polygon_centers;
	for(size_t cell=0; cell<cells.size(); ++cell)
	{
		generalizedPolygon current_cell(cells[cell]);
		cell_polygons.push_back(current_cell);
		polygon_centers.push_back(current_cell.getCenter());
	}
//	testing
//	cv::Mat center_map = room_map.clone();
//	for(size_t i=0; i<cell_polygons.size(); ++i)
//		cv::circle(center_map, cell_polygons[i].getCenter(), 2, cv::Scalar(127), CV_FILLED);
//	cv::imshow("centers", center_map);
//	cv::waitKey();

	// *********************** III. Determine the cell paths. ***********************
	// determine the start cell that is closest to the start position
	double min_distance = 1e10;
	int min_index = 0;
	cv::Point starting_point(starting_position.x, starting_position.y); // conversion of Pose2D to cv::Point for convenience

	for(std::vector<cv::Point>::iterator point = polygon_centers.begin(); point != polygon_centers.end(); ++point)
	{
		double distance = cv::norm(starting_point - *point);

		if(distance <= min_distance)
		{
			min_distance = distance;
			min_index = point - polygon_centers.begin();
		}
	}

	// determine the optimal visiting order of the cells
	ConcordeTSPSolver tsp_solver;
	std::vector<int> optimal_order = tsp_solver.solveConcordeTSP(room_map, polygon_centers, 0.25, 0.0, map_resolution, min_index, 0);

	// go trough the cells and determine the boustrophedon paths
	int fow_radius_as_int = (int) std::floor(fow_fitting_circle_radius); // convert fow-radius to int
	cv::Point robot_pos = starting_point; // Point that keeps track of the last point after the boustrophedon path in each cell
	for(size_t cell=0; cell<polygon_centers.size(); ++cell)
	{
		// access current cell
		generalizedPolygon current_cell = cell_polygons[cell];

		// get a map that has only the current cell drawn in
		cv::Mat current_cell_map;
		current_cell.drawPolygon(current_cell_map, cv::Scalar(127));

		// get the min/max x/y values for this cell
		int min_x, max_x, min_y, max_y;
		current_cell.getMinMaxCoordinates(min_x, max_x, min_y, max_y);

		// get the left and right edges of the path
		std::vector<cv::Point> edge_points;
		int y = min_y, dx;
		do
		{
			// check the for the current row to the right/left for the first white pixel (valid position), starting from
			// the minimal/maximal x value
			dx = min_x;
			bool found = false;
			// get the leftmost edges of the path
			do
			{
				if(room_map.at<uchar>(y, dx) == 255)
				{
					edge_points.push_back(cv::Point(dx, y));
					found = true;
				}
				else
					++dx;
			}while(dx < current_cell_map.cols && found == false); // dx < ... --> safety, should never hold

			// get the rightmost edges of the path
			dx = max_x;
			found = false;
			do
			{
				if(room_map.at<uchar>(y, dx) == 255)
				{
					edge_points.push_back(cv::Point(dx, y));
					found = true;
				}
				else
					--dx;
			}while(dx > 0 && found == false);

			// increase y by given fow-radius value
			y += fow_radius_as_int;
		}while(y <= max_y);

//		testing
//		for(size_t i=0; i<edge_points.size(); ++i)
//			cv::circle(current_cell_map, edge_points[i], 2, cv::Scalar(200), CV_FILLED);
//		cv::imshow("path edges", current_cell_map);
//		cv::waitKey();

		// TODO: get nearest point
		// get the edge nearest to the current robot position to start the boustrophedon path at

		// TODO: calculate boustrophedon path --> check if left or right and go in other direction until out of cell
		// calculate the points between the edge points and create the boustrophedon path with this
	}
}
