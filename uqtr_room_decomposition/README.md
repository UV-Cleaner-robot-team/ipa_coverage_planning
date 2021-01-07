- This pkg is for getting the decompositon zone centers in a .csv file fil "cfg" folder in this package (The path is changeable in the launch file).

- There is a launch file to configure and run the decompositin.

- Converage visualization added.

- Coverage rate.

- Elliminating useless zones to elliminate.

- Drive the robot through the zone centers to cover the obstacles with UV.

**Launch file configuration:**

Load the  map-linked .yaml file .

map_image_path: the path of the .pgm file of the saved map.

csv_file_path: the path of the csv file to save the zone centers cordinates in the real world.

min_cell_area: Cells with area less than this value will be merged with bigger cell (keeping small values is recommended).
min_cell_width: cells with width less than this value will be merged with bigger cells (keeping small values is recommended).
rotation_offset: value to start the slicing line rotation before starting the decomposition.
map_correction_closing_neighborhood_size: obstacle gaps with width less than this value will be closed (Inpixels).
max_uv_distance_range: distance of themax range ofthe UV rays.
robot_radius: the robot maxdimention (length, diamiter, ...).