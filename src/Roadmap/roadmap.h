#ifndef __ROADMAP_H__
#define __ROADMAP_H__

#include "../Utility/utility.h"
#include "../Log/logger.h"
#include "../Robot_manager/robot_manager.h"

#include <iostream>
#include <stdio.h>
#include <stdexcept>
#include <boost/geometry.hpp>
#include <string.h>
#include <map>

using pt = boost::geometry::model::d2::point_xy<double>;
using Polygon_boost = boost::geometry::model::polygon<pt>;


typedef struct list_of_obstacles
{
	polygon *head = NULL;
	polygon *tail = NULL;
	polygon *offset_head = NULL;
	polygon *offset_tail = NULL;
	double offset = -0.105;
	int size = 0;
	int offset_size = 0;

	~list_of_obstacles();
	void delete_offsetted_list();
} list_of_obstacles;


typedef struct points_map
{
	// Lists of points belonging respectively to the arena and the obstacles
	point_list* arena = NULL; // If the robots touches the wall Game Over -> Inflate the arena
	point_list* shrinked_arena = NULL;
	list_of_obstacles* obstacles = new list_of_obstacles;
	list_of_polygons* gates = new list_of_polygons;
	list_of_polygons* free_space = new list_of_polygons;
	// Robot *robot = NULL; // to update using list_of_robots
	map<string, Robot*> robot;
	logger *log;

	points_map(logger *l) { log = l; };

	void add_arena_points(point_list *ArenaPoints);
	void add_robot(Robot *r);
	void set_robot_position(string robot_id, double x, double y, double th);
	void add_obstacle(polygon *ob);
	void add_gate(polygon *gt);
	void merge_obstacles();
	void make_free_space_cells_triangular(int res = 3);
	void make_free_space_cells_squares(int res=4);
	void print_info();
	Mat plot_arena(int x_dim, int y_dim, bool show_original_polygons = true);
	void shrink_arena(double offset=0);
	void del_map();
	// void ~points_map();
} points_map;

// outsider functions
point_list *boost_polygon_to_point_list(Polygon_boost p);
polygon *boost_polygon_to_polygon(Polygon_boost p);
polygon *boost_polygon_to_polygon(Polygon_boost p);
list_of_polygons *subset_polygon(polygon *p, int levels = 1);
vector<Polygon_boost> difference_of_vectors(vector<Polygon_boost> arena,
											vector<Polygon_boost> obstacles);
#endif
