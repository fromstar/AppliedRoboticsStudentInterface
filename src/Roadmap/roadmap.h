#ifndef __ROADMAP_H__
#define __ROADMAP_H__

#include "../Utility/utility.h"
#include "../Log/logger.h"
#include "../Robot_manager/robot_manager.h"
#include "../Connector/connector.h"

#include <iostream>
#include <stdio.h>
#include <stdexcept>
#include <boost/geometry.hpp>
#include <string.h>
#include <map>

using pt = boost::geometry::model::d2::point_xy<double>;
using Polygon_boost = boost::geometry::model::polygon<pt>;

/*
typedef struct list_of_obstacles
{
	polygon *head = NULL;
	polygon *tail = NULL;
	polygon *offset_head = NULL;
	polygon *offset_tail = NULL;
	double offset = 105e-3;
	int size = 0;
	int offset_size = 0;

	~list_of_obstacles();
	void delete_offsetted_list();
} list_of_obstacles;
*/

/**
 * \struct points_map
 * This structure represents all the entity in an arena.
 * The attributes are:
 * @param point_list *arena: Points of the arena limits
 * @param point_list *shrinked_arena: Points of the arena limits with an offset
 * @param list_of_obstacles *obstacles: List of the obstacles in the arena
 * @param list_of_polygons *gates: List of the gates in the arena
 * @param list_of_polygons *free_space: List of free space cells
 * @param Connection_map connections:
 * @param map<string, Robot*> robot: List of the robots in the arena
 * @param logger log:
 * 
 * Available methods are:
 * @see points_map(logger *l): Constructor
 * @see void add_robot(Robot *r): Add a robot to the arena 
 * @see void set_robot_position(string robot_id, double x, double y, double th): Set the robot position given it's id
 * @see void add_obstacle(polygon *ob): Add an obstacle in the arena
 * @see void add_gate(polygon *gt): Add a gate in the arena
 * @see void merge_obstacles(): Merge intersecting obstacles
 * @see void convexify_obstacles(): Convex the space occupied by the obstacles
 * @see void make_free_space_cells_triangular(int res = 3): Create triangular free space cells
 * @see void make_free_space_cells_squares(int res = 3): Create squares triangular free space cells
 * @see void print_info(): Print the information about the arena
 * @see Mat plot_arena(int x_dim, int y_dim, bool show_original_polygons=true,
                   bool show_cells_id=false): Return an img with plotted all the objects in the arena
 * @see void shrink_arena(double offset = 0): Add an offset to the arena limits
 * @see void del_map(): Delete the map
 */
typedef struct points_map
{
	// Lists of points belonging respectively to the arena and the obstacles
	point_list *arena = NULL; // If the robots touches the wall Game Over -> Inflate the arena
	point_list *shrinked_arena = NULL;
	list_of_obstacles *obstacles = new list_of_obstacles;
	list_of_polygons *gates = new list_of_polygons;
	list_of_polygons *free_space = new list_of_polygons;
    Connection_map connections;
	// Robot *robot = NULL; // to update using list_of_robots
	map<string, Robot *> robot;
	logger *log;

	points_map(logger *l) { log = l; };

	void add_arena_points(point_list *ArenaPoints, double offset = 101e-3);
	void add_robot(Robot *r);
	void set_robot_position(string robot_id, double x, double y, double th);
	void add_obstacle(polygon *ob);
	void add_gate(polygon *gt);
	void merge_obstacles();
	void convexify_obstacles();
	void make_free_space_cells_triangular(int res = 3);
	void make_free_space_cells_squares(int res = 3);
	void print_info();
	Mat plot_arena(int x_dim, int y_dim, bool show_original_polygons=true,
                   bool show_cells_id=false);
	void shrink_arena(double offset = 0);
	void del_map();
	// void ~points_map();
} points_map;

// outsider functions
// point_list *boost_polygon_to_point_list(Polygon_boost p);
// polygon *boost_polygon_to_polygon(Polygon_boost p);
// polygon *boost_polygon_to_polygon(Polygon_boost p);
list_of_polygons *subset_polygon(polygon *p, int levels = 1);
vector<Polygon_boost> difference_of_vectors(vector<Polygon_boost> arena,
											vector<Polygon_boost> obstacles);
#endif
