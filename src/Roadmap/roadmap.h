#ifndef __ROADMAP_H__
#define __ROADMAP_H__

#include "../Utility/utility.h"
#include "../Log/logger.h"

#include <iostream>
#include <stdio.h>
#include <stdexcept>
#include <boost/geometry.hpp>
#include <string.h>


using pt         = boost::geometry::model::d2::point_xy<double>;
using Polygon       = boost::geometry::model::polygon<pt>;

/**
 * \enum {fugitive, catcher, undefined}
 * These types identify the role of the robot in the environment.
 * fugitive: The robot must chose an exit point in the arena (i.e. gate) and
 * reach it without colliding with anything.
 * catcher: The robot must catch the fugitives before they reach the exit
 * point.
 * undefined: The robot has yet to be assigned a role.
 */
typedef enum {fugitive, catcher, undefined} Robot_type;

/**
 * \struct Robot
 * This struct represent a robot in the environment. Its parameters are:
 * @param ID: string. It is the unique identifier of the robot.
 * @param type: Robot_type. It is the role of the robot in the environment.
 * @param x: double. It is the robot location over the abscissa axis.
 * @param y: double. It is the robot location over the ordinates axis.
 * @param max_curvature_angle: double. It is the maximum angle that the robot
 * is able to reach during a turning action.
 * @param offset: double. It is the value of which the robot physical
 * dimensions are increased to account for a safe movement in the environment.
 * @param next: Robot\*. Is the pointer to another robot instance, useful in
 * creating lists of robots.
 *
 * Available methods are:
 * @see set_id(string _id): Sets the robot id.
 * @see set_robot_type(Robot_type rt): Sets the type of the robot.
 * @see info(): Prints all the informations regarding the robot.
 */
typedef struct Robot{
	// Take into account orientation -> Changes the available movements
	string ID;
	Robot_type type = undefined;
	point_node* location = NULL;
	double max_curvature_angle = 0;
	double offset = 1;
	Robot* next = NULL;

	Robot(){
		ID = "Default_id";
		location = new point_node(0, 0);
	}

	void set_id(string _id);
	void set_robot_type(Robot_type rt);
	point_node* where();
	void info();
} Robot;

typedef struct list_of_robots{
	Robot* head = NULL;
	Robot* tail = NULL;

	~list_of_robots();
	void add_robot(Robot* r);
}list_of_robots;

typedef struct list_of_obstacles {
  polygon *head = NULL;
  polygon *tail = NULL;
  polygon *offset_head = NULL;
  polygon *offset_tail = NULL;
  double offset = 0.105;
  int size = 0;
  int offset_size = 0;
	
  ~list_of_obstacles();
  void delete_offsetted_list();
}list_of_obstacles;

typedef struct list_of_polygons{
	polygon *head = NULL;
	polygon *tail = NULL;
	int size = 0;

	void add_polygon(polygon* p);
	void append_other_list(list_of_polygons* p);
}list_of_polygons;

typedef struct points_map {
  // Lists of points belonging respectively to the arena and the obstacles
  point_list *arena = NULL;  // If the robots touches the wall Game Over -> Inflate the arena
  list_of_obstacles *obstacles = new list_of_obstacles;
  list_of_polygons *gates = new list_of_polygons;
  list_of_polygons *free_space = new list_of_polygons;
  Robot *robot = NULL; // to update using list_of_robots
  logger* log;

  points_map(logger* l){log=l;};

  void add_arena_points(point_list *ArenaPoints);
  void set_robot_position(double x, double y);
  void add_obstacle(polygon* ob);
  void add_gate(polygon *gt);
  void merge_obstacles();
  void make_free_space_cells(int res=2);
  void print_info();
  Mat plot_arena(int x_dim, int y_dim, bool show_original_polygons=true);
  void del_map();
  void reduce_arena();
  // void ~points_map();
} points_map;

// outsider functions
point_list* boost_polygon_to_point_list(Polygon p);
list_of_polygons* subset_polygon(polygon* p, int levels=1);
vector<Polygon> difference_of_vectors(vector<Polygon> arena,
									  vector<Polygon> obstacles);
#endif
