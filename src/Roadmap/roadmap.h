#ifndef __ROADMAP_H__
#define __ROADMAP_H__

#include "../Utility/utility.h"
#include <iostream>
#include <stdio.h>
#include <stdexcept>
#include <boost/geometry.hpp>

using pt         = boost::geometry::model::d2::point_xy<double>;
using Polygon       = boost::geometry::model::polygon<pt>;

typedef struct Robot{
	char *ID = NULL;
	double x;
	double y;
	double max_curvature_angle;
	double offset = 1;

	Robot(){
		x = 0.0;
		y = 0.0;
	}
} Robot;

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

typedef struct list_of_gates{
	polygon *head = NULL;
	polygon *tail = NULL;
}list_of_gates;

typedef struct points_map {
  // Lists of points belonging respectively to the arena and the obstacles
  // Supposed that the points are in clockwise or counterclockwise order
  point_list *arena = NULL;
  list_of_obstacles *obstacles = new list_of_obstacles;
  list_of_gates *gates = new list_of_gates;
  Robot *robot = NULL;
  
  void add_arena_points(point_list *ArenaPoints);
  void set_robot_position(double x, double y);
  void add_obstacle(polygon* ob);
  void add_gate(polygon *gt);
  void merge_obstacles();
  void print_info();
  Mat plot_arena(int, int);
  void del_map();
  // void ~points_map();
} points_map;

point_list* boost_polygon_to_point_list(Polygon p);
#endif
