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
	// Take into account orientation -> Changes the available movements
	char *ID = NULL;
	double x;
	double y;
	double max_curvature_angle;
	double offset = 1;
	Robot* next = NULL;

	Robot(){
		x = 0.0;
		y = 0.0;
	}
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

  void add_arena_points(point_list *ArenaPoints);
  void set_robot_position(double x, double y);  // To update using list
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
