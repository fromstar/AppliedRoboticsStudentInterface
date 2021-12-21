#ifndef __ROADMAP_H__
#define __ROADMAP_H__

#include "../Dubins/dubins.h"
#include <iostream>
#include <stdio.h>

typedef struct Robot{
	char *ID = NULL;
	double x;
	double y;
	double max_curvature_angle;

	Robot(){
		x = 0.0;
		y = 0.0;
	}
} Robot;

typedef struct polygon{
	point_list *pl;
	polygon *pnext = NULL;
	polygon(point_list* pls){
		pl = pls;
	};
}polygon;

typedef struct list_of_obstacles {
  polygon *head = NULL;
  polygon *tail = NULL;
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
  void print_info();
  Mat plot_arena();
  // void ~points_map();
} points_map;

#endif
