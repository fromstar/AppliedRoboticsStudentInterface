#include "roadmap.h"


void points_map::add_arena_points(point_list *ArenaPoints){
	arena = ArenaPoints;
};

void points_map::set_robot_position(double x, double y){
  robot = new(Robot);
  robot -> x = x;
  robot -> y = y;
};

void points_map::add_gate(polygon *gt){
	if(gates->head == NULL)
	{
		gates->head = gt;
		gates->tail = gates->head;
		return;
	}
	gates->tail->pnext = gt;
	gates->tail = gates->tail->pnext;
};

void points_map::add_obstacle(polygon *ob){
	if(obstacles->head == NULL)
	{
		obstacles->head = ob;
		obstacles->tail = obstacles->head;
		return;
	}
	obstacles->tail->pnext = ob;
	obstacles->tail = obstacles->tail->pnext;
};

void points_map::print_info(){
  cout<<"Robot location: " << robot->x << "- "<< robot->y <<endl;
};

Mat points_map::plot_arena(){
	Mat img_arena(600,600, CV_8UC3, Scalar(255, 255, 255));
	
	img_arena = plot_points(arena, img_arena, Scalar(0,0,0),true);
	
	polygon *tmp = obstacles->head;
	while(tmp != NULL)
	{
		img_arena = plot_points(tmp->pl,img_arena,Scalar(0,0,255),true);
		tmp = tmp->pnext;
	}
	
	tmp = gates->head;
	while(tmp != NULL)
	{
		img_arena = plot_points(tmp->pl,img_arena,Scalar(0,255,0),true);
		tmp = tmp->pnext;
	}
	
	return img_arena;
}
