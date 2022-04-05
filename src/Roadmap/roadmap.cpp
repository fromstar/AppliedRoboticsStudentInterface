#include "roadmap.h"
#include <vector>


#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>

#include <boost/foreach.hpp>

#include <string.h>

namespace bg = boost::geometry;
namespace bgm = bg::model;


using pt         = bgm::d2::point_xy<double>;
using Polygon       = bgm::polygon<pt>;
using Multi_Polygon = bgm::multi_polygon<Polygon>;


list_of_obstacles::~list_of_obstacles(){
    polygon *tmp = head;
    polygon *otmp = offset_head;
    while(head!=NULL)
    {
      tmp = head;
      head = head->pnext;
      delete tmp;
    }
    while(offset_head!=NULL)
    {
      tmp = offset_head;
      offset_head = offset_head->pnext;
      delete tmp;
    }
  };

void list_of_obstacles::delete_offsetted_list(){
	polygon *tmp;
	while (offset_head != NULL)
	{
		tmp = offset_head;
		offset_head=offset_head->pnext;
		delete tmp;
	}
	offset_size=0;
};

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
	obstacles->size++;
	obstacles->offset_size++;
	polygon *offsetted_ob = ob->add_offset(obstacles->offset); 
	if(obstacles->head == NULL)
	{
		obstacles->head = ob;
		obstacles->tail = obstacles->head;
		obstacles->offset_head = offsetted_ob;
		obstacles->offset_tail = obstacles->offset_head;
		return;
	}
	obstacles->tail->pnext = ob;
	obstacles->tail = obstacles->tail->pnext;
	obstacles->offset_tail->pnext = offsetted_ob;
	obstacles->offset_tail = obstacles->offset_tail->pnext;
};

void points_map::print_info(){
  cout<<"Robot location: " << robot->x << " - "<< robot->y <<endl;
};

Mat points_map::plot_arena(int x_dim, int y_dim){
	Mat img_arena(x_dim, y_dim, CV_8UC3, Scalar(255, 255, 255));
	
	img_arena = plot_points(arena, img_arena, Scalar(0,0,0),true);
	
	// Plot obstacles as they are
	polygon *tmp = obstacles->head;
	while(tmp != NULL)
	{
		img_arena = plot_points(tmp->pl,img_arena,Scalar(255,0,0),true);
		tmp = tmp->pnext;
	}
	
	// Plot obstacles enlarged
	tmp = obstacles->offset_head;
	while(tmp != NULL)
	{
		img_arena = plot_points(tmp->pl,img_arena,Scalar(0,0,255),true);
		tmp = tmp->pnext;
	}
	
	// plot gates
	tmp = gates->head;
	while(tmp != NULL)
	{
		img_arena = plot_points(tmp->pl,img_arena,Scalar(0,255,0),true);
		tmp = tmp->pnext;
	}
	
	// plot robots
	point_list *robot_loc = new point_list;
	robot_loc -> add_node(new point_node(robot->x, robot->y));
	robot_loc -> add_node(new point_node(robot->x, robot->y));
	img_arena = plot_points(robot_loc, img_arena, Scalar(210, 26, 198),
							false, 5);

	return img_arena;
}

// Use boost library to merge polygons
void points_map::merge_obstacles()
{
	// if(obstacles->size < 2)
	// 	return;

	std::vector<Polygon> polys;

 	polygon *pol_iter = obstacles->offset_head;

	// Convert polygons in Boost polygon object
	while(pol_iter != NULL)
	{
		polys.push_back(pol_iter->to_boost_polygon());
		pol_iter = pol_iter->pnext;
	}

	// check which polygons intersect
	vector<Polygon> output;
	int i=0; // number of polygons present
	double psize = polys.size();
	while(i < psize)
	{
		int j = i+1;
		while(j < psize)
		{
			if(boost::geometry::intersects(polys[i],polys[j])){
				// Update the polygon list
				boost::geometry::union_(polys[i], polys[j], output);
				polys.erase(polys.begin() + i);
				polys.erase(polys.begin() + j);
				polys.push_back(output[output.size()-1]);
				psize = polys.size();
				i=0, j=1;
			}
			j++;
		}
		i++;
	}
	
	// Check intersections with arena
	if(arena != NULL){
		polygon *arena_pol = new polygon(arena);
		Polygon _arena = arena_pol->to_boost_polygon();
		vector<Polygon> tmp_pols;

		for (int k=0; k<i; k++){
			if (boost::geometry::intersects(_arena, polys[k])){
				boost::geometry::intersection(polys[k],_arena, tmp_pols);
				polys[k] = tmp_pols[tmp_pols.size()-1];
			};
		};
	};
	
	// Delete the offsetted list
	obstacles->delete_offsetted_list();

	// Repopulate with updatate offsetted polygons
	for(i=0; i < polys.size(); i++)
	{
		point_list *pl = new point_list();
		for(auto it = boost::begin(boost::geometry::exterior_ring(polys[i]));
			it != boost::end(boost::geometry::exterior_ring(polys[i])); ++it)
		{
			double x = bg::get<0>(*it);
			double y = bg::get<1>(*it);
			
			pl->add_node(new point_node(x,y));
		}

		// pl->print_list();

		if(obstacles->offset_head == NULL)
		{
			obstacles->offset_head = new polygon(pl);
			obstacles->offset_tail = obstacles->offset_head;
		}
		else
		{
			obstacles->offset_tail->pnext = new polygon(pl);
			obstacles->offset_tail = obstacles->offset_tail->pnext;
		}
		obstacles->offset_size++;
	}
}
