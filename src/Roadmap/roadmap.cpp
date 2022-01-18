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

Mat points_map::plot_arena(){
	Mat img_arena(1200,1200, CV_8UC3, Scalar(255, 255, 255));
	
	img_arena = plot_points(arena, img_arena, Scalar(0,0,0),true);
	
	polygon *tmp = obstacles->head;
	while(tmp != NULL)
	{
		img_arena = plot_points(tmp->pl,img_arena,Scalar(255,0,0),true);
		tmp = tmp->pnext;
	}
	
	tmp = obstacles->offset_head;
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

// Use boost library to merge polygons
void list_of_obstacles::merge_polygons()
{
	if(size < 2)
		return;

	std::vector<Polygon> polys;

 	polygon *pol_iter = offset_head;

	// Convert polygons int Boost polygon object
	while(pol_iter != NULL)
	{
		point_node *pn = pol_iter->pl->head;

		string pts = "POLYGON((";
		while(pn != NULL)
		{
			pts.append(to_string(pn->x));
			pts.append(" ");
			pts.append(to_string(pn->y));
			pts.append(",");

			pn=pn->pnext;
		}
		pts.append("))");

		Polygon p;
		boost::geometry::read_wkt(pts,p);
		if(!boost::geometry::is_valid(p))
		//Funzione importante per fare il merge ma fa sbarellare un immagine nella funzione di test. Vedere perchè
			boost::geometry::correct(p);

		polys.push_back(p);
		pol_iter = pol_iter->pnext;
	}

	// Delete the offsetted list
	polygon *tmp;
	while (offset_head!=NULL)
	{
		tmp = offset_head;
		offset_head=offset_head->pnext;
		delete tmp;
	}
	offset_size=0;


	vector<Polygon> output;
	int i=0;
	double psize = polys.size();
	while(i<psize)
	{
		int j=i+1;
		while(j<psize)
		{
			if(boost::geometry::intersects(polys[i],polys[j])){
				boost::geometry::union_(polys[i], polys[j], output);
				// Update the polygon list
				boost::geometry::union_(polys[i], polys[j], output);
				polys.erase(polys.begin() + i);
				polys.erase(polys.begin() + j);
				polys.push_back(output[output.size()-1]);
				psize = polys.size();
				i=0,j=1;
			}
			j++;
		}
		i++;
	}

	// Repopulate with updatate offsetted polygons
	point_list *pl = new point_list();
	for(i=0;i<polys.size();i++)
	{
		for(auto it = boost::begin(boost::geometry::exterior_ring(polys[i])); it != boost::end(boost::geometry::exterior_ring(polys[i])); ++it)
		{
			double x = bg::get<0>(*it);
			double y = bg::get<1>(*it);
			
			pl->add_node(new point_node(x,y));
		}
		pl->print_list();
		if(offset_head == NULL)
		{
			offset_head = new polygon(pl);
			offset_tail = offset_head;
		}
		else
		{
			offset_tail->pnext=new polygon(pl);
			offset_tail = offset_tail->pnext;
		}
		offset_size++;
	}
}
