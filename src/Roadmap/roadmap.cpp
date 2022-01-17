#include "roadmap.h"
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>

#include <boost/foreach.hpp>
#include <string.h>

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
	
	typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > Polygon;

	std::vector<Polygon> polys;

 	polygon *pol_iter = offset_head;

	// Convert polygons int Boost polygon object
	for(int i=0;i<size;i++)
	{
		point_node *pn = pol_iter->pl->head;
		int pts_size = pol_iter->pl->size;

		string pts = "POLYGON((";
		for(int j=0;j<pts_size;j++)
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
		polys.push_back(p);
		pol_iter->pnext;
	}

	// Delete the offsetted list
	/*polygon *tmp;
	while (offset_head!=NULL)
	{
		tmp = offset_head;
		offset_head=offset_head->pnext;
		delete tmp;
	}*/

	std::vector<Polygon> output;

	int i=0;
	while(i<size)
	{
		int j=i+1;
		while(j<size)
		{
			// Check if two polygons intersects. If yes merge them.
			if(boost::geometry::intersects(polys[i],polys[j]))
			{
				cout<<"Intersezione\n";
				boost::geometry::union_(polys[i], polys[j], output);
				// Update the polygon list
				polys.erase(polys.begin() + i);
				polys.erase(polys.begin() + j);
				polys.push_back(output[output.size()-1]);

				i=0,j=0;
			}
			j++;
		}
		i++;
	}
	
	/*point_list *pl = new point_list();
	for(i=0;i<output.size();i++){
		for(auto it = boost::begin(boost::geometry::exterior_ring(output[i])); it != boost::end(boost::geometry::exterior_ring(output[i])); ++it)
		{
			double x = boost::geometry::get<0>(*it);
			double y = boost::geometry::get<1>(*it);

			cout<<"x: "<<x<<" | y: "<<y<<endl;			
			pl->add_node(new point_node(x,y));
		}
		if(offset_head==NULL){
			offset_head = new polygon(pl);
			offset_tail = offset_head;
		}
		else{
			offset_tail->pnext = new polygon(pl);
			offset_tail=offset_tail->pnext;
		}
	}*/
}
