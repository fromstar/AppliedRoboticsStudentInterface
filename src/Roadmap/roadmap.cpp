#include "roadmap.h"
#include <vector>


#include <boost/geometry.hpp>
// #include <boost/geometry/geometries/point_xy.hpp>
// #include <boost/geometry/geometries/polygon.hpp>
// #include <boost/geometry/io/wkt/wkt.hpp>

#include <boost/foreach.hpp>
#include <boost/polygon/voronoi_builder.hpp>
#include <boost/polygon/voronoi_diagram.hpp>
#include <string.h>

namespace bg = boost::geometry;
namespace bgm = bg::model;


using pt         = bgm::d2::point_xy<double>;
using Polygon       = bgm::polygon<pt>;
using Multi_Polygon = bgm::multi_polygon<Polygon>;


list_of_robots::~list_of_robots(){
	Robot* tmp = head;
	while(tmp != NULL){
		Robot* deleter = tmp;
		tmp = tmp -> next;
		delete deleter;
	};
};

void list_of_robots::add_robot(Robot* r){
	if (head == NULL){
		head = r;
		tail = head;
	}else{
		tail->next = r;
		tail = tail->next;
	};
};

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

void list_of_polygons::add_polygon(polygon *p){
	if(head == NULL)
	{
		head = p;
		tail = head;
		return;
	}
	tail->pnext = p;
	tail = tail->pnext;
	size += 1;
};

void list_of_polygons::append_other_list(list_of_polygons* p){
	if (head == NULL){
		head = p -> head;
		tail = p -> tail;
	}else{
		tail -> pnext = p -> head;
		tail = p -> tail;
	};
	size += p->size;
};

void points_map::add_gate(polygon* gt){ // is a shortcut
	gates -> add_polygon(gt);
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

void points_map::reduce_arena(){
	polygon *copy_arena = new polygon(arena);
	copy_arena = copy_arena -> add_offset(-0.1);
	arena = copy_arena -> pl;
};

Mat points_map::plot_arena(int x_dim, int y_dim, bool show_original_polygons){
	Mat img_arena(x_dim, y_dim, CV_8UC3, Scalar(255, 255, 255));
	
	img_arena = plot_points(arena, img_arena, Scalar(0,0,0),true);
	
	polygon* tmp = NULL;

	if(show_original_polygons){
		// Plot obstacles as they are
		tmp = obstacles->head;
		while(tmp != NULL)
		{
			img_arena = plot_points(tmp->pl, img_arena, Scalar(255,0,0), true);
			tmp = tmp->pnext;
		}
	};

	// Plot obstacles enlarged
	tmp = obstacles->offset_head;
	while(tmp != NULL)
	{
		img_arena = plot_points(tmp->pl, img_arena, Scalar(0,0,255), true);
		tmp = tmp->pnext;
	}
	
	// plot gates
	tmp = gates->head;
	while(tmp != NULL)
	{
		img_arena = plot_points(tmp->pl, img_arena, Scalar(0,255,0), true);
		tmp = tmp->pnext;
	}
	
	// plot free_space
	tmp = free_space -> head;
	while(tmp != NULL){
		img_arena = plot_points(tmp->pl, img_arena, Scalar(0, 255, 255),
								true, 2);
		tmp = tmp->pnext;
	};

	// plot robots
	point_list *robot_loc = new point_list;
	robot_loc -> add_node(new point_node(robot->x, robot->y));
	robot_loc -> add_node(new point_node(robot->x, robot->y));
	img_arena = plot_points(robot_loc, img_arena, Scalar(210, 26, 198),
							false, 5);

	return img_arena;
}


point_list* boost_polygon_to_point_list(Polygon p){
	point_list *pl = new point_list();
	for(auto it = boost::begin(boost::geometry::exterior_ring(p));
		it != boost::end(boost::geometry::exterior_ring(p)); ++it)
	{
		double x = bg::get<0>(*it);
		double y = bg::get<1>(*it);
		
		pl->add_node(new point_node(x,y));
	}
	return pl;
};


// Use boost library to merge polygons
void points_map::merge_obstacles()
{
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
	}else{
		printf("No arena points setted. Unable to check intersections.");
	};
	
	// Check intersections with gates
	if (gates != NULL){
		polygon* tmp_gate = gates->head;
		gates = NULL;  // Delete old gates list
		vector<Polygon> tmp_new_gates;
		points_map* tmp_map = new points_map;

		while (tmp_gate != NULL){
			Polygon _gate = tmp_gate -> to_boost_polygon();
			for (int k=0; k<i; k++){
				if (boost::geometry::intersects(_gate, polys[k])){
					boost::geometry::difference(_gate, polys[k],
												  tmp_new_gates);
					point_list* pl = boost_polygon_to_point_list(
									 tmp_new_gates[tmp_new_gates.size()-1]
									 );
					polygon *new_gate = new polygon(pl);
					new_gate->pnext = tmp_gate->pnext;
					tmp_gate = new_gate;
				};
			};
			tmp_map->add_gate(tmp_gate);
			tmp_gate = tmp_gate->pnext;
		};
		gates = tmp_map->gates;
	};

	// Delete the offsetted list
	obstacles->delete_offsetted_list();

	// Repopulate with updatate offsetted polygons
	for(i=0; i < polys.size(); i++)
	{
		point_list *pl = boost_polygon_to_point_list(polys[i]);

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

list_of_polygons* subset_polygon(polygon* p, int levels){
	list_of_polygons* subset_list = new list_of_polygons;

	point_node* tmp_point_start = p -> pl -> head;

	while(tmp_point_start->pnext != NULL){
		point_list* tmp_point_list = new point_list;
		
		double s_x = tmp_point_start -> x;
		double s_y = tmp_point_start -> y;
		double e_x = tmp_point_start -> pnext -> x;
		double e_y = tmp_point_start -> pnext -> y;

		tmp_point_list -> add_node(new point_node(s_x, s_y));
		tmp_point_list -> add_node(new point_node(e_x, e_y));
		tmp_point_list -> add_node(p->centroid);
		
		subset_list -> add_polygon(new polygon(tmp_point_list));
		tmp_point_start = tmp_point_start -> pnext;

		if (tmp_point_start -> pnext == NULL){
			tmp_point_list = new point_list;
			
			double s_x = tmp_point_start -> x;
			double s_y = tmp_point_start -> y;
			double e_x = p -> pl -> head -> x;
			double e_y = p -> pl -> head -> y;

			tmp_point_list -> add_node(new point_node(s_x, s_y));
			tmp_point_list -> add_node(new point_node(e_x, e_y));
			tmp_point_list -> add_node(p -> centroid);
			subset_list -> add_polygon(new polygon(tmp_point_list));
		};
	};
	for(int i=0; i<levels-1; i++){
		polygon* tmp_pol = subset_list -> head;
		list_of_polygons* new_subset_list = new list_of_polygons;
		while(tmp_pol != NULL){
			polygon* tmp_list = new polygon(tmp_pol -> pl);
			new_subset_list->append_other_list(subset_polygon(tmp_list));
			tmp_pol = tmp_pol -> pnext;
		};
		subset_list = new_subset_list;
	};
	return subset_list;
};


/**
 * Subtract a vector o polygons from another one.
 * @param arena: vector<Polygon>. Is the vector of polygons from which
 * the other polygons will be subtracted.
 * @param obstacles: vector<Polygon>. Is the vector of polygons that will
 * be subtracted.
 */
vector<Polygon> difference_of_vectors(vector<Polygon> arena,
									  vector<Polygon> obstacles){
	vector<Polygon> output;
	vector<Polygon> tmp_output;

	int prev_output = 0;
	int arena_size = arena.size();
	int arena_ob_size = obstacles.size();
	for (int i=0; i<arena_size; i++){
		for (int j=0; j<arena_ob_size; j++){
			if (bg::intersects(arena[i], obstacles[j])){
				bg::difference(arena[i], obstacles[j], output);
				int diff = output.size() - prev_output;
				prev_output = output.size();

				// printf("Polygon %d and obstacle %d intersects", i, j);
				// printf(" -> %d new cells\n", diff);

				arena[i] = output[output.size()-1];
				if (diff > 1){
					tmp_output = arena;
					for(int k=1; k < diff; k++){
						int output_idx = output.size()-1-k;
						vector<Polygon>::iterator it;
						it = tmp_output.begin();
						tmp_output.insert(it+i+k, output[output_idx]);
						arena_size += 1;
					};
					arena = tmp_output;
				};
			};
		};
	};
	return arena;
};


void points_map::make_free_space_cells(int res){
	// Subset arena -> free space idealization
	polygon* _arena = new polygon(arena);
	list_of_polygons* _arena_subset = subset_polygon(_arena, res);
	

	// Arena subsets to boost::polygons;
	vector<Polygon> arena_polys;
	vector<Polygon> arena_obstacles;
	vector<Polygon> output;
	vector<Polygon> tmp_output;

	polygon* pol = _arena_subset -> head;
	while(pol != NULL){
		arena_polys.push_back(pol->to_boost_polygon());
		pol = pol -> pnext;
	};
	printf("Arena converted to boost\n");

	pol = obstacles -> offset_head;
	while(pol != NULL){
		arena_obstacles.push_back(pol->to_boost_polygon());
		pol = pol -> pnext;
	};
	printf("Obstacles converted to boost\n");

	// Remove the obstacles from the free space and compute the new shapes
	if (arena_obstacles.size() > 0){
		// Remove obstacles from the sectors of the arena.
		arena_polys = difference_of_vectors(arena_polys, arena_obstacles);
		
		// Remove occlusions caused by the free space with themselves
		int prev_output = output.size();
		int arena_size = arena_polys.size();
		int arena_ob_size = arena_obstacles.size();

		output.clear();
		tmp_output.clear();
		prev_output = 0;
		for (int i=0; i<arena_size; i++){
			for (int j=0; j<arena_size; j++){
				if (i != j){
					if (bg::overlaps(arena_polys[i], arena_polys[j])){
						bg::difference(arena_polys[i], arena_polys[j], output);
						int diff = output.size() - prev_output;
						prev_output = output.size();

						// printf("Polygon %d and polygon %d intersects", i, j);
						// printf(" -> %d new cells\n", diff);

						arena_polys[i] = output[output.size()-1];
						if (diff > 1){
							tmp_output = arena_polys;
							for(int k=1; k < diff; k++){
								int output_idx = output.size()-1-k;
								vector<Polygon>::iterator it;
								it = tmp_output.begin();
								tmp_output.insert(it+i+k, output[output_idx]);
								arena_size += 1;
							};
							arena_polys = tmp_output;
						};
					};
				};
			};
		};
	};

	// printf("Obstacles removed\n");
	output = arena_polys;
	// cout << "Found " << output.size() << " cells" << endl;

	// convert boost polygons to polygons and update free space variable
	for(int i=0; i<output.size(); i++){
		point_list* new_space_points = boost_polygon_to_point_list(output[i]);
		polygon* new_space = new polygon(new_space_points);
		free_space->add_polygon(new_space);
	};
	printf("Free space generated\n");
};

