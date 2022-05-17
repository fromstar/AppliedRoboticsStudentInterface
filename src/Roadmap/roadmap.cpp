#include "roadmap.h"
#include <vector>
#include <sys/stat.h> // used to create folders
#include <iostream>
#include <fstream>

// #include <boost/geometry.hpp>
// #include <boost/geometry/geometries/point_xy.hpp>
// #include <boost/geometry/geometries/polygon.hpp>
// #include <boost/geometry/io/wkt/wkt.hpp>

// #include <boost/foreach.hpp>
// #include <boost/polygon/voronoi_builder.hpp>
// #include <boost/polygon/voronoi_diagram.hpp>
// #include <string.h>

namespace bg = boost::geometry;
namespace bgm = bg::model;

using pt = bgm::d2::point_xy<double>;
using Polygon_boost = bgm::polygon<pt>;
using Multi_Polygon_boost = bgm::multi_polygon<Polygon_boost>;

list_of_obstacles::~list_of_obstacles()
{
	polygon *tmp = head;
	polygon *otmp = offset_head;
	while (head != NULL)
	{
		tmp = head;
		head = head->pnext;
		delete tmp;
	}
	while (offset_head != NULL)
	{
		tmp = offset_head;
		offset_head = offset_head->pnext;
		delete tmp;
	}
};


void list_of_obstacles::delete_offsetted_list()
{
	polygon *tmp;
	while (offset_head != NULL)
	{
		tmp = offset_head;
		offset_head = offset_head->pnext;
		delete tmp;
	}
	offset_size = 0;
};

void points_map::add_arena_points(point_list *ArenaPoints)
{
	arena = ArenaPoints;
};

/**
 * \fun void points_map::add_robot(Robot\* r)
 * Use this function to add a robot to the mapping of the environments.
 * The functions checks whether a robot with the same id exists in the mapping
 * and if so it changes the id adding an index at its end.
 * @param r: Robot. Is the Robot object to add to the mapping.
 */
void points_map::add_robot(Robot *r)
{
	int existing = robot.count(r->ID);
	if (existing > 0)
	{ // A robot with same id already exists.
		r->ID += "_" + to_string(existing);
		add_robot(r); // Recursive call, dangerous but ensures uniqueness.
	}
	else
	{ // No robot with that id
		robot[r->ID] = r;
	};
};

void points_map::set_robot_position(string robot_id, double x, double y, double th = 0)
{
	int existing = robot.count(robot_id);
	if (existing == 0)
	{
		cout << "No Robot found having id = \"" << robot_id << "\"" << endl;
		return;
	};
	Robot *_robot = robot[robot_id]; // If no element in container it adds it.
	delete (_robot->location);		 // free memory of previous location.
	_robot->location = new point_node(x, y);
	_robot->theta=th;
};

void points_map::add_gate(polygon *gt)
{ // is a shortcut
	gates->add_polygon(gt);
};

void points_map::add_obstacle(polygon *ob)
{
	obstacles->size++;
	obstacles->offset_size++;
	polygon *offsetted_ob = ob->add_offset(obstacles->offset);
	if (obstacles->head == NULL)
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

void points_map::print_info()
{
	// Print robots informations.
	for (map<string, Robot *>::iterator robot_it = robot.begin();
		 robot_it != robot.end(); ++robot_it)
	{
		robot_it->second->info();
		cout << endl;
	};
};

void points_map::reduce_arena()
{ // Buggy function, must be re_seen
	polygon *copy_arena = new polygon(arena);
	copy_arena = copy_arena->add_offset(-0.1);
	arena = copy_arena->pl;
};

Mat points_map::plot_arena(int x_dim, int y_dim, bool show_original_polygons)
{
	Mat img_arena(x_dim, y_dim, CV_8UC3, Scalar(255, 255, 255));

	img_arena = plot_points(arena, img_arena, Scalar(0, 0, 0), true);

	polygon *tmp = NULL;

	if (show_original_polygons)
	{
		// Plot obstacles as they are
		tmp = obstacles->head;
		while (tmp != NULL)
		{
			img_arena = plot_points(tmp->pl, img_arena, Scalar(255, 0, 0), true);
			tmp = tmp->pnext;
		}
	};

	// Plot obstacles enlarged
	tmp = obstacles->offset_head;
	while (tmp != NULL)
	{
		img_arena = plot_points(tmp->pl, img_arena, Scalar(0, 0, 255), true);
		tmp = tmp->pnext;
	}

	// plot gates
	tmp = gates->head;
	while (tmp != NULL)
	{
		img_arena = plot_points(tmp->pl, img_arena, Scalar(0, 255, 0), true);
		tmp = tmp->pnext;
	}

	// plot free_space
	cout << endl << "Plotting free space" << endl;
	tmp = free_space->head;
	while (tmp != NULL)
	{
		img_arena = plot_points(tmp->pl, img_arena, Scalar(0, 255, 255),
								true, 1);
		
		point_list *free_space_centroid = new point_list;
		free_space_centroid->add_node(tmp->centroid->copy());
		free_space_centroid->add_node(tmp->centroid->copy());
		img_arena = plot_points(free_space_centroid, img_arena, Scalar(0, 255, 255),
								false, 2);
		tmp = tmp->pnext;
	};

	// plot robots

	for (map<string, Robot *>::iterator robot_it = robot.begin();
		 robot_it != robot.end(); ++robot_it)
	{

		point_list *robot_loc = new point_list;
		robot_loc->add_node(robot_it->second->location->copy());
		robot_loc->add_node(robot_it->second->location->copy());
		img_arena = plot_points(robot_loc, img_arena, Scalar(210, 26, 198),
								false, 5);
		delete (robot_loc);
	};

	return img_arena;
}

/**
 * This function is used to convert a boost polygon object into a list of
 * points ready to be interpreted by others functions in the framework.
 * The only purpose of this function is to reduce the amount of code written.
 * @param[in] p: boost::geometry::model::Polygon_boost. Is the polygon to convert.
 * @parma[out] pl: point_list pointer. Is the resulting point list of the
 * polygon.
 */
point_list *boost_polygon_to_point_list(Polygon_boost p)
{
	point_list *pl = new point_list();
	for (auto it = boost::begin(boost::geometry::exterior_ring(p));
		 it != boost::end(boost::geometry::exterior_ring(p)); ++it)
	{
		double x = bg::get<0>(*it);
		double y = bg::get<1>(*it);
		
		pl->add_node(new point_node(x, y));
	}
	point_list* new_pol_list = pl->pop();
	return new_pol_list;
};


polygon *boost_polygon_to_polygon(Polygon_boost p)
{
	point_list *pl = new point_list();
	pt new_centroid;
	boost::geometry::centroid(p, new_centroid);
	for (auto it = boost::begin(boost::geometry::exterior_ring(p));
		 it != boost::end(boost::geometry::exterior_ring(p)); ++it)
	{
		double x = bg::get<0>(*it);
		double y = bg::get<1>(*it);
		
		pl->add_node(new point_node(x, y));
	}
	point_list* new_pol_list = pl->pop();
	polygon *new_pol = new polygon(new_pol_list);
	new_pol->centroid = new point_node(new_centroid.x(), new_centroid.y());
	return new_pol;
};


/**
 * This function is used to merge the obstacles that touch each others in the
 * arena space.
 */
void points_map::merge_obstacles()
{
	std::vector<Polygon_boost> polys;

	polygon *pol_iter = obstacles->offset_head;

	// Convert polygons in Boost polygon object
	while (pol_iter != NULL)
	{
		polys.push_back(pol_iter->to_boost_polygon());
		pol_iter = pol_iter->pnext;
	}

	// check which polygons intersect
	vector<Polygon_boost> output;
	int i = 0; // number of polygons present
	double psize = polys.size();
	while (i < psize)
	{
		int j = i + 1;
		while (j < psize)
		{
			if (boost::geometry::intersects(polys[i], polys[j]))
			{
				// Update the polygon list
				boost::geometry::union_(polys[i], polys[j], output);
				// polys.erase(polys.begin() + i);
				// polys.erase(polys.begin() + j);
				// polys.push_back(output[output.size()-1]);
				polys[i] = output[output.size() - 1];
				// psize = polys.size();
				// i=0, j=1;
			}
			j++;
		}
		i++;
	}

	// Check intersections with arena
	if (arena != NULL)
	{
		polygon *arena_pol = new polygon(arena);
		Polygon_boost _arena = arena_pol->to_boost_polygon();
		vector<Polygon_boost> tmp_pols;

		for (int k = 0; k < i; k++)
		{
			if (boost::geometry::intersects(_arena, polys[k]))
			{
				boost::geometry::intersection(polys[k], _arena, tmp_pols);
				polys[k] = tmp_pols[tmp_pols.size() - 1];
			};
		};
	}
	else
	{
		printf("No arena points setted. Unable to check intersections.");
	};

	// Check intersections with gates
	if (gates != NULL)
	{
		polygon *tmp_gate = gates->head;
		gates = NULL; // Delete old gates list
		vector<Polygon_boost> tmp_new_gates;
		points_map *tmp_map = new points_map(NULL);

		while (tmp_gate != NULL)
		{
			Polygon_boost _gate = tmp_gate->to_boost_polygon();
			for (int k = 0; k < i; k++)
			{
				if (boost::geometry::intersects(_gate, polys[k]))
				{
					boost::geometry::difference(_gate, polys[k],
												tmp_new_gates);
					point_list *pl = boost_polygon_to_point_list(
						tmp_new_gates[tmp_new_gates.size() - 1]);
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
	for (i = 0; i < polys.size(); i++)
	{
		point_list *pl = boost_polygon_to_point_list(polys[i]);

		// pl->print_list();

		if (obstacles->offset_head == NULL)
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
	log->add_event("Obstacles in touch merged\n");
}

/**
 * This function is used to subset a polygon into a finite number of cells.
 * The cells are formed creating a triange connecting every edge of the polygon
 * with the centroid of the object. The number of cells is num_edges^(1+levels).
 * @param[in] levels: int. Is the number of iterations to perform during the
 * polygon subsetting.
 * @param[out] subset_list: list_of_polygons.
 */
list_of_polygons *subset_polygon(polygon *p, int levels)
{
	list_of_polygons *subset_list = new list_of_polygons;

	point_node *tmp_point_start = p->pl->head;

	while (tmp_point_start->pnext != NULL)
	{
		point_list *tmp_point_list = new point_list;

		double s_x = tmp_point_start->x;
		double s_y = tmp_point_start->y;
		double e_x = tmp_point_start->pnext->x;
		double e_y = tmp_point_start->pnext->y;

		tmp_point_list->add_node(new point_node(s_x, s_y));
		tmp_point_list->add_node(new point_node(e_x, e_y));
		tmp_point_list->add_node(p->centroid);

		subset_list->add_polygon(new polygon(tmp_point_list));
		tmp_point_start = tmp_point_start->pnext;

		if (tmp_point_start->pnext == NULL)
		{
			tmp_point_list = new point_list;

			double s_x = tmp_point_start->x;
			double s_y = tmp_point_start->y;
			double e_x = p->pl->head->x;
			double e_y = p->pl->head->y;

			tmp_point_list->add_node(new point_node(s_x, s_y));
			tmp_point_list->add_node(new point_node(e_x, e_y));
			tmp_point_list->add_node(p->centroid);
			subset_list->add_polygon(new polygon(tmp_point_list));
		};
	};
	for (int i = 0; i < levels - 1; i++)
	{
		polygon *tmp_pol = subset_list->head;
		list_of_polygons *new_subset_list = new list_of_polygons;
		while (tmp_pol != NULL)
		{
			polygon *tmp_list = new polygon(tmp_pol->pl);
			new_subset_list->append_other_list(subset_polygon(tmp_list));
			tmp_pol = tmp_pol->pnext;
		};
		subset_list = new_subset_list;
	};
	return subset_list;
};

/**
 * Subtract a vector o polygons from another one.
 * @param arena: vector<Polygon_boost>. Is the vector of polygons from which
 * the other polygons will be subtracted.
 * @param obstacles: vector<Polygon_boost>. Is the vector of polygons that will
 * be subtracted.
 */
vector<Polygon_boost> difference_of_vectors(vector<Polygon_boost> arena,
											vector<Polygon_boost> obstacles)
{

	if(arena.size() == 0){
		cout << "Arena vector is empty." << endl;
	};
	if(obstacles.size() == 0){
		cout << "Obstacle vector is empty." << endl;
	};

	vector<Polygon_boost> output;
	vector<Polygon_boost> tmp_output;

	int prev_output = 0;
	int arena_size = arena.size();
	int arena_ob_size = obstacles.size();
	for (int i = 0; i < arena_size; i++)
	{
		for (int j = 0; j < arena_ob_size; j++)
		{
			if (bg::intersects(arena[i], obstacles[j]))
			{
				bg::difference(arena[i], obstacles[j], output);
				int diff = output.size() - prev_output;
				prev_output = output.size();

				// printf("Polygon_boost %d and obstacle %d intersects", i, j);
				// printf(" -> %d new cells\n", diff);

				arena[i] = output[output.size() - 1];
				if (diff > 1)
				{
					tmp_output = arena;
					for (int k = 1; k < diff; k++)
					{
						int output_idx = output.size() - 1 - k;
						vector<Polygon_boost>::iterator it;
						it = tmp_output.begin();
						tmp_output.insert(it + i + k, output[output_idx]);
						arena_size += 1;
					};
					arena = tmp_output;
				};
			};
		};
	};
	return arena;
};



void points_map::make_free_space_cells_squares(int res){
	// generate arena subsetting
	list_of_polygons *tmp_list = new list_of_polygons();
	tmp_list->add_polygon(new polygon(arena));
	
	for(int i=0; i<res; i++){
		polygon *tmp_pol = tmp_list->head;
		list_of_polygons *tmp_output = new list_of_polygons();
		while(tmp_pol != NULL){
			Edge_list *e_list = tmp_pol->edgify();
			Edge *tmp = e_list->head;

			Edge *start = e_list->head;
			while(start != NULL){
				Edge *end;
				if(start->next == NULL){
					end = e_list->head;
				}else{
					end = start->next;
				};
			
				point_list * pl_temp = new point_list();
				pl_temp->add_node(new point_node(tmp_pol->centroid->x,
												 tmp_pol->centroid->y));
				pl_temp->add_node(start->middle_point());
				pl_temp->add_node(start->points->tail);
				pl_temp->add_node(end->middle_point());
				
				polygon *cell = new polygon(pl_temp);
				tmp_output -> add_polygon(cell);

				start = start->next;
			};
		tmp_pol = tmp_pol->pnext;
		};
		tmp_list = tmp_output;
	};
	

	polygon *tmp_pol;

	// convert cells to boost polygons
	vector<Polygon_boost> cells;
	tmp_pol = tmp_list->head;
	while(tmp_pol != NULL){
		cout << "Before becaming boost: " << tmp_pol->pl->size << endl;
		cells.push_back(tmp_pol->to_boost_polygon());
		tmp_pol = tmp_pol->pnext;
	};
	cout << "Cells in vector: " << cells.size() << endl;

	// convert obstacles to boost ones
	vector<Polygon_boost> ob_boost;
	tmp_pol = obstacles->offset_head;
	while(tmp_pol != NULL){
		ob_boost.push_back(tmp_pol->to_boost_polygon());
		tmp_pol = tmp_pol->pnext;
	};

	// add gates to list of obstacles	
	/*
	tmp_pol = gates->head;
	while(tmp_pol != NULL){
		ob_boost.push_back(tmp_pol->to_boost_polygon());
		tmp_pol = tmp_pol->pnext;
	};
	*/

	// Remove obstacles from the cells
	
	vector<Polygon_boost> output;
	for(int i_o=0; i_o != ob_boost.size(); i_o++){
		for(int i_c=0; i_c != cells.size(); i_c++){
			if (bg::intersects(cells[i_c], ob_boost[i_o])){
				bg::difference(cells[i_c], ob_boost[i_o], output);
				cells[i_c] = output[output.size()-1];
			};
		};
	};

	cells = difference_of_vectors(cells, ob_boost);
	
	list_of_polygons *new_cells = new list_of_polygons();
	for(int i=0; i<cells.size(); i++){
		polygon *p = boost_polygon_to_polygon(cells[i]);
		// p->recompute_centroid();
		new_cells -> add_polygon(p);
	};
	tmp_list = new_cells;

	polygon *pol_pointer = tmp_list->head;
	while(pol_pointer != NULL){
		free_space -> add_polygon(pol_pointer);
		pol_pointer = pol_pointer->pnext;
	};
};


/**
 * This function is used to detect the free space in the arena provided.
 * The algorithm works by subtracting to the arena the obstacles identified
 * in it.
 * The arena is cut into several cells which have the arena centroid as the
 * reference point. Then the obstacles are subtracted from those cells
 * retaining only the free space in it.
 * @param[in] res: int. Is the resolution to apply at the arena subsetting.
 */
void points_map::make_free_space_cells_triangular(int res)
{	
	// Subset arena -> free space idealization
	polygon *_arena = new polygon(arena);
	list_of_polygons *_arena_subset = subset_polygon(_arena, res);
	log->add_event("Arena subsetted");

	// Arena subsets to boost::polygons;
	vector<Polygon_boost> arena_polys;
	vector<Polygon_boost> arena_obstacles;
	vector<Polygon_boost> output;
	vector<Polygon_boost> tmp_output;

	polygon *pol = _arena_subset->head;
	while (pol != NULL)
	{
		arena_polys.push_back(pol->to_boost_polygon());
		pol = pol->pnext;
	};
	log->add_event("Arena converted to boost");

	pol = obstacles->offset_head;
	while (pol != NULL)
	{
		arena_obstacles.push_back(pol->to_boost_polygon());
		pol = pol->pnext;
	};
	log->add_event("Obstacles converted to boost");

	pol = gates->head;
	while (pol != NULL)
	{
		arena_obstacles.push_back(pol->to_boost_polygon());
		pol = pol->pnext;
	};
	log->add_event("Arena gates added to obstacles list");

	// Remove the obstacles from the free space and compute the new shapes
	if (arena_obstacles.size() > 0)
	{
		// Remove obstacles from the sectors of the arena.
		arena_polys = difference_of_vectors(arena_polys, arena_obstacles);

		// Remove occlusions caused by the free space with themselves
		int prev_output = output.size();
		int arena_size = arena_polys.size();
		int arena_ob_size = arena_obstacles.size();

		output.clear();
		tmp_output.clear();
		prev_output = 0;
		for (int i = 0; i < arena_size; i++)
		{
			for (int j = 0; j < arena_size; j++)
			{
				if (i != j)
				{
					if (bg::overlaps(arena_polys[i], arena_polys[j]))
					{
						bg::difference(arena_polys[i], arena_polys[j], output);
						int diff = output.size() - prev_output;
						prev_output = output.size();

						// printf("Polygon_boost %d and polygon %d intersects", i, j);
						// printf(" -> %d new cells\n", diff);

						arena_polys[i] = output[output.size() - 1];
						if (diff > 1)
						{
							tmp_output = arena_polys;
							for (int k = 1; k < diff; k++)
							{
								int output_idx = output.size() - 1 - k;
								vector<Polygon_boost>::iterator it;
								it = tmp_output.begin();
								tmp_output.insert(it + i + k, output[output_idx]);
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
	for (int i = 0; i < output.size(); i++)
	{
		point_list *new_space_points = boost_polygon_to_point_list(output[i]);
		polygon *new_space = new polygon(new_space_points);
		free_space->add_polygon(new_space);
	};
	log->add_event("Free space generated");
};
