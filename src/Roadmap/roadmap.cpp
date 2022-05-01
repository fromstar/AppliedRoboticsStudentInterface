#include "roadmap.h"
#include <vector>
#include <sys/stat.h>  // used to create folders

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

using pt         = bgm::d2::point_xy<double>;
using Polygon_boost       = bgm::polygon<pt>;
using Multi_Polygon_boost = bgm::multi_polygon<Polygon_boost>;


Robot::Robot(string _id, Robot_type _type, point_node* _loc,
			 double _max_curvature, double _offset){
	ID = _id;
	type = _type;
	location = _loc;
	max_curvature_angle = _max_curvature;
	offset = _offset;
};

/**
 * \fn void Robot::set_id(string _id)
 * Sets the identifier of the robot.
 * @param _id: string. New identifier to apply.
 */
void Robot::set_id(string _id){
	ID = _id;
};

/**
 * \fn void Robot::set_robot_type(Robot_type rt)
 * Sets the type of the robot, it will be further used during the pddl
 * creation.
 * @param rt: Robot_type. The type to apply at the robot.
 */
void Robot::set_robot_type(Robot_type rt){
	type=rt;
};

/**
 * \fun point_node* where();
 * Use this function to return the robot current position.
 * @return robot location: point_node\*.
 */
point_node* Robot::where(){
	return location;
};

/**
 * \func
 * This function is used to return the string type
 * representing the role of the robot.
 * @return char*.
 */
string Robot::get_type(){
	string _type;
	switch(type){
		case fugitive: 	_type = "fugitive";
					  	break;
		case catcher: 	_type = "catcher";
					  	break;
		default: 		_type = "undefined";
				 		break;
	};
	return _type;
};

void Robot::set_plan(vector<string> p){
	plan = p;
};

/**
 * \fn void Robot::info()
 * This fuction serves the purpose to print on screen the details of the
 * robot on which it is called.
 */
void Robot::info(){
	/*string _type;
	switch(type){
		case fugitive: 	_type = "Fugitive";
					  	break;
		case catcher: 	_type = "Catcher";
					  	break;
		default: 		_type = "Undefined";
				 		break;
	}; */
	cout << std::setprecision(2) << std::fixed;

	cout << "Robot: " << ID << endl << "Type: " << get_type() << endl
		 << "Location: " << location -> x << " - " << location -> y << endl
		 << "Max curvature angle: " << max_curvature_angle << endl
		 << "Offset in use: " << offset << endl;

	// Print plan if available
	if (plan.size() != 0){
		for(int i=0; i< plan.size(); i++){
			cout << "\t- " << i << ": " << plan[i] << endl;
		};
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

/**
 * \fun void points_map::add_robot(Robot\* r)
 * Use this function to add a robot to the mapping of the environments.
 * The functions checks whether a robot with the same id exists in the mapping
 * and if so it changes the id adding an index at its end.
 * @param r: Robot. Is the Robot object to add to the mapping.
 */
void points_map::add_robot(Robot* r){
	int existing = robot.count(r->ID);
	if (existing > 0){  // A robot with same id already exists.
		r -> ID += "_" + to_string(existing);
		add_robot(r);  // Recursive call, dangerous but ensures uniqueness.
	}else{ // No robot with that id
		robot[r->ID] = r;
	};
};

void points_map::set_robot_position(string robot_id, double x, double y){
	int existing = robot.count(robot_id);
	if (existing == 0){
		cout << "No Robot found having id = \"" << robot_id << "\"" << endl;
		return;
	};
	Robot* _robot = robot[robot_id];  // If no element in container it adds it.
	delete(_robot->location);  // free memory of previous location.
	_robot -> location = new point_node(x, y);
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
	// Print robots informations.
	for(map<string, Robot*>::iterator robot_it = robot.begin();
		robot_it != robot.end(); ++robot_it){
		robot_it -> second -> info();
		cout << endl;
	};

};

void points_map::reduce_arena(){ // Buggy function, must be re_seen
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

	for(map<string, Robot*>::iterator robot_it = robot.begin();
		robot_it != robot.end(); ++robot_it){

		point_list* robot_loc = new point_list;
		robot_loc -> add_node(robot_it -> second -> location -> copy());
		robot_loc -> add_node(robot_it -> second -> location -> copy());
		img_arena = plot_points(robot_loc, img_arena, Scalar(210, 26, 198),
								false, 5);
		delete(robot_loc);
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
point_list* boost_polygon_to_point_list(Polygon_boost p){
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


/**
 * This function is used to merge the obstacles that touch each others in the
 * arena space.
 */
void points_map::merge_obstacles()
{
	std::vector<Polygon_boost> polys;

 	polygon *pol_iter = obstacles->offset_head;

	// Convert polygons in Boost polygon object
	while(pol_iter != NULL)
	{
		polys.push_back(pol_iter->to_boost_polygon());
		pol_iter = pol_iter->pnext;
	}

	// check which polygons intersect
	vector<Polygon_boost> output;
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
		Polygon_boost _arena = arena_pol->to_boost_polygon();
		vector<Polygon_boost> tmp_pols;

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
		vector<Polygon_boost> tmp_new_gates;
		points_map* tmp_map = new points_map(NULL);

		while (tmp_gate != NULL){
			Polygon_boost _gate = tmp_gate -> to_boost_polygon();
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
	log -> add_event("Obstacles in touch merged\n");
}

/**
 * This function is used to subset a polygon into a finite number of cells.
 * The cells are formed creating a triange connecting every edge of the polygon
 * with the centroid of the object. The number of cells is num_edges^(1+levels).
 * @param[in] levels: int. Is the number of iterations to perform during the
 * polygon subsetting.
 * @param[out] subset_list: list_of_polygons.
 */
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
 * @param arena: vector<Polygon_boost>. Is the vector of polygons from which
 * the other polygons will be subtracted.
 * @param obstacles: vector<Polygon_boost>. Is the vector of polygons that will
 * be subtracted.
 */
vector<Polygon_boost> difference_of_vectors(vector<Polygon_boost> arena,
									  vector<Polygon_boost> obstacles){
	vector<Polygon_boost> output;
	vector<Polygon_boost> tmp_output;

	int prev_output = 0;
	int arena_size = arena.size();
	int arena_ob_size = obstacles.size();
	for (int i=0; i<arena_size; i++){
		for (int j=0; j<arena_ob_size; j++){
			if (bg::intersects(arena[i], obstacles[j])){
				bg::difference(arena[i], obstacles[j], output);
				int diff = output.size() - prev_output;
				prev_output = output.size();
				
				// printf("Polygon_boost %d and obstacle %d intersects", i, j);
				// printf(" -> %d new cells\n", diff);

				arena[i] = output[output.size()-1];
				if (diff > 1){
					tmp_output = arena;
					for(int k=1; k < diff; k++){
						int output_idx = output.size()-1-k;
						vector<Polygon_boost>::iterator it;
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

/**
 * This function is used to detect the free space in the arena provided.
 * The algorithm works by subtracting to the arena the obstacles identified
 * in it.
 * The arena is cut into several cells which have the arena centroid as the
 * reference point. Then the obstacles are subtracted from those cells
 * retaining only the free space in it.
 * @param[in] res: int. Is the resolution to apply at the arena subsetting.
 */
void points_map::make_free_space_cells(int res){
	// Subset arena -> free space idealization
	polygon* _arena = new polygon(arena);
	list_of_polygons* _arena_subset = subset_polygon(_arena, res);
	log -> add_event("Arena subsetted");	

	// Arena subsets to boost::polygons;
	vector<Polygon_boost> arena_polys;
	vector<Polygon_boost> arena_obstacles;
	vector<Polygon_boost> output;
	vector<Polygon_boost> tmp_output;

	polygon* pol = _arena_subset -> head;
	while(pol != NULL){
		arena_polys.push_back(pol->to_boost_polygon());
		pol = pol -> pnext;
	};
	log->add_event("Arena converted to boost");

	pol = obstacles -> offset_head;
	while(pol != NULL){
		arena_obstacles.push_back(pol->to_boost_polygon());
		pol = pol -> pnext;
	};
	log->add_event("Obstacles converted to boost");

	pol = gates -> head;
	while (pol != NULL){
		arena_obstacles.push_back(pol->to_boost_polygon());
		pol = pol -> pnext;
	};
	log->add_event("Arena gates added to obstacles list");

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

						// printf("Polygon_boost %d and polygon %d intersects", i, j);
						// printf(" -> %d new cells\n", diff);

						arena_polys[i] = output[output.size()-1];
						if (diff > 1){
							tmp_output = arena_polys;
							for(int k=1; k < diff; k++){
								int output_idx = output.size()-1-k;
								vector<Polygon_boost>::iterator it;
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
	log->add_event("Free space generated");
};

/**
 * Is the default constructor.
 */
World_node::World_node(string id, polygon* p, double prob){
	cell_id = id;
	cell = p;
	probability = prob;
};

/**
 * \func void set_id(string id)
 * Use this function to set the unique identifier of the cell.
 */
void World_node::set_id(string id){
	cell_id = id;
};

/**
 * \func
 * This function is used to print relevant informations of the world_node.
 */
void World_node::info(){
	cout << "Cell ID: " << cell_id << endl << "Reaching probability: "
		 << probability << endl;
};

/**
 * \func void add_cell(World_node cell, bool gate)
 * This function allows to add a cell to the abstract representation of
 * the environment.
 * Its parameters are:
 * @param cell: World_node. Is the space representation to add.
 * @param gate: bool. This flag specifies whether or not the cell is a gate.
 */
void World_representation::add_cell(World_node cell, bool gate){
	if (!gate){
		int existing = world_free_cells.count(cell.cell_id);
		if (existing > 0){
			cell.cell_id += "_" + to_string(existing);
		};
		world_free_cells[cell.cell_id] = cell;
	}else{
		int existing = world_gates.count(cell.cell_id);
		if (existing > 0){
			cell.cell_id += "_" + to_string(existing);
		};
		world_gates[cell.cell_id] = cell;
	};
};

/**
 * \func
 * Is the default constructor.
 * @param cells: list_of_polygons\*. Are the free space cells.
 * @param gate_cells: list_of_polygons\*. Are the cells representing the gates.
 * @param agents: map<string, Robot\*>. Are the agents available.
 * @param log: logger\*. Is the log class used to record the activity.
 */
World_representation:: World_representation(list_of_polygons* cells,
											list_of_polygons* gate_cells,
						 					map<string, Robot*> agents,
											logger* log){
	l = log;
	l->add_event("Created world representation");

	polygon* pol_pointer = cells->head;
	int cells_counter = 0;
	while(pol_pointer != NULL){
		World_node temp = World_node("Cell_" + to_string(cells_counter),
									 pol_pointer);
		add_cell(temp);
		pol_pointer = pol_pointer->pnext;
		cells_counter += 1;
	};

	l -> add_event("Added free space cells to world representation");

	cells_counter = 0;
	pol_pointer = gate_cells->head;
	while(pol_pointer != NULL){
		World_node temp = World_node("Gate_" + to_string(cells_counter),
									 pol_pointer);
		add_cell(temp, true);
		pol_pointer = pol_pointer->pnext;
		cells_counter += 1;
	};
	
	l -> add_event("Added gates cells to world representation");

	world_robots = agents;
	
	l -> add_event("Added robot agents to world representation");
	l -> add_event("End world representation creation");
};

/**
 * This function runs the planner and saves it to the specified plan file.
 * The default location of the plan is the current working directory and
 * the filename is MyPlan.plan
 * @param planner_path: string. Is the path in which the callable executable
 * file of the planner is located.
 * @param domain_file_path: string. Is the path of the domain file to use.
 * @patma problem_file_path: string. Is the path of the problem file to use.
 */
void run_planner(string planner_path, string domain_file_path,
				 string problem_file_path, string plan_path="MyPlan.plan"){

	cout << "called planner for: " << planner_path << endl << domain_file_path
		 << endl << problem_file_path << endl << plan_path << endl;

	string command = "exec " + planner_path + " --alias lama-first " +
					 "--plan-file " + plan_path + " " + 
					 domain_file_path + " " + problem_file_path;
	system(command.c_str());
};

/**
 * \func
 * This function is used to generate a pddl problem file from the abstract
 * world representation.
 * The .pddl file is overwritten every time.
 * @param path_pddl_problem_file: string. It is the path in which the problem
 * @param domain_name: string. It is the default name of the domain to use.
 * file will be saved. Default is the current path with name problem.pddl .
 * @param symmetrical: bool. Tells whether or not to suppose that two cells
 * can be accessed in both ways.
 * @param fugitive: bool. Tells whether or not the problem file has been
 * created for a fugitive-type agent. This flag changes the goal to achieve.
 */
void World_representation::to_pddl(string path_pddl_problem_file,
								   string problem_name,
				 				   string domain_name,
								   bool fugitive_agent){

	l -> add_event("Started Creation of pddl problem file.");
	string pddl_file = "";

	// Write first two lines defining the problem and domain names
	pddl_file += "(define (problem " + problem_name + ")\n\t(:domain "
				 + domain_name + " )\n";

	// Write the objects available.
	pddl_file += "\t(:objects\n";

	for (map<string, World_node>::iterator it = world_free_cells.begin();
		 it != world_free_cells.end(); ++it){
		pddl_file += "\t\t" + it->first + " - cell\n";	
	};
	
	for (map<string, World_node>::iterator it = world_gates.begin();
		 it != world_gates.end(); ++it){
		pddl_file += "\t\t" + it->first + " - gate\n";	
	};
	
	for (map<string, Robot*>::iterator it = world_robots.begin();
		 it != world_robots.end(); ++it){
		if (it -> second -> type != undefined){
			pddl_file += "\t\t" + it->first + " - " + it->second->get_type() +
						 "\n";
		};
	};

	pddl_file += "\t)\n";
	
	// Write the initial state
	// 1 Write connections among cells
	pddl_file += "\t(:init\n";
	for (map<string, World_node>::iterator it_1 = world_free_cells.begin();
		 it_1 != world_free_cells.end(); ++it_1){
		for (map<string, World_node>::iterator it_2 = world_free_cells.begin();
		 it_2 != world_free_cells.end(); ++it_2){
			if (it_1 != it_2){
				Polygon_boost pol_1 = it_1->second.cell->to_boost_polygon();
				Polygon_boost pol_2 = it_2->second.cell->to_boost_polygon();

				if (bg::touches(pol_1, pol_2)){ // works also if one point in common !!!!
					pddl_file += "\t\t( connected " + it_1->first + " " +
								 it_2->first + " )\n";
				};
			};
		};
	};
	
	// 2 Write connections among gates and cells.
	for (map<string, World_node>::iterator it_g = world_gates.begin();
		 it_g != world_gates.end(); ++it_g){
		for (map<string, World_node>::iterator it_c = world_free_cells.begin();
		 	 it_c != world_free_cells.end(); ++it_c){

			Polygon_boost pol_g = it_g->second.cell->to_boost_polygon();
			Polygon_boost pol_c = it_c->second.cell->to_boost_polygon();

			if (bg::touches(pol_g, pol_c)){ // works also if one point in common !!!!
				pddl_file += "\t\t( connected " + it_g->first + " " +
							 it_c->first + " )\n";
			};
		};
	};


	// 3 Write agents location
	using boost_point = bgm::d2::point_xy<double>;

	for (map<string, Robot*>::iterator it_r = world_robots.begin();
		 it_r != world_robots.end(); ++it_r){
		for (map<string, World_node>::iterator it_c = world_free_cells.begin();
		 	 it_c != world_free_cells.end(); ++it_c){

			boost_point robot_pos = boost_point(it_r->second->location->x,
												it_r->second->location->y);
			Polygon_boost pol_c = it_c->second.cell->to_boost_polygon();

			if (bg::covered_by(robot_pos, pol_c)){
				pddl_file += "\t\t( is_in " + it_r->first + " " +
							 it_c->first + " )\n";
				break; // skip to next robot -> it can be only in one place
			};
		};
	};
	pddl_file += "\t)\n";

	// Write goal
	pddl_file += "\t(:goal\n"; // "\t\t( and\n";
	if (fugitive_agent){
		int fugitives = 0;
		for (map<string, Robot*>::iterator it_r = world_robots.begin();
			 it_r != world_robots.end(); ++it_r){
			// cout << it_r -> first.c_str() << " " << it_r -> second -> ID << endl;
			vector<vector<string>> all_plans;
			if (it_r -> second -> type == fugitive){
				fugitives += 1;
				// Chose nearest gate -> run planner for each gate and retain
				// The safest one -> must find way to interlace catcher and gate
				// distance
				
				/*
				int escape_gate = ((rand()%world_gates.size()-1))+1;
				map<string, World_node>::iterator it_g = world_gates.begin();
				for(int i=0; i < escape_gate; i++){++it_g;};
				// cout << it_r -> first.c_str() << " " << it_g -> first.c_str() << endl;
				pddl_file += "\t\t\t( is_in " + it_r->first + " " +
							 it_g->first + " )\n";
				*/

				// make variable to store the best_plan.
				vector<string> fugitive_plans;

				// make temporary folder for the plans.
				int tmp_folder = mkdir(".tmp", 0777);
				if (tmp_folder != 0){
					l->add_event("Temporary folder for planning correctly "
								 "created");
				}else{
					l->add_event("Unable to create temporary folder");					
				};

				// Write pddl problem specific to each fugitve
				for (map<string, World_node>::iterator it_g=world_gates.begin();
					 it_g != world_gates.end(); ++it_g){
					string pddl_file_fugitive = pddl_file + "\t\t( is_in " +
										 it_r->first + " " +
										 it_g->first + " )";
					// write file ending
					pddl_file_fugitive += "\n\t)\n)";
					string file_name = ".tmp/" + it_r->first + "_" +
									   it_g->first;

					FILE* tmp_out = fopen((file_name + ".pddl").c_str(), "w");
					fprintf(tmp_out, "%s", pddl_file_fugitive.c_str());
					fclose(tmp_out);

					string user_folder = getenv("USER");
					string curr_dir = get_current_dir_name();
					run_planner("/home/" + user_folder +
								"/.planutils/packages/downward/run",
								curr_dir + "/Pddl/domain_fugitive_catcher.pddl",
								curr_dir + "/" + file_name + ".pddl",
								curr_dir + "/" + file_name + ".plan");

					// Must add a flag to check if everything went good
					FILE* tmp_in = fopen((file_name+".plan").c_str(), "r");
					char tmp[1000];
					cout << "reading file" << endl;
					while(fgets(tmp, 1000, tmp_in)){
						fugitive_plans.push_back(tmp);
					};
					fclose(tmp_in);
					fugitive_plans.pop_back();
					all_plans.push_back(fugitive_plans);
				};

				// Find lowest number of steps plan
				int min_plan_idx=0;
				for (int i=0; i<all_plans.size(); i++){
					if (all_plans[i].size() <
						all_plans[min_plan_idx].size()){
							min_plan_idx = i;
					};
				};
				cout << "Apply plan" << endl;
				// Set plan for robot fugitive
				it_r -> second -> set_plan(all_plans[min_plan_idx]);
				it_r -> second -> info();
			};
		};
		if (fugitives == 0){printf("No fugitives found\n"); return;};
	}else{
		int catchers = 0;
		for (map<string, Robot*>::iterator it_r = world_robots.begin();
			 it_r != world_robots.end(); ++it_r){
			if (it_r -> second -> type == fugitive){
				pddl_file += "\t\t\t(captured " + it_r->first + ")\n";
			}else{  // Undefined treated as catchers
				catchers += 1;
			};
		};
		if (catchers == 0){printf("No catchers found\n"); return;};
	};
	// pddl_file += "\t\t)";
	pddl_file += "\n\t)";

	// Write ending of the pddl file
	pddl_file += "\n)";

	FILE* pddl_out = fopen(path_pddl_problem_file.c_str(), "w");
	fprintf(pddl_out, "%s", pddl_file.c_str());  //write files out
	fclose(pddl_out);
	l -> add_event("Ended Creation of pddl problem file.");
};

void World_representation::info(){
	cout << "World representation contains:\n\t" << "- "
		 << world_free_cells.size() << " Cells\n\t" << "- "
		 << world_gates.size() << " Gates\n\t" << "- "
		 << world_robots.size() << " Robots"
		 << endl;
};
