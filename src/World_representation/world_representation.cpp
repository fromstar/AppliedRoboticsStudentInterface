#include "world_representation.h"
#include <../boost/geometry.hpp>
#include <sys/stat.h> // used to create folders
#include <iostream>
#include <fstream>

namespace bg = boost::geometry;
namespace bgm = bg::model;

/**
 * Is the default constructor.
 */
World_node::World_node(string id, polygon *p, double prob)
{
	cell_id = id;
	cell = p;
	probability = prob;
};

/**
 * \func void set_id(string id)
 * Use this function to set the unique identifier of the cell.
 */
void World_node::set_id(string id)
{
	cell_id = id;
};

/**
 * \func
 * This function is used to print relevant informations of the world_node.
 */
void World_node::info()
{
	cout << "Cell ID: " << cell_id << endl
		 << "Reaching probability: "
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
void World_representation::add_cell(World_node cell, bool gate)
{
	if (!gate)
	{
		int existing = world_free_cells.count(cell.cell_id);
		if (existing > 0)
		{
			cell.cell_id += "_" + to_string(existing);
		};
		world_free_cells[cell.cell_id] = cell;
	}
	else
	{
		int existing = world_gates.count(cell.cell_id);
		if (existing > 0)
		{
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
World_representation::World_representation(list_of_polygons *cells,
										   list_of_polygons *gate_cells,
										   logger *log)
{
	l = log;
	l->add_event("Created world representation");

	polygon *pol_pointer = cells->head;
	int cells_counter = 0;
	while (pol_pointer != NULL)
	{
		World_node temp = World_node("cell_" + to_string(cells_counter),
									 pol_pointer);
		add_cell(temp);
		pol_pointer = pol_pointer->pnext;
		cells_counter += 1;
	};

	l->add_event("Added free space cells to world representation");

	cells_counter = 0;
	pol_pointer = gate_cells->head;
	while (pol_pointer != NULL)
	{
		World_node temp = World_node("gate_" + to_string(cells_counter),
									 pol_pointer);
		add_cell(temp, true);
		pol_pointer = pol_pointer->pnext;
		cells_counter += 1;
	};

	l->add_event("Added gates cells to world representation");

	l->add_event("End world representation creation");
};

tuple<vector<double>, vector<double>> World_representation::get_path(vector<string> plan)
{

	// double x_path[plan.size() + 1]; // Path size + 1 for the starting position
	// double y_path[plan.size() + 1];

	vector<double> x_path;
	vector<double> y_path;

	for (int i = 0; i < plan.size(); i++)
	{
		string word;
		stringstream iss(plan[i]);
		vector<string> path;
		while (iss >> word)
			path.push_back(word);

		if(path[3].back() == ')' )
			path[3].resize(path[3].size() - 1);

		// There is only one case that path[2] is not contained in world free cell.
		// This case is the last row of the catcher plan where in that position there is
		// the fugitive id.
		if (world_free_cells.count(path[2]) != 0)
		{
			x_path.push_back(world_free_cells[path[2]].cell->centroid->x);
			y_path.push_back(world_free_cells[path[2]].cell->centroid->y);

			if (i == plan.size() - 1)
			{
				x_path.push_back(world_gates[path[3]].cell->centroid->x);
				y_path.push_back(world_gates[path[3]].cell->centroid->y);
			}
		}
		else
		{
			x_path.push_back(world_free_cells[path[3]].cell->centroid->x);
			y_path.push_back(world_free_cells[path[3]].cell->centroid->y);
		}
	}

	return make_tuple(x_path, y_path);
}

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

/*
void World_representation::to_pddl(string path_pddl_problem_file,
								   string problem_name,
								   string domain_name,
								   bool fugitive_agent)
{

	path_pddl_problem_file = "/home/davide/workspace/project/src/Pddl/problem_cathcer.pddl";

	l->add_event("Started Creation of pddl problem file.");
	string pddl_file = "";

	// Write first two lines defining the problem and domain names
	pddl_file += "(define (problem " + problem_name + ")\n\t(:domain " + domain_name + " )\n";

	// Write the objects available.
	pddl_file += "\t(:objects\n";

	for (map<string, World_node>::iterator it = world_free_cells.begin();
		 it != world_free_cells.end(); ++it)
	{
		pddl_file += "\t\t" + it->first + " - cell\n";
	};

	for (map<string, World_node>::iterator it = world_gates.begin();
		 it != world_gates.end(); ++it)
	{
		pddl_file += "\t\t" + it->first + " - gate\n";
	};

	for (map<string, Robot *>::iterator it = world_robots.begin();
		 it != world_robots.end(); ++it)
	{
		if (it->second->type != undefined)
		{
			pddl_file += "\t\t" + it->first + " - " + it->second->get_type() +
						 "\n";
		};
	};

	pddl_file += "\t)\n";

	// Write the initial state
	// 1 Write connections among cells
	pddl_file += "\t(:init\n";
	for (map<string, World_node>::iterator it_1 = world_free_cells.begin();
		 it_1 != world_free_cells.end(); ++it_1)
	{
		for (map<string, World_node>::iterator it_2 = world_free_cells.begin();
			 it_2 != world_free_cells.end(); ++it_2)
		{
			if (it_1 != it_2)
			{
				Polygon_boost pol_1 = it_1->second.cell->to_boost_polygon();
				Polygon_boost pol_2 = it_2->second.cell->to_boost_polygon();

				if (bg::touches(pol_1, pol_2))
				{ // works also if one point in common !!!!
					pddl_file += "\t\t( connected " + it_1->first + " " +
								 it_2->first + " )\n";
				};
			};
		};
	};
	// 2 Write connections among gates and cells.
	for (map<string, World_node>::iterator it_g = world_gates.begin();
		 it_g != world_gates.end(); ++it_g)
	{
		for (map<string, World_node>::iterator it_c = world_free_cells.begin();
			 it_c != world_free_cells.end(); ++it_c)
		{

			Polygon_boost pol_g = it_g->second.cell->to_boost_polygon();
			Polygon_boost pol_c = it_c->second.cell->to_boost_polygon();

			if (bg::touches(pol_g, pol_c))
			{ // works also if one point in common !!!!
				pddl_file += "\t\t( connected " + it_g->first + " " +
							 it_c->first + " )\n";
			};
		};
	};

	// 3 Write agents location
	using boost_point = bgm::d2::point_xy<double>;

	for (map<string, Robot *>::iterator it_r = world_robots.begin();
		 it_r != world_robots.end(); ++it_r)
	{
		for (map<string, World_node>::iterator it_c = world_free_cells.begin();
			 it_c != world_free_cells.end(); ++it_c)
		{

			boost_point robot_pos = boost_point(it_r->second->location->x,
												it_r->second->location->y);
			Polygon_boost pol_c = it_c->second.cell->to_boost_polygon();

			if (bg::covered_by(robot_pos, pol_c))
			{
				pddl_file += "\t\t( is_in " + it_r->first + " " +
							 it_c->first + " )\n";
				break; // skip to next robot -> it can be only in one place
			};
		};
	};
	pddl_file += "\t)\n";

	// Write goal
	pddl_file += "\t(:goal\n"; // "\t\t( and\n";
	if (fugitive_agent)
	{
		int fugitives = 0;
		for (map<string, Robot *>::iterator it_r = world_robots.begin();
			 it_r != world_robots.end(); ++it_r)
		{
			// cout << it_r -> first.c_str() << " " << it_r -> second -> ID << endl;
			vector<vector<string>> all_plans;
			if (it_r->second->type == fugitive)
			{
				fugitives += 1;
				// Chose nearest gate -> run planner for each gate and retain
				// The safest one -> must find way to interlace catcher and gate
				// distance

				// make variable to store the best_plan.
				vector<string> fugitive_plans;

				// make temporary folder for the plans.
				// system("mkdir .tmp");
				// l->add_event("Temporary folder for planning correctly "
				// 				 "created");

				int tmp_folder = mkdir(".tmp", 0777);
				if (tmp_folder != 0)
				{
					l->add_event("Temporary folder for planning correctly "
								 "created");
				}
				else
				{
					l->add_event("Unable to create temporary folder");
				};

				// Write pddl problem specific to each fugitve
				for (map<string, World_node>::iterator it_g = world_gates.begin();
					 it_g != world_gates.end(); ++it_g)
				{
					string pddl_file_fugitive = pddl_file + "\t\t( is_in " +
												it_r->first + " " +
												it_g->first + " )";
					// write file ending
					pddl_file_fugitive += "\n\t)\n)";
					string file_name = ".tmp/" + it_r->first + "_" +
									   it_g->first;


					ofstream tmp_out("/home/davide/workspace/project/src/"+file_name + ".pddl");
					if(tmp_out.is_open())
					{
						tmp_out << pddl_file_fugitive;
						tmp_out.close();
					}
					string user_folder = getenv("USER");
					string curr_dir = "/home/davide/workspace/project/src"; //get_current_dir_name();
					run_planner("/home/" + user_folder +
									"/.planutils/packages/downward/run",
								curr_dir + "/Pddl/domain_fugitive_catcher.pddl",
								curr_dir + "/" + file_name + ".pddl",
								curr_dir + "/" + file_name + ".plan");

					// Must add a flag to check if everything went good
					string tmp;
					ifstream tmp_in("/home/davide/workspace/project/src/"+file_name + ".plan");
					if(tmp_in.is_open())
					{
						while(tmp_in)
						{
							getline(tmp_in, tmp);
							fugitive_plans.push_back(tmp);
						}
						tmp_in.close();
					}

					fugitive_plans.pop_back();
					fugitive_plans.pop_back();
					all_plans.push_back(fugitive_plans);
				};

				// Find lowest number of steps plan
				int min_plan_idx = 0;
				for (int i = 0; i < all_plans.size(); i++)
				{
					if (all_plans[i].size() <
						all_plans[min_plan_idx].size())
					{
						min_plan_idx = i;
					};
				};
				cout << "Apply plan" << endl;
				// Set plan for robot fugitive
				it_r->second->set_plan(all_plans[min_plan_idx]);
				it_r->second->info();
			};
		};
		if (fugitives == 0)
		{
			printf("No fugitives found\n");
			return;
		};
	}
	else
	{
		int catchers = 0;
		for (map<string, Robot *>::iterator it_r = world_robots.begin();
			 it_r != world_robots.end(); ++it_r)
		{
			if (it_r->second->type == fugitive)
			{
				pddl_file += "\t\t\t(captured " + it_r->first + ")\n";
			}
			else
			{ // Undefined treated as catchers
				catchers += 1;
			};
		};
		if (catchers == 0)
		{
			printf("No catchers found\n");
			return;
		};
	};
	// pddl_file += "\t\t)";
	pddl_file += "\n\t)";

	// Write ending of the pddl file
	pddl_file += "\n)";;

	ofstream pddl_out(path_pddl_problem_file);
	if(pddl_out.is_open())
	{
		pddl_out << pddl_file;
		pddl_out.close();
	}
};
*/
void World_representation::info()
{
	cout << "World representation contains:\n\t"
		 << "- "
		 << world_free_cells.size() << " Cells\n\t"
		 << "- "
		 << world_gates.size() << " Gates\n\t"
		 << endl;
};
