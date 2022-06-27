#include "world_representation.h"
#include <../boost/geometry.hpp>
#include <sys/stat.h> // used to create folders
#include <iostream>
#include <fstream>
#include <algorithm>

namespace bg = boost::geometry;
namespace bgm = bg::model;

/**
 * \fun
 * Is the default constructor.
 */
World_node::World_node(string id, polygon *p, double prob)
{
	set_id(id);
	cell = p;
	probability = prob;
};

/**
 * \func void set_id(string id)
 * Use this function to set the unique identifier of the cell.
 */
void World_node::set_id(string id)
{
	// cout << "Pre id: " << id << endl;
	transform(id.begin(), id.end(), id.begin(), ::toupper);
	// cout << "Post id: " << id << endl << endl;
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
World_representation::World_representation(logger *log,
										   list_of_polygons *cells,
										   list_of_polygons *gate_cells,
										   list_of_obstacles *obs,
										   string connections)
{
	l = log;
	l->add_event("Creating world representation");

	l->add_event("Adding free space cells to world representation");
	polygon *pol_pointer = cells->head;
	int cells_counter = 0;
	World_node temp;
	while (pol_pointer != NULL)
	{
		if (pol_pointer->id == "NaN")
		{
			temp = World_node("cell_" + to_string(cells_counter), pol_pointer);
		}
		else
		{
			temp = World_node(pol_pointer->id, pol_pointer);
		}
		add_cell(temp);
		cells_counter += 1;
		pol_pointer = pol_pointer->pnext;
	};

	l->add_event("Adding gates cells to world representation");
	cells_counter = 0;
	pol_pointer = gate_cells->head;
	while (pol_pointer != NULL)
	{
		if (pol_pointer->id == "NaN")
		{
			temp = World_node("gate_" + to_string(cells_counter), pol_pointer);
		}
		else
		{
			temp = World_node(pol_pointer->id, pol_pointer);
		}
		add_cell(temp, true);
		cells_counter += 1;
		pol_pointer = pol_pointer->pnext;
	};

	if (obs != NULL)
	{
		polygon *tmp = obs->head;
		while (tmp != NULL)
		{
			World_node ob_temp = World_node("Obstacle", tmp);
			obstacles.push_back(ob_temp);
			tmp = tmp->pnext;
		};
		l->add_event("Added obstacles");
	};

	if (connections != "NaN")
	{
		pddl_connections = connections;
		l->add_event("Added pddl connections");
	};
	l->add_event("Ended world representation creation");
};

/**
 * \fun
 * This method is used to parse a plan and transform it into two vectors
 * representing the x and y coordinates of the cells contained in the plan.
 * @param plan: vector<string>. It is the plan to parse.
 */
tuple<vector<double>, vector<double>> World_representation::get_path(vector<string> plan, polygon *pol)
{

	vector<double> x_path;
	vector<double> y_path;
	point_node *middle_point = NULL;
	Edge *common_edge = NULL;
	string prec_id;
	double x, y;

	for (int i = 0; i < plan.size(); i++)
	{
		string word;
		stringstream iss(plan[i]);
		vector<string> path;
		while (iss >> word)
        {
			path.push_back(word);
        }

		if (path[3].back() == ')')
			path[3].resize(path[3].size() - 1);

		// There is only one case that path[2] is not contained in world free cell.
		// This case is the last row of the catcher plan where in that position there is
		// the fugitive id.
		if (world_free_cells.count(path[2]) != 0)
		{
			x = world_free_cells[path[2]].cell->centroid->x;
			y = world_free_cells[path[2]].cell->centroid->y;
            /*
            cout << path[2] << endl;
			if (i != 0)
			{
				cout << "prec_id: " << prec_id << endl;
				cout << "actual_id: " << path[2] << endl;
				cout<<"SIZE: " << world_free_cells[prec_id].cell->common_edges.size() << endl;
				cout << "OKEE\n"<<endl;

				// common_edge = world_free_cells[prec_id].cell->common_edges[path[2]]->head;
                
                polygon *pol_a = world_free_cells[prec_id].cell;
                polygon *pol_b = world_free_cells[path[2]].cell;
                common_edge = find_common_edge(pol_a, pol_b);
				//common_edge->info();
				middle_point = los(new point_node(x_path[i - 1], y_path[i - 1]),
                                   new point_node(x, y), pol, common_edge);
				if (middle_point != NULL)
				{
					// cout << "NO LOS." << endl;
					// cout << "middle ponint: " << middle_point->x <<":"<<middle_point->y<<endl;
					x_path.push_back(middle_point->x);
					y_path.push_back(middle_point->y);
				}
                cout << "Hello 1" << endl;
			}
            */
			x_path.push_back(x);
			y_path.push_back(y);

			if (i == plan.size() - 1)
			{
				x = world_gates[path[3]].cell->centroid->x;
				y = world_gates[path[3]].cell->centroid->y;
				/*
                if (i != 0)
				{
					//Potrebbe essere che il gate in realtà non sia adiacente alla cella. Occhio!
					// common_edge = world_free_cells[prec_id].cell->common_edges[path[3]]->head;
                    polygon *pol_a = world_free_cells[prec_id].cell;
                    polygon *pol_b = world_gates[path[3]].cell;
                    common_edge = find_common_edge(pol_a, pol_b);

					middle_point = los(new point_node(x_path[i - 1], y_path[i - 1]),
                                       new point_node(x, y), pol, common_edge);
                    cout << "Hello 2" << endl;
					if (middle_point != NULL)
					{
						x_path.push_back(middle_point->x);
						y_path.push_back(middle_point->y);
					}
				}
                */
				x_path.push_back(x);
				y_path.push_back(y);
			}
		}
		else
		{
			x = world_free_cells[path[3]].cell->centroid->x;
			y = world_free_cells[path[3]].cell->centroid->y;
            /*
			if (i != 0)
			{
				// common_edge = world_free_cells[prec_id].cell->common_edges[path[3]]->head;

                polygon *pol_a = world_free_cells[prec_id].cell;
                polygon *pol_b = world_free_cells[path[3]].cell;
                common_edge = find_common_edge(pol_a, pol_b);

				middle_point = los(new point_node(x_path[i - 1], y_path[i - 1]),
                                   new point_node(x, y), pol, common_edge);
				if (middle_point != NULL)
				{
					x_path.push_back(middle_point->x);
					y_path.push_back(middle_point->y);
				}
                cout << "Hello 3" << endl;

			}
            */
			x_path.push_back(x);
			y_path.push_back(y);
		}
		prec_id = path[2];
		// cout << "ID: " << prec_id << endl;
	}

	return make_tuple(x_path, y_path);
}

void World_representation::set_connections(string connections)
{
	pddl_connections = connections;
};

/**
 * \fun
 * This method is used to return a pddl representation of the connections
 * among the free space cells.
 * @return string: the pddl string representing the connections;
 */
string World_representation::find_pddl_connections()
{
	l->add_event("World_representation: Finding cells connections.");
	if (pddl_connections != "NaN")
	{
		l->add_event("World_representation: "
					 "Cells connections already in memory -> returning.");
		return pddl_connections;
	}
	else
	{
		// string *connections = new string("");
		pddl_connections = "";
		map<string, World_node>::iterator it_1;
		map<string, World_node>::iterator it_2;

		for (it_1 = world_free_cells.begin(); it_1 != world_free_cells.end();
			 ++it_1)
		{

			Polygon_boost p1 = it_1->second.cell->to_boost_polygon();
			for (it_2 = world_free_cells.begin();
				 it_2 != world_free_cells.end();
				 ++it_2)
			{
				if (it_1 != it_2)
				{

					Polygon_boost p2 = it_2->second.cell->to_boost_polygon();
					if (bg::touches(p1, p2))
					{
						pddl_connections += "\t\t( connected " + it_1->first + " " +
											it_2->first + " )\n";
					};

					polygon *temp_1 = it_1->second.cell;
					polygon *temp_2 = it_2->second.cell;
					int points_in_common = temp_1->points_in_common(temp_2);
					if (points_in_common == 1)
					{
						pddl_connections += "\t\t( is_diagonal " + it_1->first +
											" " + it_2->first + " )\n";
					};
				};
			};
		};
		// pddl_connections = connections;
		return pddl_connections;
	};
};

/**
 * \fun
 * This method allows to print relevant informations regarding the
 * abstract representation of the environment.
 */
void World_representation::info()
{
	cout << "World representation contains:\n\t"
		 << "- "
		 << world_free_cells.size() << " Cells\n\t"
		 << "- "
		 << world_gates.size() << " Gates"
		 << endl;
};
