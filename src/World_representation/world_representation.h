#ifndef __WORLD_REPRESENTATION_H__
#define __WORLD_REPRESENTATION_H__

#include "../Utility/utility.h"
#include "../Log/logger.h"

/**
 * \struct World_node
 * This structure is used as an abstract representation of the environment.
 * This new representation will be further used to generate the ppdl file
 * for the problem solver.
 * The parameters available are:
 * @param cell_id: string. It is the identifier of the cell.
 * @param cell: polygon\*. It is the cell represented by the node.
 * @param prob: double. It is a slot in which to save the probability of
 * chosing this cell as next step by the robot.
 *
 * The methods available are:
 * @see void set_id(string id): use it to set the id of the cell.
 * @see void info(): use this function to print informations regarding the
 * current node.
 */
typedef struct World_node
{
	string cell_id;
	polygon *cell = NULL;
	double probability = 0;

	World_node(string id = "Default_cell_id", polygon *p = NULL, double _prob = 0);
	void set_id(string id);
	void info();
} World_node;

typedef struct World_representation
{
	logger *l = NULL;
	map<string, World_node> world_free_cells;
	map<string, World_node> world_gates;
	vector<World_node> obstacles;

	string pddl_connections = "NaN";
    string cell_predicates = "NaN";
    string cells_distances = "NaN";

	// map<string, Robot *> world_robots;

	World_representation(logger *log = new logger(),
                         list_of_polygons *cells = NULL,
                         list_of_polygons *gate_cells = NULL,
						 list_of_obstacles *obs=NULL,
						 string connections="NaN",
                         string cells_pred="NaN",
                         string cells_dist="NaN");
	void add_cell(World_node cell, bool gate = false);
	tuple<vector<double>, vector<double>> get_path(vector<string> plan, polygon *pol);
	string find_pddl_connections();
    string get_gates_predicates();
    string get_cells_predicates();
    string get_cells_conditional_distances();
    double distance(string el1, string el2);  // Distance among elements centroid
	void set_connections(string connections);
	void info();

} World_representation;
#endif
