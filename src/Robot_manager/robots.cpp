#include "robot_manager.h"
#include <sys/stat.h>
#include <../boost/geometry.hpp>
#include <fstream>
#include <bits/stdc++.h>
#include <filesystem>
#include <algorithm>

using pt = boost::geometry::model::d2::point_xy<double>;
using Polygon_boost = boost::geometry::model::polygon<pt>;

/**
 * \fun
 * Use this function to remove the first and last characters of a sentence
 * or a token.
 * @param input_string: string. It is the input string to modify
 * @returns string. It is the transformed string.
 */
string remove_first_and_last_char(string input_string)
{
	string pre_processed_string = input_string;
	pre_processed_string.erase(0, 1); // Remove starting (
	pre_processed_string.pop_back();  // Remove ending )
	return pre_processed_string;
};

/**
 * \fun
 * Use this function to return a word with the first letter capital.
 * It is useful on the problem file.
 * @param word: string. Is the input word.
 * @return capitalized word: string.
 */
string upperify(string word)
{
	word[0] = toupper(word[0]);
	return word;
};

/**
 * \fun
 * Use this function to return a word with the first letter in lowercase.
 * It is useful on the problem file.
 * @param word: string. Is the input word.
 * @return lowercase word: string.
 */
string lowerify(string word)
{
	word[0] = tolower(word[0]);
	return word;
};

/**
 * \fun
 * This function allows to split a string with respect to a precise token
 * @param sentence: string. Is the string to split.
 * @param token: string. Is the token that when found will trigger the spliting.
 *
 * @return tmp: vector<string>. Is the list containing the input string splitted.
 */
vector<string> string_to_vector(string sentence, string token)
{
	stringstream ss(sentence);
	string word;
	vector<string> tmp;
	while (ss >> word)
	{
		if (word != token)
		{
			tmp.push_back(word);
		};
	};
	return tmp;
};

/**
 * \fun
 * This is the default constructor for the robot struct.
 * @param _id: string. It is the unique id of the robot.
 * @param _type: Robot_type. It is the type associated to the robot and defines
 * its behaviour in the environment. Available values are:
 * - catcher: The robot will try to capture the fugitives.
 * - fugitive: The robot will try to escape from the catcher and reach a gate.
 * - undefined: It is a general_purpose type, never to be used.
 * @param _l: logger\*. It is the logger instance which will be used to 
 * print out the events regarding the Robot instance.
 * @param _loc: point_node\*. It is the current location of the robot.
 * @param _max_curvature: double. It is the maximum curvature angle that
 * the robot can achieve -> physical limit. It is expressed in radiants.
 * @param _offset: double. It is the offset to be used in enlargin the robot
 * to avoid collisions with the obstacles during the planning phase.
 */
Robot::Robot(string _id, Robot_type _type, logger* _l, point_node *_loc,
			 double _max_curvature, double _offset)
{
	set_id(_id);
	type = _type;
	location = _loc;
	max_curvature_angle = _max_curvature;
	offset = _offset;
	l = _l;
};

/**
 * \fun
 * This method allows to set the logger to be used by the Robot instance.
 * @param _l: logger\*. It is the logger instance to be used for logging.
 */
void Robot::set_logger(logger *_l)
{
	l = _l;
};

/**
 * \fn void Robot::set_id(string _id)
 * Sets the identifier of the robot.
 * @param _id: string. New identifier to apply.
 */
void Robot::set_id(string _id)
{
	// Make everything lowercase
	_id = lowerify(_id);

	// Remove spaces from the ID
	replace(_id.begin(), _id.end(), ' ', '_');

	// Write new ID
	ID = _id;
};

/**
 * \fn void Robot::set_robot_type(Robot_type rt)
 * Sets the type of the robot, it will be further used during the pddl
 * creation.
 * @param rt: Robot_type. The type to apply at the robot.
 */
void Robot::set_robot_type(Robot_type rt)
{
	type = rt;
};

/**
 * \fun point_node* where();
 * Use this function to return the robot current position.
 * @return robot location: point_node\*.
 */
point_node *Robot::where()
{
	return location;
};

/**
 * \func
 * This function is used to return the string type
 * representing the role of the robot.
 * @return char*.
 */
string Robot::get_type()
{
	string _type;
	switch (type)
	{
	case fugitive:
		_type = "fugitive";
		break;
	case catcher:
		_type = "catcher";
		break;
	default:
		_type = "undefined";
		break;
	};
	return _type;
};

/**
 * \fun set_plan(vector<string> p)
 * This method is use to set the plan that the robot has to follow. The plan is
 * a sequence of string representing pddl expressions.
 * @param p: vector<string>. The vector of commands to execute.
 */
void Robot::set_plan(vector<string> p)
{
	plan = p;
};

/**
 * \fun
 * This method is used to set the goal of that robot has to pursue. It must be
 * written as a PDDL expression. It will be further used in the creation of the
 * pddl problem file.
 * @param d: string. Is the goal written in pddl.
 */
void Robot::set_desire(string d)
{
	desire = d;
};

/**
 * \fn void Robot::info()
 * This fuction serves the purpose to print on screen the details of the
 * robot on which it is called.
 */
void Robot::info()
{
	cout << std::setprecision(2) << std::fixed;

	cout << "Robot: " << ID << endl
		 << "Type: " << get_type() << endl
		 << "Location: " << location->x << " - " << location->y << endl
		 << "Max curvature angle: " << max_curvature_angle << endl
		 << "Offset in use: " << offset << endl;
	if (desire != "NaN")
	{
		cout << "Desire: " << desire << endl;
	};

	// Print plan if available
	if (plan.size() != 0)
	{
		cout << "Plan:" << endl;
		for (int i = 0; i < plan.size(); i++)
		{
			cout << "\t- " << i << ": " << plan[i] << endl;
		};
	};
};

/**
 * \fun get_behaviour()
 * This method is used to set the behaviour of the robot in chosing the gate
 * from which to escape. Available values are:
 * - least_steps: minimum number of moves to reach the gate.
 * - undeterministic: chose gate randomically.
 * - aware: chose the best gate maximizing the distance among its antagonists.
 */
string robot_fugitive::get_behaviour()
{
	string _behaviour;
	switch (behaviour)
	{
	case least_steps:
		_behaviour = "least_steps";
		break;
	case undeterministic:
		_behaviour = "undeterministic";
		break;
	case aware:
		_behaviour = "aware";
		break;
	};
	return _behaviour;
};

/**
 * \fun
 * This is the default constructor for the structure.
 * @param _self: Robot\*. Is the robot in use.
 * @param path: string. Is the path in which the robot pddl files will be
 * created.
 */
robot_fugitive::robot_fugitive(Robot *_self, string path,
							   behaviour_fugitive b)
{
	self = _self;
	filesPath = path;
	behaviour = b;
};

/**
 * \fun
 * This function is used to write a log message exploiting the Robot
 * logger attribute existing in the Robot instance inside the robot_fugitive
 * struct
 * @param message: string. It is the message to write on the logger
 */
void robot_fugitive::to_log(string message){
	self->l->add_event(self->ID + ": " + message);
};

/**
 * \fun
 * This function is used to add a Robot instance to the list of antagonists
 * of the fugitive.
 * @param r_ant: Robot\*. It is the instance of the antagonist.
 */
void robot_fugitive::add_antagonist(Robot *r_ant)
{
	antagonists.push_back(r_ant);
};

/**
 * \fun
 * This function is used to set the behaviour of the fugitive robot.
 * @param b: behaviour_fugitive. It is the enumerative value representing the
 * behaviour in use by the fugitive. Available values are:
 * - least_steps: the fugitive will reach the nearest gate -> nearest gate
 *   is the one reachable in the least number of movements.
 * - undeterministic: the fugitive will chose a gate randomly.
 * - aware: the fugitive will chose the gate as the one being at maximum
 *   distance from the nearest catcher.
 */

void robot_fugitive::set_behaviour(behaviour_fugitive b)
{
	behaviour = b;
};

/**
 * \fun
 * This method is used to write the files for the fugitive robot.
 * @param file_name: string. It is the path in which to write the file.
 * @param what_to_Write: string. It is the content of the file.
 * @param extension: string. It is the file extension to be used.
 */
void robot_fugitive::write_file(string file_name, string what_to_write,
								string extension)
{
	ofstream file_out(filesPath + "/" + file_name + extension);
	if (file_out.is_open())
	{
		file_out << what_to_write;
		file_out.close();
	}
	else
	{
		string error_msg = "Unable to open output stream towards " + file_name;
		to_log(error_msg);
		throw std::logic_error(error_msg);
	};
};

/**
 * \fun
 * This method is used to create the domain file for the fugitive robot.
 *  */
string robot_fugitive::make_pddl_domain_file()
{
	to_log("Writing domain file");

	// Define name of the domain
	string domain_name = "domain_" + self->ID;

	// Write header of the file
	string domain_file =
				"(define (domain " + domain_name + ")\n"
				
				// Write requirements
				"\t(:requirements "
				"\n\t\t:strips :typing :negative-preconditions"
				"\n\t\t:disjunctive-preconditions\n\t)\n"

				// Write types
				"\t(:types\n\t\tfugitive catcher - robot"
			    "\n\t\tcell gate - location\n\t)\n"

				// Write predicates
				"\t(:predicates\n"
				"\t\t(is_in ?r - robot ?loc - location)\n"
				"\t\t(connected ?loc_start - location ?loc_end - location)"
				"\n\t)"

				// Write actions
				"\n\t(:action move"
			    "\n\t\t:parameters (?r - fugitive ?loc_start - location "
			    "?loc_end - location)"
			    "\n\t\t:precondition"
			    "\n\t\t\t(and"
			    "\n\t\t\t\t( is_in ?r ?loc_start )"
			    "\n\t\t\t\t( or"
			    "\n\t\t\t\t\t( connected ?loc_start ?loc_end )"
			    "\n\t\t\t\t\t( connected ?loc_end ?loc_start )"
			    "\n\t\t\t\t)"
			    "\n\t\t\t)"
			    "\n\t\t:effect"
			    "\n\t\t\t(and"
			    "\n\t\t\t\t( not ( is_in ?r ?loc_start ) )"
			    "\n\t\t\t\t( is_in ?r ?loc_end)"
			    "\n\t\t\t)"
			    "\n\t)"

				// Write file end
				"\n)";

	// make folder if it doesn't exist
	int tmp_folder = mkdir(filesPath.c_str(), 0777);
	write_file(domain_name, domain_file, ".pddl");
	return domain_name;
};

string robot_fugitive::make_pddl_problem_file(World_representation wr)
{
	to_log("Writing problem file");

	int tmp_folder = mkdir(filesPath.c_str(), 0777);

	// Define name of the problem
	string problem_name = "problem_" + self->ID;
	string domain_name = "domain_" + self->ID;

	// Write header of the file
	string problem_file = "(define (problem " + problem_name + ")\n";

	// Write domain in use
	problem_file += "\n\t(:domain " + domain_name + " )\n";

	// Write objects available
	to_log("Writing available objects");
	problem_file += "\t(:objects\n";
	map<string, World_node>::iterator it;
	for (it = wr.world_free_cells.begin(); it != wr.world_free_cells.end(); ++it)
	{
		problem_file += "\t\t\t" + it->first + " - cell\n";
	};
	for (it = wr.world_gates.begin(); it != wr.world_gates.end(); ++it)
	{
		problem_file += "\t\t\t" + it->first + " - gate\n";
	};

	// write agents
	to_log("Writing agents");
	problem_file += "\t\t\t" + self->ID + " - fugitive";
	if (antagonists.size() != 0)
	{
		for (int i = 0; i < antagonists.size(); i++)
		{
			problem_file += "\n\t\t\t" + antagonists[i]->ID + " - " +
							antagonists[i]->get_type();
		}
	};
	problem_file += "\n\t)\n";

	// Write initial state
	problem_file += "\t(:init\n";
	map<string, World_node>::iterator it_1;
	map<string, World_node>::iterator it_2;

	// cells connected
	to_log("Writing cells connections");
	problem_file += wr.find_pddl_connections();

	/*
	for (it_1 = wr.world_free_cells.begin(); it_1 != wr.world_free_cells.end();
		 ++it_1)
	{
		Polygon_boost p1 = it_1->second.cell->to_boost_polygon();
		for (it_2 = wr.world_free_cells.begin(); it_2 != wr.world_free_cells.end();
			 ++it_2)
		{
			if (it_1 != it_2)
			{
				Polygon_boost p2 = it_2->second.cell->to_boost_polygon();
				if (boost::geometry::touches(p1, p2))
				{
					problem_file += "\t\t( connected " + it_1->first + " " +
									it_2->first + " )\n";
				};
			};
		};
	};
	*/

	// gates in cells
	to_log("Find and write in which cells the gates are");
	for (it_1 = wr.world_free_cells.begin(); it_1 != wr.world_free_cells.end();
		 ++it_1)
	{
		Polygon_boost p1 = it_1->second.cell->to_boost_polygon();
		for (it_2 = wr.world_gates.begin(); it_2 != wr.world_gates.end();
			 ++it_2)
		{
			Polygon_boost p2 = it_2->second.cell->to_boost_polygon();
			if (boost::geometry::intersects(p1, p2) ||
				boost::geometry::covered_by(p2, p1))
			{
				problem_file += "\t\t( connected " + it_2->first + " " +
								it_1->first + " )\n";
			};
		};
	};

	// agents location
	// Fugitive
	to_log("Find and write agents location");
	pt fugitive_location = pt(self->location->x, self->location->y);
	for (it_1 = wr.world_free_cells.begin(); it_1 != wr.world_free_cells.end();
		 ++it_1)
	{
		Polygon_boost p1 = it_1->second.cell->to_boost_polygon();

		if (boost::geometry::covered_by(fugitive_location, p1))
		{
			problem_file += "\t\t( is_in " + self->ID + " " +
							it_1->first + " )\n";
			break; // can only be in one position
		};
	};
	to_log("Ended fugitive search");

	// Antagonists
	for (it_1 = wr.world_free_cells.begin(); it_1 != wr.world_free_cells.end();
		 ++it_1)
	{
		Polygon_boost p1 = it_1->second.cell->to_boost_polygon();
		for (int i = 0; i < antagonists.size(); i++)
		{
			pt antagonist_pos = pt(antagonists[i]->location->x,
								   antagonists[i]->location->y);
			if (boost::geometry::covered_by(antagonist_pos, p1))
			{
				problem_file += "\t\t( is_in " + antagonists[i]->ID + " " +
								it_1->first + " )\n";
			};
		};
	};
	problem_file += "\t)\n";
	to_log("Ended antagonists search");

	// Write goal
	problem_file += "\t(:goal\n";
	switch (behaviour)
	{
	case least_steps:
	{
		vector<vector<string>> all_plans;
		for (it_1 = wr.world_gates.begin(); it_1 != wr.world_gates.end();
			 ++it_1)
		{
			string tmp_out = problem_file + "\t\t( is_in " + self->ID +
							 " " + it_1->first + " )\n\t)\n\n)";
			string tmp_name = problem_name + "_" + it_1->first;
			write_file(tmp_name, tmp_out, ".pddl");
			all_plans.push_back(make_plan(false, domain_name,
										  tmp_name, tmp_name));

			// remove files, they are useless now -> also checked in future
			remove((filesPath + "/" + tmp_name + ".plan").c_str());
		};
		int idx_least = 0;

		for (int i = 0; i < all_plans.size(); i++)
		{
			if (all_plans[i].size() < all_plans[idx_least].size())
			{
				idx_least = i;
			};
		};

		// identify desire and write goal
		it_1 = wr.world_gates.begin();
		for (int i = 0; i < idx_least; i++)
		{
			++it_1;
		};
		self->set_plan(all_plans[idx_least]);
		self->set_desire("( is_in " + self->ID + " " + it_1->first + " )");
		to_log("End least_steps behaviour");
		return problem_name + "_" + it_1->first;
		break;
	};
	case undeterministic:
	{
		cout << "In undeterministic branch" << endl;
		// chose gate randomly
		srand(time(0));
		int idx_gate = rand() % wr.world_gates.size();
		map<string, World_node>::iterator it_g = wr.world_gates.begin();
		for (int i = 0; i < idx_gate; i++)
		{
			++it_g;
		};
		self->set_desire("( is_in " + self->ID + " " + it_g->first + " )");
		string tmp_file = problem_file + "\t\t" + self->desire +
						  "\n\t)\n\n)";
		string tmp_problem_name = problem_name + "_" + it_g->first;
		write_file(tmp_problem_name, tmp_file, ".pddl");
		make_plan(true, domain_name, tmp_problem_name,
				  tmp_problem_name);

		// Remove useless files.
		remove((filesPath + "/" + tmp_problem_name + ".plan").c_str());
		to_log("End undeterministic behaviour");
		return tmp_problem_name;
		break;
	};
	case aware:
	{
		// Check if behaviour can be done
		if (antagonists.size() == 0)
		{
			cout << "No antagonist found, use method \"trade_fugitives()\""
					" on the robot manager if any antagonist exists"
				 << endl;
			return "NaN";
		};

		// Chose the gate which distance is less or equal than the
		// distance from the fugitive to its antagonists.
		map<string, World_node>::iterator it_g;
		map<string, World_node>::iterator it_c;

		// vector<string> antagonists_locations;
		vector<vector<string>> antagonists_distance;
		vector<vector<string>> gates_distance;

		// Compute distance from fugitive to gates
		int counter = 0;
		int idx_min_dist = 0;
		int prev_dist = 0;
		for (it_g = wr.world_gates.begin(); it_g != wr.world_gates.end();
			 ++it_g)
		{
			string tmp_out = problem_file + "\t\t( is_in " + self->ID +
							 " " + it_g->first + " )\n\t)\n\n)";
			string tmp_name = problem_name + "_" + it_g->first;
			write_file(tmp_name, tmp_out, ".pddl");
			vector<string> plan = make_plan(false, "domain_" + self->ID,
											tmp_name, tmp_name);

			// Remove useless files
			remove((filesPath + "/" + tmp_name + ".plan").c_str());

			if (counter == 0)
			{ // computes min distance index gate
				prev_dist = plan.size();
			}
			else
			{
				if (plan.size() < gates_distance[idx_min_dist].size())
				{
					idx_min_dist = counter;
				};
			};
			gates_distance.push_back(plan);
			counter++;
		};

		// compute distance from fugitive to antagonists
		for (int i = 0; i < antagonists.size(); i++)
		{
			pt antagonist_pos = pt(antagonists[i]->location->x,
								   antagonists[i]->location->y);
			for (it_c = wr.world_free_cells.begin();
				 it_c != wr.world_free_cells.end(); ++it_c)
			{
				Polygon_boost cell = it_c->second.cell->to_boost_polygon();
				if (boost::geometry::covered_by(antagonist_pos, cell))
				{
					// antagonists_locations.push_back(it_c->first);
					string tmp_out = problem_file + "\t\t( is_in " +
									 self->ID + " " + it_c->first +
									 " )\n\t)\n\n)";
					string tmp_name = problem_name + "_" +
									  antagonists[i]->ID;
					write_file(tmp_name, tmp_out, ".pddl");
					antagonists_distance.push_back(make_plan(
						false,
						"domain_" + self->ID,
						tmp_name, tmp_name));
					// Remove plan
					remove((filesPath + "/" + tmp_name + ".plan").c_str());
					break; // can only be in one location.
				};
			};
		};

		// Apply rule to chose which gate
		// Step 1: find nearest catcher to the fugitive
		int idx_near = 0;
		for (int i = 0; i < antagonists_distance.size(); i++)
		{
			if (antagonists_distance[i] < antagonists_distance[idx_near])
			{
				idx_near = i;
			};
		};
		// Step 2: find gate which distance is less or equal than the
		// minimum distance between fugitive and antagonists.
		int idx_gate = -1;
		for (int i = 0; i < gates_distance.size(); i++)
		{
			if (gates_distance[i].size() < antagonists_distance[idx_near].size())
			{
				if (gates_distance[i].size() < gates_distance[idx_gate].size())
				{
					idx_gate = i;
				};
			};
		};
		if (idx_gate == -1)
		{
			idx_gate = idx_near;
			cout << "No gates satisfying the needs found, relying on "
					"minimum distance"
				 << endl;
		};

		// set_plan
		// cout << "Setting plan" << endl;
		self->set_plan(gates_distance[idx_gate]);
		// cout << "Print plan chosen:" << endl;
		// for(int i=0; i<gates_distance[idx_gate].size(); i++){
		//     cout << gates_distance[idx_gate][i] << endl;
		// };
		// cout << endl;

		// set desire
		it_g = wr.world_gates.begin();
		for (int i = 0; i < idx_gate; i++)
		{
			++it_g;
		};
		self->set_desire("( is_in " + self->ID + " " + it_g->first + " )");
		to_log("End aware behaviour");
		return problem_name + "_" + it_g->first;
		break;
	};
	default:
	{
		return "NaN";
		break;
	};
	};
};

vector<string> robot_fugitive::make_plan(bool apply, string domain_name,
										 string problem_name,
										 string plan_name)
{
	run_planner("/home/" + string(getenv("USER")) +	"/FF-v2.3/",
				"/home/" + string(getenv("USER")) + "/.ros/" + filesPath + "/" + domain_name + ".pddl",
				"/home/" + string(getenv("USER")) + "/.ros/" + filesPath + "/" + problem_name + ".pddl",
				"/home/" + string(getenv("USER")) + "/.ros/" + filesPath + "/" + plan_name + ".plan");

	string tmp_line;
	ifstream pddl_in(filesPath + "/" + plan_name + ".plan");
	vector<string> tmp_plan;
	bool pushing = false;
	if (pddl_in.is_open())
	{
		while (pddl_in)
		{
			getline(pddl_in, tmp_line);
			if (tmp_line.substr(0, tmp_line.find(" ")) == "step")
			{
				pushing = true;
			}

			if (pushing)
			{
				if (tmp_line.empty())
				{
					pushing = false;
				}
				else
				{
					tmp_line.erase(0, 11);
					// cout << tmp_line << endl;
					tmp_plan.push_back(tmp_line);
				}
			}
		};
		// Delete last element that is an empty line
		//tmp_plan.pop_back();
	}
	else
	{
		cout << "Unable to open input plan file, probably no plan found"
			 << endl;
	};
	// using ifstream returns an empty line after the last one in file
	tmp_plan.pop_back(); // remove empty line

	if (apply)
	{
		self->set_plan(tmp_plan);
	};
	return tmp_plan;
};

/**
 * This method is used to print relevant informations regarding the fugitive
 * robot.
 */
void robot_fugitive::info()
{
	self->info();
	cout << "Behaviour: " << get_behaviour() << endl;
	if (antagonists.size() != 0)
	{
		cout << "Antagonists: " << endl;
		for (int i = 0; i < antagonists.size(); i++)
		{
			cout << "\t- " << antagonists[i]->ID << endl;
		};
	};
	cout << "Path to PDDL files: " << filesPath << endl;
};

/**
 * \fun
 * This is the default constructor for the structure.
 * @param _self: Robot\*. Is the robot in use.
 * @param path: string. Is the path in which the robot pddl files will be
 * created.
 */
robot_catcher::robot_catcher(Robot *_self, string path)
{
	self = _self;
	filesPath = path;
};

void robot_catcher::add_antagonist(Robot *r_ant)
{
	antagonists.push_back(r_ant);
};

void robot_catcher::write_file(string file_name, string what_to_write,
							   string extension)
{
	ofstream file_out(filesPath + "/" + file_name + extension);
	if (file_out.is_open())
	{
		file_out << what_to_write;
		file_out.close();
	}
	else
	{
		cout << "Unable to open output stream" << endl;
	};
};

/**
 * \fun
 * This function is used to write a log message exploiting the Robot
 * logger attribute existing in the Robot instance inside the robot_catcher
 * struct
 * @param message: string. It is the message to write on the logger
 */
void robot_catcher::to_log(string message){
	self->l->add_event(self->ID + ": " + message);
};

void robot_catcher::make_pddl_files(World_representation wr,
									behaviour_fugitive b_ant, bool do_plan)
{
	if (antagonists.size() == 0)
	{
		string error_msg = "No antagonists assigned to " + self->ID +
						   " try running the method \"trade_fugitives()\""
						   " of the struct Robot_manager.\n";
		to_log(error_msg);
		throw std::logic_error(error_msg);
		/*
		cout << "No antagonists assigned to " << self->ID
			 << " try running the method \"trade_fugitives()\" of the struct"
			 << " robot manager." << endl;
		*/
		return;
	};
	cout << "First line catcher " << endl;
	string domain_name = "domain_" + self->ID;

	// Write pddl header
	string pddl_domain = "(define (domain " + domain_name + ")\n";

	// Write requirements
	pddl_domain += "\t(:requirements\n"
				   "\t\t:strips :typing :negative-preconditions\n"
				   "\t\t:disjunctive-preconditions\n"
				   "\t\t:conditional-effects\n"
				   "\t)\n";

	// create plans for each antagonists -> used to make an aware decision
	to_log("Creating plans for the ghost antagonists");
	for (int i = 0; i < antagonists.size(); i++)
	{
		robot_fugitive ghost_fugitive = robot_fugitive(antagonists[i],
													   ".tmp", b_ant);
		// Suppose that the catcher is the only threat
		ghost_fugitive.add_antagonist(self);
		string g_domain_name = ghost_fugitive.make_pddl_domain_file();
		string g_problem_name = ghost_fugitive.make_pddl_problem_file(wr);
		string id_ghost = ghost_fugitive.self->ID;
		antagonists_plans[id_ghost] = ghost_fugitive.make_plan(false,
															   g_domain_name,
															   g_problem_name,
															   g_problem_name);
		// Remove file
		remove((filesPath + "/" + g_problem_name + ".plan").c_str());
	};

	set<string> cells;
	map<string, vector<string>>::iterator it;

	// find subset of cells used in the plans of the antagonists
	to_log("Searching subset of cells in plan");
	for (it = antagonists_plans.begin(); it != antagonists_plans.end(); ++it)
	{
		// iterate until last step of plan to avoid to misclassify the gate
		for (int i = 0; i < it->second.size() - 1; i++)
		{
			// string plan_step = remove_first_and_last_char(it->second[i]);

			// vector<string> temp = string_to_vector(plan_step, " ");

			vector<string> temp = string_to_vector(it->second[i], " ");			
			for (int j = temp.size() - 1; j > temp.size() - 3; j--)
			{
				cells.insert(temp[j]);
			}
		}
	}

	// write types
	to_log("Writing types");
	pddl_domain += "\t(:types\n"
				   "\t\tfugitive catcher - robot\n";

	for (int i = 0; i < antagonists.size(); i++)
	{
		pddl_domain += "\t\t" + antagonists[i]->ID + " - fugitive\n";
	};
	pddl_domain += "\t\tCELL gate - location\n";
	for (set<string>::iterator it_s = cells.begin(); it_s != cells.end(); ++it_s)
	{
		pddl_domain += "\t\t" + *it_s + " - CELL\n";
	};
	pddl_domain += "\t)\n";

	// write predicates
	to_log("Writing predicates");
	pddl_domain += "\t(:predicates\n"
				   "\t\t(is_in ?r - robot ?loc - location)\n"
				   "\t\t(connected ?loc_start - location ?loc_end - location)\n"
				   "\t\t(captured ?r - fugitive)\n"
				   "\t\t(escaped ?r - fugitive)\n" // new
				   "\t)\n";
	
	// write actions
	to_log("Writing action \"Move\"");
	pddl_domain += "\t(:action move\n"
				   "\t\t:parameters\n"
				   "\t\t\t(\n"
				   "\t\t\t\t?r_c - catcher\n"
				   "\t\t\t\t?loc_start - location\n"
				   "\t\t\t\t?loc_end - location\n";
	
	to_log("Started searching subset of cells in plan");
	vector<string> antagonists_pddl;
	for (int i = 0; i < antagonists.size(); i++)
	{
		vector<string> tmp_s = string_to_vector(antagonists[i]->ID, "_");
		antagonists_pddl.push_back("?r_f_" + tmp_s[tmp_s.size() - 1]);
		pddl_domain += "\t\t\t\t" + 
					   antagonists_pddl[antagonists_pddl.size()-1]
					   + " - "
					   + antagonists[i]->ID
					   + "\n";
	};

	for (set<string>::iterator it_s = cells.begin(); it_s != cells.end();
		 ++it_s)
	{
		vector<string> tmp_s = string_to_vector(*it_s, "_");
		pddl_domain += "\t\t\t\t?c_" + tmp_s[tmp_s.size() - 1] +
					   " - " + *it_s + "\n";
	};
	pddl_domain += "\t\t\t)\n";

	// write preconditions
	pddl_domain += "\t\t:precondition\n"
				   "\t\t\t(and\n"
				   "\t\t\t\t(is_in ?r_c ?loc_start)\n"
				   "\t\t\t\t(or\n"
				   "\t\t\t\t\t( connected ?loc_start ?loc_end )\n"
				   "\t\t\t\t\t( connected ?loc_end ?loc_start )\n"
				   "\t\t\t\t)\n";
	// new
	if (antagonists_pddl.size() == 1){
		pddl_domain += "\t\t\t\t(not ( escaped " + antagonists_pddl[0] +
					   ") )\n";
	}else{
		pddl_domain += "\t\t\t\t( or\n";
		for(int i=0; i < antagonists_pddl.size(); i++){
			pddl_domain += "\t\t\t\t\t(not ( escaped " + antagonists_pddl[i] +
					   	   ") )\n";
		};
		pddl_domain += "\t\t\t\t)\n";
	};
	// end new

	pddl_domain += "\t\t\t)\n";

	// write effects
	pddl_domain += "\t\t:effect\n"
				   "\t\t\t(and\n"
				   "\t\t\t\t( not ( is_in ?r_c ?loc_start ) )\n"
				   "\t\t\t\t( is_in ?r_c ?loc_end )\n";
	
	to_log("Parsing antagonists plans");
	for (it = antagonists_plans.begin(); it != antagonists_plans.end(); ++it)
	{
		pddl_domain += "\n\t\t\t\t; Plan for fugitive: " + it->first;
		// make a condition for each step of the plan
		vector<string> tmp_s = string_to_vector(it->first, "_");
		// Must catch it before it goes to the gate, i.e -> n-1 plan steps
		for (int i = it->second.size() - 2; i >= 0; i--)
		{
			vector<string> tmp_plan_p1 = string_to_vector(it->second[i], " ");
			vector<string> tmp_plan_p2 = string_to_vector(
				tmp_plan_p1[tmp_plan_p1.size() - 1],
				")");
			vector<string> cell_s = string_to_vector(tmp_plan_p1[tmp_plan_p1.size() - 2], "_");
			vector<string> cell_e = string_to_vector(tmp_plan_p2[0], "_");

			pddl_domain += "\n\t\t\t\t(when\n"
						   "\t\t\t\t\t(is_in ?r_f_" +
						   tmp_s[tmp_s.size() - 1] +
						   " ?c_" + cell_s[cell_s.size() - 1] + " )\n"
						   "\t\t\t\t\t(and\n"
						   "\t\t\t\t\t\t( is_in ?r_f_" +
						   tmp_s[tmp_s.size() - 1] +
						   " ?c_" + cell_e[cell_e.size() - 1] + " )\n"
						   "\t\t\t\t\t\t( not ( is_in ?r_f_" +
						   tmp_s[tmp_s.size() - 1] + " ?c_" +
						   cell_s[cell_s.size() - 1] + " ) )"
						   "\n\t\t\t\t\t)\n"
						   "\t\t\t\t)";

			// new -> tells when fugitive has escaped
			if (i == it->second.size()-2){
				pddl_domain += "\n\t\t\t\t(when\n"
							   "\t\t\t\t\t(is_in ?r_f_" +
							   tmp_s[tmp_s.size() - 1] +
							   " ?c_" + cell_s[cell_s.size() - 1] + " )\n"
							   "\t\t\t\t\t(escaped " + tmp_s[tmp_s.size()-1] + 
							   ")\n"
							   "\t\t\t\t)";
			};
			// end new
		};
	};

	to_log("Writing action \"Capture\"");
	pddl_domain += "\n\t\t\t)\n";
	pddl_domain += "\t)\n";

	pddl_domain += "\t(:action capture\n"
				   "\t\t:parameters (?r_catcher - catcher "
				   "?r_fugitive - fugitive ?loc - cell)\n"
				   "\t\t:precondition\n"
				   "\t\t\t(and\n"
				   "\t\t\t\t( is_in ?r_catcher ?loc)\n"
				   "\t\t\t\t( is_in ?r_fugitive ?loc)\n"
				   "\t\t\t)\n"
				   "\t\t:effect ( captured ?r_fugitive)\n";
	pddl_domain += "\t)\n";

	// Write end of pddl domain
	pddl_domain += "\n)";

	// Write domain file to disk
	// cout << "Catcher domain: -------------" << endl
	//     << pddl_domain << endl << endl;
	int tmp_folder = mkdir(filesPath.c_str(), 0777);
	// cout << "Made temporary folder: " << tmp_folder << endl;
	write_file(domain_name, pddl_domain, ".pddl");
	// cout << "Domain PDDL written" << endl;

	// Write problem file ----------------------------------------------------
	to_log("Writing problem file");
	string problem_name = "problem_" + self->ID;
	string pddl_problem = "(define (problem " + problem_name + " )\n"
															   "\t(:domain " +
						  domain_name + ")\n";

	// write objects
	pddl_problem += "\t(:objects\n";

	// Part 1: write cells
	to_log("Writing cells");
	string upper_id;
	map<string, World_node>::iterator it_node;
	for (it_node = wr.world_free_cells.begin();
		 it_node != wr.world_free_cells.end(); ++it_node)
	{
		pddl_problem += "\t\t" + upperify(it_node->first) + " - ";
		if (cells.count(it_node->first) != 0)
		{
			pddl_problem += it_node->first + "\n";
		}
		else
		{
			pddl_problem += "CELL\n";
		};
	};

	// Part 2: write gates
	to_log("Writing gates");
	for (it_node = wr.world_gates.begin(); it_node != wr.world_gates.end();
		 ++it_node)
	{
		pddl_problem += "\t\t" + upperify(it_node->first) + " - gate\n";
	};

	// Part 3: write agents
	to_log("Writing agents");
	pddl_problem += "\t\t" + upperify(self->ID) + " - " +
					self->get_type() + "\n";

	for (int i = 0; i < antagonists.size(); i++)
	{
		pddl_problem += "\t\t" + upperify(antagonists[i]->ID) + " - " +
						antagonists[i]->ID + "\n";
	};

	pddl_problem += "\t)\n";

	// Write initial state
	to_log("Writing cell connections");
	pddl_problem += "\t(:init\n";
	pddl_problem += wr.find_pddl_connections();

	// Write initial location of the agents
	to_log("Writing initial location of the agents");
	pt self_location = pt(self->location->x, self->location->y);
	for (it_node = wr.world_free_cells.begin();
		 it_node != wr.world_free_cells.end(); ++it_node)
	{
		Polygon_boost cell_p = it_node->second.cell->to_boost_polygon();
		if (boost::geometry::covered_by(self_location, cell_p))
		{
			pddl_problem += "\t\t( is_in " + upperify(self->ID) + " " +
							upperify(it_node->first) + ")\n";
			break;
		};
	};
	to_log("Written location of catcher");

	for (int i = 0; i < antagonists.size(); i++)
	{
		pt ant_location = pt(antagonists[i]->location->x,
							 antagonists[i]->location->y);
		for (it_node = wr.world_free_cells.begin();
			 it_node != wr.world_free_cells.end(); ++it_node)
		{
			Polygon_boost cell_p = it_node->second.cell->to_boost_polygon();
			if (boost::geometry::covered_by(ant_location, cell_p))
			{
				pddl_problem += "\t\t( is_in " +
								upperify(antagonists[i]->ID) +
								" " + upperify(it_node->first) +
								")\n";
				break;
			};
		};
	};
	to_log("Written location of antagonists");
	pddl_problem += "\t)\n";

	// write goal
	to_log("Writing goal");
	pddl_problem += "\t(:goal\n";
	int ant_size = antagonists.size();
	if (ant_size > 1)
	{
		pddl_problem += "\t\t(and\n";
	};
	for (int i = 0; i < ant_size; i++)
	{
		pddl_problem += "\t\t\t ( captured " + upperify(antagonists[i]->ID) +
						" )\n";
	};
	if (ant_size > 1)
	{
		pddl_problem += "\t\t)\n";
	};
	pddl_problem += "\t)\n";

	// write file end
	pddl_problem += ")\n";

	// write file to disk
	write_file(problem_name, pddl_problem, ".pddl");
	
	to_log("Calling planner");
	// make and apply plan for the catcher
	string plan_name = "catcher_plan";
	make_plan(true, domain_name, problem_name, plan_name);
	// Remove plan
	remove((filesPath + "/" + plan_name + ".plan").c_str());
};

/**
 * \fun make_plan
 * Use this method to call the planner over the domain and problem files to
 * generate the plan.
 * @param apply: bool. Set the flag to true if the plan found has to be saved
 * inside the homonymous variable in the self attribute.
 * @param domain_name: string. Is the path in which the domain file can be
 * found.
 * @param problem_name: string. Is the path in which the problem file can be
 * found.
 * @param plan_name: string. Is the path and name in which to save the plan
 * file.
 *
 * @return vector<string>: the plan found.
 */
vector<string> robot_catcher::make_plan(bool apply, string domain_name,
										string problem_name,
										string plan_name)
{
	// to_log(self->ID + ": Started running planner");
	run_planner("/home/" + string(getenv("USER")) + "/FF-v2.3/",
				"/home/" + string(getenv("USER")) + "/.ros/" + filesPath + "/" + domain_name + ".pddl",
				"/home/" + string(getenv("USER")) + "/.ros/" + filesPath + "/" + problem_name + ".pddl",
				"/home/" + string(getenv("USER")) + "/.ros/" + filesPath + "/" + plan_name + ".plan");
	// to_log(self->ID + ": Ended planner run");

	string tmp_line;
	ifstream pddl_in(filesPath + "/" + plan_name + ".plan");
	vector<string> tmp_plan;
	bool pushing = false;
	if (pddl_in.is_open())
	{
		while (pddl_in)
		{
			getline(pddl_in, tmp_line);
			if (tmp_line.substr(0, tmp_line.find(" ")) == "step")
			{
				pushing = true;
			}

			if (pushing)
			{
				if (tmp_line.empty())
				{
					pushing = false;
				}
				else
				{
					tmp_line.erase(0, 11);
					// cout << tmp_line << endl;
					tmp_plan.push_back(tmp_line);
				}
			};
			// tmp_plan.pop_back();
		};
	}
	else
	{
		string error_msg = "Unable to open input plan file. Either folder "
						   "permission error or No plan Fout -> exit code 12";
		to_log(error_msg);
		throw std::logic_error(error_msg);
	};
	// using ifstream returns an empty line after the last one in file
	tmp_plan.pop_back(); // remove empty line
	
	if (apply)
	{
		self->set_plan(tmp_plan);
		to_log("Plan found and loaded in structure");
	};

	return tmp_plan;
};

/**
 * \fun info()
 * This method is used to print relevant informations regarding the catcher
 * robot.
 */
void robot_catcher::info()
{
	self->info();
	if (antagonists.size() != 0)
	{
		cout << "Antagonists: " << endl;
		for (int i = 0; i < antagonists.size(); i++)
		{
			cout << "\t- " << antagonists[i]->ID << endl;
		};
	};
	cout << "Path to PDDL files: " << filesPath << endl;
};

/**
 * \fun run_planner
 * This function runs the planner and saves it to the specified plan file.
 * The default location of the plan is the current working directory and
 * the filename is MyPlan.plan
 * @param planner_path: string. Is the path in which the callable executable
 * file of the planner is located.
 * @param domain_file_path: string. Is the path of the domain file to use.
 * @param problem_file_path: string. Is the path of the problem file to use.
 */
void run_planner(string planner_path, string domain_file_path,
				 string problem_file_path, string plan_path = "MyPlan.plan")
{

	cout << "called planner for: " << planner_path << "ff" << endl
		 << domain_file_path
		 << endl
		 << problem_file_path << endl
		 << plan_path << endl;

	string command = "cd " + planner_path + " \n ./ff -o " + domain_file_path +
					 " -f " + problem_file_path + " > " + plan_path;
	std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"),
												  pclose);
};
