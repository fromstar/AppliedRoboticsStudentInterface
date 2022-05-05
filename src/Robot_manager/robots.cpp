#include "robot_manager.h"
#include <sys/stat.h>
#include <../boost/geometry.hpp>
#include <fstream>
#include <bits/stdc++.h>

using pt = boost::geometry::model::d2::point_xy<double>;
using Polygon_boost = boost::geometry::model::polygon<pt>;

vector<string> string_to_vector(string sentence, string token){
	stringstream ss(sentence);
	string word;
	vector<string> tmp;
	while( ss >> word){
		if (word != token){
			tmp.push_back(word);
		};
	};
	return tmp;
};

/**
 * \fun 
 * This is the default constructor for the robot struct.
 */
Robot::Robot(string _id, Robot_type _type, point_node *_loc,
			 double _max_curvature, double _offset)
{
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
void Robot::set_id(string _id)
{
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
void Robot::set_desire(string d){
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
	if (desire != "NaN"){
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
string robot_fugitive::get_behaviour(){
	string _behaviour;
	switch(behaviour){
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
robot_fugitive::robot_fugitive(Robot* _self, string path,
							   behaviour_fugitive b){
	self = _self;
	filesPath = path;
	behaviour = b;
};

void robot_fugitive::add_antagonist(Robot* r_ant){
	antagonists.push_back(r_ant);
};

void robot_fugitive::set_behaviour(behaviour_fugitive b){
	behaviour = b;
};

void robot_fugitive::write_file(string file_name, string what_to_write,
								string extension){
	ofstream file_out(filesPath + "/" + file_name + extension);
	if (file_out.is_open()){
		file_out << what_to_write;
		file_out.close();
	}else{
		cout << "Unable to open output stream" << endl;
	};

};

string robot_fugitive::make_pddl_domain_file(World_representation wr){
	// Define name of the domain
	string domain_name = "domain_" + self->ID;

	// Write header of the file
	string domain_file = "(define (domain " + domain_name + ")\n";
	
	// Write requirements
	domain_file += "\t(:requirements "
				   "\n\t\t:strips :typing :negative-preconditions"
				   "\n\t\t:disjunctive-preconditions\n\t)\n";

	// Write types
	domain_file += "\t(:types\n\t\tfugitive catcher - robot"
				   "\n\t\tcell gate - location\n\t)\n";

	// Write predicates
	domain_file += "\t(:predicates\n"
				   "\t\t(is_in ?r - robot ?loc - location)\n"
				   "\t\t(connected ?loc_start - location ?loc_end - location)"
				   "\n\t)";
	
	// Write actions
	domain_file += "\n\t(:action move"
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
				   "\n\t)";

	// Write file end
	domain_file += "\n)";
	
	// make folder if it doesn't exist
	int tmp_folder = mkdir(filesPath.c_str(), 0777);
	write_file(domain_name, domain_file, ".pddl");
	return domain_name;
};

string robot_fugitive::make_pddl_problem_file(World_representation wr){
	int tmp_folder = mkdir(filesPath.c_str(), 0777);

	// Define name of the problem
	string problem_name = "problem_" + self->ID;
	string domain_name = "domain_" + self->ID;

	// Write header of the file
	string problem_file = "(define (problem " + problem_name + ")\n";

	// Write domain in use
	problem_file += "\n\t(:domain " + domain_name + " )\n";
	
	// Write objects available
	problem_file += "\t(:objects\n";
	map<string, World_node>::iterator it;
	for(it=wr.world_free_cells.begin(); it != wr.world_free_cells.end(); ++it){
		problem_file += "\t\t\t" + it->first + " - cell\n";
	};
	for(it=wr.world_gates.begin(); it != wr.world_gates.end(); ++it){
		problem_file += "\t\t\t" + it->first + " - gate\n";
	};

	// write agents
	problem_file += "\t\t\t" + self->ID + " - fugitive";
	if (antagonists.size()!=0){
		for(int i=0; i<antagonists.size(); i++){
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
	for(it_1=wr.world_free_cells.begin(); it_1 != wr.world_free_cells.end();
		++it_1){
		Polygon_boost p1 = it_1 -> second.cell -> to_boost_polygon();
		for(it_2=wr.world_free_cells.begin(); it_2 != wr.world_free_cells.end();
			++it_2){
			if(it_1 != it_2){
				Polygon_boost p2 = it_2 -> second.cell -> to_boost_polygon();
				if (boost::geometry::touches(p1, p2)){
					problem_file += "\t\t( connected " + it_1->first + " " +
									it_2->first + " )\n";
				};
			};
		};
	};

	// gates connected with cells
	for(it_1=wr.world_free_cells.begin(); it_1 != wr.world_free_cells.end();
		++it_1){
		Polygon_boost p1 = it_1 -> second.cell -> to_boost_polygon();
		for(it_2 = wr.world_gates.begin(); it_2 != wr.world_gates.end();
			++it_2){
			Polygon_boost p2 = it_2 -> second.cell -> to_boost_polygon();
			if (boost::geometry::touches(p1, p2)){
				problem_file += "\t\t( connected " + it_1->first + " " +
								it_2->first + " )\n";
			};
		};
	};
	
	// agents location
	pt fugitive_location = pt(self->location->x, self->location->y);
	for(it_1=wr.world_free_cells.begin(); it_1 != wr.world_free_cells.end();
		++it_1){
		Polygon_boost p1 = it_1 -> second.cell -> to_boost_polygon();
		
		if (boost::geometry::covered_by(fugitive_location, p1)){
			problem_file += "\t\t( is_in " + self->ID + " " +
							it_1->first + " )\n";
			break; // can only be in one position
		};
	};

	for(it_1=wr.world_free_cells.begin(); it_1 != wr.world_free_cells.end();
		++it_1){
		Polygon_boost p1 = it_1 -> second.cell -> to_boost_polygon();
		for(int i=0; i<antagonists.size(); i++){
			pt antagonist_pos = pt(antagonists[i]->location->x,
								   antagonists[i]->location->y);
			if (boost::geometry::covered_by(antagonist_pos, p1)){
				problem_file += "\t\t( is_in " + antagonists[i]->ID + " " +
								it_1->first + " )\n";
			};
		};
	};
	problem_file += "\t)\n";
	
	// Write goal
	problem_file += "\t(:goal\n";
	switch(behaviour){
		case least_steps:{
			vector<vector<string>> all_plans;
			for (it_1 = wr.world_gates.begin(); it_1 != wr.world_gates.end();
				 ++it_1){
				string tmp_out = problem_file + "\t\t( is_in " + self->ID +
								 " " + it_1->first + " )\n\t)\n\n)";
				string tmp_name = problem_name + "_" + it_1->first;
				write_file(tmp_name, tmp_out, ".pddl");
				all_plans.push_back(make_plan(false, "domain_" + self->ID,
											  tmp_name, tmp_name));
			};
			int idx_least = 0;

			for(int i=0; i<all_plans.size(); i++){
				if (all_plans[i].size() < all_plans[idx_least].size()){
					idx_least = i;
				};
			};

			// identify desire and write goal
			it_1 = wr.world_gates.begin();
			for (int i=0; i<idx_least; i++){
				++it_1;
			};
			self->set_plan(all_plans[idx_least]);
			self->set_desire("( is_in "+ self->ID + " " + it_1->first + " )");
			return problem_name + "_" + it_1->first;
			break;
		};
		case undeterministic: {
			cout << "In undeterministic branch" << endl;
			// chose gate randomly
			srand(time(0));
			int idx_gate = rand()%wr.world_gates.size();
			map<string, World_node>::iterator it_g=wr.world_gates.begin();
			for (int i=0; i<idx_gate; i++){
				++it_g;
			};
			self->set_desire("( is_in " + self->ID + " " + it_g->first + " )");
			string tmp_file = problem_file + "\t\t" + self->desire +
							  "\n\t)\n\n)";
			string tmp_problem_name = problem_name + "_" + it_g->first;
			write_file(tmp_problem_name, tmp_file, ".pddl");
			make_plan(true, "domain_" + self->ID, tmp_problem_name,
					  tmp_problem_name);
			return tmp_problem_name;
			break;
		};
		case aware: {
			// Check if behaviour can be done
			if (antagonists.size() == 0){
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
			for(it_g=wr.world_gates.begin(); it_g != wr.world_gates.end();
				++it_g){
				string tmp_out = problem_file + "\t\t( is_in " + self->ID +
								 " " + it_g->first + " )\n\t)\n\n)";
				string tmp_name = problem_name + "_" + it_g->first;
				write_file(tmp_name, tmp_out, ".pddl");
				vector<string> plan = make_plan(false, "domain_" + self->ID,
										 		tmp_name, tmp_name);
				if (counter==0){ // computes min distance index gate
					prev_dist=plan.size();
				}else{
					if (plan.size() < gates_distance[idx_min_dist].size()){
						idx_min_dist = counter;
					};
				}
				;
				gates_distance.push_back(plan);
				counter++;
			};

			// compute distance from fugitive to antagonists
			for (int i=0; i<antagonists.size(); i++){
				pt antagonist_pos = pt(antagonists[i]->location->x,
									   antagonists[i]->location->y);
				for(it_c=wr.world_free_cells.begin();
					it_c != wr.world_free_cells.end(); ++it_c){
					Polygon_boost cell = it_c->second.cell->to_boost_polygon();
					if(boost::geometry::covered_by(antagonist_pos, cell)){
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
						break; // can only be in one location.
					};
				};
			};

			// Apply rule to chose which gate
			// Step 1: find nearest catcher to the fugitive
			int idx_near=0;
			for (int i=0; i<antagonists_distance.size(); i++){
				if (antagonists_distance[i] < antagonists_distance[idx_near]){
					idx_near = i;
				};
			};
			// Step 2: find gate which distance is less or equal than the
			// minimum distance between fugitive and antagonists.
			int idx_gate = -1;
			for (int i=0; i<gates_distance.size(); i++){
				if (gates_distance[i].size() < antagonists_distance[idx_near].size()){
					if (gates_distance[i].size() < gates_distance[idx_gate].size()){
						idx_gate = i;
					};
				};
			};
			if (idx_gate == -1){
				idx_gate = idx_near;
				cout << "No gates satisfying the needs found, relying on "
						"minimum distance" << endl;
			};
			
			// set_plan
			self->set_plan(gates_distance[idx_gate]);
			// set desire
			it_g = wr.world_gates.begin();
			for(int i=0; i<idx_gate; i++){
				++it_g;
			};
			self->set_desire("( is_in " + self->ID + " " + it_g->first + " )");
			return problem_name + "_" + it_g->first;
			break;
		};

	};
};

vector<string> robot_fugitive::make_plan(bool apply, string domain_name, 
										 string problem_name,
										 string plan_name){
	run_planner("/home/" + string(getenv("USER")) +
				"/.planutils/packages/downward/run",
				filesPath + "/" + domain_name + ".pddl",
				filesPath + "/" + problem_name + ".pddl",
				filesPath + "/" + plan_name + ".plan");
	string tmp_line;
	ifstream pddl_in(filesPath + "/" + plan_name + ".plan");
	vector<string> tmp_plan;
	if (pddl_in.is_open()){
		while(pddl_in){
			getline(pddl_in, tmp_line);
			tmp_plan.push_back(tmp_line);
		};
	}else{
		cout << "Unable to open input plan file, probably no plan found"
			 << endl;
	};
	
	// using ifstream returns an empty line after the last one in file
	tmp_plan.pop_back(); // remove empty line
	tmp_plan.pop_back(); // remove cost line

	if (apply){
		self->set_plan(tmp_plan);
	};
	return tmp_plan;
};

/**
 * This method is used to print relevant informations regarding the fugitive
 * robot.
 */
void robot_fugitive::info(){
	self->info();
	cout << "Behaviour: " << get_behaviour() << endl;
	if (antagonists.size() != 0){
		cout << "Antagonists: " << endl;
		for(int i=0; i<antagonists.size(); i++){
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
robot_catcher::robot_catcher(Robot* _self, string path){
	self = _self;
	filesPath = path;
};

void robot_catcher::add_antagonist(Robot* r_ant){
	antagonists.push_back(r_ant);
};

void robot_catcher::write_file(string file_name, string what_to_write,
		  		  			   string extension){
	ofstream file_out(filesPath + "/" + file_name + extension);
	if (file_out.is_open()){
		file_out << what_to_write;
		file_out.close();
	}else{
		cout << "Unable to open output stream" << endl;
	};
};

void robot_catcher::make_pddl_files(World_representation wr,
									behaviour_fugitive b_ant){
	if (antagonists.size() == 0){
		cout << "No antagonists assigned to " << self->ID
			 << " try running the method \"trade_fugitives()\" of the struct"
			 << " robot manager." << endl;
		return;
	};

	// Write pddl header
	string pddl_domain = "(define (domain domain_" + self->ID + ")\n";

	// Write requirements
	pddl_domain += "\t(:requirements\n"
				   "\t\t:strips :typing :negative-preconditions\n"
				   "\t\t:disjunctive-preconditions\n"
				   "\t\t:conditional-effects\n"
				   "\t)\n";
	
	// create plans for each antagonists -> used to make an aware decision
	for(int i=0; i<antagonists.size(); i++){
		robot_fugitive ghost_fugitive = robot_fugitive(antagonists[i],
													   ".tmp", b_ant);
		// Suppose that the catcher is the only threat
		ghost_fugitive.add_antagonist(self);
		string g_domain_name = ghost_fugitive.make_pddl_domain_file(wr);
		string g_problem_name = ghost_fugitive.make_pddl_problem_file(wr);
		string id_ghost = ghost_fugitive.self->ID;
		antagonists_plans[id_ghost]=ghost_fugitive.make_plan(false,
															 g_domain_name,
															 g_problem_name,
															 g_problem_name);
	};

	set<string> cells;
	map<string, vector<string>>::iterator it;

	// find subset of cells used in the plans of the antagonists
	for(it=antagonists_plans.begin(); it != antagonists_plans.end(); ++it){
		// iterate until last step of plan to avoid to misclassify the gate
		for(int i=0; i<it->second.size()-1; it++){
			vector<string> temp = string_to_vector(it->second[i], " ");
			for(int j=temp.size()-1; j > temp.size()-3; j--){
				cells.insert(temp[j]);
			};
		};
	};

	// write types
	pddl_domain += "\t(:types\n"
				   "\t\tfugitive catcher - robot\n";
	for(int i=0; i<antagonists.size(); i++){
		pddl_domain += "\t\t" + antagonists[i]->ID + " - fugitive\n";
	};
	pddl_domain += "\t\tcell gate - location\n";
	for(set<string>::iterator it_s=cells.begin(); it_s != cells.end(); ++it_s){
		pddl_domain += "\t\t" + *it_s + " - cell\n";
	};
	pddl_domain += "\t)\n";

	// write predicates
	pddl_domain += "\t(:predicates\n"
				   "\t\t(is_in ?r - robot ?loc - location)\n"
				   "\t\t(connected ?loc_start - location ?loc_end - location)\n"
				   "\t\t(captured ?r - fugitive)\n"
				   "\t)\n";

	// write actions
	pddl_domain += "\t(:action move\n"
				   "\t\t:parameters\n"
				   "\t\t\t(\n"
				   "\t\t\t?r_c - catcher\n"
				   "\t\t\t?loc_start - location\n"
				   "\t\t\t?loc_end - location\n";
	for(int i=0; i<antagonists.size(); i++){
		vector<string> tmp_s = string_to_vector(antagonists[i]->ID, "_");
		pddl_domain += "\t\t\t?r_f_" + tmp_s[tmp_s.size()-1] + " - " + 
					   antagonists[i]->ID + "\n";
	};

	for(set<string>::iterator it_s=cells.begin(); it_s != cells.end(); ++it_s){
		vector<string> tmp_s = string_to_vector(*it_s, "_");
		pddl_domain += "\t\t\t?c_" + tmp_s[tmp_s.size()-1] +
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
				   "\t\t\t\t)\n"
				   "\t\t\t)\n";

	// write effects
	pddl_domain += "\t\t:effect\n"
				   "\t\t\t(and\n"
				   "\t\t\t\t( not ( is_in ?r_c ?loc_start ) )\n"
				   "\t\t\t\t( is_in ?r_c ?loc_end )\n";
	for(it=antagonists_plans.begin(); it != antagonists_plans.end(); ++it){
		pddl_domain += "\t\t\t\t; Plan for fugitive: " + it->first;
		// make a condition for each step of the plan
		vector<string> tmp_s = string_to_vector(it->first, "_");
		for(int i=0; i<it->second.size()-1; i++){
			vector<string> tmp_plan_p1 = string_to_vector(it->second[i], " ");
			vector<string> tmp_plan_p2 = string_to_vector(
											tmp_plan_p1[tmp_plan_p1.size()-1],
											")");
			vector<string> cell_s = string_to_vector(tmp_plan_p1[tmp_plan_p1.size()-2], "_");
			vector<string> cell_e = string_to_vector(tmp_plan_p2[0], "_");

			pddl_domain += "\t\t\t\t(when\n"
						   "\t\t\t\t\t(is_in ?r_f_" + tmp_s[tmp_s.size()-1] +
						   " ?c_" + cell_s[cell_s.size()-1] + " )\n"
						   "\t\t\t\t\t(and\n"
						   "\t\t\t\t\t\t( is_in ?r_f_" + tmp_s[tmp_s.size()-1]+
						   " ?c_" + cell_e[cell_e.size()-1] + " )\n"
						   "\t\t\t\t\t\t( not ( is_in ?r_f_" +
						   tmp_s[tmp_s.size()-1] + " ?c_" +
						   cell_s[cell_s.size()-1] + " ) )"
						   "\n\t\t\t\t\t)\n"
						   "\t\t\t\t\t)\n";
			pddl_domain += "\t\t\t\t)\n";
		};
	};
	pddl_domain += "\t\t\t)\n";
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
	int tmp_folder = mkdir(filesPath.c_str(), 0777);
	write_file("domain_" + self->ID, pddl_domain, ".pddl");
};

// bool robot_catcher::make_pddl_problem_file(World_representation wr){
// 	return true;
// };

void robot_catcher::make_plan(){
};

/**
 * This method is used to print relevant informations regarding the catcher
 * robot.
 */
void robot_catcher::info(){
	self->info();
	if (antagonists.size() != 0){
		cout << "Antagonists: " << endl;
		for(int i=0; i<antagonists.size(); i++){
			cout << "\t- " << antagonists[i]->ID << endl;
		};
	};
	cout << "Path to PDDL files: " << filesPath << endl;
};

/**
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

	cout << "called planner for: " << planner_path << endl
		 << domain_file_path
		 << endl
		 << problem_file_path << endl
		 << plan_path << endl;

	string command = "exec " + planner_path + " --alias lama-first " +
					 "--plan-file " + plan_path + " " +
					 domain_file_path + " " + problem_file_path;
	system(command.c_str());
};