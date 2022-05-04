#include "robot_manager.h"
#include <sys/stat.h>
#include <../boost/geometry.hpp>
#include <fstream>

using pt = boost::geometry::model::d2::point_xy<double>;
using Polygon_boost = boost::geometry::model::polygon<pt>;

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
robot_fugitive::robot_fugitive(Robot* _self, string path){
	self = _self;
	filesPath = path;
};

void robot_fugitive::add_antagonist(Robot* r_ant){
	antagonists.push_back(r_ant);
};

void robot_fugitive::set_behaviour(behaviour_fugitive b){
	behaviour = b;
};

void robot_fugitive::make_pddl_domain_file(World_representation wr){
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
	ofstream pddl_out(filesPath + "/" + domain_name + ".pddl");
	if (pddl_out.is_open()){
		pddl_out << domain_file;
		pddl_out.close();
	}else{
		cout << "Unable to open output stream" << endl;
	};
};

void robot_fugitive::make_pddl_problem_file(World_representation wr){
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
				ofstream pddl_out(filesPath + "/" + tmp_name + ".pddl");
				if (pddl_out.is_open()){
					pddl_out << tmp_out;
					pddl_out.close();
				}else{
					cout << "Unable to open output stream" << endl;
				};
				run_planner("/home/" + string(getenv("USER")) +
							"/.planutils/packages/downward/run",
							filesPath + "/" + "domain_" + self->ID + ".pddl",
							filesPath + "/" + tmp_name + ".pddl",
							filesPath + "/" + tmp_name + ".plan");
				string tmp_line;
				ifstream pddl_in(filesPath + "/" + tmp_name + ".plan");
				vector<string> tmp_plan;
				if (pddl_in.is_open()){
					while(pddl_in){
						getline(pddl_in, tmp_line);
						tmp_plan.push_back(tmp_line);
					};
					tmp_plan.pop_back();
					all_plans.push_back(tmp_plan);
				}else{
					cout << "Unable to open input plan file" << endl;
				};
			};
			int idx_least = 0;
			for(int i=0; i<all_plans.size(); i++){
				if (all_plans[i].size() < all_plans[idx_least].size()){
					idx_least = i;
				};
			};
			self->set_plan(all_plans[idx_least]);
			break;
		};
		case undeterministic:{
			break;
		};
		case aware:{
			break;
		};

	};
	/*
	problem_file += "\t)\n";

	// Write file end
	problem_file += "\n)";
	
	// make folder if it doesn't exist
	ofstream pddl_out(filesPath + "/" + problem_name + ".pddl");
	if (pddl_out.is_open()){
		pddl_out << problem_file;
		pddl_out.close();
	}else{
		cout << "Unable to open output stream" << endl;
	};
	*/

};

void robot_fugitive::make_plan(){
	run_planner("/home/" + string(getenv("USER")) +
				"/.planutils/packages/downward/run",
				filesPath + "/" + "domain_" + self->ID + ".pddl",
				filesPath + "/" + "problem_" + self->ID + ".pddl",
				filesPath + "/" + self->ID + ".plan");
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
	cout << endl << "Path to PDDL files: " << filesPath << endl;
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

bool robot_catcher::make_pddl_domain_file(World_representation wr){
	return true;
};

bool robot_catcher::make_pddl_problem_file(World_representation wr){
	return true;
};

bool robot_catcher::make_plan(){
	return true;
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
	cout << endl << "Path to PDDL files: " << filesPath << endl;
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
