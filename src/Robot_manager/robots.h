#ifndef __ROBOTS_H__
#define __ROBOTS_H__

#include <string>
#include <vector>

#include "../Log/logger.h"
#include "../Utility/utility.h"
#include "../World_representation/world_representation.h"

/**
 * \enum {fugitive, catcher, undefined}
 * These types identify the role of the robot in the environment.
 * fugitive: The robot must chose an exit point in the arena (i.e. gate) and
 * reach it without colliding with anything.
 * catcher: The robot must catch the fugitives before they reach the exit
 * point.
 * undefined: The robot has yet to be assigned a role.
 */
typedef enum { fugitive, catcher, undefined } Robot_type;

/**
 * \enum {least_steps, undeterministic, aware}
 * This type serves the purpose to define the behaviour of the fugitive robot.
 * Specifically:
 * 	- least_steps: the robot will chose as gate the one reachable in the
 * minimum number of steps.
 * 	- undeterministic: the robot will chose as gate a random one.
 * 	- aware: the robot will chose as gate the one which allows for a better
 * 	  escape depending the position of the catcher.
 */
typedef enum { least_steps, undeterministic, aware } behaviour_fugitive;

/**
 * \struct Robot
 * This struct represent a robot in the environment. Its parameters are:
 * @param ID: string. It is the unique identifier of the robot.
 * @param type: Robot_type. It is the role of the robot in the environment.
 * @param x: double. It is the robot location over the abscissa axis.
 * @param y: double. It is the robot location over the ordinates axis.
 * @param theta: double. Initial orientation angle.
 * @param max_curvature_angle: double. It is the maximum angle that the robot
 * is able to reach during a turning action.
 * @param offset: double. It is the value of which the robot physical
 * dimensions are increased to account for a safe movement in the environment.
 * @param desire: string. Is the goal state in which the robot is required to
 * be. The desire is written as a PDDL statement.
 * @param plan: pointer of type vector<string>. It is the plan that the robot
 * has to execute, default to NULL. The plan is written as a sequence of PDDL
 * expressions.
 *
 * Available methods are:
 * @see set_id(string _id): Sets the robot id.
 * @see set_robot_type(Robot_type rt): Sets the type of the robot.
 * @se where(): This method return the location of the robot.
 * @see get_type(): This method returns the type of the robot in a
 * string format.
 * @see set_plan(vector<string> p): This method allows to set the plan that the
 * robot has to execute.
 * @see set_desire(string d): This method allows to set the goal of the robot.
 * @see info(): Prints all the informations regarding the robot.
 */
typedef struct Robot {
  // Take into account orientation -> Changes the available movements
  string ID;
  logger* l=NULL;
  Robot_type type = undefined;
  point_node *location = NULL;
  double theta = 0;
  double max_curvature_angle = 0;
  double offset = 1;
  string desire = "NaN";
  vector<string> plan;

  Robot(string _id = "Default_id", Robot_type _type = undefined,
        point_node *loc = new point_node(0, 0), double _max_curvature = 0,
        double _offset = 1, logger* _l=new logger());

  void set_id(string _id);
  void set_robot_type(Robot_type rt);
  point_node *where();
  string get_type();
  void set_logger(logger* _l);
  void set_plan(vector<string> p);
  void set_desire(string d);
  void info();
} Robot;

/**
 * This struct is used to represent a fugitve robot, it expand the Robot struct.
 * @param behaviour: behaviour_type. Defines the type of behaviour in use by
 * the robot.
 * @param self: Robot\*. Is the struct representing the fugitive robot in use.
 * @param antagonists: vector<Robot\*>. Is the list of agents that must be
 * outperformed.
 * @param filesPath: string. It is the path in which are created all the files
 * that the robot needs.
 *
 * @see add_antagonist(Robot\* r): This method is used to add an antagonist to
 * the list of antagonists.
 * @see set_behaviour(behaviour_fugitive b): This method is used to set the
 * behaviour of the robot in chosing the gate from which to escape.
 * @see make_pddl_domain_file(): This method creates the pddl domain file.
 * @see make_pddl_problem_file(): This method creates the pddl problem file.
 * @see make_plan(): This method is used to create the plan for the robot.
 * @see get_behaviour(): This method is used to return the behaviourla type of
 * the robot as a string.
 * @see info(): This method prints informations regarding the robot.
 */
typedef struct robot_fugitive {
  behaviour_fugitive behaviour = least_steps;
  Robot *self = NULL;
  vector<Robot*> antagonists;
  string filesPath;

  robot_fugitive(Robot* _self, string path=".tmp",
		  		 behaviour_fugitive behaviour=least_steps);

  // Methods
  void add_antagonist(Robot* r_ant);
  void set_behaviour(behaviour_fugitive b);
  string make_pddl_domain_file(World_representation wr);
  string make_pddl_problem_file(World_representation wr);
  void write_file(string file_name, string what_to_write,
		  		  string extension=".pddl");
  vector<string> make_plan(bool apply=false, string domain_name="domain",
		  				   string problem_name="problem",
						   string plan_name="MyPlan");
  string get_behaviour();
  void info();
} robot_fugitive;

/**
 * This struct is used to represent a catcher robot, it expand the Robot struct.
 * @param behaviour: behaviour_type. Defines the type of behaviour in use by
 * the robot.
 * @param self: Robot\*. Is the struct representing the catcher robot in use.
 * @param antagonists: vector<Robot\*>. It is the list of agents that the robot
 * has to outperfrom.
 * @param filesPath: string. It is the path in which are created all the files
 * that the robot needs.
 *
 * @see make_pddl_domain_file(): This method creates the pddl domain file.
 * @see make_pddl_problem_file(): This method creates the pddl problem file.
 * @see make_plan(): This method is used to create the plan for the robot.
 * @see get_behaviour(): This method is used to return the behaviourla type of
 * the robot as a string.
 * @see info(): This method prints informations regarding the robot.
 */
typedef struct robot_catcher {
  Robot *self = NULL;
  vector<Robot*> antagonists;
  map<string, vector<string>> antagonists_plans;
  string filesPath;
  
  robot_catcher(Robot* _self, string path=".tmp");

  // Methods
  void add_antagonist(Robot* r_ant);
  void write_file(string file_name, string what_to_write,
		  		  string extension=".pddl");
  void make_pddl_files(World_representation wr,
		  			   behaviour_fugitive b_ant=least_steps,
					   bool do_plan=true);
  vector<string> make_plan(bool apply=true, string domain_name="domain.pddl",
		  				   string problem_name="problem.pddl",
						   string plan_name="MyPlan.plan");
  void info();
} robot_catcher;

/**
 * \fun
 * Use this function to remove the first and last characters of a sentence
 * or a token.
 * @param input_string: string. It is the input string to modify
 * @returns string. It is the transformed string.
 */
string remove_first_and_last_char(string input_string);

/**
 * \fun
 * Use this function to return a word with the first letter capital.
 * It is useful on the problem file.
 * @param word: string. Is the input word.
 * @return capitalized word: string.
 */
string upperify(string word);

/**
 * \fun
 * Use this function to return a word with the first letter capital.
 * It is useful on the problem file.
 * @param word: string. Is the input word.
 * @return capitalized word: string.
 */
string lowerify(string word);

/**
 * \fun run_planner
 * Use this function to run the downward planner
 */
void run_planner(string planner_path, string domain_file_path,
				 string problem_file_path, string plan_path);

/**
 * \fun string_to_vector(string sentence, string token).
 * Use this function to split a string with respect to a token.
 * @return vector<string>
 */
vector<string> string_to_vector(string sentence, string token);
#endif
