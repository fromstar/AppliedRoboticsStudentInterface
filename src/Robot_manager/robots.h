#ifndef __ROBOTS_H__
#define __ROBOTS_H__

#include <vector>
#include <string>

#include "../Utility/utility.h"
#include "../Log/logger.h"

/**
 * \enum {fugitive, catcher, undefined}
 * These types identify the role of the robot in the environment.
 * fugitive: The robot must chose an exit point in the arena (i.e. gate) and
 * reach it without colliding with anything.
 * catcher: The robot must catch the fugitives before they reach the exit
 * point.
 * undefined: The robot has yet to be assigned a role.
 */
typedef enum {fugitive,	catcher, undefined} Robot_type;
typedef enum {least_steps, undeterministic, aware} behaviour_type;

/**
 * \struct Robot
 * This struct represent a robot in the environment. Its parameters are:
 * @param ID: string. It is the unique identifier of the robot.
 * @param type: Robot_type. It is the role of the robot in the environment.
 * @param x: double. It is the robot location over the abscissa axis.
 * @param y: double. It is the robot location over the ordinates axis.
 * @param max_curvature_angle: double. It is the maximum angle that the robot
 * is able to reach during a turning action.
 * @param offset: double. It is the value of which the robot physical
 * dimensions are increased to account for a safe movement in the environment.
 * @param plan: pointer of type vector<string>. It is the plan that the robot
 * has to execute, default to NULL.
 *
 * Available methods are:
 * @see set_id(string _id): Sets the robot id.
 * @see set_robot_type(Robot_type rt): Sets the type of the robot.
 * @see info(): Prints all the informations regarding the robot.
 */
typedef struct Robot
{
	// Take into account orientation -> Changes the available movements
	string ID;
	Robot_type type = undefined;
	point_node *location = NULL;
	double max_curvature_angle = 0;
	double offset = 1;
	string desire = "NaN";
	vector<string> plan;

	Robot(string _id = "Default_id", Robot_type _type = undefined,
		  point_node *loc = new point_node(0, 0), double _max_curvature = 0,
		  double _offset = 1);

	void set_id(string _id);
	void set_robot_type(Robot_type rt);
	point_node *where();
	string get_type();
	void set_plan(vector<string> p);
	void set_desire();
	void info();
} Robot;

typedef struct robot_fugitive{
	behaviour_type behaviour = least_steps;
	Robot* self = NULL;
	Robot* antagonist = NULL;
	string filesPath;

	// Methods
	bool make_pddl_domain_file();
	bool make_pddl_problem_file();
	bool make_plan();
	string get_behaviour();
	void info();
}robot_fugitive;

typedef struct robot_catcher{
	Robot* self = NULL;
	Robot* antagonist = NULL;
	string filesPath;

	// Methods
	string custom_domain();
	bool make_pddl_domain_file();
	bool make_pddl_problem_file();
	bool make_plan();
}robot_catcher;

#endif
