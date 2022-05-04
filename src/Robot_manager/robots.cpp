#include "robot_manager.h"


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

void Robot::set_plan(vector<string> p)
{
	plan = p;
};

/**
 * \fn void Robot::info()
 * This fuction serves the purpose to print on screen the details of the
 * robot on which it is called.
 */
void Robot::info()
{
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

	cout << "Robot: " << ID << endl
		 << "Type: " << get_type() << endl
		 << "Location: " << location->x << " - " << location->y << endl
		 << "Max curvature angle: " << max_curvature_angle << endl
		 << "Offset in use: " << offset << endl;

	// Print plan if available
	if (plan.size() != 0)
	{
		for (int i = 0; i < plan.size(); i++)
		{
			cout << "\t- " << i << ": " << plan[i] << endl;
		};
	};
};
