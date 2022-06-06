#ifndef __ROBOT_MANAGER_H__
#define  __ROBOT_MANAGER_H__

#include "robots.h"
#include <string>
#include <math.h>
#include <thread> 

/**
 * \struct
 * This struct is made to represent a supervisor class of all the agents
 * inside the environment.
 * Attributes available are:
 * @param fugitives: map<string, robot_fugitive\*>. It is the map containing
 * all the agents having the role of fugitives.
 * @param catchers: map<string, robot_catcher\*>. It is the map containing
 * all the agents having the role of catchers.
 * @param rm_logger: logger\*. It is the logger instance used to record events.
 * @param trading_done: bool. It is a flag telling whether or not the
 * association of the antagonists have been done.
 *
 * Available methods are:
 * @see parse_map_robots(map<string, Robot\*> map_r, string f_path). This
 * method is used to parse a map of agents and add it to the manager classified
 * using their type.
 * @see void add_robot(Robot* r, string f_path). This method is used to add a
 * single agent to the manager depending by its type.
 * @see info(bool detailed). This method is used to print relevant informations
 * regarding the manager. If the flag detailed is turned on it will print also
 * all the informations of the robots it contains.
*/
typedef struct robot_manager{
	map<string, robot_fugitive*> fugitives;
	map<string, robot_catcher*> catchers;
	logger* rm_logger = NULL;
	bool trading_done = false;

	robot_manager(logger* l=new logger);
	void parse_map_robots(map<string, Robot*> map_r, string f_path=".tmp");
	void add_robot(Robot* r, string f_path=".tmp");
	void trade_fugitives();
	void info(bool detailed=false);
	void run_agents_planners(World_representation wr,
							 behaviour_fugitive supposed_fugitive_behaviour);
}robot_manager;

void thread_fugitive_planner(map<string, robot_fugitive *>::iterator f_it,
							 World_representation wr);
void thread_catcher_planner(map<string, robot_catcher *>::iterator c_it,
  						    World_representation wr,
							behaviour_fugitive type);

#endif
