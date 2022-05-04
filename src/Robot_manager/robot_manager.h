#ifndef __ROBOT_MANAGER_H__
#define  __ROBOT_MANAGER_H__

#include "robots.h"
#include <string>
// #include "../World_representation/world_representation.h"

typedef struct robot_manager{
	map<string, robot_fugitive*> fugitives;
	map<string, robot_catcher*> catchers;

	void parse_map_robots(map<string, Robot*> map_r, string f_path=".tmp");
	void add_robot(Robot* r, string f_path=".tmp");
	void trade_fugitives();
	void info();
}robot_manager;

#endif
