#ifndef __ROBOT_MANAGER_H__
#define  __ROBOT_MANAGER_H__

#include "robots.h"
#include <string>
#include "../World_representation/world_representation.h"

typedef struct robot_manager{
	map<string, robot_fugitive*> fugitives;
	map<string, robot_catcher*> catchers;
	void trade_fugitives();
}robot_manager;

#endif
