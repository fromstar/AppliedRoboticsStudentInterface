#include "robot_manager.h"

/**
 * \fun
 * This is the default constructor of the robot_manager struct.
 * @param l: logger\*. It is the logger instance which will be used to save
 * the events.
 */
robot_manager::robot_manager(logger* l){
	rm_logger = l;
};
 
/**
 * \fun
 * This method allows to add a robot to the struct.
 * @param r: Robot\*. It is a pointer to the Robot instance to be added to
 * the manager.
 * @param f_path: string. It is the path in which the robot will save files
 * necessary to perform its tasks.
 */
void robot_manager::add_robot(Robot* r, string f_path){
	switch(r->type){
		case catcher:
			catchers[r->ID] = new robot_catcher(r, f_path);
			break;
		case fugitive:
			fugitives[r->ID] = new robot_fugitive(r, f_path);
			break;
		default:
			// All robots non fugitives are treated as potential threats for
			// the others hence catchers.
			catchers[r->ID] = new robot_catcher(r, f_path);
			break;
	};
	rm_logger -> add_event("Added robot " + r->ID + " to the robot manager");
};

/**
 * \fun
 * This method allows to parse a map of Robot instance and add its content
 * to the menager.
 * @param map_r: map<string, Robot\*>. It is the map containing the Robot
 * instances using as keys the ID of the robots.
 * @param f_path: string. It is the path in which the added robots will save
 * the files necessary for their execution.
 */
void robot_manager::parse_map_robots(map<string, Robot*> map_r, string f_path){
	for( map<string, Robot*>::iterator it = map_r.begin(); it != map_r.end();
		 ++it){
		add_robot(it->second, f_path);
	};
	rm_logger -> add_event("After parsing added " + map_r.size() +
						   string(" robots"));
};

/**
 * \fun
 * This method is used to chose the antagonist of each robot.
 * It works under geometrical assumptions.
 */
void robot_manager::trade_fugitives(){
	if (fugitives.size() == catchers.size()){
		map<string, robot_fugitive*>::iterator it_f = fugitives.begin();
		map<string, robot_catcher*>::iterator it_c = catchers.begin();

		it_f -> second -> add_antagonist(it_c->second->self);
		it_c -> second -> add_antagonist(it_f->second->self);
	};
	
	// Add other options

	rm_logger -> add_event("Ended Robot trading");
};

/**
 * \fun
 * This method is used to print relevant informations regarding the robot
 * manager.
 * @param detailed: bool. It is a flag telling whether or not to show also
 * the informations regarding the robots contained in the robot manager.
 * It is setted to false by default.
 */
void robot_manager::info(bool detailed){
	cout << "Robot Menager info:" << endl
		 << "\tCatchers available: " << catchers.size() << endl
		 << "\tFugitives available: " << fugitives.size() << endl;
	if (detailed){
		map<string, robot_fugitive*>::iterator it_f;
		map<string, robot_catcher*>::iterator it_c;

		cout << string(80, '-') << endl;
		// print info for fugitives
		for (it_f=fugitives.begin(); it_f != fugitives.end(); ++it_f){
			it_f -> second -> info();
			cout << endl;
		};
		// print info for catchers
		for (it_c=catchers.begin(); it_c != catchers.end(); ++it_c){
			it_c -> second -> info();
			cout << endl;
		};
		cout << string(80, '-') << endl;
	};
};
