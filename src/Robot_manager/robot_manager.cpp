#include "robot_manager.h"

robot_manager::robot_manager(logger* l){
	rm_logger = l;
};
 
void robot_manager::add_robot(Robot* r, string f_path){
	switch(r->type){
		case catcher:
			catchers[r->ID] = new robot_catcher(r, f_path);
			break;
		case fugitive:
			fugitives[r->ID] = new robot_fugitive(r, f_path);
			break;
		default:
			// All robots not fugitives are treated as potential thrests
			// hence catchers
			catchers[r->ID] = new robot_catcher(r, f_path);
			break;
	};
	rm_logger -> add_event("Added robot " + r->ID + " to the robot manager");
};

void robot_manager::parse_map_robots(map<string, Robot*> map_r, string f_path){
	for( map<string, Robot*>::iterator it = map_r.begin(); it != map_r.end();
		 ++it){
		add_robot(it->second, f_path);
	};
	
	rm_logger -> add_event("After parsing added " + map_r.size() + string(" robots"));
};

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
